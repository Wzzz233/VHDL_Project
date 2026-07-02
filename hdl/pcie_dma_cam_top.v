//////////////////////////////////////////////////////////////////////////////////
// Module: pcie_dma_cam_top
// Description: PCIe DMA with Camera and DDR3 support
//              Based on working pcie_dma_test example with added DDR3/Camera
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
`define UD #1

module pcie_dma_cam_top #(
    parameter MEM_ROW_ADDR_WIDTH   = 15,
    parameter MEM_COL_ADDR_WIDTH   = 10,
    parameter MEM_BADDR_WIDTH      = 3,
    parameter MEM_DQ_WIDTH         = 16
)(
    // System clock - 25MHz
    input                               sys_clk,
    
    // User button (active low)
    input                               button_rst_n,
    
    // PCIe interface
    input                               ref_clk_p,
    input                               ref_clk_n,
    input                               perst_n,
    input   [1:0]                       rxn,
    input   [1:0]                       rxp,
    output  [1:0]                       txn,
    output  [1:0]                       txp,
    
    // LED signals
    output reg                          ref_led,
    output reg                          pclk_led,
    
    // DDR3 Interface
    output                              mem_rst_n,
    output                              mem_ck,
    output                              mem_ck_n,
    output                              mem_cke,
    output                              mem_cs_n,
    output                              mem_ras_n,
    output                              mem_cas_n,
    output                              mem_we_n,
    output                              mem_odt,
    output  [MEM_ROW_ADDR_WIDTH-1:0]    mem_a,
    output  [MEM_BADDR_WIDTH-1:0]       mem_ba,
    inout   [MEM_DQ_WIDTH/8-1:0]        mem_dqs,
    inout   [MEM_DQ_WIDTH/8-1:0]        mem_dqs_n,
    inout   [MEM_DQ_WIDTH-1:0]          mem_dq,
    output  [MEM_DQ_WIDTH/8-1:0]        mem_dm,
    
    // CMOS Camera Interface
    inout                               cmos_scl,
    inout                               cmos_sda,
    input                               cmos_vsync,
    input                               cmos_href,
    input                               cmos_pclk,
    input   [7:0]                       cmos_data,
    output                              cmos_reset
);

//=============================================================================
// Parameters
//=============================================================================
localparam DEVICE_TYPE = 3'b000;        // PCIe Endpoint
localparam AXIS_SLAVE_NUM = 3;
localparam CTRL_ADDR_WIDTH = MEM_ROW_ADDR_WIDTH + MEM_BADDR_WIDTH + MEM_COL_ADDR_WIDTH;

// Video parameters - 1280x720
localparam H_ACTIVE = 12'd1280;
localparam V_ACTIVE = 12'd720;

//=============================================================================
// Internal signals - PCIe
//=============================================================================
wire            axis_master_tready_mem;
wire            cross_4kb_boundary;
wire            dma_axis_slave0_tvalid;
wire    [127:0] dma_axis_slave0_tdata;
wire            dma_axis_slave0_tlast;
wire            dma_axis_slave0_tuser;

wire            sync_button_rst_n;
wire            sync_perst_n;
wire            ref_core_rst_n;
wire            s_pclk_rstn;

wire            pclk_div2;
wire            pclk;
wire            ref_clk;
wire            core_rst_n;

wire            axis_master_tvalid;
wire            axis_master_tready;
wire    [127:0] axis_master_tdata;
wire    [3:0]   axis_master_tkeep;
wire            axis_master_tlast;
wire    [7:0]   axis_master_tuser;

wire            axis_slave0_tready;
wire            axis_slave0_tvalid;
wire    [127:0] axis_slave0_tdata;
wire            axis_slave0_tlast;
wire            axis_slave0_tuser;

wire            axis_slave1_tready;
wire            axis_slave1_tvalid;
wire    [127:0] axis_slave1_tdata;
wire            axis_slave1_tlast;
wire            axis_slave1_tuser;

wire            axis_slave2_tready;
wire            axis_slave2_tvalid;
wire    [127:0] axis_slave2_tdata;
wire            axis_slave2_tlast;
wire            axis_slave2_tuser;

wire    [7:0]   cfg_pbus_num;
wire    [4:0]   cfg_pbus_dev_num;
wire    [2:0]   cfg_max_rd_req_size;
wire    [2:0]   cfg_max_payload_size;
wire            cfg_rcb;

wire    [4:0]   smlh_ltssm_state;
wire            smlh_link_up;
wire            rdlh_link_up;

reg     [22:0]  ref_led_cnt;
reg     [26:0]  pclk_led_cnt;

wire    [31:0]  p_rdata_pcie;
wire    [31:0]  p_rdata_dma;
wire            p_rdy_pcie;
wire            p_rdy_dma;

//=============================================================================
// DDR3 signals
//=============================================================================
wire            ddr_init_done;
wire            core_clk;

// DDR AXI interface (directly from/to frame buffer)
wire    [CTRL_ADDR_WIDTH-1:0]   axi_awaddr;
wire    [3:0]                   axi_awid;
wire    [3:0]                   axi_awlen;
wire    [2:0]                   axi_awsize;
wire    [1:0]                   axi_awburst;
wire                            axi_awready;
wire                            axi_awvalid;
wire    [MEM_DQ_WIDTH*8-1:0]    axi_wdata;
wire    [MEM_DQ_WIDTH-1:0]      axi_wstrb;
wire                            axi_wlast;
wire                            axi_wvalid;
wire                            axi_wready;

wire    [CTRL_ADDR_WIDTH-1:0]   axi_araddr;
wire    [3:0]                   axi_arid;
wire    [3:0]                   axi_arlen;
wire    [2:0]                   axi_arsize;
wire    [1:0]                   axi_arburst;
wire                            axi_arvalid;
wire                            axi_arready;
wire                            axi_rready;
wire    [MEM_DQ_WIDTH*8-1:0]    axi_rdata;
wire                            axi_rvalid;
wire                            axi_rlast;
wire    [3:0]                   axi_rid;

//=============================================================================
// Camera signals
//=============================================================================
wire            cfg_clk;
wire            locked;
wire            cmos_init_done;

wire            pixel_clk;
wire            cmos_de;
wire    [15:0]  cmos_pixel_data;

// Frame buffer output
wire            fram_buf_init_done;
wire            vout_de;
wire    [127:0] vout_data;

//=============================================================================
// PLL DISABLED for testing
//=============================================================================
/*
PLL u_pll (
    .clkin1     (sys_clk),
    .clkout0    (),
    .clkout1    (cfg_clk),
    .clkout2    (),
    .lock       (locked)
);
*/
assign locked = 1'b1;
assign cfg_clk = 1'b0;

reg [15:0] rstn_1ms;
always @(posedge cfg_clk) begin
    if (!locked)
        rstn_1ms <= 16'd0;
    else if (rstn_1ms != 16'h2710)
        rstn_1ms <= rstn_1ms + 1'b1;
end
wire rstn_out = (rstn_1ms == 16'h2710);

//=============================================================================
// Camera modules DISABLED for testing - will add back one by one
//=============================================================================
// Tie off camera outputs
assign cmos_reset = 1'b0;
assign cmos_scl = 1'bz;
assign cmos_sda = 1'bz;

// Tie off DDR3 AXI write interface (not used without camera)
assign axi_awaddr = 0;
assign axi_awid = 0;
assign axi_awlen = 0;
assign axi_awsize = 0;
assign axi_awburst = 0;
assign axi_awvalid = 0;
assign axi_wdata = 0;
assign axi_wstrb = 0;
assign axi_wlast = 0;
assign axi_wvalid = 0;

// Tie off DDR3 AXI read interface
assign axi_araddr = 0;
assign axi_arid = 0;
assign axi_arlen = 0;
assign axi_arsize = 0;
assign axi_arburst = 0;
assign axi_arvalid = 0;
assign axi_rready = 0;

/*
// COMMENTED OUT - power_on_delay
wire camera_rstn;
wire camera_pwnd;
wire initial_en;
power_on_delay u_power_on_delay (
    .clk_50M        (cfg_clk),
    .reset_n        (rstn_out),
    .camera1_rstn   (camera_rstn),
    .camera2_rstn   (),
    .camera_pwnd    (camera_pwnd),
    .initial_en     (initial_en)
);
assign cmos_reset = camera_rstn;

// COMMENTED OUT - reg_config
wire i2c_sclk_out;
wire i2c_sdat_io;
reg_config u_reg_config (
    .clk_25M        (cfg_clk),
    .camera_rstn    (initial_en),
    .reg_conf_done  (cmos_init_done),
    .i2c_sclk       (i2c_sclk_out),
    .i2c_sdat       (i2c_sdat_io)
);
assign cmos_scl = i2c_sclk_out;
assign cmos_sda = i2c_sdat_io;

// COMMENTED OUT - cmos_8_16bit
cmos_8_16bit u_cmos_8_16bit (
    .pclk       (cmos_pclk),
    .rst_n      (cmos_init_done),
    .de_i       (cmos_href),
    .pdata_i    (cmos_data),
    .vs_i       (cmos_vsync),
    .pixel_clk  (pixel_clk),
    .de_o       (cmos_de),
    .pdata_o    (cmos_pixel_data)
);

// COMMENTED OUT - fram_buf
fram_buf #(
    .MEM_ROW_WIDTH      (MEM_ROW_ADDR_WIDTH),
    .MEM_COLUMN_WIDTH   (MEM_COL_ADDR_WIDTH),
    .MEM_BANK_WIDTH     (MEM_BADDR_WIDTH),
    .MEM_DQ_WIDTH       (MEM_DQ_WIDTH),
    .H_NUM              (H_ACTIVE),
    .V_NUM              (V_ACTIVE),
    .PIX_WIDTH          (16)
) u_fram_buf (
    .vin_clk            (pixel_clk),
    .wr_fsync           (cmos_vsync),
    .wr_en              (cmos_de),
    .wr_data            (cmos_pixel_data),
    .init_done          (fram_buf_init_done),
    .ddr_clk            (core_clk),
    .ddr_rstn           (ddr_init_done),
    .vout_clk           (pclk_div2),
    .rd_fsync           (1'b1),
    .rd_en              (1'b1),
    .vout_de            (vout_de),
    .vout_data          (vout_data),
    .axi_awaddr         (axi_awaddr),
    .axi_awid           (axi_awid),
    .axi_awlen          (axi_awlen),
    .axi_awsize         (axi_awsize),
    .axi_awburst        (axi_awburst),
    .axi_awready        (axi_awready),
    .axi_awvalid        (axi_awvalid),
    .axi_wdata          (axi_wdata),
    .axi_wstrb          (axi_wstrb),
    .axi_wlast          (axi_wlast),
    .axi_wvalid         (axi_wvalid),
    .axi_wready         (axi_wready),
    .axi_bid            (4'd0),
    .axi_araddr         (axi_araddr),
    .axi_arid           (axi_arid),
    .axi_arlen          (axi_arlen),
    .axi_arsize         (axi_arsize),
    .axi_arburst        (axi_arburst),
    .axi_arvalid        (axi_arvalid),
    .axi_arready        (axi_arready),
    .axi_rready         (axi_rready),
    .axi_rdata          (axi_rdata),
    .axi_rvalid         (axi_rvalid),
    .axi_rlast          (axi_rlast),
    .axi_rid            (axi_rid)
);
*/

//=============================================================================
// Reset debounce (from example)
//=============================================================================
hsst_rst_cross_sync_v1_0 #(
    .RST_CNTR_VALUE(16'hC000)
) u_refclk_buttonrstn_debounce (
    .clk        (ref_clk),
    .rstn_in    (button_rst_n),
    .rstn_out   (sync_button_rst_n)
);

hsst_rst_cross_sync_v1_0 #(
    .RST_CNTR_VALUE(16'hC000)
) u_refclk_perstn_debounce (
    .clk        (ref_clk),
    .rstn_in    (perst_n),
    .rstn_out   (sync_perst_n)
);

hsst_rst_sync_v1_0 u_ref_core_rstn_sync (
    .clk        (ref_clk),
    .rst_n      (core_rst_n),
    .sig_async  (1'b1),
    .sig_synced (ref_core_rst_n)
);

hsst_rst_sync_v1_0 u_pclk_core_rstn_sync (
    .clk        (pclk),
    .rst_n      (core_rst_n),
    .sig_async  (1'b1),
    .sig_synced (s_pclk_rstn)
);

//=============================================================================
// LED logic
//=============================================================================
always @(posedge ref_clk or negedge sync_perst_n) begin
    if (!sync_perst_n) begin
        ref_led_cnt <= 23'd0;
        ref_led <= 1'b1;
    end else if (smlh_link_up & rdlh_link_up) begin
        ref_led_cnt <= ref_led_cnt + 23'd1;
        if(&ref_led_cnt)
            ref_led <= ~ref_led;
    end
end

always @(posedge pclk or negedge s_pclk_rstn) begin
    if (!s_pclk_rstn) begin
        pclk_led_cnt <= 27'd0;
        pclk_led <= 1'b1;
    end else if (smlh_link_up & rdlh_link_up) begin
        pclk_led_cnt <= pclk_led_cnt + 27'd1;
        if(&pclk_led_cnt)
            pclk_led <= ~pclk_led;
    end
end

//=============================================================================
// DMA Controller
//=============================================================================
ips2l_pcie_dma #(
    .DEVICE_TYPE    (DEVICE_TYPE),
    .AXIS_SLAVE_NUM (AXIS_SLAVE_NUM)
) u_ips2l_pcie_dma (
    .clk                    (pclk_div2),
    .rst_n                  (core_rst_n),
    
    .i_cfg_pbus_num         (cfg_pbus_num),
    .i_cfg_pbus_dev_num     (cfg_pbus_dev_num),
    .i_cfg_max_rd_req_size  (cfg_max_rd_req_size),
    .i_cfg_max_payload_size (cfg_max_payload_size),
    
    .i_axis_master_tvld     (axis_master_tvalid),
    .o_axis_master_trdy     (axis_master_tready_mem),
    .i_axis_master_tdata    (axis_master_tdata),
    .i_axis_master_tkeep    (axis_master_tkeep),
    .i_axis_master_tlast    (axis_master_tlast),
    .i_axis_master_tuser    (axis_master_tuser),
    
    .i_axis_slave0_trdy     (axis_slave0_tready),
    .o_axis_slave0_tvld     (dma_axis_slave0_tvalid),
    .o_axis_slave0_tdata    (dma_axis_slave0_tdata),
    .o_axis_slave0_tlast    (dma_axis_slave0_tlast),
    .o_axis_slave0_tuser    (dma_axis_slave0_tuser),
    
    .i_axis_slave1_trdy     (axis_slave1_tready),
    .o_axis_slave1_tvld     (axis_slave1_tvalid),
    .o_axis_slave1_tdata    (axis_slave1_tdata),
    .o_axis_slave1_tlast    (axis_slave1_tlast),
    .o_axis_slave1_tuser    (axis_slave1_tuser),
    
    .i_axis_slave2_trdy     (axis_slave2_tready),
    .o_axis_slave2_tvld     (axis_slave2_tvalid),
    .o_axis_slave2_tdata    (axis_slave2_tdata),
    .o_axis_slave2_tlast    (axis_slave2_tlast),
    .o_axis_slave2_tuser    (axis_slave2_tuser),
    
    .i_cfg_ido_req_en       (1'b0),
    .i_cfg_ido_cpl_en       (1'b0),
    .i_xadm_ph_cdts         (8'b0),
    .i_xadm_pd_cdts         (12'b0),
    .i_xadm_nph_cdts        (8'b0),
    .i_xadm_npd_cdts        (12'b0),
    .i_xadm_cplh_cdts       (8'b0),
    .i_xadm_cpld_cdts       (12'b0),
    
    .i_apb_psel             (1'b0),
    .i_apb_paddr            (9'b0),
    .i_apb_pwdata           (32'b0),
    .i_apb_pstrb            (4'b0),
    .i_apb_pwrite           (1'b0),
    .i_apb_penable          (1'b0),
    .o_apb_prdy             (p_rdy_dma),
    .o_apb_prdata           (p_rdata_dma),
    .o_cross_4kb_boundary   (cross_4kb_boundary)
);

assign axis_slave0_tvalid = dma_axis_slave0_tvalid;
assign axis_slave0_tlast  = dma_axis_slave0_tlast;
assign axis_slave0_tuser  = dma_axis_slave0_tuser;
assign axis_slave0_tdata  = dma_axis_slave0_tdata;
assign axis_master_tready = axis_master_tready_mem;

//=============================================================================
// PCIe IP
//=============================================================================
pcie_test u_ips2l_pcie_wrap (
    .button_rst_n           (sync_button_rst_n),
    .power_up_rst_n         (sync_perst_n),
    .perst_n                (sync_perst_n),
    
    .pclk                   (pclk),
    .pclk_div2              (pclk_div2),
    .ref_clk                (ref_clk),
    .ref_clk_n              (ref_clk_n),
    .ref_clk_p              (ref_clk_p),
    .core_rst_n             (core_rst_n),
    
    .p_sel                  (1'b0),
    .p_strb                 (4'b0),
    .p_addr                 (16'b0),
    .p_wdata                (32'b0),
    .p_ce                   (1'b0),
    .p_we                   (1'b0),
    .p_rdy                  (p_rdy_pcie),
    .p_rdata                (p_rdata_pcie),
    
    .rxn                    (rxn),
    .rxp                    (rxp),
    .txn                    (txn),
    .txp                    (txp),
    .pcs_nearend_loop       ({4{1'b0}}),
    .pma_nearend_ploop      ({4{1'b0}}),
    .pma_nearend_sloop      ({4{1'b0}}),
    
    .axis_master_tvalid     (axis_master_tvalid),
    .axis_master_tready     (axis_master_tready),
    .axis_master_tdata      (axis_master_tdata),
    .axis_master_tkeep      (axis_master_tkeep),
    .axis_master_tlast      (axis_master_tlast),
    .axis_master_tuser      (axis_master_tuser),
    
    .axis_slave0_tready     (axis_slave0_tready),
    .axis_slave0_tvalid     (axis_slave0_tvalid),
    .axis_slave0_tdata      (axis_slave0_tdata),
    .axis_slave0_tlast      (axis_slave0_tlast),
    .axis_slave0_tuser      (axis_slave0_tuser),
    
    .axis_slave1_tready     (axis_slave1_tready),
    .axis_slave1_tvalid     (axis_slave1_tvalid),
    .axis_slave1_tdata      (axis_slave1_tdata),
    .axis_slave1_tlast      (axis_slave1_tlast),
    .axis_slave1_tuser      (axis_slave1_tuser),
    
    .axis_slave2_tready     (axis_slave2_tready),
    .axis_slave2_tvalid     (axis_slave2_tvalid),
    .axis_slave2_tdata      (axis_slave2_tdata),
    .axis_slave2_tlast      (axis_slave2_tlast),
    .axis_slave2_tuser      (axis_slave2_tuser),
    
    .pm_xtlh_block_tlp      (),
    .cfg_send_cor_err_mux   (),
    .cfg_send_nf_err_mux    (),
    .cfg_send_f_err_mux     (),
    .cfg_sys_err_rc         (),
    .cfg_aer_rc_err_mux     (),
    .radm_cpl_timeout       (),
    
    .cfg_max_rd_req_size    (cfg_max_rd_req_size),
    .cfg_bus_master_en      (),
    .cfg_max_payload_size   (cfg_max_payload_size),
    .cfg_ext_tag_en         (),
    .cfg_rcb                (cfg_rcb),
    .cfg_mem_space_en       (),
    .cfg_pm_no_soft_rst     (),
    .cfg_crs_sw_vis_en      (),
    .cfg_no_snoop_en        (),
    .cfg_relax_order_en     (),
    .cfg_tph_req_en         (),
    .cfg_pf_tph_st_mode     (),
    .rbar_ctrl_update       (),
    .cfg_atomic_req_en      (),
    
    .cfg_pbus_num           (cfg_pbus_num),
    .cfg_pbus_dev_num       (cfg_pbus_dev_num),
    
    .radm_idle              (),
    .radm_q_not_empty       (),
    .radm_qoverflow         (),
    .diag_ctrl_bus          (2'b0),
    .cfg_link_auto_bw_mux   (),
    .cfg_bw_mgt_mux         (),
    .cfg_pme_mux            (),
    .app_ras_des_sd_hold_ltssm(1'b0),
    .app_ras_des_tba_ctrl   (2'b0),
    
    .dyn_debug_info_sel     (4'b0),
    .debug_info_mux         (),
    
    .smlh_link_up           (smlh_link_up),
    .rdlh_link_up           (rdlh_link_up),
    .smlh_ltssm_state       (smlh_ltssm_state)
);

//=============================================================================
// DDR3 Controller DISABLED for testing
//=============================================================================
// Tie off DDR3 outputs
assign mem_rst_n = 1'b0;
assign mem_ck = 1'b0;
assign mem_ck_n = 1'b1;
assign mem_cke = 1'b0;
assign mem_cs_n = 1'b1;
assign mem_ras_n = 1'b1;
assign mem_cas_n = 1'b1;
assign mem_we_n = 1'b1;
assign mem_odt = 1'b0;
assign mem_a = 0;
assign mem_ba = 0;
assign mem_dm = 0;
assign core_clk = 1'b0;
assign ddr_init_done = 1'b0;

/*
DDR3_50H u_DDR3 (
    .resetn                 (rstn_out),
    .core_clk               (core_clk),
    .pll_lock               (),
    .phy_pll_lock           (),
    .gpll_lock              (),
    .rst_gpll_lock          (),
    .ddrphy_cpd_lock        (),
    .ddr_init_done          (ddr_init_done),
    
    // AXI Write channel
    .axi_awaddr             (axi_awaddr),
    .axi_awuser_ap          (1'b0),
    .axi_awuser_id          (axi_awid),
    .axi_awlen              (axi_awlen),
    .axi_awready            (axi_awready),
    .axi_awvalid            (axi_awvalid),
    .axi_wdata              (axi_wdata),
    .axi_wstrb              ({MEM_DQ_WIDTH{1'b1}}),
    .axi_wready             (axi_wready),
    .axi_wusero_id          (),
    .axi_wusero_last        (axi_wlast),
    
    // AXI Read channel
    .axi_araddr             (axi_araddr),
    .axi_aruser_ap          (1'b0),
    .axi_aruser_id          (axi_arid),
    .axi_arlen              (axi_arlen),
    .axi_arready            (axi_arready),
    .axi_arvalid            (axi_arvalid),
    .axi_rdata              (axi_rdata),
    .axi_rid                (axi_rid),
    .axi_rlast              (axi_rlast),
    .axi_rvalid             (axi_rvalid),
    
    // APB interface
    .apb_clk                (cfg_clk),
    .apb_rst_n              (rstn_out),
    .apb_sel                (1'b0),
    .apb_enable             (1'b0),
    .apb_addr               (8'b0),
    .apb_write              (1'b0),
    .apb_ready              (),
    .apb_wdata              (16'b0),
    .apb_rdata              (),
    
    // DDR3 physical interface
    .mem_cs_n               (mem_cs_n),
    .mem_rst_n              (mem_rst_n),
    .mem_ck                 (mem_ck),
    .mem_ck_n               (mem_ck_n),
    .mem_cke                (mem_cke),
    .mem_ras_n              (mem_ras_n),
    .mem_cas_n              (mem_cas_n),
    .mem_we_n               (mem_we_n),
    .mem_odt                (mem_odt),
    .mem_a                  (mem_a),
    .mem_ba                 (mem_ba),
    .mem_dqs                (mem_dqs),
    .mem_dqs_n              (mem_dqs_n),
    .mem_dq                 (mem_dq),
    .mem_dm                 (mem_dm),
    
    // Debug signals
    .dbg_gate_start         (1'b0),
    .dbg_cpd_start          (1'b0),
    .dbg_ddrphy_rst_n       (1'b1),
    .dbg_gpll_scan_rst      (1'b0),
    .samp_position_dyn_adj  (1'b0),
    .init_samp_position_even(16'b0),
    .init_samp_position_odd (16'b0),
    .wrcal_position_dyn_adj (1'b0),
    .init_wrcal_position    (16'b0),
    .force_read_clk_ctrl    (1'b0),
    .init_slip_step         (8'b0),
    .init_read_clk_ctrl     (6'b0),
    .debug_calib_ctrl       (),
    .dbg_slice_status       (),
    .dbg_slice_state        (),
    .debug_data             (),
    .dbg_dll_upd_state      (),
    .debug_gpll_dps_phase   (),
    .dbg_rst_dps_state      (),
    .dbg_tran_err_rst_cnt   (),
    .dbg_ddrphy_init_fail   (),
    .debug_cpd_offset_adj   (1'b0),
    .debug_cpd_offset_dir   (1'b0),
    .debug_cpd_offset       (10'b0),
    .debug_dps_cnt_dir0     (),
    .debug_dps_cnt_dir1     (),
    .ck_dly_en              (1'b0),
    .init_ck_dly_step       (8'b0),
    .ck_dly_set_bin         (),
    .align_error            (),
    .debug_rst_state        (),
    .debug_cpd_state        ()
);
*/

endmodule
