// pango reference design
// Based on vendor demo, extended with DDR3 and power_on_delay test path
module pcie_dma_ddr3_cam1 #(
    parameter MEM_ROW_ADDR_WIDTH   = 15,
    parameter MEM_COL_ADDR_WIDTH   = 10,
    parameter MEM_BADDR_WIDTH      = 3,
    parameter MEM_DQ_WIDTH         = 16
)(
	input					button_rst_n,
	input					sys_clk,		// 25MHz for PLL -> DDR3

	// PCIE interface
	input					ref_clk_p,
	input					ref_clk_n,
	input					perst_n,
	input		[1:0]		rxn,
	input		[1:0]		rxp,
	output wire	[1:0]		txn,
	output wire	[1:0]		txp,

	// LED signals
	output reg				ref_led,
	output reg				pclk_led,

	// DDR3 Interface
	output				mem_rst_n,
	output				mem_ck,
	output				mem_ck_n,
	output				mem_cke,
	output				mem_cs_n,
	output				mem_ras_n,
	output				mem_cas_n,
	output				mem_we_n,
	output				mem_odt,
	output	[MEM_ROW_ADDR_WIDTH-1:0]	mem_a,
	output	[MEM_BADDR_WIDTH-1:0]		mem_ba,
	inout	[MEM_DQ_WIDTH/8-1:0]		mem_dqs,
	inout	[MEM_DQ_WIDTH/8-1:0]		mem_dqs_n,
	inout	[MEM_DQ_WIDTH-1:0]			mem_dq,
	output	[MEM_DQ_WIDTH/8-1:0]		mem_dm,

	// Camera reset (from power_on_delay)
	output				cmos_reset,

	// Camera 1 I2C interface
	output				cmos1_scl,
	inout				cmos1_sda,

	// Camera 1 data interface
	input				cmos1_pclk,		// Pixel clock from camera
	input				cmos1_vsync,	// Vertical sync
	input				cmos1_href,		// Horizontal reference (data valid)
	input		[7:0]	cmos1_data		// 8-bit pixel data
);

localparam DEVICE_TYPE = 3'b000;			// @IPC enum 3'b000, 3'b001, 3'b100
localparam AXIS_SLAVE_NUM = 3;				// @IPC enum 1 2 3

// Test unit mode signals
wire			pcie_cfg_ctrl_en;
wire			axis_master_tready_cfg;

wire			cfg_axis_slave0_tvalid;
wire	[127:0]	cfg_axis_slave0_tdata;
wire			cfg_axis_slave0_tlast;
wire			cfg_axis_slave0_tuser;

// For mux
wire			axis_master_tready_mem;
wire			axis_master_tvalid_mem;
wire	[127:0]	axis_master_tdata_mem;
wire	[3:0]	axis_master_tkeep_mem;

wire			axis_master_tlast_mem;
wire	[7:0]	axis_master_tuser_mem;

wire			cross_4kb_boundary;

wire			dma_axis_slave0_tvalid;
wire	[127:0]	dma_axis_slave0_tdata;
wire			dma_axis_slave0_tlast;
wire			dma_axis_slave0_tuser;

// Reset debounce and sync
wire			sync_button_rst_n;
wire			sync_perst_n;
wire			ref_core_rst_n;
wire			s_pclk_rstn;

// Internal signal
wire			pclk_div2/*synthesis PAP_MARK_DEBUG="1"*/;  	// 鐢ㄦ埛鏃堕挓锛寈2 5gt/s鏃讹紝涓?25MHZ 2.5gt/s鏃朵负62.5
wire			pclk/*synthesis PAP_MARK_DEBUG="1"*/;			// 鐢ㄦ埛鏃堕挓锛寈2 5gt/s鏃讹紝涓?25MHZ 2.5gt/s鏃朵负62.5
wire			ref_clk;
wire			core_rst_n;

wire			axis_master_tvalid/*synthesis PAP_MARK_DEBUG="1"*/;
wire			axis_master_tready/*synthesis PAP_MARK_DEBUG="1"*/;
wire	[127:0]	axis_master_tdata/*synthesis PAP_MARK_DEBUG="1"*/;
wire	[3:0]	axis_master_tkeep/*synthesis PAP_MARK_DEBUG="1"*/;
wire			axis_master_tlast/*synthesis PAP_MARK_DEBUG="1"*/;
wire	[7:0]	axis_master_tuser/*synthesis PAP_MARK_DEBUG="1"*/;

// AXI4-Stream slave 0 interface
wire			axis_slave0_tready;
wire			axis_slave0_tvalid;
wire	[127:0]	axis_slave0_tdata;
wire			axis_slave0_tlast;
wire			axis_slave0_tuser;
// AXI4-Stream slave 1 interface
wire			axis_slave1_tready;
wire			axis_slave1_tvalid;
wire	[127:0]	axis_slave1_tdata;
wire			axis_slave1_tlast;
wire			axis_slave1_tuser;
// AXI4-Stream slave 2 interface
wire			axis_slave2_tready_raw;
wire			axis_slave2_tready_fc;
wire			axis_slave2_tvalid;
wire	[127:0]	axis_slave2_tdata;
wire			axis_slave2_tlast;
wire			axis_slave2_tuser;

wire	[7:0]	cfg_pbus_num;
wire	[4:0]	cfg_pbus_dev_num;
wire	[2:0]	cfg_max_rd_req_size;
wire	[2:0]	cfg_max_payload_size;
wire			cfg_rcb;

wire			cfg_ido_req_en;
wire			cfg_ido_cpl_en;
wire	[7:0]	xadm_ph_cdts;
wire	[11:0]	xadm_pd_cdts;
wire	[7:0]	xadm_nph_cdts;
wire	[11:0]	xadm_npd_cdts;
wire	[7:0]	xadm_cplh_cdts;
wire	[11:0]	xadm_cpld_cdts;

wire	[4:0]	smlh_ltssm_state/*synthesis PAP_MARK_DEBUG="1"*/;//link鐘舵€佹満

// Led lights up signal
reg		[22:0]	ref_led_cnt;
reg		[26:0]	pclk_led_cnt;
wire			smlh_link_up;
wire			rdlh_link_up/*synthesis PAP_MARK_DEBUG="1"*/;

// Uart to APB 32bits
wire			uart_p_sel;
wire	[3:0]	uart_p_strb;
wire	[15:0]	uart_p_addr;
wire	[31:0]	uart_p_wdata;
wire			uart_p_ce;
wire			uart_p_we;
wire			uart_p_rdy;
wire	[31:0]	uart_p_rdata;

// APB signal
wire	[3:0]	p_strb;
wire	[15:0]	p_addr;
wire	[31:0]	p_wdata;
wire			p_ce;
wire			p_we;

// APB MUX signal
// 0~5: HSSTLP 6: Reserved 7: PCIe
// 8: config
// 9: DMA
wire			p_sel_pcie;
wire			p_sel_cfg;
wire			p_sel_dma;

wire	[31:0]	p_rdata_pcie;
wire	[31:0]	p_rdata_cfg;
wire	[31:0]	p_rdata_dma;

wire			p_rdy_pcie;
wire			p_rdy_cfg;
wire			p_rdy_dma;

// External BAR2 read override for MWR frame data
wire			mwr_rd_clk_en;
wire	[11:0]	mwr_rd_addr;
wire	[127:0]	mwr_rd_data;
wire			mwr_cmd_start;
wire			frame_done_pulse;
wire    [31:0]  fpga_prep_ctrl;
wire    [31:0]  fpga_prep_clahe;
wire    [31:0]  fpga_prep_usm;
wire    [31:0]  fpga_prep_med;
wire    [31:0]  fpga_roi_x1y1;
wire    [31:0]  fpga_roi_x2y2;
wire    [31:0]  fpga_roi_ctrl;

wire			cfg_msi_en;
wire			ven_msi_grant;
wire			ven_msi_req;
wire	[4:0]	ven_msi_vector;
wire	[2:0]	ven_msi_tc;
wire	[31:0]	cfg_msi_pending;
reg				msi_pending;

assign cfg_ido_req_en	=	1'b0;
assign cfg_ido_cpl_en	=	1'b0;
assign xadm_ph_cdts		=	8'b0;
assign xadm_pd_cdts		=	12'b0;
assign xadm_nph_cdts	=	8'b0;
assign xadm_npd_cdts	=	12'b0;
assign xadm_cplh_cdts	=	8'b0;
assign xadm_cpld_cdts	=	12'b0;
assign ven_msi_vector	=	5'd0;
assign ven_msi_tc		=	3'd0;
assign cfg_msi_pending	=	32'd0;
assign ven_msi_req		=	msi_pending & cfg_msi_en;

always @(posedge pclk_div2 or negedge core_rst_n) begin
	if (!core_rst_n) begin
		msi_pending <= 1'b0;
	end else begin
		if (ven_msi_grant)
			msi_pending <= 1'b0;
		if (frame_done_pulse)
			msi_pending <= 1'b1;
	end
end

// Rst debounce
hsst_rst_cross_sync_v1_0 #(
	.RST_CNTR_VALUE		(16'hC000)
) u_refclk_buttonrstn_debounce (
	.clk				(ref_clk),
	.rstn_in			(button_rst_n),
	.rstn_out			(sync_button_rst_n)
);

hsst_rst_cross_sync_v1_0 #(
	.RST_CNTR_VALUE		(16'hC000)
) u_refclk_perstn_debounce (
	.clk				(ref_clk),
	.rstn_in			(perst_n),
	.rstn_out			(sync_perst_n)
);

hsst_rst_sync_v1_0  u_ref_core_rstn_sync (
	.clk				(ref_clk),
	.rst_n				(core_rst_n),
	.sig_async			(1'b1),
	.sig_synced			(ref_core_rst_n)
);

hsst_rst_sync_v1_0  u_pclk_core_rstn_sync (
	.clk				(pclk),
	.rst_n				(core_rst_n),
	.sig_async			(1'b1),
	.sig_synced			(s_pclk_rstn)
);

// Clk led
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

// UART TO APB
pgr_uart2apb_top_32bit #(
	.CLK_DIV_P		(16'd145)
) u_uart2apb_top (
	.clk			(ref_clk),
	.rst_n			(ref_core_rst_n),
	.txd			(txd),
	.rxd			(rxd),
	.p_sel			(uart_p_sel),
	.p_strb			(uart_p_strb),
	.p_addr			(uart_p_addr),
	.p_wdata		(uart_p_wdata),
	.p_ce			(uart_p_ce),
	.p_we			(uart_p_we),
	.p_rdy			(uart_p_rdy),
	.p_rdata		(uart_p_rdata)
);

// APB MUX
ips2l_expd_apb_mux u_ips2l_pcie_expd_apb_mux (
	// From ref_clk domain
	.i_uart_clk				(ref_clk),
	.i_uart_rst_n			(ref_core_rst_n),
	.i_uart_p_sel			(uart_p_sel),
	.i_uart_p_strb			(uart_p_strb),
	.i_uart_p_addr			(uart_p_addr),
	.i_uart_p_wdata			(uart_p_wdata),
	.i_uart_p_ce			(uart_p_ce),
	.i_uart_p_we			(uart_p_we),
	.o_uart_p_rdy			(uart_p_rdy),
	.o_uart_p_rdata			(uart_p_rdata),
	// To pclk_div2 clock domain
	.i_pclk_div2_clk		(pclk_div2),
	.i_pclk_div2_rst_n		(core_rst_n),

	.o_pclk_div2_p_strb		(p_strb),
	.o_pclk_div2_p_addr		(p_addr),
	.o_pclk_div2_p_wdata	(p_wdata),
	.o_pclk_div2_p_ce		(p_ce),
	.o_pclk_div2_p_we		(p_we),

	// To PCIe
	.o_pcie_p_sel			(p_sel_pcie),
	.i_pcie_p_rdy			(p_rdy_pcie),
	.i_pcie_p_rdata			(p_rdata_pcie),

	// To DMA
	.o_dma_p_sel			(p_sel_dma),
	.i_dma_p_rdy			(p_rdy_dma),
	.i_dma_p_rdata			(p_rdata_dma),

	// To config
	.o_cfg_p_sel			(p_sel_cfg),
	.i_cfg_p_rdy			(p_rdy_cfg),
	.i_cfg_p_rdata			(p_rdata_cfg)
);

// DMA CTRL      BASE ADDR = 0x8000
ips2l_pcie_dma #(
	.DEVICE_TYPE			(DEVICE_TYPE),
	.AXIS_SLAVE_NUM			(AXIS_SLAVE_NUM)
) u_ips2l_pcie_dma (
	.clk					(pclk_div2),
	.rst_n					(core_rst_n),

	// Num
	.i_cfg_pbus_num			(cfg_pbus_num),
	.i_cfg_pbus_dev_num		(cfg_pbus_dev_num),
	.i_cfg_max_rd_req_size	(cfg_max_rd_req_size),
	.i_cfg_max_payload_size	(cfg_max_payload_size),

	// AXI4-Stream master interface
	.i_axis_master_tvld		(axis_master_tvalid_mem),
	.o_axis_master_trdy		(axis_master_tready_mem),
	.i_axis_master_tdata	(axis_master_tdata_mem),
	.i_axis_master_tkeep	(axis_master_tkeep_mem),

	.i_axis_master_tlast	(axis_master_tlast_mem),
	.i_axis_master_tuser	(axis_master_tuser_mem),

	// AXI4-Stream slave0 interface
	.i_axis_slave0_trdy		(axis_slave0_tready),
	.o_axis_slave0_tvld		(dma_axis_slave0_tvalid),
	.o_axis_slave0_tdata	(dma_axis_slave0_tdata),
	.o_axis_slave0_tlast	(dma_axis_slave0_tlast),
	.o_axis_slave0_tuser	(dma_axis_slave0_tuser),

	// AXI4-Stream slave1 interface
	.i_axis_slave1_trdy		(axis_slave1_tready),
	.o_axis_slave1_tvld		(axis_slave1_tvalid),
	.o_axis_slave1_tdata	(axis_slave1_tdata),
	.o_axis_slave1_tlast	(axis_slave1_tlast),
	.o_axis_slave1_tuser	(axis_slave1_tuser),

	// AXI4-Stream slave2 interface
	// Gate slave2 ready with frame stream readiness so DMA payload waits for a warmed frame stream.
	.i_axis_slave2_trdy		(axis_slave2_tready_fc),
	.o_axis_slave2_tvld		(axis_slave2_tvalid),
	.o_axis_slave2_tdata	(axis_slave2_tdata),
	.o_axis_slave2_tlast	(axis_slave2_tlast),
	.o_axis_slave2_tuser	(axis_slave2_tuser),

	// From pcie
	.i_cfg_ido_req_en		(cfg_ido_req_en),
	.i_cfg_ido_cpl_en		(cfg_ido_cpl_en),
	.i_xadm_ph_cdts			(xadm_ph_cdts),
	.i_xadm_pd_cdts			(xadm_pd_cdts),
	.i_xadm_nph_cdts		(xadm_nph_cdts),
	.i_xadm_npd_cdts		(xadm_npd_cdts),
	.i_xadm_cplh_cdts		(xadm_cplh_cdts),
	.i_xadm_cpld_cdts		(xadm_cpld_cdts),

	// APB interface
	.i_apb_psel				(p_sel_dma),
	.i_apb_paddr			(p_addr[8:0]),
	.i_apb_pwdata			(p_wdata),
	.i_apb_pstrb			(p_strb),
	.i_apb_pwrite			(p_we),
	.i_apb_penable			(p_ce),
	.o_apb_prdy				(p_rdy_dma),
	.o_apb_prdata			(p_rdata_dma),
	.o_cross_4kb_boundary	(cross_4kb_boundary),	//4k杈圭晫
	.o_tx_restart_ext		(mwr_cmd_start),
	.o_frame_done_pulse_ext	(frame_done_pulse),
	.o_prep_ctrl_ext		(fpga_prep_ctrl),
	.o_prep_clahe_ext		(fpga_prep_clahe),
	.o_prep_usm_ext			(fpga_prep_usm),
	.o_prep_med_ext			(fpga_prep_med),
	.o_roi_x1y1_ext			(fpga_roi_x1y1),
	.o_roi_x2y2_ext			(fpga_roi_x2y2),
	.o_roi_ctrl_ext			(fpga_roi_ctrl),
	// External BAR2 read override for MWR frame data
	.o_bar2_rd_clk_en_ext	(mwr_rd_clk_en),
	.o_bar2_rd_addr_ext		(mwr_rd_addr),
	.i_ext_bar2_rd_data		(mwr_rd_data),
	.i_ext_bar2_rd_sel		(1'b1)				// Always use external frame data
);

// CFG CTRL
generate
	if (DEVICE_TYPE == 3'd4) begin:rc
	//CFG TLP TX RX     BASE ADDR = 0x9000
		pcie_cfg_ctrl u_pcie_cfg_ctrl (
			//from APB
			.pclk_div2				(pclk_div2),				//125mhz    x2 5gt/s
			.apb_rst_n				(core_rst_n),
			.p_sel					(p_sel_cfg),
			.p_strb					(p_strb),
			.p_addr					(p_addr[7:0]),
			.p_wdata				(p_wdata),
			.p_ce					(p_ce),
			.p_we					(p_we),
			.p_rdy					(p_rdy_cfg),
			.p_rdata				(p_rdata_cfg),
			.pcie_cfg_ctrl_en		(pcie_cfg_ctrl_en),

			//To PCIE ctrl
			.axis_slave_tready		(axis_slave0_tready),
			.axis_slave_tvalid		(cfg_axis_slave0_tvalid),
			.axis_slave_tlast		(cfg_axis_slave0_tlast),
			.axis_slave_tuser		(cfg_axis_slave0_tuser),
			.axis_slave_tdata		(cfg_axis_slave0_tdata),

			.axis_master_tready		(axis_master_tready_cfg),
			.axis_master_tvalid		(axis_master_tvalid),
			.axis_master_tlast		(axis_master_tlast),

			.axis_master_tkeep		(axis_master_tkeep),


			.axis_master_tdata		(axis_master_tdata)
		);

		// Logic mux
		assign axis_slave0_tvalid      = pcie_cfg_ctrl_en ? cfg_axis_slave0_tvalid  : dma_axis_slave0_tvalid;
		assign axis_slave0_tlast       = pcie_cfg_ctrl_en ? cfg_axis_slave0_tlast   : dma_axis_slave0_tlast;
		assign axis_slave0_tuser       = pcie_cfg_ctrl_en ? cfg_axis_slave0_tuser   : dma_axis_slave0_tuser;
		assign axis_slave0_tdata       = pcie_cfg_ctrl_en ? cfg_axis_slave0_tdata   : dma_axis_slave0_tdata;

		assign axis_master_tvalid_mem  = pcie_cfg_ctrl_en ? 1'b0                    : axis_master_tvalid;
		assign axis_master_tdata_mem   = pcie_cfg_ctrl_en ? 128'b0                  : axis_master_tdata;
		assign axis_master_tkeep_mem   = pcie_cfg_ctrl_en ? 4'b0                    : axis_master_tkeep;
		assign axis_master_tlast_mem   = pcie_cfg_ctrl_en ? 1'b0                    : axis_master_tlast;
		assign axis_master_tuser_mem   = pcie_cfg_ctrl_en ? 8'b0                    : axis_master_tuser;

		assign axis_master_tready      = pcie_cfg_ctrl_en ? axis_master_tready_cfg  : axis_master_tready_mem;
	end else begin:ep
		assign p_rdy_cfg               = 1'b0;
		assign p_rdata_cfg             = 32'b0;

		assign axis_slave0_tvalid      = dma_axis_slave0_tvalid;
		assign axis_slave0_tlast       = dma_axis_slave0_tlast;
		assign axis_slave0_tuser       = dma_axis_slave0_tuser;
		assign axis_slave0_tdata       = dma_axis_slave0_tdata;

		assign axis_master_tvalid_mem  = axis_master_tvalid;
		assign axis_master_tdata_mem   = axis_master_tdata;
		assign axis_master_tkeep_mem   = axis_master_tkeep;
		assign axis_master_tlast_mem   = axis_master_tlast;
		assign axis_master_tuser_mem   = axis_master_tuser;

		assign axis_master_tready      = axis_master_tready_mem;
	end
endgenerate

// PCIe IP TOP : HSSTLP : 0x0000~6000 PCIe BASE ADDR : 0x7000
pcie_test u_ips2l_pcie_wrap (
	.button_rst_n				(sync_button_rst_n),
	.power_up_rst_n				(sync_perst_n),
	.perst_n					(sync_perst_n),

	// The clock and reset signals
	.pclk						(pclk),
	.pclk_div2					(pclk_div2),
	.ref_clk					(ref_clk),
	.ref_clk_n					(ref_clk_n),
	.ref_clk_p					(ref_clk_p),
	.core_rst_n					(core_rst_n),

	// APB interface to DBI config
	.p_sel						(p_sel_pcie),
	.p_strb						(uart_p_strb),
	.p_addr						(uart_p_addr),
	.p_wdata					(uart_p_wdata),
	.p_ce						(uart_p_ce),
	.p_we						(uart_p_we),
	.p_rdy						(p_rdy_pcie),
	.p_rdata					(p_rdata_pcie),

	// PHY diff signals
	.rxn						(rxn),
	.rxp						(rxp),
	.txn						(txn),
	.txp						(txp),
	.pcs_nearend_loop			({4{1'b0}}),
	.pma_nearend_ploop			({4{1'b0}}),
	.pma_nearend_sloop			({4{1'b0}}),

	// AXI4-Stream master interface
	.axis_master_tvalid			(axis_master_tvalid),
	.axis_master_tready			(axis_master_tready),
	.axis_master_tdata			(axis_master_tdata),
	.axis_master_tkeep			(axis_master_tkeep),

	.axis_master_tlast			(axis_master_tlast),
	.axis_master_tuser			(axis_master_tuser),

	// AXI4-Stream slave 0 interface
	.axis_slave0_tready			(axis_slave0_tready),
	.axis_slave0_tvalid			(axis_slave0_tvalid),
	.axis_slave0_tdata			(axis_slave0_tdata),
	.axis_slave0_tlast			(axis_slave0_tlast),
	.axis_slave0_tuser			(axis_slave0_tuser),

	// AXI4-Stream slave 1 interface
	.axis_slave1_tready			(axis_slave1_tready),
	.axis_slave1_tvalid			(axis_slave1_tvalid),
	.axis_slave1_tdata			(axis_slave1_tdata),
	.axis_slave1_tlast			(axis_slave1_tlast),
	.axis_slave1_tuser			(axis_slave1_tuser),

		// AXI4-Stream slave 2 interface
		.axis_slave2_tready			(axis_slave2_tready_raw),
		.axis_slave2_tvalid			(axis_slave2_tvalid),
		.axis_slave2_tdata			(axis_slave2_tdata),
		.axis_slave2_tlast			(axis_slave2_tlast),
		.axis_slave2_tuser			(axis_slave2_tuser),

		.pm_xtlh_block_tlp			(),

		.ven_msi_vector				(ven_msi_vector),
		.cfg_msi_pending			(cfg_msi_pending),
		.cfg_msi_en					(cfg_msi_en),
		.ven_msi_req				(ven_msi_req),
		.ven_msi_tc					(ven_msi_tc),
		.ven_msi_grant				(ven_msi_grant),

		.cfg_send_cor_err_mux		(),
	.cfg_send_nf_err_mux		(),
	.cfg_send_f_err_mux			(),
	.cfg_sys_err_rc				(),
	.cfg_aer_rc_err_mux			(),

	// The radm timeout
	.radm_cpl_timeout			(),

	// Configuration signals
	.cfg_max_rd_req_size		(cfg_max_rd_req_size),
	.cfg_bus_master_en			(),
	.cfg_max_payload_size		(cfg_max_payload_size),
	.cfg_ext_tag_en				(),
	.cfg_rcb					(cfg_rcb),
	.cfg_mem_space_en			(),
	.cfg_pm_no_soft_rst			(),
	.cfg_crs_sw_vis_en			(),
	.cfg_no_snoop_en			(),
	.cfg_relax_order_en			(),
	.cfg_tph_req_en				(),
	.cfg_pf_tph_st_mode			(),
	.rbar_ctrl_update			(),
	.cfg_atomic_req_en			(),

	.cfg_pbus_num				(cfg_pbus_num),
	.cfg_pbus_dev_num			(cfg_pbus_dev_num),

	// Debug signals
	.radm_idle					(),
	.radm_q_not_empty			(),
	.radm_qoverflow				(),
	.diag_ctrl_bus				(2'b0),
	.cfg_link_auto_bw_mux		(),
	.cfg_bw_mgt_mux				(),
	.cfg_pme_mux				(),
	.app_ras_des_sd_hold_ltssm	(1'b0),
	.app_ras_des_tba_ctrl		(2'b0),

	.dyn_debug_info_sel			(4'b0),
	.debug_info_mux				(),

	// System signal
	.smlh_link_up				(smlh_link_up),			//link鐘舵€?
	.rdlh_link_up				(rdlh_link_up),			//link鐘舵€?
	.smlh_ltssm_state			(smlh_ltssm_state)
);

//=============================================================================
// PLL for DDR3 (generates clocks from 25MHz sys_clk)
//=============================================================================
wire cfg_clk;
wire ddr_clk;
wire pll_locked;

PLL u_pll (
    .clkin1     (sys_clk),
    .clkout0    (cfg_clk),      // Configuration clock
    .clkout1    (),
    .clkout2    (ddr_clk),      // DDR reference clock
    .lock       (pll_locked)
);

// Reset generation from PLL
reg [15:0] rstn_1ms;
always @(posedge cfg_clk or negedge pll_locked) begin
    if (!pll_locked)
        rstn_1ms <= 16'd0;
    else if (rstn_1ms != 16'h2710)
        rstn_1ms <= rstn_1ms + 1'b1;
end
wire ddr_rstn = (rstn_1ms == 16'h2710);

//=============================================================================
// Camera power-on delay
//=============================================================================
wire camera_rstn;
wire camera_pwnd;
wire initial_en;

power_on_delay u_power_on_delay (
    .clk_50M        (sys_clk),      // Stage 1: keep delay logic in 25MHz system clock domain
    .reset_n        (ddr_rstn),
    .camera1_rstn   (camera_rstn),
    .camera2_rstn   (),
    .camera_pwnd    (camera_pwnd),
    .initial_en     (initial_en)
);

assign cmos_reset = camera_rstn;

//=============================================================================
// Camera 1 I2C Configuration (OV5640)
//=============================================================================
wire cmos1_init_done /*synthesis PAP_MARK_DEBUG="1"*/;
wire [8:0] cmos1_init_reg_index /*synthesis PAP_MARK_DEBUG="1"*/;
wire cmos1_init_ack_fail_sticky /*synthesis PAP_MARK_DEBUG="1"*/;
wire [8:0] cmos1_init_ack_fail_first_index /*synthesis PAP_MARK_DEBUG="1"*/;
wire [8:0] cmos1_init_ack_fail_count /*synthesis PAP_MARK_DEBUG="1"*/;
localparam SENSOR_TEST_PATTERN_EN = 1'b0;

reg_config #(
    .SENSOR_TEST_PATTERN_EN (SENSOR_TEST_PATTERN_EN)
) u_cmos1_config (
    .clk_25M        (sys_clk),          // Stage 1: force camera config logic to 25MHz
    .camera_rstn    (camera_rstn),      // From power_on_delay
    .initial_en     (initial_en),       // From power_on_delay
    .i2c_sclk       (cmos1_scl),        // I2C clock output
    .i2c_sdat       (cmos1_sda),        // I2C data bidirectional
    .reg_conf_done  (cmos1_init_done),  // Configuration done flag
    .reg_index      (cmos1_init_reg_index),
    .ack_fail_sticky(cmos1_init_ack_fail_sticky),
    .ack_fail_first_index(cmos1_init_ack_fail_first_index),
    .ack_fail_count (cmos1_init_ack_fail_count),
    .clock_20k      ()                  // Debug: I2C clock
);

//=============================================================================
// Camera 1 Data Capture (8-bit to 16-bit RGB565)
//=============================================================================
wire [15:0] cmos1_d_16bit;
wire        cmos1_href_16bit;
wire        cmos1_vsync_16bit;
wire        cmos1_pclk_16bit;
wire        cmos1_pix_vld;
localparam  CMOS1_CAPTURE_NEGEDGE = 1'b0;

cmos_8_16bit #(
    .CAPTURE_ON_NEGEDGE (CMOS1_CAPTURE_NEGEDGE)
) u_cmos1_8_16bit (
    .pclk       (cmos1_pclk),           // Pixel clock from camera
    .rst_n      (cmos1_init_done),      // Reset after I2C config done
    .pdata_i    (cmos1_data),           // 8-bit input data
    .de_i       (cmos1_href),           // Data enable (href)
    .vs_i       (cmos1_vsync),          // Vsync
    .pixel_clk  (cmos1_pclk_16bit),     // Output: divided pixel clock
    .pix_vld_o  (cmos1_pix_vld),        // Output: 16-bit pixel valid pulse
    .pdata_o    (cmos1_d_16bit),        // Output: 16-bit RGB565
    .de_o       (cmos1_href_16bit),     // Output: line active
    .vs_o       (cmos1_vsync_16bit)     // Output: frame sync aligned to capture domain
);

// Stage 1 color normalization: keep hardware output explicitly defined.
localparam CAM_SWAP_RB = 1'b0;
wire [15:0] cmos1_rgb565_fmt = CAM_SWAP_RB ?
    {cmos1_d_16bit[4:0], cmos1_d_16bit[10:5], cmos1_d_16bit[15:11]} :
    cmos1_d_16bit;

// Debug injection switches (default disabled).
localparam FORCE_COLOR_BAR_PRE_DDR = 1'b0;
localparam FORCE_PATTERN_POST_DDR  = 1'b0;
localparam DMA_OUTPUT_BGRX         = 1'b1;
// Mainline V3 default: keep bypass unless explicitly enabled.
localparam PREPROC_ENABLE_DEFAULT  = 1'b0;

reg [11:0] cmos1_bar_x;
reg        cmos1_href_16bit_d;
reg        cmos1_vsync_16bit_d;

function [15:0] color_bar_bgr565;
    input [11:0] x;
begin
    if (x < 12'd160)       color_bar_bgr565 = 16'h001F; // Red
    else if (x < 12'd320)  color_bar_bgr565 = 16'h07E0; // Green
    else if (x < 12'd480)  color_bar_bgr565 = 16'hF800; // Blue
    else if (x < 12'd640)  color_bar_bgr565 = 16'h07FF; // Yellow
    else if (x < 12'd800)  color_bar_bgr565 = 16'hFFE0; // Cyan
    else if (x < 12'd960)  color_bar_bgr565 = 16'hF81F; // Magenta
    else if (x < 12'd1120) color_bar_bgr565 = 16'hFFFF; // White
    else                   color_bar_bgr565 = 16'h0000; // Black
end
endfunction

always @(posedge cmos1_pclk or negedge cmos1_init_done) begin
    if (!cmos1_init_done) begin
        cmos1_bar_x <= 12'd0;
        cmos1_href_16bit_d <= 1'b0;
        cmos1_vsync_16bit_d <= 1'b0;
    end else begin
        cmos1_href_16bit_d <= cmos1_href_16bit;
        cmos1_vsync_16bit_d <= cmos1_vsync_16bit;

        if ((~cmos1_vsync_16bit_d) && cmos1_vsync_16bit) begin
            cmos1_bar_x <= 12'd0;
        end else if ((~cmos1_href_16bit_d) && cmos1_href_16bit) begin
            cmos1_bar_x <= 12'd0;
        end else if (cmos1_pix_vld) begin
            if (cmos1_bar_x == 12'd1279)
                cmos1_bar_x <= 12'd0;
            else
                cmos1_bar_x <= cmos1_bar_x + 12'd1;
        end
    end
end

wire [15:0] cmos1_bar_data = color_bar_bgr565(cmos1_bar_x);
wire [15:0] cmos1_wr_data_pre = FORCE_COLOR_BAR_PRE_DDR ? cmos1_bar_data : cmos1_rgb565_fmt;
wire [15:0] cmos1_wr_data = cmos1_wr_data_pre;

//=============================================================================
// Frame Buffer (Camera 鈫?DDR3)
//=============================================================================
// AXI signals between fram_buf and DDR3
localparam CTRL_ADDR_WIDTH = MEM_ROW_ADDR_WIDTH + MEM_BADDR_WIDTH + MEM_COL_ADDR_WIDTH;

wire [CTRL_ADDR_WIDTH-1:0] axi_awaddr;
wire [3:0]                 axi_awuser_id;
wire [3:0]                 axi_awlen;
wire                       axi_awready;
wire                       axi_awvalid;
wire [MEM_DQ_WIDTH*8-1:0]  axi_wdata;
wire [MEM_DQ_WIDTH-1:0]    axi_wstrb;
wire                       axi_wready;
wire                       axi_wusero_last;

wire [CTRL_ADDR_WIDTH-1:0] axi_araddr;
wire [3:0]                 axi_aruser_id;
wire [3:0]                 axi_arlen;
wire                       axi_arready;
wire                       axi_arvalid;
wire [MEM_DQ_WIDTH*8-1:0]  axi_rdata;
wire [3:0]                 axi_rid;
wire                       axi_rlast;
wire                       axi_rvalid;

wire                       ddr_init_done /*synthesis PAP_MARK_DEBUG="1"*/;
wire                       core_clk_ddr;
wire                       fram_buf_init_done /*synthesis PAP_MARK_DEBUG="1"*/;
wire [127:0]               frame_rd_data;
wire                       frame_rd_data_valid;
wire                       frame_rd_data_ready;

//=============================================================================
// MWR Data Source (frame data for DMA transfer to host)
//=============================================================================
// Start one read session on DMA command start and keep the session
// active across chunk gaps until a full frame has been consumed.
localparam [17:0]          FRAME_WORDS_565  = (1280 * 720 * 16) / 128;
localparam [17:0]          FRAME_WORDS_BGRX = (1280 * 720 * 32) / 128;
localparam [17:0]          FRAME_SRC_WORDS  = FRAME_WORDS_565;
localparam [2:0]           PREP_BOOTSTRAP_WORDS = 3'd4;
localparam [2:0]           RAW_BOOTSTRAP_WORDS = 3'd2;
localparam                  PREP_TIMING_CLAHE_ENABLE = 1'b0;
localparam                  PREP_TIMING_USM_ENABLE = 1'b0;
localparam [7:0]           PREP_FETCH_WORDS_PER_LINE = 8'd160;
reg                        dma_session_active;
reg  [17:0]                dma_rd_word_count;
reg  [17:0]                frame_src_req_count;
reg  [5:0]                 rd_fsync_stretch_cnt;
reg  [8:0]                 post_ddr_word_x;
reg  [9:0]                 post_ddr_line_y;
reg  [7:0]                 prep_word_x;
reg  [9:0]                 prep_line_y;
reg                        prep_linebuf_req_valid_d0;
reg                        prep_linebuf_req_valid_d1;
reg  [7:0]                 prep_linebuf_req_word_x_d0;
reg  [7:0]                 prep_linebuf_req_word_x_d1;
reg  [9:0]                 prep_linebuf_req_line_y_d0;
reg  [9:0]                 prep_linebuf_req_line_y_d1;
reg  [63:0]                prep_linebuf_prev1_rd_d0;
reg  [63:0]                prep_linebuf_prev1_rd_d1;
reg  [63:0]                prep_linebuf_prev2_rd_d0;
reg  [63:0]                prep_linebuf_prev2_rd_d1;
reg  [2:0]                 frame_src_bootstrap_count;
reg                        dma_expand_phase;
reg  [11:0]                mwr_rd_addr_d;
reg                        mwr_rd_clk_en_d;
reg  [127:0]               frame_rd_data_hold;
reg                        raw_startup_active;
reg                        raw_start_word_valid;
reg  [127:0]               raw_start_word;
reg  [7:0]                 frame_id;
reg                        prep_session_en;
reg                        prep_session_target_all;
reg                        prep_session_a_fmt_yenh;
reg                        prep_session_ocr_stroke;
reg                        prep_session_median_en;
reg                        prep_session_clahe_en;
reg                        prep_session_usm_en;
reg  [31:0]                prep_session_clahe_cfg;
reg  [31:0]                prep_session_usm_cfg;
reg  [31:0]                prep_session_med_cfg;
reg                        roi_session_active;
reg                        roi_session_left_bias;
reg  [11:0]                roi_session_x1;
reg  [11:0]                roi_session_x2;
reg  [10:0]                roi_session_y1;
reg  [10:0]                roi_session_y2;
reg  [7:0]                 roi_timeout_count;
reg  [31:0]                roi_cfg_seen_x1y1;
reg  [31:0]                roi_cfg_seen_x2y2;
reg  [31:0]                roi_cfg_seen_ctrl;
reg  [7:0]                 prep_top_hist_0_q;
reg  [7:0]                 prep_top_hist_1_q;
reg  [7:0]                 prep_mid_hist_0_q;
reg  [7:0]                 prep_mid_hist_1_q;
reg  [7:0]                 prep_bot_hist_0_q;
reg  [7:0]                 prep_bot_hist_1_q;
reg                        prep_stage_a_valid;
reg  [127:0]               prep_stage_a_src_word;
reg  [63:0]                prep_stage_a_luma_top_word;
reg  [63:0]                prep_stage_a_luma_mid_word;
reg  [63:0]                prep_stage_a_luma_bot_word;
reg  [15:0]                prep_stage_a_roi_mode_word;
reg                        prep_stage_a_first_word;
reg  [7:0]                 prep_stage_a_top_hist_0;
reg  [7:0]                 prep_stage_a_top_hist_1;
reg  [7:0]                 prep_stage_a_mid_hist_0;
reg  [7:0]                 prep_stage_a_mid_hist_1;
reg  [7:0]                 prep_stage_a_bot_hist_0;
reg  [7:0]                 prep_stage_a_bot_hist_1;
reg                        prep_stage_b_valid;
reg  [127:0]               prep_stage_b_src_word;
reg  [63:0]                prep_stage_b_luma_bot_word;
reg  [15:0]                prep_stage_b_roi_mode_word;
reg                        prep_stage_b_first_word;
reg  [63:0]                prep_stage_b_median_word;
reg  [63:0]                prep_stage_b_min_word;
reg  [63:0]                prep_stage_b_max_word;
reg                        prep_stage_c_valid;
reg  [127:0]               prep_stage_c_src_word;
reg  [63:0]                prep_stage_c_y_word;
reg                        prep_stage_c_first_word;
reg                        out_pair_active_valid;
reg  [127:0]               out_pair_active_src_word;
reg                        out_pair_active_first_word;
reg                        out_pair_next_valid;
reg  [127:0]               out_pair_next_src_word;
reg                        out_pair_next_first_word;
reg                        prep_pair_active_valid;
reg  [127:0]               prep_pair_active_src_word;
reg  [63:0]                prep_pair_active_y_word;
reg                        prep_pair_active_first_word;
reg                        prep_pair_next_valid;
reg  [127:0]               prep_pair_next_src_word;
reg  [63:0]                prep_pair_next_y_word;
reg                        prep_pair_next_first_word;
wire                       dma_session_start;
wire                       rd_fsync_pclk_div2;
wire                       dma_expand_mode = DMA_OUTPUT_BGRX;
wire                       preproc_en = PREPROC_ENABLE_DEFAULT | fpga_prep_ctrl[0];
wire                       prep_bypass = fpga_prep_ctrl[1];
wire                       prep_median_en = fpga_prep_ctrl[2];
wire                       prep_clahe_en = fpga_prep_ctrl[3];
wire                       prep_usm_en = fpga_prep_ctrl[4];
wire                       prep_target_ocr_only = fpga_prep_ctrl[5];
wire                       prep_a_fmt_yenh = fpga_prep_ctrl[6];
wire                       prep_ocr_stroke_mode = fpga_prep_ctrl[7];
wire                       prep_active = preproc_en & ~prep_bypass;
wire                       prep_target_all = prep_active & ~prep_target_ocr_only;
wire                       prep_active_latched = prep_session_en;
wire                       prep_target_all_latched = prep_session_target_all;
wire                       prep_a_fmt_yenh_latched = prep_session_a_fmt_yenh;
wire                       prep_ocr_stroke_latched = prep_session_ocr_stroke;
wire                       prep_median_en_latched = prep_session_median_en;
wire                       prep_clahe_en_latched = PREP_TIMING_CLAHE_ENABLE ? prep_session_clahe_en : 1'b0;
wire                       prep_usm_en_latched = PREP_TIMING_USM_ENABLE ? prep_session_usm_en : 1'b0;
wire [31:0]                prep_clahe_cfg_latched = PREP_TIMING_CLAHE_ENABLE ? prep_session_clahe_cfg : 32'd0;
wire [31:0]                prep_usm_cfg_latched = PREP_TIMING_USM_ENABLE ? prep_session_usm_cfg : 32'd0;
wire [31:0]                prep_med_cfg_latched = prep_session_med_cfg;
wire                       roi_cfg_enable = fpga_roi_ctrl[0];
wire                       roi_cfg_left_bias = fpga_roi_ctrl[1];
wire [7:0]                 roi_cfg_timeout_frames = (fpga_roi_ctrl[15:8] != 8'd0) ? fpga_roi_ctrl[15:8] : 8'd3;
wire [11:0]                roi_cfg_x1 = fpga_roi_x1y1[11:0];
wire [11:0]                roi_cfg_x2 = fpga_roi_x2y2[11:0];
wire [10:0]                roi_cfg_y1 = fpga_roi_x1y1[26:16];
wire [10:0]                roi_cfg_y2 = fpga_roi_x2y2[26:16];
wire                       roi_cfg_bbox_valid = (roi_cfg_x2 >= roi_cfg_x1) && (roi_cfg_y2 >= roi_cfg_y1);
wire                       roi_cfg_changed = (fpga_roi_x1y1 != roi_cfg_seen_x1y1) ||
                                            (fpga_roi_x2y2 != roi_cfg_seen_x2y2) ||
                                            (fpga_roi_ctrl != roi_cfg_seen_ctrl);
wire [17:0]                frame_words_cfg = dma_expand_mode ? FRAME_WORDS_BGRX : FRAME_WORDS_565;
wire [2:0]                 frame_bootstrap_words = prep_active_latched ? PREP_BOOTSTRAP_WORDS
                                                                        : RAW_BOOTSTRAP_WORDS;
// Count chunk first beat as a valid step to prevent boundary phase slip.
wire                       bar2_addr_step = mwr_rd_clk_en &&
                                            ((mwr_rd_addr != mwr_rd_addr_d) || (~mwr_rd_clk_en_d));
wire                       raw_start_bootstrap_req = raw_startup_active &&
                                                    (frame_src_bootstrap_count < RAW_BOOTSTRAP_WORDS);
wire                       frame_rd_req_en_raw = dma_session_active &&
                                                frame_rd_data_ready &&
                                                (frame_src_req_count < FRAME_SRC_WORDS) &&
                                                (raw_start_bootstrap_req ||
                                                 (bar2_addr_step & (~dma_expand_mode | ~dma_expand_phase)));
wire                       frame_rd_req_en_prep = dma_session_active &&
                                                 frame_rd_data_ready &&
                                                 (frame_src_req_count < FRAME_SRC_WORDS) &&
                                                 ((frame_src_bootstrap_count < frame_bootstrap_words) ||
                                                  (bar2_addr_step & (~dma_expand_mode | ~dma_expand_phase)));
wire                       frame_rd_req_en = prep_active_latched ? frame_rd_req_en_prep : frame_rd_req_en_raw;
wire [11:0]                post_ddr_x_pix = dma_expand_mode ? {1'b0, post_ddr_word_x, 2'b00}
                                                             : {post_ddr_word_x, 3'b000};
wire [15:0]                post_ddr_color_base = color_bar_bgr565(post_ddr_x_pix);
wire [15:0]                post_ddr_color_data = post_ddr_color_base;
wire [8:0]                 post_ddr_words_per_line = dma_expand_mode ? 9'd320 : 9'd160;
// Read the prep line buffers through registered request/data phases so the
// synthesizer can map them as RAM instead of a wide async mux tree.
reg  [63:0]                prep_linebuf_prev1 [0:PREP_FETCH_WORDS_PER_LINE-1];
reg  [63:0]                prep_linebuf_prev2 [0:PREP_FETCH_WORDS_PER_LINE-1];
wire [7:0]                 prep_data_word_x = prep_linebuf_req_word_x_d1;
wire [9:0]                 prep_data_line_y = prep_linebuf_req_line_y_d1;
wire [11:0]                prep_x_pix_base = {prep_data_word_x, 3'b000};
function [7:0] expand5_to_8;
    input [4:0] value5;
begin
    expand5_to_8 = {value5, value5[4:2]};
end
endfunction

function [7:0] expand6_to_8;
    input [5:0] value6;
begin
    expand6_to_8 = {value6, value6[5:4]};
end
endfunction

function [7:0] preproc_alpha_from_bgr565;
    input [15:0] pix565;
    reg [7:0] r8;
    reg [7:0] g8;
    reg [7:0] b8;
    reg [7:0] y8;
    reg [7:0] drg;
    reg [7:0] dgb;
    reg       edge_flag;
    reg       thresh_flag;
    reg [1:0] color_class;
begin
    r8 = expand5_to_8(pix565[4:0]);
    g8 = expand6_to_8(pix565[10:5]);
    b8 = expand5_to_8(pix565[15:11]);
    y8 = (r8 >> 2) + (g8 >> 1) + (b8 >> 2);

    if (r8 >= g8)
        drg = r8 - g8;
    else
        drg = g8 - r8;

    if (g8 >= b8)
        dgb = g8 - b8;
    else
        dgb = b8 - g8;

    edge_flag = ({1'b0, drg} + {1'b0, dgb}) > 9'd64;
    thresh_flag = (y8 > 8'd96);

    if ((r8 > 8'd150) && ({1'b0, r8} > ({1'b0, g8} + 9'd24)) && ({1'b0, r8} > ({1'b0, b8} + 9'd24)))
        color_class = 2'b11;
    else if ((b8 > 8'd110) && ({1'b0, b8} > ({1'b0, g8} + 9'd16)) && ({1'b0, b8} > ({1'b0, r8} + 9'd16)))
        color_class = 2'b01;
    else if ((g8 > 8'd90) && (r8 > 8'd70) && (b8 < 8'd140))
        color_class = 2'b10;
    else
        color_class = 2'b00;

    preproc_alpha_from_bgr565 = {1'b1, edge_flag, thresh_flag, 1'b0, color_class, 2'b00};
end
endfunction

function [7:0] absdiff8;
    input [7:0] a;
    input [7:0] b;
begin
    absdiff8 = (a >= b) ? (a - b) : (b - a);
end
endfunction

function [7:0] luma_from_rgb888;
    input [7:0] r8;
    input [7:0] g8;
    input [7:0] b8;
begin
    luma_from_rgb888 = (r8 >> 2) + (g8 >> 1) + (b8 >> 2);
end
endfunction

function [7:0] luma_from_bgr565;
    input [15:0] pix565;
begin
    luma_from_bgr565 = luma_from_rgb888(expand5_to_8(pix565[4:0]),
                                        expand6_to_8(pix565[10:5]),
                                        expand5_to_8(pix565[15:11]));
end
endfunction

function [63:0] luma_word_from_src128;
    input [127:0] src_word;
begin
    luma_word_from_src128 = {
        luma_from_bgr565(src_word[127:112]),
        luma_from_bgr565(src_word[111:96]),
        luma_from_bgr565(src_word[95:80]),
        luma_from_bgr565(src_word[79:64]),
        luma_from_bgr565(src_word[63:48]),
        luma_from_bgr565(src_word[47:32]),
        luma_from_bgr565(src_word[31:16]),
        luma_from_bgr565(src_word[15:0])
    };
end
endfunction

function [7:0] clamp_u8_int;
    input integer v;
begin
    if (v < 0)
        clamp_u8_int = 8'd0;
    else if (v > 255)
        clamp_u8_int = 8'd255;
    else
        clamp_u8_int = v[7:0];
end
endfunction

function [7:0] max3_u8;
    input [7:0] a;
    input [7:0] b;
    input [7:0] c;
    reg [7:0] t;
begin
    t = (a >= b) ? a : b;
    max3_u8 = (t >= c) ? t : c;
end
endfunction

function [7:0] min3_u8;
    input [7:0] a;
    input [7:0] b;
    input [7:0] c;
    reg [7:0] t;
begin
    t = (a <= b) ? a : b;
    min3_u8 = (t <= c) ? t : c;
end
endfunction

function [7:0] median3_u8;
    input [7:0] a;
    input [7:0] b;
    input [7:0] c;
begin
    if ((a >= b && a <= c) || (a <= b && a >= c))
        median3_u8 = a;
    else if ((b >= a && b <= c) || (b <= a && b >= c))
        median3_u8 = b;
    else
        median3_u8 = c;
end
endfunction

function [1:0] roi_boost_mode_xy;
    input [11:0] x;
    input [10:0] y;
    input        active;
    input        left_bias;
    input [11:0] x1;
    input [10:0] y1;
    input [11:0] x2;
    input [10:0] y2;
    integer roi_w;
    integer x_rel;
    integer roi_w_mul28;
    integer x_rel_mul100;
begin
    roi_boost_mode_xy = 2'd0;
    if (active && (x >= x1) && (x <= x2) && (y >= y1) && (y <= y2)) begin
        roi_boost_mode_xy = 2'd1;
        if (left_bias) begin
            roi_w = ({20'd0, x2} - {20'd0, x1}) + 1;
            x_rel = ({20'd0, x} - {20'd0, x1}) + 1;
            roi_w_mul28 = (roi_w << 4) + (roi_w << 3) + (roi_w << 2);
            x_rel_mul100 = (x_rel << 6) + (x_rel << 5) + (x_rel << 2);
            if (x_rel_mul100 <= roi_w_mul28)
                roi_boost_mode_xy = 2'd2;
        end
    end
end
endfunction

function [23:0] rgb24_from_bgr565;
    input [15:0] pix565;
    reg [7:0] r8;
    reg [7:0] g8;
    reg [7:0] b8;
begin
    r8 = expand5_to_8(pix565[4:0]);
    g8 = expand6_to_8(pix565[10:5]);
    b8 = expand5_to_8(pix565[15:11]);
    rgb24_from_bgr565 = {r8, g8, b8};
end
endfunction

function [31:0] rgb24_to_bgrx32;
    input [23:0] rgb24;
    input [7:0] alpha8;
begin
    rgb24_to_bgrx32 = {alpha8, rgb24[23:16], rgb24[15:8], rgb24[7:0]};
end
endfunction

function [7:0] median9_u8;
    input [7:0] a0;
    input [7:0] a1;
    input [7:0] a2;
    input [7:0] b0;
    input [7:0] b1;
    input [7:0] b2;
    input [7:0] c0;
    input [7:0] c1;
    input [7:0] c2;
    reg [7:0] min_a;
    reg [7:0] min_b;
    reg [7:0] min_c;
    reg [7:0] med_a;
    reg [7:0] med_b;
    reg [7:0] med_c;
    reg [7:0] max_a;
    reg [7:0] max_b;
    reg [7:0] max_c;
    reg [7:0] lo_bound;
    reg [7:0] mid_bound;
    reg [7:0] hi_bound;
begin
    min_a = min3_u8(a0, a1, a2);
    min_b = min3_u8(b0, b1, b2);
    min_c = min3_u8(c0, c1, c2);
    med_a = median3_u8(a0, a1, a2);
    med_b = median3_u8(b0, b1, b2);
    med_c = median3_u8(c0, c1, c2);
    max_a = max3_u8(a0, a1, a2);
    max_b = max3_u8(b0, b1, b2);
    max_c = max3_u8(c0, c1, c2);
    lo_bound = max3_u8(min_a, min_b, min_c);
    mid_bound = median3_u8(med_a, med_b, med_c);
    hi_bound = min3_u8(max_a, max_b, max_c);
    median9_u8 = median3_u8(lo_bound, mid_bound, hi_bound);
end
endfunction

function [7:0] spatial_enhance_y;
    input [7:0] y_src;
    input [7:0] y_med;
    input [7:0] y_min;
    input [7:0] y_max;
    input        en_median;
    input        en_clahe;
    input        en_usm;
    input        ocr_stroke_mode;
    input [1:0]  roi_boost_mode;
    input [31:0] clahe_cfg;
    input [31:0] usm_cfg;
    input [31:0] med_cfg;
    integer y_cur;
    integer y_mid;
    integer delta;
    integer boost;
    integer clip_span;
    integer noise_gate;
    integer range_gate;
    integer local_range;
    integer usm_thr;
    integer usm_limit;
    integer usm_gain_q4_4;
    integer abs_detail;
    integer detail;
    integer clahe_strength;
begin
    y_cur = {1'b0, y_src};
    noise_gate = med_cfg[7:0];
    range_gate = med_cfg[15:8];
    if (noise_gate < 1)
        noise_gate = 12;
    if (range_gate < 1)
        range_gate = 8;

    if (ocr_stroke_mode) begin
        if (noise_gate > 2)
            noise_gate = noise_gate - 2;
        if (range_gate > 2)
            range_gate = range_gate - 2;
    end

    if (roi_boost_mode == 2'd1) begin
        if (noise_gate > 1)
            noise_gate = noise_gate - 1;
    end else if (roi_boost_mode == 2'd2) begin
        if (noise_gate > 2)
            noise_gate = noise_gate - 2;
        if (range_gate > 2)
            range_gate = range_gate - 2;
    end

    if (en_median) begin
        if (absdiff8(y_src, y_med) >= noise_gate[7:0])
            y_cur = {1'b0, y_med};
        else
            y_cur = ({1'b0, y_src} + {1'b0, y_med}) >> 1;
    end

    local_range = {1'b0, y_max} - {1'b0, y_min};

    if (en_clahe && (local_range >= range_gate)) begin
        clahe_strength = clahe_cfg[31:24];
        clip_span = clahe_cfg[23:16];
        if (clip_span < 1)
            clip_span = 16;
        if (roi_boost_mode == 2'd1)
            clip_span = clip_span + 2;
        else if (roi_boost_mode == 2'd2)
            clip_span = clip_span + 4;
        if (clip_span > 48)
            clip_span = 48;

        y_mid = ({1'b0, y_min} + {1'b0, y_max}) >> 1;
        delta = y_cur - y_mid;
        boost = 0;
        if (clahe_strength >= 16)
            boost = delta >>> 3;
        if (clahe_strength >= 64)
            boost = delta >>> 2;
        if (clahe_strength >= 128)
            boost = delta >>> 1;
        if (clahe_strength >= 192)
            boost = (delta >>> 1) + (delta >>> 2);
        if (boost > clip_span)
            boost = clip_span;
        else if (boost < -clip_span)
            boost = -clip_span;
        y_cur = y_cur + boost;
    end

    if (en_usm) begin
        usm_gain_q4_4 = usm_cfg[7:0];
        usm_thr = usm_cfg[15:8];
        usm_limit = usm_cfg[23:16];
        if (usm_thr < 1)
            usm_thr = 3;
        if (usm_limit < 1)
            usm_limit = 8;
        if (ocr_stroke_mode && usm_thr > 1)
            usm_thr = usm_thr - 1;
        if (roi_boost_mode == 2'd1)
            usm_limit = usm_limit + 2;
        else if (roi_boost_mode == 2'd2)
            usm_limit = usm_limit + 4;
        if (usm_limit > 48)
            usm_limit = 48;

        detail = $signed({1'b0, y_src}) - $signed({1'b0, y_med});
        abs_detail = (detail < 0) ? -detail : detail;
        if (abs_detail >= usm_thr) begin
            boost = 0;
            if (usm_gain_q4_4 >= 4)
                boost = detail >>> 3;
            if (usm_gain_q4_4 >= 8)
                boost = detail >>> 2;
            if (usm_gain_q4_4 >= 16)
                boost = detail >>> 1;
            if (usm_gain_q4_4 >= 24)
                boost = (detail >>> 1) + (detail >>> 2);
            if (usm_gain_q4_4 >= 32)
                boost = detail;
            if (boost > usm_limit)
                boost = usm_limit;
            else if (boost < -usm_limit)
                boost = -usm_limit;
            y_cur = y_cur + boost;
        end
    end

    spatial_enhance_y = clamp_u8_int(y_cur);
end
endfunction

function [23:0] rgb24_apply_y;
    input [23:0] rgb24;
    input [7:0] y_new;
    reg [7:0] r8;
    reg [7:0] g8;
    reg [7:0] b8;
    integer y_old;
    integer delta_y;
    integer rr;
    integer gg;
    integer bb;
begin
    r8 = rgb24[23:16];
    g8 = rgb24[15:8];
    b8 = rgb24[7:0];
    y_old = luma_from_rgb888(r8, g8, b8);
    delta_y = $signed({1'b0, y_new}) - $signed(y_old);
    if (delta_y > 64)
        delta_y = 64;
    else if (delta_y < -64)
        delta_y = -64;
    rr = $signed({1'b0, r8}) + delta_y;
    gg = $signed({1'b0, g8}) + delta_y;
    bb = $signed({1'b0, b8}) + delta_y;
    rgb24_apply_y = {clamp_u8_int(rr), clamp_u8_int(gg), clamp_u8_int(bb)};
end
endfunction

function [127:0] pack_4rgb_bgrx;
    input [23:0] rgb0;
    input [23:0] rgb1;
    input [23:0] rgb2;
    input [23:0] rgb3;
    input [7:0]  a0;
    input [7:0]  a1;
    input [7:0]  a2;
    input [7:0]  a3;
begin
    pack_4rgb_bgrx = {
        rgb24_to_bgrx32(rgb3, a3),
        rgb24_to_bgrx32(rgb2, a2),
        rgb24_to_bgrx32(rgb1, a1),
        rgb24_to_bgrx32(rgb0, a0)
    };
end
endfunction

function [31:0] bgr565_to_bgrx32;
    input [15:0] pix565;
    input [7:0] alpha8;
begin
    bgr565_to_bgrx32 = rgb24_to_bgrx32(rgb24_from_bgr565(pix565), alpha8);
end
endfunction

function [127:0] pack_4pix_bgrx;
    input [15:0] p0;
    input [15:0] p1;
    input [15:0] p2;
    input [15:0] p3;
    input [7:0]  a0;
    input [7:0]  a1;
    input [7:0]  a2;
    input [7:0]  a3;
begin
    pack_4pix_bgrx = {
        bgr565_to_bgrx32(p3, a3),
        bgr565_to_bgrx32(p2, a2),
        bgr565_to_bgrx32(p1, a1),
        bgr565_to_bgrx32(p0, a0)
    };
end
endfunction

function [31:0] prep_pixel_bgrx32;
    input [15:0] pix565;
    input [7:0]  y_new;
    input        target_all;
    input        a_fmt_yenh;
    input        mark_frame_id;
    input [7:0]  frame_id_in;
    reg   [7:0]  alpha8;
    reg   [23:0] rgb24;
begin
    rgb24 = rgb24_from_bgr565(pix565);
    alpha8 = a_fmt_yenh ? y_new : preproc_alpha_from_bgr565(pix565);
    if (mark_frame_id && ~a_fmt_yenh)
        alpha8 = frame_id_in;
    if (target_all)
        prep_pixel_bgrx32 = rgb24_to_bgrx32(rgb24_apply_y(rgb24, y_new), alpha8);
    else
        prep_pixel_bgrx32 = bgr565_to_bgrx32(pix565, alpha8);
end
endfunction

function [127:0] pack_4prep_bgrx;
    input [15:0] p0;
    input [15:0] p1;
    input [15:0] p2;
    input [15:0] p3;
    input [7:0]  y0;
    input [7:0]  y1;
    input [7:0]  y2;
    input [7:0]  y3;
    input        target_all;
    input        a_fmt_yenh;
    input        mark_first;
    input [7:0]  frame_id_in;
begin
    pack_4prep_bgrx = {
        prep_pixel_bgrx32(p3, y3, target_all, a_fmt_yenh, 1'b0,       frame_id_in),
        prep_pixel_bgrx32(p2, y2, target_all, a_fmt_yenh, 1'b0,       frame_id_in),
        prep_pixel_bgrx32(p1, y1, target_all, a_fmt_yenh, 1'b0,       frame_id_in),
        prep_pixel_bgrx32(p0, y0, target_all, a_fmt_yenh, mark_first, frame_id_in)
    };
end
endfunction

reg  [63:0] prep_stage_b_median_word_c;
reg  [63:0] prep_stage_b_min_word_c;
reg  [63:0] prep_stage_b_max_word_c;
reg  [63:0] prep_stage_c_y_word_c;
reg  [7:0]  prep_top_hist_0_d;
reg  [7:0]  prep_top_hist_1_d;
reg  [7:0]  prep_mid_hist_0_d;
reg  [7:0]  prep_mid_hist_1_d;
reg  [7:0]  prep_bot_hist_0_d;
reg  [7:0]  prep_bot_hist_1_d;
reg  [7:0]  prep_top_cur_r;
reg  [7:0]  prep_mid_cur_r;
reg  [7:0]  prep_bot_cur_r;
reg  [7:0]  prep_top_min_r;
reg  [7:0]  prep_mid_min_r;
reg  [7:0]  prep_bot_min_r;
reg  [7:0]  prep_top_max_r;
reg  [7:0]  prep_mid_max_r;
reg  [7:0]  prep_bot_max_r;
reg  [7:0]  prep_y_med_r;
reg  [7:0]  prep_y_min_r;
reg  [7:0]  prep_y_max_r;
reg  [7:0]  prep_stage_c_bot_cur_r;
reg  [7:0]  prep_stage_c_y_med_r;
reg  [7:0]  prep_stage_c_y_min_r;
reg  [7:0]  prep_stage_c_y_max_r;
reg  [1:0]  prep_stage_c_roi_mode_r;
integer     prep_k;
integer     prep_stage_c_k;

wire [63:0] frame_rd_luma_word = luma_word_from_src128(frame_rd_data);
wire [63:0] prep_luma_word_cur = frame_rd_luma_word;
wire [63:0] prep_luma_word_top = (prep_data_line_y == 10'd0) ? prep_luma_word_cur :
                                 (prep_data_line_y == 10'd1) ? prep_linebuf_prev1_rd_d1 : prep_linebuf_prev2_rd_d1;
wire [63:0] prep_luma_word_mid = (prep_data_line_y == 10'd0) ? prep_luma_word_cur : prep_linebuf_prev1_rd_d1;
wire        out_pair_pop = bar2_addr_step & (~dma_expand_mode | dma_expand_phase);
wire        raw_capture_fire = dma_session_active && frame_rd_data_valid;
wire        raw_frame_hold_en = raw_capture_fire;
wire        raw_startup_use_word = raw_startup_active && raw_start_word_valid;
wire        raw_startup_done = raw_startup_use_word && out_pair_pop;
wire [127:0] raw_lo_src_word = raw_startup_use_word ? raw_start_word : frame_rd_data;
wire [127:0] raw_hi_src_word = raw_startup_use_word ? raw_start_word : frame_rd_data_hold;
wire        prep_capture_fire = dma_session_active && frame_rd_data_valid && prep_linebuf_req_valid_d1;
wire        prep_capture_first_word = (prep_data_word_x == 8'd0) && (prep_data_line_y == 10'd0);
wire        pair_capture_fire = prep_active_latched ? prep_capture_fire : raw_capture_fire;
wire        pair_capture_first_word = prep_active_latched ? prep_capture_first_word : 1'b0;

wire [1:0] prep_roi_mode_0 = roi_boost_mode_xy(prep_x_pix_base + 12'd0, {1'b0, prep_data_line_y},
                                               roi_session_active, roi_session_left_bias,
                                               roi_session_x1, roi_session_y1, roi_session_x2, roi_session_y2);
wire [1:0] prep_roi_mode_1 = roi_boost_mode_xy(prep_x_pix_base + 12'd1, {1'b0, prep_data_line_y},
                                               roi_session_active, roi_session_left_bias,
                                               roi_session_x1, roi_session_y1, roi_session_x2, roi_session_y2);
wire [1:0] prep_roi_mode_2 = roi_boost_mode_xy(prep_x_pix_base + 12'd2, {1'b0, prep_data_line_y},
                                               roi_session_active, roi_session_left_bias,
                                               roi_session_x1, roi_session_y1, roi_session_x2, roi_session_y2);
wire [1:0] prep_roi_mode_3 = roi_boost_mode_xy(prep_x_pix_base + 12'd3, {1'b0, prep_data_line_y},
                                               roi_session_active, roi_session_left_bias,
                                               roi_session_x1, roi_session_y1, roi_session_x2, roi_session_y2);
wire [1:0] prep_roi_mode_4 = roi_boost_mode_xy(prep_x_pix_base + 12'd4, {1'b0, prep_data_line_y},
                                               roi_session_active, roi_session_left_bias,
                                               roi_session_x1, roi_session_y1, roi_session_x2, roi_session_y2);
wire [1:0] prep_roi_mode_5 = roi_boost_mode_xy(prep_x_pix_base + 12'd5, {1'b0, prep_data_line_y},
                                               roi_session_active, roi_session_left_bias,
                                               roi_session_x1, roi_session_y1, roi_session_x2, roi_session_y2);
wire [1:0] prep_roi_mode_6 = roi_boost_mode_xy(prep_x_pix_base + 12'd6, {1'b0, prep_data_line_y},
                                               roi_session_active, roi_session_left_bias,
                                               roi_session_x1, roi_session_y1, roi_session_x2, roi_session_y2);
wire [1:0] prep_roi_mode_7 = roi_boost_mode_xy(prep_x_pix_base + 12'd7, {1'b0, prep_data_line_y},
                                               roi_session_active, roi_session_left_bias,
                                               roi_session_x1, roi_session_y1, roi_session_x2, roi_session_y2);
wire [15:0] prep_roi_mode_word = {prep_roi_mode_7, prep_roi_mode_6, prep_roi_mode_5, prep_roi_mode_4,
                                  prep_roi_mode_3, prep_roi_mode_2, prep_roi_mode_1, prep_roi_mode_0};
wire [7:0] prep_stage_a_top_hist_0_seed = (prep_data_word_x == 8'd0) ? prep_luma_word_top[7:0] :
                                          (prep_stage_a_valid ? prep_stage_a_luma_top_word[55:48] : prep_top_hist_0_q);
wire [7:0] prep_stage_a_top_hist_1_seed = (prep_data_word_x == 8'd0) ? prep_luma_word_top[7:0] :
                                          (prep_stage_a_valid ? prep_stage_a_luma_top_word[63:56] : prep_top_hist_1_q);
wire [7:0] prep_stage_a_mid_hist_0_seed = (prep_data_word_x == 8'd0) ? prep_luma_word_mid[7:0] :
                                          (prep_stage_a_valid ? prep_stage_a_luma_mid_word[55:48] : prep_mid_hist_0_q);
wire [7:0] prep_stage_a_mid_hist_1_seed = (prep_data_word_x == 8'd0) ? prep_luma_word_mid[7:0] :
                                          (prep_stage_a_valid ? prep_stage_a_luma_mid_word[63:56] : prep_mid_hist_1_q);
wire [7:0] prep_stage_a_bot_hist_0_seed = (prep_data_word_x == 8'd0) ? prep_luma_word_cur[7:0] :
                                          (prep_stage_a_valid ? prep_stage_a_luma_bot_word[55:48] : prep_bot_hist_0_q);
wire [7:0] prep_stage_a_bot_hist_1_seed = (prep_data_word_x == 8'd0) ? prep_luma_word_cur[7:0] :
                                          (prep_stage_a_valid ? prep_stage_a_luma_bot_word[63:56] : prep_bot_hist_1_q);

always @* begin
    prep_stage_b_median_word_c = 64'd0;
    prep_stage_b_min_word_c = 64'd0;
    prep_stage_b_max_word_c = 64'd0;
    prep_top_hist_0_d = prep_stage_a_top_hist_0;
    prep_top_hist_1_d = prep_stage_a_top_hist_1;
    prep_mid_hist_0_d = prep_stage_a_mid_hist_0;
    prep_mid_hist_1_d = prep_stage_a_mid_hist_1;
    prep_bot_hist_0_d = prep_stage_a_bot_hist_0;
    prep_bot_hist_1_d = prep_stage_a_bot_hist_1;

    for (prep_k = 0; prep_k < 8; prep_k = prep_k + 1) begin
        prep_top_cur_r = prep_stage_a_luma_top_word[(prep_k * 8) +: 8];
        prep_mid_cur_r = prep_stage_a_luma_mid_word[(prep_k * 8) +: 8];
        prep_bot_cur_r = prep_stage_a_luma_bot_word[(prep_k * 8) +: 8];
        prep_top_min_r = min3_u8(prep_top_hist_0_d, prep_top_hist_1_d, prep_top_cur_r);
        prep_mid_min_r = min3_u8(prep_mid_hist_0_d, prep_mid_hist_1_d, prep_mid_cur_r);
        prep_bot_min_r = min3_u8(prep_bot_hist_0_d, prep_bot_hist_1_d, prep_bot_cur_r);
        prep_top_max_r = max3_u8(prep_top_hist_0_d, prep_top_hist_1_d, prep_top_cur_r);
        prep_mid_max_r = max3_u8(prep_mid_hist_0_d, prep_mid_hist_1_d, prep_mid_cur_r);
        prep_bot_max_r = max3_u8(prep_bot_hist_0_d, prep_bot_hist_1_d, prep_bot_cur_r);
        prep_y_med_r = median9_u8(prep_top_hist_0_d, prep_top_hist_1_d, prep_top_cur_r,
                                  prep_mid_hist_0_d, prep_mid_hist_1_d, prep_mid_cur_r,
                                  prep_bot_hist_0_d, prep_bot_hist_1_d, prep_bot_cur_r);
        prep_y_min_r = min3_u8(prep_top_min_r, prep_mid_min_r, prep_bot_min_r);
        prep_y_max_r = max3_u8(prep_top_max_r, prep_mid_max_r, prep_bot_max_r);
        prep_stage_b_median_word_c[(prep_k * 8) +: 8] = prep_y_med_r;
        prep_stage_b_min_word_c[(prep_k * 8) +: 8] = prep_y_min_r;
        prep_stage_b_max_word_c[(prep_k * 8) +: 8] = prep_y_max_r;
        prep_top_hist_0_d = prep_top_hist_1_d;
        prep_top_hist_1_d = prep_top_cur_r;
        prep_mid_hist_0_d = prep_mid_hist_1_d;
        prep_mid_hist_1_d = prep_mid_cur_r;
        prep_bot_hist_0_d = prep_bot_hist_1_d;
        prep_bot_hist_1_d = prep_bot_cur_r;
    end
end

always @* begin
    prep_stage_c_y_word_c = 64'd0;
    for (prep_stage_c_k = 0; prep_stage_c_k < 8; prep_stage_c_k = prep_stage_c_k + 1) begin
        prep_stage_c_bot_cur_r = prep_stage_b_luma_bot_word[(prep_stage_c_k * 8) +: 8];
        prep_stage_c_y_med_r = prep_stage_b_median_word[(prep_stage_c_k * 8) +: 8];
        prep_stage_c_y_min_r = prep_stage_b_min_word[(prep_stage_c_k * 8) +: 8];
        prep_stage_c_y_max_r = prep_stage_b_max_word[(prep_stage_c_k * 8) +: 8];
        prep_stage_c_roi_mode_r = prep_stage_b_roi_mode_word[(prep_stage_c_k * 2) +: 2];
        prep_stage_c_y_word_c[(prep_stage_c_k * 8) +: 8] = spatial_enhance_y(prep_stage_c_bot_cur_r,
                                                                              prep_stage_c_y_med_r,
                                                                              prep_stage_c_y_min_r,
                                                                              prep_stage_c_y_max_r,
                                                                              prep_median_en_latched,
                                                                              prep_clahe_en_latched,
                                                                              prep_usm_en_latched,
                                                                              prep_ocr_stroke_latched,
                                                                              prep_stage_c_roi_mode_r,
                                                                              prep_clahe_cfg_latched,
                                                                              prep_usm_cfg_latched,
                                                                              prep_med_cfg_latched);
    end
end

wire [7:0]  prep_pair_active_y_0 = prep_pair_active_y_word[7:0];
wire [7:0]  prep_pair_active_y_1 = prep_pair_active_y_word[15:8];
wire [7:0]  prep_pair_active_y_2 = prep_pair_active_y_word[23:16];
wire [7:0]  prep_pair_active_y_3 = prep_pair_active_y_word[31:24];
wire [7:0]  prep_pair_active_y_4 = prep_pair_active_y_word[39:32];
wire [7:0]  prep_pair_active_y_5 = prep_pair_active_y_word[47:40];
wire [7:0]  prep_pair_active_y_6 = prep_pair_active_y_word[55:48];
wire [7:0]  prep_pair_active_y_7 = prep_pair_active_y_word[63:56];
wire [127:0] out_pair_active_raw_lo_pack = pack_4pix_bgrx(
    raw_lo_src_word[15:0], raw_lo_src_word[31:16],
    raw_lo_src_word[47:32], raw_lo_src_word[63:48],
    8'h00, 8'h00, 8'h00, 8'h00);
wire [127:0] out_pair_active_raw_hi_pack = pack_4pix_bgrx(
    raw_hi_src_word[79:64], raw_hi_src_word[95:80],
    raw_hi_src_word[111:96], raw_hi_src_word[127:112],
    8'h00, 8'h00, 8'h00, 8'h00);
wire [127:0] prep_pair_active_prep_lo_pack = pack_4prep_bgrx(
    prep_pair_active_src_word[15:0], prep_pair_active_src_word[31:16],
    prep_pair_active_src_word[47:32], prep_pair_active_src_word[63:48],
    prep_pair_active_y_0, prep_pair_active_y_1, prep_pair_active_y_2, prep_pair_active_y_3,
    prep_target_all_latched,
    prep_a_fmt_yenh_latched,
    prep_pair_active_first_word,
    frame_id);
wire [127:0] prep_pair_active_prep_hi_pack = pack_4prep_bgrx(
    prep_pair_active_src_word[79:64], prep_pair_active_src_word[95:80],
    prep_pair_active_src_word[111:96], prep_pair_active_src_word[127:112],
    prep_pair_active_y_4, prep_pair_active_y_5, prep_pair_active_y_6, prep_pair_active_y_7,
    prep_target_all_latched,
    prep_a_fmt_yenh_latched,
    1'b0,
    frame_id);
wire [127:0] frame_dma_data_raw = dma_expand_phase ? out_pair_active_raw_hi_pack : out_pair_active_raw_lo_pack;
wire [127:0] frame_dma_data_prep = dma_expand_phase ? prep_pair_active_prep_hi_pack : prep_pair_active_prep_lo_pack;
wire        prep_output_active = prep_active_latched & out_pair_active_valid & prep_pair_active_valid;
wire [127:0] post_ddr_pattern_data_565 = {8{post_ddr_color_data}};
wire [127:0] post_ddr_pattern_data_bgrx = {4{bgr565_to_bgrx32(post_ddr_color_data, 8'h00)}};
wire [127:0] post_ddr_pattern_data = dma_expand_mode ? post_ddr_pattern_data_bgrx : post_ddr_pattern_data_565;
wire [127:0] frame_dma_data = dma_expand_mode
    ? (prep_output_active ? frame_dma_data_prep : frame_dma_data_raw)
    : frame_rd_data;
wire        prep_pipe_valid = prep_active_latched ? (out_pair_active_valid & prep_pair_active_valid)
                                                  : frame_rd_data_ready;
wire        frame_stream_ready = ~dma_session_active | prep_pipe_valid;

assign axis_slave2_tready_fc = axis_slave2_tready_raw & frame_stream_ready;
always @(posedge pclk_div2 or negedge core_rst_n) begin
    if (!core_rst_n)
        mwr_rd_addr_d <= 12'd0;
    else
        mwr_rd_addr_d <= mwr_rd_addr;
end

always @(posedge pclk_div2 or negedge core_rst_n) begin
    if (!core_rst_n)
        mwr_rd_clk_en_d <= 1'b0;
    else
        mwr_rd_clk_en_d <= mwr_rd_clk_en;
end

always @(posedge pclk_div2 or negedge core_rst_n) begin
    if (!core_rst_n) begin
        dma_session_active <= 1'b0;
        dma_rd_word_count <= 18'd0;
        frame_src_req_count <= 18'd0;
        rd_fsync_stretch_cnt <= 6'd0;
        prep_word_x <= 8'd0;
        prep_line_y <= 10'd0;
        prep_linebuf_req_valid_d0 <= 1'b0;
        prep_linebuf_req_valid_d1 <= 1'b0;
        prep_linebuf_req_word_x_d0 <= 8'd0;
        prep_linebuf_req_word_x_d1 <= 8'd0;
        prep_linebuf_req_line_y_d0 <= 10'd0;
        prep_linebuf_req_line_y_d1 <= 10'd0;
        prep_linebuf_prev1_rd_d0 <= 64'd0;
        prep_linebuf_prev1_rd_d1 <= 64'd0;
        prep_linebuf_prev2_rd_d0 <= 64'd0;
        prep_linebuf_prev2_rd_d1 <= 64'd0;
        frame_src_bootstrap_count <= 3'd0;
        dma_expand_phase <= 1'b0;
        frame_rd_data_hold <= 128'd0;
        raw_startup_active <= 1'b0;
        raw_start_word_valid <= 1'b0;
        raw_start_word <= 128'd0;
        frame_id <= 8'd0;
        prep_session_en <= 1'b0;
        prep_session_target_all <= 1'b0;
        prep_session_a_fmt_yenh <= 1'b0;
        prep_session_ocr_stroke <= 1'b0;
        prep_session_median_en <= 1'b0;
        prep_session_clahe_en <= 1'b0;
        prep_session_usm_en <= 1'b0;
        prep_session_clahe_cfg <= 32'd0;
        prep_session_usm_cfg <= 32'd0;
        prep_session_med_cfg <= 32'd0;
        roi_session_active <= 1'b0;
        roi_session_left_bias <= 1'b0;
        roi_session_x1 <= 12'd0;
        roi_session_x2 <= 12'd0;
        roi_session_y1 <= 11'd0;
        roi_session_y2 <= 11'd0;
        roi_timeout_count <= 8'd0;
        roi_cfg_seen_x1y1 <= 32'd0;
        roi_cfg_seen_x2y2 <= 32'd0;
        roi_cfg_seen_ctrl <= 32'd0;
        prep_top_hist_0_q <= 8'd0;
        prep_top_hist_1_q <= 8'd0;
        prep_mid_hist_0_q <= 8'd0;
        prep_mid_hist_1_q <= 8'd0;
        prep_bot_hist_0_q <= 8'd0;
        prep_bot_hist_1_q <= 8'd0;
        prep_stage_a_valid <= 1'b0;
        prep_stage_a_src_word <= 128'd0;
        prep_stage_a_luma_top_word <= 64'd0;
        prep_stage_a_luma_mid_word <= 64'd0;
        prep_stage_a_luma_bot_word <= 64'd0;
        prep_stage_a_roi_mode_word <= 16'd0;
        prep_stage_a_first_word <= 1'b0;
        prep_stage_a_top_hist_0 <= 8'd0;
        prep_stage_a_top_hist_1 <= 8'd0;
        prep_stage_a_mid_hist_0 <= 8'd0;
        prep_stage_a_mid_hist_1 <= 8'd0;
        prep_stage_a_bot_hist_0 <= 8'd0;
        prep_stage_a_bot_hist_1 <= 8'd0;
        prep_stage_b_valid <= 1'b0;
        prep_stage_b_src_word <= 128'd0;
        prep_stage_b_luma_bot_word <= 64'd0;
        prep_stage_b_roi_mode_word <= 16'd0;
        prep_stage_b_first_word <= 1'b0;
        prep_stage_b_median_word <= 64'd0;
        prep_stage_b_min_word <= 64'd0;
        prep_stage_b_max_word <= 64'd0;
        prep_stage_c_valid <= 1'b0;
        prep_stage_c_src_word <= 128'd0;
        prep_stage_c_y_word <= 64'd0;
        prep_stage_c_first_word <= 1'b0;
        out_pair_active_valid <= 1'b0;
        out_pair_active_src_word <= 128'd0;
        out_pair_active_first_word <= 1'b0;
        out_pair_next_valid <= 1'b0;
        out_pair_next_src_word <= 128'd0;
        out_pair_next_first_word <= 1'b0;
        prep_pair_active_valid <= 1'b0;
        prep_pair_active_src_word <= 128'd0;
        prep_pair_active_y_word <= 64'd0;
        prep_pair_active_first_word <= 1'b0;
        prep_pair_next_valid <= 1'b0;
        prep_pair_next_src_word <= 128'd0;
        prep_pair_next_y_word <= 64'd0;
        prep_pair_next_first_word <= 1'b0;
    end else if (frame_done_pulse) begin
        dma_session_active <= 1'b0;
        dma_rd_word_count <= 18'd0;
        frame_src_req_count <= 18'd0;
        rd_fsync_stretch_cnt <= 6'd0;
        prep_word_x <= 8'd0;
        prep_line_y <= 10'd0;
        prep_linebuf_req_valid_d0 <= 1'b0;
        prep_linebuf_req_valid_d1 <= 1'b0;
        prep_linebuf_req_word_x_d0 <= 8'd0;
        prep_linebuf_req_word_x_d1 <= 8'd0;
        prep_linebuf_req_line_y_d0 <= 10'd0;
        prep_linebuf_req_line_y_d1 <= 10'd0;
        prep_linebuf_prev1_rd_d0 <= 64'd0;
        prep_linebuf_prev1_rd_d1 <= 64'd0;
        prep_linebuf_prev2_rd_d0 <= 64'd0;
        prep_linebuf_prev2_rd_d1 <= 64'd0;
        frame_src_bootstrap_count <= 3'd0;
        dma_expand_phase <= 1'b0;
        raw_startup_active <= 1'b0;
        raw_start_word_valid <= 1'b0;
        raw_start_word <= 128'd0;
        prep_session_en <= 1'b0;
        prep_session_target_all <= 1'b0;
        prep_session_a_fmt_yenh <= 1'b0;
        prep_session_ocr_stroke <= 1'b0;
        prep_session_median_en <= 1'b0;
        prep_session_clahe_en <= 1'b0;
        prep_session_usm_en <= 1'b0;
        prep_session_clahe_cfg <= 32'd0;
        prep_session_usm_cfg <= 32'd0;
        prep_session_med_cfg <= 32'd0;
        roi_session_active <= 1'b0;
        roi_session_left_bias <= 1'b0;
        prep_top_hist_0_q <= 8'd0;
        prep_top_hist_1_q <= 8'd0;
        prep_mid_hist_0_q <= 8'd0;
        prep_mid_hist_1_q <= 8'd0;
        prep_bot_hist_0_q <= 8'd0;
        prep_bot_hist_1_q <= 8'd0;
        prep_stage_a_valid <= 1'b0;
        prep_stage_a_src_word <= 128'd0;
        prep_stage_a_luma_top_word <= 64'd0;
        prep_stage_a_luma_mid_word <= 64'd0;
        prep_stage_a_luma_bot_word <= 64'd0;
        prep_stage_a_roi_mode_word <= 16'd0;
        prep_stage_a_first_word <= 1'b0;
        prep_stage_a_top_hist_0 <= 8'd0;
        prep_stage_a_top_hist_1 <= 8'd0;
        prep_stage_a_mid_hist_0 <= 8'd0;
        prep_stage_a_mid_hist_1 <= 8'd0;
        prep_stage_a_bot_hist_0 <= 8'd0;
        prep_stage_a_bot_hist_1 <= 8'd0;
        prep_stage_b_valid <= 1'b0;
        prep_stage_b_src_word <= 128'd0;
        prep_stage_b_luma_bot_word <= 64'd0;
        prep_stage_b_roi_mode_word <= 16'd0;
        prep_stage_b_first_word <= 1'b0;
        prep_stage_b_median_word <= 64'd0;
        prep_stage_b_min_word <= 64'd0;
        prep_stage_b_max_word <= 64'd0;
        prep_stage_c_valid <= 1'b0;
        prep_stage_c_src_word <= 128'd0;
        prep_stage_c_y_word <= 64'd0;
        prep_stage_c_first_word <= 1'b0;
        out_pair_active_valid <= 1'b0;
        out_pair_active_src_word <= 128'd0;
        out_pair_active_first_word <= 1'b0;
        out_pair_next_valid <= 1'b0;
        out_pair_next_src_word <= 128'd0;
        out_pair_next_first_word <= 1'b0;
        prep_pair_active_valid <= 1'b0;
        prep_pair_active_src_word <= 128'd0;
        prep_pair_active_y_word <= 64'd0;
        prep_pair_active_first_word <= 1'b0;
        prep_pair_next_valid <= 1'b0;
        prep_pair_next_src_word <= 128'd0;
        prep_pair_next_y_word <= 64'd0;
        prep_pair_next_first_word <= 1'b0;
    end else begin
        if (dma_session_start) begin
            dma_session_active <= 1'b1;
            dma_rd_word_count <= 18'd0;
            frame_src_req_count <= 18'd0;
            rd_fsync_stretch_cnt <= 6'd31;
            prep_word_x <= 8'd0;
            prep_line_y <= 10'd0;
            prep_linebuf_req_valid_d0 <= 1'b0;
            prep_linebuf_req_valid_d1 <= 1'b0;
            prep_linebuf_req_word_x_d0 <= 8'd0;
            prep_linebuf_req_word_x_d1 <= 8'd0;
            prep_linebuf_req_line_y_d0 <= 10'd0;
            prep_linebuf_req_line_y_d1 <= 10'd0;
            prep_linebuf_prev1_rd_d0 <= 64'd0;
            prep_linebuf_prev1_rd_d1 <= 64'd0;
            prep_linebuf_prev2_rd_d0 <= 64'd0;
            prep_linebuf_prev2_rd_d1 <= 64'd0;
            frame_src_bootstrap_count <= 3'd0;
            dma_expand_phase <= 1'b0;
            frame_rd_data_hold <= 128'd0;
            raw_startup_active <= ~prep_active;
            raw_start_word_valid <= 1'b0;
            raw_start_word <= 128'd0;
            frame_id <= frame_id + 8'd1;
            prep_session_en <= prep_active;
            prep_session_target_all <= prep_target_all;
            prep_session_a_fmt_yenh <= prep_a_fmt_yenh;
            prep_session_ocr_stroke <= prep_ocr_stroke_mode;
            prep_session_median_en <= prep_median_en;
            prep_session_clahe_en <= prep_clahe_en;
            prep_session_usm_en <= prep_usm_en;
            prep_session_clahe_cfg <= fpga_prep_clahe;
            prep_session_usm_cfg <= fpga_prep_usm;
            prep_session_med_cfg <= fpga_prep_med;
            roi_cfg_seen_x1y1 <= fpga_roi_x1y1;
            roi_cfg_seen_x2y2 <= fpga_roi_x2y2;
            roi_cfg_seen_ctrl <= fpga_roi_ctrl;
            prep_top_hist_0_q <= 8'd0;
            prep_top_hist_1_q <= 8'd0;
            prep_mid_hist_0_q <= 8'd0;
            prep_mid_hist_1_q <= 8'd0;
            prep_bot_hist_0_q <= 8'd0;
            prep_bot_hist_1_q <= 8'd0;
            prep_stage_a_valid <= 1'b0;
            prep_stage_a_src_word <= 128'd0;
            prep_stage_a_luma_top_word <= 64'd0;
            prep_stage_a_luma_mid_word <= 64'd0;
            prep_stage_a_luma_bot_word <= 64'd0;
            prep_stage_a_roi_mode_word <= 16'd0;
            prep_stage_a_first_word <= 1'b0;
            prep_stage_a_top_hist_0 <= 8'd0;
            prep_stage_a_top_hist_1 <= 8'd0;
            prep_stage_a_mid_hist_0 <= 8'd0;
            prep_stage_a_mid_hist_1 <= 8'd0;
            prep_stage_a_bot_hist_0 <= 8'd0;
            prep_stage_a_bot_hist_1 <= 8'd0;
            prep_stage_b_valid <= 1'b0;
            prep_stage_b_src_word <= 128'd0;
            prep_stage_b_luma_bot_word <= 64'd0;
            prep_stage_b_roi_mode_word <= 16'd0;
            prep_stage_b_first_word <= 1'b0;
            prep_stage_b_median_word <= 64'd0;
            prep_stage_b_min_word <= 64'd0;
            prep_stage_b_max_word <= 64'd0;
            prep_stage_c_valid <= 1'b0;
            prep_stage_c_src_word <= 128'd0;
            prep_stage_c_y_word <= 64'd0;
            prep_stage_c_first_word <= 1'b0;
            out_pair_active_valid <= 1'b0;
            out_pair_active_src_word <= 128'd0;
            out_pair_active_first_word <= 1'b0;
            out_pair_next_valid <= 1'b0;
            out_pair_next_src_word <= 128'd0;
            out_pair_next_first_word <= 1'b0;
            prep_pair_active_valid <= 1'b0;
            prep_pair_active_src_word <= 128'd0;
            prep_pair_active_y_word <= 64'd0;
            prep_pair_active_first_word <= 1'b0;
            prep_pair_next_valid <= 1'b0;
            prep_pair_next_src_word <= 128'd0;
            prep_pair_next_y_word <= 64'd0;
            prep_pair_next_first_word <= 1'b0;
            if (prep_active && prep_ocr_stroke_mode && prep_target_ocr_only) begin
                if (roi_cfg_changed) begin
                    if (roi_cfg_enable && roi_cfg_bbox_valid) begin
                        roi_session_active <= 1'b1;
                        roi_session_left_bias <= roi_cfg_left_bias;
                        roi_session_x1 <= roi_cfg_x1;
                        roi_session_x2 <= roi_cfg_x2;
                        roi_session_y1 <= roi_cfg_y1;
                        roi_session_y2 <= roi_cfg_y2;
                        roi_timeout_count <= roi_cfg_timeout_frames;
                    end else begin
                        roi_session_active <= 1'b0;
                        roi_session_left_bias <= 1'b0;
                        roi_timeout_count <= 8'd0;
                    end
                end else if (roi_session_active) begin
                    if (roi_timeout_count != 8'd0)
                        roi_timeout_count <= roi_timeout_count - 8'd1;
                    else begin
                        roi_session_active <= 1'b0;
                        roi_session_left_bias <= 1'b0;
                    end
                end else begin
                    roi_timeout_count <= 8'd0;
                end
            end else begin
                roi_session_active <= 1'b0;
                roi_session_left_bias <= 1'b0;
                roi_timeout_count <= 8'd0;
            end
        end else begin
            prep_linebuf_req_valid_d1 <= prep_linebuf_req_valid_d0;
            prep_linebuf_req_word_x_d1 <= prep_linebuf_req_word_x_d0;
            prep_linebuf_req_line_y_d1 <= prep_linebuf_req_line_y_d0;
            prep_linebuf_prev1_rd_d1 <= prep_linebuf_prev1_rd_d0;
            prep_linebuf_prev2_rd_d1 <= prep_linebuf_prev2_rd_d0;
            prep_linebuf_req_valid_d0 <= 1'b0;

            if (dma_session_active && bar2_addr_step) begin
                if (dma_rd_word_count == frame_words_cfg - 1'b1) begin
                    dma_rd_word_count <= 18'd0;
                    dma_session_active <= 1'b0;
                    prep_session_en <= 1'b0;
                    prep_session_target_all <= 1'b0;
                    prep_session_a_fmt_yenh <= 1'b0;
                    prep_session_ocr_stroke <= 1'b0;
                    prep_session_median_en <= 1'b0;
                    prep_session_clahe_en <= 1'b0;
                    prep_session_usm_en <= 1'b0;
                end else begin
                    dma_rd_word_count <= dma_rd_word_count + 18'd1;
                end
            end

            if (dma_expand_mode && dma_session_active && bar2_addr_step)
                dma_expand_phase <= ~dma_expand_phase;

            if (raw_frame_hold_en) begin
                frame_rd_data_hold <= frame_rd_data;
                if (raw_startup_active && !raw_start_word_valid) begin
                    raw_start_word <= frame_rd_data;
                    raw_start_word_valid <= 1'b1;
                end
            end

            if (raw_startup_done)
                raw_startup_active <= 1'b0;

            if (rd_fsync_stretch_cnt != 6'd0)
                rd_fsync_stretch_cnt <= rd_fsync_stretch_cnt - 6'd1;

            if (frame_rd_req_en) begin
                frame_src_req_count <= frame_src_req_count + 18'd1;
                if (frame_src_bootstrap_count != frame_bootstrap_words)
                    frame_src_bootstrap_count <= frame_src_bootstrap_count + 3'd1;
                prep_linebuf_req_valid_d0 <= 1'b1;
                prep_linebuf_req_word_x_d0 <= prep_word_x;
                prep_linebuf_req_line_y_d0 <= prep_line_y;
                prep_linebuf_prev1_rd_d0 <= prep_linebuf_prev1[prep_word_x];
                prep_linebuf_prev2_rd_d0 <= prep_linebuf_prev2[prep_word_x];
                if (prep_word_x == (PREP_FETCH_WORDS_PER_LINE - 1'b1)) begin
                    prep_word_x <= 8'd0;
                    if (prep_line_y == 10'd719)
                        prep_line_y <= 10'd0;
                    else
                        prep_line_y <= prep_line_y + 10'd1;
                end else begin
                    prep_word_x <= prep_word_x + 8'd1;
                end
            end

            if (out_pair_pop) begin
                if (out_pair_next_valid) begin
                    out_pair_active_valid <= 1'b1;
                    out_pair_active_src_word <= out_pair_next_src_word;
                    out_pair_active_first_word <= out_pair_next_first_word;
                    out_pair_next_valid <= 1'b0;
                end else begin
                    out_pair_active_valid <= 1'b0;
                end

                if (prep_pair_next_valid) begin
                    prep_pair_active_valid <= 1'b1;
                    prep_pair_active_src_word <= prep_pair_next_src_word;
                    prep_pair_active_y_word <= prep_pair_next_y_word;
                    prep_pair_active_first_word <= prep_pair_next_first_word;
                    prep_pair_next_valid <= 1'b0;
                end else begin
                    prep_pair_active_valid <= 1'b0;
                end
            end

            prep_stage_b_valid <= prep_stage_a_valid;
            if (prep_stage_a_valid) begin
                prep_top_hist_0_q <= prep_stage_a_luma_top_word[55:48];
                prep_top_hist_1_q <= prep_stage_a_luma_top_word[63:56];
                prep_mid_hist_0_q <= prep_stage_a_luma_mid_word[55:48];
                prep_mid_hist_1_q <= prep_stage_a_luma_mid_word[63:56];
                prep_bot_hist_0_q <= prep_stage_a_luma_bot_word[55:48];
                prep_bot_hist_1_q <= prep_stage_a_luma_bot_word[63:56];
                prep_stage_b_src_word <= prep_stage_a_src_word;
                prep_stage_b_luma_bot_word <= prep_stage_a_luma_bot_word;
                prep_stage_b_roi_mode_word <= prep_stage_a_roi_mode_word;
                prep_stage_b_first_word <= prep_stage_a_first_word;
                prep_stage_b_median_word <= prep_stage_b_median_word_c;
                prep_stage_b_min_word <= prep_stage_b_min_word_c;
                prep_stage_b_max_word <= prep_stage_b_max_word_c;
            end

            prep_stage_c_valid <= prep_stage_b_valid;
            if (prep_stage_b_valid) begin
                prep_stage_c_src_word <= prep_stage_b_src_word;
                prep_stage_c_y_word <= prep_stage_c_y_word_c;
                prep_stage_c_first_word <= prep_stage_b_first_word;
            end

            prep_stage_a_valid <= 1'b0;
            if (pair_capture_fire) begin
                if (!out_pair_active_valid || (out_pair_pop && !out_pair_next_valid)) begin
                    out_pair_active_valid <= 1'b1;
                    out_pair_active_src_word <= frame_rd_data;
                    out_pair_active_first_word <= pair_capture_first_word;
                end else if (!out_pair_next_valid || out_pair_pop) begin
                    out_pair_next_valid <= 1'b1;
                    out_pair_next_src_word <= frame_rd_data;
                    out_pair_next_first_word <= pair_capture_first_word;
                end
            end

            if (prep_capture_fire) begin
                prep_linebuf_prev2[prep_data_word_x] <= prep_linebuf_prev1_rd_d1;
                prep_linebuf_prev1[prep_data_word_x] <= frame_rd_luma_word;

                if (prep_active_latched) begin
                    prep_stage_a_valid <= 1'b1;
                    prep_stage_a_src_word <= frame_rd_data;
                    prep_stage_a_luma_top_word <= prep_luma_word_top;
                    prep_stage_a_luma_mid_word <= prep_luma_word_mid;
                    prep_stage_a_luma_bot_word <= prep_luma_word_cur;
                    prep_stage_a_roi_mode_word <= prep_roi_mode_word;
                    prep_stage_a_first_word <= prep_capture_first_word;
                    prep_stage_a_top_hist_0 <= prep_stage_a_top_hist_0_seed;
                    prep_stage_a_top_hist_1 <= prep_stage_a_top_hist_1_seed;
                    prep_stage_a_mid_hist_0 <= prep_stage_a_mid_hist_0_seed;
                    prep_stage_a_mid_hist_1 <= prep_stage_a_mid_hist_1_seed;
                    prep_stage_a_bot_hist_0 <= prep_stage_a_bot_hist_0_seed;
                    prep_stage_a_bot_hist_1 <= prep_stage_a_bot_hist_1_seed;
                end
            end

            if (prep_stage_c_valid) begin
                if (!prep_pair_active_valid || (out_pair_pop && !prep_pair_next_valid)) begin
                    prep_pair_active_valid <= 1'b1;
                    prep_pair_active_src_word <= prep_stage_c_src_word;
                    prep_pair_active_y_word <= prep_stage_c_y_word;
                    prep_pair_active_first_word <= prep_stage_c_first_word;
                end else if (!prep_pair_next_valid || out_pair_pop) begin
                    prep_pair_next_valid <= 1'b1;
                    prep_pair_next_src_word <= prep_stage_c_src_word;
                    prep_pair_next_y_word <= prep_stage_c_y_word;
                    prep_pair_next_first_word <= prep_stage_c_first_word;
                end
            end
        end
    end
end

assign dma_session_start = mwr_cmd_start & ~dma_session_active;
assign rd_fsync_pclk_div2 = (rd_fsync_stretch_cnt != 6'd0);
assign mwr_rd_data = FORCE_PATTERN_POST_DDR ? post_ddr_pattern_data : frame_dma_data;

// Post-DDR pattern coordinate counters (only used when FORCE_PATTERN_POST_DDR=1)
always @(posedge pclk_div2 or negedge core_rst_n) begin
    if (!core_rst_n) begin
        post_ddr_word_x <= 9'd0;
        post_ddr_line_y <= 10'd0;
    end else if (dma_session_start) begin
        post_ddr_word_x <= 9'd0;
        post_ddr_line_y <= 10'd0;
    end else if (bar2_addr_step) begin
        if (post_ddr_word_x == post_ddr_words_per_line - 1'b1) begin
            post_ddr_word_x <= 9'd0;
            if (post_ddr_line_y == 10'd719)
                post_ddr_line_y <= 10'd0;
            else
                post_ddr_line_y <= post_ddr_line_y + 10'd1;
        end else begin
            post_ddr_word_x <= post_ddr_word_x + 9'd1;
        end
    end
end

// Frame Buffer: Camera RGB565 鈫?DDR3 鈫?Read out
fram_buf #(
    .MEM_ROW_WIDTH      (MEM_ROW_ADDR_WIDTH),
    .MEM_COLUMN_WIDTH   (MEM_COL_ADDR_WIDTH),
    .MEM_BANK_WIDTH     (MEM_BADDR_WIDTH),
    .MEM_DQ_WIDTH       (MEM_DQ_WIDTH),
    .H_NUM              (12'd1280),     // 720p horizontal
    .V_NUM              (12'd720),      // 720p vertical
    .PIX_WIDTH          (16)            // RGB565
) u_fram_buf (
    // DDR clock domain
    .ddr_clk            (core_clk_ddr),
    .ddr_rstn           (ddr_init_done),
    .ddr_init_ready     (ddr_init_done),

    // Camera input (write to DDR)
    .vin_clk            (cmos1_pclk),
    .wr_fsync           (cmos1_vsync_16bit),
    .wr_en              (cmos1_href_16bit),
    .wr_data_vld        (cmos1_pix_vld),
    .wr_data            (cmos1_wr_data),
    .init_done          (fram_buf_init_done),

    // Read output (for future PCIe DMA) - tied off for now
    .vout_clk           (pclk_div2),
    .rd_fsync           (rd_fsync_pclk_div2),
    .rd_en              (frame_rd_req_en),
    .vout_de            (frame_rd_data_valid),
    .vout_data          (frame_rd_data),
    .rd_data_ready      (frame_rd_data_ready),

    // AXI Write channel
    .axi_awaddr         (axi_awaddr),
    .axi_awid           (axi_awuser_id),
    .axi_awlen          (axi_awlen),
    .axi_awsize         (),             // DDR3 IP doesn't have this
    .axi_awburst        (),             // DDR3 IP doesn't have this
    .axi_awready        (axi_awready),
    .axi_awvalid        (axi_awvalid),
    .axi_wdata          (axi_wdata),
    .axi_wstrb          (axi_wstrb),
    .axi_wlast          (axi_wusero_last),  // DDR3 provides this
    .axi_wvalid         (),             // DDR3 IP doesn't have this
    .axi_wready         (axi_wready),
    .axi_bid            (4'd0),         // Unused

    // AXI Read channel
    .axi_araddr         (axi_araddr),
    .axi_arid           (axi_aruser_id),
    .axi_arlen          (axi_arlen),
    .axi_arsize         (),             // DDR3 IP doesn't have this
    .axi_arburst        (),             // DDR3 IP doesn't have this
    .axi_arvalid        (axi_arvalid),
    .axi_arready        (axi_arready),
    .axi_rready         (),             // DDR3 IP doesn't have this
    .axi_rdata          (axi_rdata),
    .axi_rvalid         (axi_rvalid),
    .axi_rlast          (axi_rlast),
    .axi_rid            (axi_rid)
);

//=============================================================================
// DDR3 Controller
//=============================================================================
DDR3_50H u_DDR3 (
    .ref_clk                (sys_clk),      // 25MHz input clock (was MISSING from template!)
    .resetn                 (ddr_rstn),
    .core_clk               (core_clk_ddr),
    .pll_lock               (),
    .phy_pll_lock           (),
    .gpll_lock              (),
    .rst_gpll_lock          (),
    .ddrphy_cpd_lock        (),
    .ddr_init_done          (ddr_init_done),

    // AXI Write channel - connected to fram_buf
    .axi_awaddr             (axi_awaddr),
    .axi_awuser_ap          (1'b0),
    .axi_awuser_id          (axi_awuser_id),
    .axi_awlen              (axi_awlen),
    .axi_awready            (axi_awready),
    .axi_awvalid            (axi_awvalid),
    .axi_wdata              (axi_wdata),
    .axi_wstrb              (axi_wstrb),
    .axi_wready             (axi_wready),
    .axi_wusero_id          (),
    .axi_wusero_last        (axi_wusero_last),

    // AXI Read channel - connected to fram_buf
    .axi_araddr             (axi_araddr),
    .axi_aruser_ap          (1'b0),
    .axi_aruser_id          (axi_aruser_id),
    .axi_arlen              (axi_arlen),
    .axi_arready            (axi_arready),
    .axi_arvalid            (axi_arvalid),
    .axi_rdata              (axi_rdata),
    .axi_rid                (axi_rid),
    .axi_rlast              (axi_rlast),
    .axi_rvalid             (axi_rvalid),

    // APB interface - tied off
    .apb_clk                (cfg_clk),
    .apb_rst_n              (ddr_rstn),
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

    // Debug signals - tied off
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

endmodule
