//pango参考例程
//基于官方demo修改 - 添加 DDR3 + power_on_delay 测试
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
wire			pclk_div2/*synthesis PAP_MARK_DEBUG="1"*/;  	// 用户时钟，x2 5gt/s时，为125MHZ 2.5gt/s时为62.5
wire			pclk/*synthesis PAP_MARK_DEBUG="1"*/;			// 用户时钟，x2 5gt/s时，为125MHZ 2.5gt/s时为62.5			
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

wire	[4:0]	smlh_ltssm_state/*synthesis PAP_MARK_DEBUG="1"*/;//link状态机

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
	// Stability hotfix: keep raw ready to avoid frame-mode deadlock on long run.
	.i_axis_slave2_trdy		(axis_slave2_tready_raw),
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
	.o_cross_4kb_boundary	(cross_4kb_boundary),	//4k边界
	.o_tx_restart_ext		(mwr_cmd_start),
	.o_frame_done_pulse_ext	(frame_done_pulse),
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
	.smlh_link_up				(smlh_link_up),			//link状态
	.rdlh_link_up				(rdlh_link_up),			//link状态
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
    .reg_index      (),                 // Debug: current register index
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
// Frame Buffer (Camera → DDR3)
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
wire                       frame_rd_data_ready;

//=============================================================================
// MWR Data Source (frame data for DMA transfer to host)
//=============================================================================
// Start one read session on DMA command start and keep the session
// active across chunk gaps until a full frame has been consumed.
localparam [17:0]          FRAME_WORDS_565  = (1280 * 720 * 16) / 128;
localparam [17:0]          FRAME_WORDS_BGRX = (1280 * 720 * 32) / 128;
reg                        dma_session_active;
reg  [17:0]                dma_rd_word_count;
reg  [5:0]                 rd_fsync_stretch_cnt;
reg  [8:0]                 post_ddr_word_x;
reg  [9:0]                 post_ddr_line_y;
reg                        dma_expand_phase;
reg                        mwr_first_beat_seen;
reg  [11:0]                mwr_rd_addr_d;
reg                        mwr_rd_clk_en_d;
reg  [127:0]               frame_rd_data_hold;
reg  [7:0]                 frame_id;
wire                       dma_session_start;
wire                       rd_fsync_pclk_div2;
wire                       dma_expand_mode = DMA_OUTPUT_BGRX;
wire                       preproc_en = PREPROC_ENABLE_DEFAULT;
wire [17:0]                frame_words_cfg = dma_expand_mode ? FRAME_WORDS_BGRX : FRAME_WORDS_565;
// Count chunk first beat as a valid step to prevent boundary phase slip.
wire                       bar2_addr_step = mwr_rd_clk_en &&
                                            ((mwr_rd_addr != mwr_rd_addr_d) || (~mwr_rd_clk_en_d));
wire                       frame_rd_fetch_en = bar2_addr_step & (~dma_expand_mode | ~dma_expand_phase);
wire [11:0]                post_ddr_x_pix = dma_expand_mode ? {1'b0, post_ddr_word_x, 2'b00}
                                                             : {post_ddr_word_x, 3'b000};
wire [15:0]                post_ddr_color_base = color_bar_bgr565(post_ddr_x_pix);
wire [15:0]                post_ddr_color_data = post_ddr_color_base;
wire [8:0]                 post_ddr_words_per_line = dma_expand_mode ? 9'd320 : 9'd160;

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

function [31:0] bgr565_to_bgrx32;
    input [15:0] pix565;
    input [7:0] alpha8;
    reg [4:0] r5;
    reg [5:0] g6;
    reg [4:0] b5;
begin
    r5 = pix565[4:0];
    g6 = pix565[10:5];
    b5 = pix565[15:11];
    bgr565_to_bgrx32 = {alpha8, expand5_to_8(r5), expand6_to_8(g6), expand5_to_8(b5)};
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

wire [7:0] alpha_lo_0_base = preproc_en ? preproc_alpha_from_bgr565(frame_rd_data[15:0]) : 8'h00;
wire [7:0] alpha_lo_1 = preproc_en ? preproc_alpha_from_bgr565(frame_rd_data[31:16]) : 8'h00;
wire [7:0] alpha_lo_2 = preproc_en ? preproc_alpha_from_bgr565(frame_rd_data[47:32]) : 8'h00;
wire [7:0] alpha_lo_3 = preproc_en ? preproc_alpha_from_bgr565(frame_rd_data[63:48]) : 8'h00;

wire [7:0] alpha_hi_0 = preproc_en ? preproc_alpha_from_bgr565(frame_rd_data_hold[79:64]) : 8'h00;
wire [7:0] alpha_hi_1 = preproc_en ? preproc_alpha_from_bgr565(frame_rd_data_hold[95:80]) : 8'h00;
wire [7:0] alpha_hi_2 = preproc_en ? preproc_alpha_from_bgr565(frame_rd_data_hold[111:96]) : 8'h00;
wire [7:0] alpha_hi_3 = preproc_en ? preproc_alpha_from_bgr565(frame_rd_data_hold[127:112]) : 8'h00;

// Reserve Pixel[0,0].A as frame watermark for future sideband lockstep.
wire first_pixel_word = dma_session_active && (dma_rd_word_count == 18'd0) && (dma_expand_phase == 1'b0);
wire [7:0] alpha_lo_0 = (preproc_en && first_pixel_word) ? frame_id : alpha_lo_0_base;

wire [127:0] frame_rd_data_bgrx_lo = pack_4pix_bgrx(
    frame_rd_data[15:0], frame_rd_data[31:16], frame_rd_data[47:32], frame_rd_data[63:48],
    alpha_lo_0, alpha_lo_1, alpha_lo_2, alpha_lo_3);
wire [127:0] frame_rd_hold_bgrx_hi = pack_4pix_bgrx(
    frame_rd_data_hold[79:64], frame_rd_data_hold[95:80], frame_rd_data_hold[111:96], frame_rd_data_hold[127:112],
    alpha_hi_0, alpha_hi_1, alpha_hi_2, alpha_hi_3);
wire [127:0] post_ddr_pattern_data_565 = {8{post_ddr_color_data}};
wire [127:0] post_ddr_pattern_data_bgrx = {4{bgr565_to_bgrx32(post_ddr_color_data, preproc_en ? 8'h80 : 8'h00)}};
wire [127:0] post_ddr_pattern_data = dma_expand_mode ? post_ddr_pattern_data_bgrx : post_ddr_pattern_data_565;
wire [127:0] frame_dma_data = dma_expand_mode
    ? (dma_expand_phase ? frame_rd_hold_bgrx_hi : frame_rd_data_bgrx_lo)
    : frame_rd_data;
wire        frame_stream_ready = ~dma_session_active | ~mwr_first_beat_seen | frame_rd_data_ready;

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
        rd_fsync_stretch_cnt <= 6'd0;
        dma_expand_phase <= 1'b0;
        mwr_first_beat_seen <= 1'b0;
        frame_rd_data_hold <= 128'd0;
        frame_id <= 8'd0;
    end else if (frame_done_pulse) begin
        dma_session_active <= 1'b0;
        dma_rd_word_count <= 18'd0;
        rd_fsync_stretch_cnt <= 6'd0;
        dma_expand_phase <= 1'b0;
        mwr_first_beat_seen <= 1'b0;
    end else begin
        if (dma_session_start) begin
            dma_session_active <= 1'b1;
            dma_rd_word_count <= 18'd0;
            rd_fsync_stretch_cnt <= 6'd31;
            dma_expand_phase <= 1'b0;
            mwr_first_beat_seen <= 1'b0;
            frame_id <= frame_id + 8'd1;
        end else begin
            if (dma_session_active && bar2_addr_step)
                mwr_first_beat_seen <= 1'b1;

            if (dma_session_active && bar2_addr_step) begin
                if (dma_rd_word_count == frame_words_cfg - 1'b1) begin
                    dma_rd_word_count <= 18'd0;
                    dma_session_active <= 1'b0;
                end else begin
                    dma_rd_word_count <= dma_rd_word_count + 18'd1;
                end
            end

            if (dma_expand_mode && dma_session_active && bar2_addr_step)
                dma_expand_phase <= ~dma_expand_phase;

            if (frame_rd_fetch_en)
                frame_rd_data_hold <= frame_rd_data;

            if (rd_fsync_stretch_cnt != 6'd0)
                rd_fsync_stretch_cnt <= rd_fsync_stretch_cnt - 6'd1;
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

// Frame Buffer: Camera RGB565 → DDR3 → Read out
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
    .rd_en              (frame_rd_fetch_en),
    .vout_de            (),
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
