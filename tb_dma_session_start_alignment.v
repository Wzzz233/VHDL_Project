`timescale 1ns / 1ps

module tb_dma_session_start_alignment;

reg         clk;
reg         rst_n;
reg         bar1_wr_en;
reg  [11:0] bar1_wr_addr;
reg  [127:0] bar1_wr_data;
reg  [15:0] bar1_wr_byte_en;
reg         gen_tlp_start;

wire        user_define_data_flag;
wire        o_mwr32_req;
wire        o_mwr64_req;
wire        o_mrd32_req;
wire        o_mrd64_req;
wire [9:0]  req_length;
wire [63:0] req_addr;
wire [31:0] req_data;
wire        mwr64_req_ack;
wire        mwr_tx_busy;
wire        o_tx_restart;
wire        o_frame_done_pulse;
wire        o_mwr_req_start_pulse;
wire        axis_slave2_tvld;
wire [127:0] axis_slave2_tdata;
wire        axis_slave2_tlast;
wire        axis_slave2_tuser;

integer cycle;
integer tx_restart_cycle;
integer req_cycle;
integer req_start_cycle;
integer busy_rise_cycle;
integer timeout_cycles;
reg     mwr_tx_busy_d;
wire    tx_done = axis_slave2_tvld && axis_slave2_tlast;
reg     frame_done_seen;

// Backpressure: deassert ready for 4 cycles after mwr_tx_busy rises.
reg        axis_trdy_bp;
reg  [3:0] trdy_delay_cnt;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        axis_trdy_bp   <= 1'b1;
        trdy_delay_cnt <= 4'd0;
    end else if (mwr_tx_busy && !mwr_tx_busy_d) begin
        axis_trdy_bp   <= 1'b0;
        trdy_delay_cnt <= 4'd4;
    end else if (trdy_delay_cnt != 4'd0) begin
        trdy_delay_cnt <= trdy_delay_cnt - 4'd1;
        if (trdy_delay_cnt == 4'd1)
            axis_trdy_bp <= 1'b1;
    end
end

task bar1_write;
    input [11:0] addr;
    input [31:0] data32;
begin
    @(posedge clk);
    bar1_wr_en <= 1'b1;
    bar1_wr_addr <= addr;
    bar1_wr_byte_en <= 16'h000F;
    bar1_wr_data <= {96'd0, data32};
    @(posedge clk);
    bar1_wr_en <= 1'b0;
    bar1_wr_addr <= 12'd0;
    bar1_wr_byte_en <= 16'd0;
    bar1_wr_data <= 128'd0;
end
endtask

ips2l_pcie_dma_controller #(
    .DEVICE_TYPE(3'd0),
    .ADDR_WIDTH(4'd12)
) controller (
    .clk                    (clk),
    .rst_n                  (rst_n),
    .i_bar1_wr_en           (bar1_wr_en),
    .i_bar1_wr_addr         (bar1_wr_addr),
    .i_bar1_wr_data         (bar1_wr_data),
    .i_bar1_wr_byte_en      (bar1_wr_byte_en),
    .i_apb_psel             (1'b0),
    .i_apb_paddr            (9'd0),
    .i_apb_pwdata           (32'd0),
    .i_apb_pstrb            (4'd0),
    .i_apb_pwrite           (1'b0),
    .i_apb_penable          (1'b0),
    .o_apb_prdy             (),
    .o_apb_prdata           (),
    .o_user_define_data_flag(user_define_data_flag),
    .o_mwr32_req            (o_mwr32_req),
    .i_mwr32_req_ack        (1'b0),
    .o_mwr64_req            (o_mwr64_req),
    .i_mwr64_req_ack        (mwr64_req_ack),
    .o_mrd32_req            (o_mrd32_req),
    .i_mrd32_req_ack        (1'b0),
    .o_mrd64_req            (o_mrd64_req),
    .i_mrd64_req_ack        (1'b0),
    .o_req_length           (req_length),
    .o_req_addr             (req_addr),
    .o_req_data             (req_data),
    .i_mwr_tx_busy          (mwr_tx_busy),
    .i_dma_check_result     (64'd0),
    .o_tx_restart           (o_tx_restart),
    .o_cross_4kb_boundary   (),
    .o_frame_done_pulse     (o_frame_done_pulse),
    .o_prep_ctrl            (),
    .o_prep_clahe           (),
    .o_prep_usm             (),
    .o_prep_med             (),
    .o_roi_x1y1             (),
    .o_roi_x2y2             (),
    .o_roi_ctrl             ()
);

ips2l_pcie_dma_mwr_tx_ctrl #(
    .DEVICE_TYPE(3'd0)
) mwr_tx (
    .clk                    (clk),
    .rst_n                  (rst_n),
    .i_cfg_pbus_num         (8'h00),
    .i_cfg_pbus_dev_num     (5'h00),
    .i_cfg_max_payload_size (3'd0),
    .i_user_define_data_flag(user_define_data_flag),
    .i_mwr32_req            (o_mwr32_req),
    .o_mwr32_req_ack        (),
    .i_mwr64_req            (o_mwr64_req),
    .o_mwr64_req_ack        (mwr64_req_ack),
    .i_req_length           (req_length),
    .i_req_addr             (req_addr),
    .i_req_data             (req_data),
    .o_rd_en                (),
    .o_rd_length            (),
    .i_gen_tlp_start        (gen_tlp_start),
    .i_rd_data              (128'h0123_4567_89ab_cdef_fedc_ba98_7654_3210),
    .i_last_data            (1'b1),
    .i_axis_slave2_trdy     (axis_trdy_bp),
    .o_axis_slave2_tvld     (axis_slave2_tvld),
    .o_axis_slave2_tdata    (axis_slave2_tdata),
    .o_axis_slave2_tlast    (axis_slave2_tlast),
    .o_axis_slave2_tuser    (axis_slave2_tuser),
    .o_mwr_tx_busy          (mwr_tx_busy),
    .o_mwr_tx_hold          (),
    .o_mwr_tlp_tx           (),
    .o_mwr_req_start_pulse  (o_mwr_req_start_pulse),
    .i_tx_restart           (o_tx_restart)
);

always #5 clk = ~clk;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        gen_tlp_start <= 1'b0;
    end else if (tx_done) begin
        gen_tlp_start <= 1'b0;
    end else if (o_mwr_req_start_pulse) begin
        gen_tlp_start <= 1'b1;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        cycle <= 0;
        tx_restart_cycle <= -1;
        req_cycle <= -1;
        req_start_cycle <= -1;
        busy_rise_cycle <= -1;
        mwr_tx_busy_d <= 1'b0;
    end else begin
        cycle <= cycle + 1;
        mwr_tx_busy_d <= mwr_tx_busy;

        if (o_tx_restart && tx_restart_cycle < 0)
            tx_restart_cycle <= cycle;
        if (o_mwr64_req && req_cycle < 0)
            req_cycle <= cycle;
        if (o_mwr_req_start_pulse && req_start_cycle < 0)
            req_start_cycle <= cycle;
        if (mwr_tx_busy && !mwr_tx_busy_d && busy_rise_cycle < 0)
            busy_rise_cycle <= cycle;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        frame_done_seen <= 1'b0;
    else if (o_frame_done_pulse)
        frame_done_seen <= 1'b1;
end

initial begin
    clk = 1'b0;
    rst_n = 1'b0;
    bar1_wr_en = 1'b0;
    bar1_wr_addr = 12'd0;
    bar1_wr_data = 128'd0;
    bar1_wr_byte_en = 16'd0;
    gen_tlp_start = 1'b0;

    #30;
    rst_n = 1'b1;

    // Match the Linux driver write order: BAR1+0x120 -> BAR1+0x110 -> BAR1+0x100.
    // Use one 128-bit beat so the TX path can complete with i_last_data=1.
    bar1_write(12'h120, 32'h0000_0000);
    bar1_write(12'h110, 32'h7EB0_0000);
    bar1_write(12'h100, 32'h8000_0004);

    timeout_cycles = 0;
    while (!frame_done_seen && timeout_cycles < 200) begin
        @(posedge clk);
        timeout_cycles = timeout_cycles + 1;
    end
    if (!frame_done_seen) begin
        $display("ERROR: timeout waiting for frame_done_pulse under startup backpressure");
        $fatal;
    end
    @(posedge clk);

    $display("TRACE tx_restart_cycle=%0d req_cycle=%0d req_start_cycle=%0d busy_rise_cycle=%0d req_addr=%h req_len=%0d",
             tx_restart_cycle, req_cycle, req_start_cycle, busy_rise_cycle, req_addr, req_length);

    if (tx_restart_cycle < 0 || req_cycle < 0 || req_start_cycle < 0 || busy_rise_cycle < 0) begin
        $display("ERROR: failed to observe command pulse / request / accept / busy ordering");
        $fatal;
    end

    if (!(tx_restart_cycle < req_cycle && req_cycle <= req_start_cycle && req_start_cycle <= busy_rise_cycle)) begin
        $display("ERROR: expected BAR1+0x110 pulse before MWR request, and MWR accept before tx busy rise");
        $fatal;
    end

    if (req_length !== 10'd4) begin
        $display("ERROR: expected a single-beat 4-DW request, got %0d", req_length);
        $fatal;
    end

    $display("PASS: controller/TX startup ordering survived post-busy backpressure without stalling frame completion.");
    $finish;
end

endmodule
