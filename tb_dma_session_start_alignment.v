`timescale 1ns / 1ps

module tb_dma_session_start_alignment;

reg         clk;
reg         rst_n;
reg         bar1_wr_en;
reg  [11:0] bar1_wr_addr;
reg  [127:0] bar1_wr_data;
reg  [15:0] bar1_wr_byte_en;
reg         mwr64_req_ack;
reg         mwr_tx_busy;
reg         mwr32_req_ack;
reg         mrd32_req_ack;
reg         mrd64_req_ack;

wire        o_mwr64_req;
wire        o_tx_restart;
wire        o_frame_req_ack_pulse;
wire        o_frame_done_pulse;

integer cycle;
integer tx_restart_cycle;
integer req_cycle;
integer req_ack_cycle;
integer busy_rise_cycle;
reg     mwr_tx_busy_d;

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
) dut (
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
    .o_user_define_data_flag(),
    .o_mwr32_req            (),
    .i_mwr32_req_ack        (mwr32_req_ack),
    .o_mwr64_req            (o_mwr64_req),
    .i_mwr64_req_ack        (mwr64_req_ack),
    .o_mrd32_req            (),
    .i_mrd32_req_ack        (mrd32_req_ack),
    .o_mrd64_req            (),
    .i_mrd64_req_ack        (mrd64_req_ack),
    .o_req_length           (),
    .o_req_addr             (),
    .o_req_data             (),
    .i_mwr_tx_busy          (mwr_tx_busy),
    .i_dma_check_result     (64'd0),
    .o_tx_restart           (o_tx_restart),
    .o_frame_req_ack_pulse  (o_frame_req_ack_pulse),
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

always #5 clk = ~clk;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        cycle <= 0;
        tx_restart_cycle <= -1;
        req_cycle <= -1;
        req_ack_cycle <= -1;
        busy_rise_cycle <= -1;
        mwr_tx_busy_d <= 1'b0;
    end else begin
        cycle <= cycle + 1;
        mwr_tx_busy_d <= mwr_tx_busy;

        if (o_tx_restart && tx_restart_cycle < 0)
            tx_restart_cycle <= cycle;
        if (o_mwr64_req && req_cycle < 0)
            req_cycle <= cycle;
        if (o_frame_req_ack_pulse && req_ack_cycle < 0)
            req_ack_cycle <= cycle;
        if (mwr_tx_busy && !mwr_tx_busy_d && busy_rise_cycle < 0)
            busy_rise_cycle <= cycle;
    end
end

initial begin
    clk = 1'b0;
    rst_n = 1'b0;
    bar1_wr_en = 1'b0;
    bar1_wr_addr = 12'd0;
    bar1_wr_data = 128'd0;
    bar1_wr_byte_en = 16'd0;
    mwr64_req_ack = 1'b0;
    mwr_tx_busy = 1'b0;
    mwr32_req_ack = 1'b0;
    mrd32_req_ack = 1'b0;
    mrd64_req_ack = 1'b0;

    #30;
    rst_n = 1'b1;

    // Match the Linux driver write order: BAR1+0x120 -> BAR1+0x110 -> BAR1+0x100.
    bar1_write(12'h120, 32'h0000_0000);
    bar1_write(12'h110, 32'h7EB0_0000);
    bar1_write(12'h100, 32'h8000_0010); // frame mode + 16 DWORDs

    wait (o_mwr64_req);
    @(posedge clk);
    mwr64_req_ack <= 1'b1;
    @(posedge clk);
    mwr64_req_ack <= 1'b0;

    @(posedge clk);
    mwr_tx_busy <= 1'b1;
    repeat (4) @(posedge clk);
    mwr_tx_busy <= 1'b0;

    wait (o_frame_done_pulse);
    @(posedge clk);

    $display("TRACE tx_restart_cycle=%0d req_cycle=%0d req_ack_cycle=%0d busy_rise_cycle=%0d",
             tx_restart_cycle, req_cycle, req_ack_cycle, busy_rise_cycle);

    if (tx_restart_cycle < 0 || req_cycle < 0 || req_ack_cycle < 0 || busy_rise_cycle < 0) begin
        $display("ERROR: failed to observe controller write/start ordering");
        $fatal;
    end

    if (!(tx_restart_cycle < req_cycle && req_cycle <= req_ack_cycle && req_ack_cycle < busy_rise_cycle)) begin
        $display("ERROR: expected BAR1 L_ADDR pulse before MWR request, request-ack pulse, and tx busy rise");
        $fatal;
    end

    $display("PASS: o_tx_restart follows BAR1+0x110, frame request accept happens later, and MWR busy rises after that.");
    $finish;
end

endmodule
