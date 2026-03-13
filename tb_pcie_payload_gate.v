`timescale 1ns / 1ps

module pcie_payload_gate_case #(
    parameter LEGACY_GATE = 1'b0
) (
    input  wire clk,
    input  wire rst_n,
    output reg  done,
    output reg  pass
);

localparam integer TIMEOUT_CYCLES = 120;

reg         mwr_req;
reg         frame_stream_ready;
reg  [3:0]  ready_delay;
reg         header_seen;
reg         payload_done_seen;
integer     cycle_count;

wire        axis_slave2_tready;
wire        axis_slave2_tvld;
wire [127:0] axis_slave2_tdata;
wire        axis_slave2_tlast;
wire        axis_slave2_tuser;
wire        mwr_req_ack;
wire        mwr_tx_busy;
wire        mwr_tx_hold;
wire        mwr_payload_active;
wire        mwr_payload_fire;
wire        gen_tlp_start;

function [23:0] rgb24_from_bgr565;
    input [15:0] pix565;
    reg [7:0] b8;
    reg [7:0] g8;
    reg [7:0] r8;
begin
    b8 = {pix565[15:11], pix565[15:13]};
    g8 = {pix565[10:5],  pix565[10:9]};
    r8 = {pix565[4:0],   pix565[4:2]};
    rgb24_from_bgr565 = {r8, g8, b8};
end
endfunction

function [31:0] rgb24_to_bgrx32;
    input [23:0] rgb24;
    input [7:0]  alpha8;
begin
    rgb24_to_bgrx32 = {alpha8, rgb24[7:0], rgb24[15:8], rgb24[23:16]};
end
endfunction

function [31:0] bgr565_to_bgrx32;
    input [15:0] pix565;
    input [7:0]  alpha8;
begin
    bgr565_to_bgrx32 = rgb24_to_bgrx32(rgb24_from_bgr565(pix565), alpha8);
end
endfunction

function [127:0] pack_4pix_bgrx;
    input [15:0] p0;
    input [15:0] p1;
    input [15:0] p2;
    input [15:0] p3;
begin
    pack_4pix_bgrx = {
        bgr565_to_bgrx32(p3, 8'h00),
        bgr565_to_bgrx32(p2, 8'h00),
        bgr565_to_bgrx32(p1, 8'h00),
        bgr565_to_bgrx32(p0, 8'h00)
    };
end
endfunction

function [15:0] pixel_word;
    input integer idx;
begin
    pixel_word = 16'h1200 + idx[15:0];
end
endfunction

function [127:0] endian_convert;
    input [127:0] data_in;
begin
    endian_convert[32*0+31:32*0+0] = {data_in[32*0+7:32*0+0], data_in[32*0+15:32*0+8], data_in[32*0+23:32*0+16], data_in[32*0+31:32*0+24]};
    endian_convert[32*1+31:32*1+0] = {data_in[32*1+7:32*1+0], data_in[32*1+15:32*1+8], data_in[32*1+23:32*1+16], data_in[32*1+31:32*1+24]};
    endian_convert[32*2+31:32*2+0] = {data_in[32*2+7:32*2+0], data_in[32*2+15:32*2+8], data_in[32*2+23:32*2+16], data_in[32*2+31:32*2+24]};
    endian_convert[32*3+31:32*3+0] = {data_in[32*3+7:32*3+0], data_in[32*3+15:32*3+8], data_in[32*3+23:32*3+16], data_in[32*3+31:32*3+24]};
end
endfunction

assign axis_slave2_tready = LEGACY_GATE
    ? frame_stream_ready
    : (~mwr_payload_active | frame_stream_ready);
assign gen_tlp_start = mwr_tx_busy;
assign mwr_payload_fire = axis_slave2_tvld && axis_slave2_tready && mwr_payload_active;

ips2l_pcie_dma_mwr_tx_ctrl #(
    .DEVICE_TYPE(3'd0)
) dut (
    .clk                    (clk),
    .rst_n                  (rst_n),
    .i_cfg_pbus_num         (8'h00),
    .i_cfg_pbus_dev_num     (5'h00),
    .i_cfg_max_payload_size (3'd0),
    .i_user_define_data_flag(1'b0),
    .i_mwr32_req            (mwr_req),
    .o_mwr32_req_ack        (mwr_req_ack),
    .i_mwr64_req            (1'b0),
    .o_mwr64_req_ack        (),
    .i_req_length           (10'd4),
    .i_req_addr             (64'd0),
    .i_req_data             (32'd0),
    .o_rd_en                (),
    .o_rd_length            (),
    .i_gen_tlp_start        (gen_tlp_start),
    .i_rd_data              (endian_convert(pack_4pix_bgrx(
                                pixel_word(0),
                                pixel_word(1),
                                pixel_word(2),
                                pixel_word(3)))),
    .i_last_data            (1'b1),
    .i_axis_slave2_trdy     (axis_slave2_tready),
    .o_axis_slave2_tvld     (axis_slave2_tvld),
    .o_axis_slave2_tdata    (axis_slave2_tdata),
    .o_axis_slave2_tlast    (axis_slave2_tlast),
    .o_axis_slave2_tuser    (axis_slave2_tuser),
    .o_mwr_tx_busy          (mwr_tx_busy),
    .o_mwr_tx_hold          (mwr_tx_hold),
    .o_mwr_tlp_tx           (mwr_payload_active),
    .o_mwr_req_start_pulse  (),
    .i_tx_restart           (1'b0)
);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        mwr_req <= 1'b0;
        frame_stream_ready <= 1'b0;
        ready_delay <= 4'd0;
        header_seen <= 1'b0;
        payload_done_seen <= 1'b0;
        cycle_count <= 0;
        done <= 1'b0;
        pass <= 1'b0;
    end else begin
        cycle_count <= cycle_count + 1;

        if (!mwr_req_ack && !mwr_tx_busy)
            mwr_req <= 1'b1;
        else
            mwr_req <= 1'b0;

        if ((dut.state == 2'd1) && axis_slave2_tready)
            header_seen <= 1'b1;

        if (!frame_stream_ready && header_seen) begin
            if (ready_delay == 4'd0)
                ready_delay <= 4'd3;
            else if (ready_delay == 4'd1)
                frame_stream_ready <= 1'b1;
            else
                ready_delay <= ready_delay - 4'd1;
        end

        if (dut.tx_done)
            payload_done_seen <= 1'b1;

        if (!done) begin
            if (!LEGACY_GATE && header_seen && payload_done_seen && !mwr_tx_busy) begin
                done <= 1'b1;
                pass <= 1'b1;
            end else if (LEGACY_GATE && (cycle_count == TIMEOUT_CYCLES)) begin
                done <= 1'b1;
                pass <= !header_seen && !payload_done_seen && !mwr_payload_fire;
            end else if (!LEGACY_GATE && (cycle_count == TIMEOUT_CYCLES)) begin
                done <= 1'b1;
                pass <= 1'b0;
            end
        end
    end
end

endmodule

module tb_pcie_payload_gate;

reg clk;
reg rst_n;

wire done_legacy;
wire pass_legacy;
wire done_fixed;
wire pass_fixed;

pcie_payload_gate_case #(.LEGACY_GATE(1'b1)) u_legacy (
    .clk(clk),
    .rst_n(rst_n),
    .done(done_legacy),
    .pass(pass_legacy)
);

pcie_payload_gate_case #(.LEGACY_GATE(1'b0)) u_fixed (
    .clk(clk),
    .rst_n(rst_n),
    .done(done_fixed),
    .pass(pass_fixed)
);

always #5 clk = ~clk;

initial begin
    clk = 1'b0;
    rst_n = 1'b0;

    #40;
    rst_n = 1'b1;

    wait (done_legacy && done_fixed);
    #20;

    if (!pass_legacy) begin
        $display("ERROR: legacy gate case did not reproduce startup deadlock");
        $fatal;
    end

    if (!pass_fixed) begin
        $display("ERROR: payload-only gate case did not complete payload correctly");
        $fatal;
    end

    $display("PASS: full ready gate blocks header startup; payload-only gate launches header and completes all payload beats.");
    $finish;
end

endmodule
