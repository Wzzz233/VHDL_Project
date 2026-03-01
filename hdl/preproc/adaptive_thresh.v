`timescale 1ns / 1ps

module adaptive_thresh (
    input               clk,
    input               rst_n,
    input               i_vld,
    input       [7:0]   i_pix,
    output  reg         o_flag,
    output  reg [7:0]   o_mean
);
    reg [15:0] mean_acc;
    wire [7:0] mean_now = mean_acc[15:8];
    wire [8:0] thr = {1'b0, mean_now} + 9'd8;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mean_acc <= 16'd0;
            o_flag <= 1'b0;
            o_mean <= 8'd0;
        end else if (i_vld) begin
            mean_acc <= mean_acc + {8'd0, i_pix} - {8'd0, mean_now};
            o_mean <= mean_now;
            o_flag <= ({1'b0, i_pix} > thr);
        end
    end
endmodule
