`timescale 1ns / 1ps

module motion_diff #(
    parameter DS_W = 320,
    parameter DS_H = 180
) (
    input               clk,
    input               rst_n,
    input               i_vld,
    input               i_sof,
    input       [11:0]  i_x,
    input       [10:0]  i_y,
    input               i_bit,
    output  reg         o_motion
);
    localparam integer DS_PIXELS = DS_W * DS_H;
    localparam integer ADDR_W = 16;

    reg [DS_PIXELS-1:0] prev_map;
    wire ds_hit = i_vld && (i_x[1:0] == 2'b00) && (i_y[1:0] == 2'b00) &&
                  (i_x[11:2] < DS_W) && (i_y[10:2] < DS_H);
    wire [ADDR_W-1:0] ds_addr = i_y[10:2] * DS_W + i_x[11:2];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prev_map <= {DS_PIXELS{1'b0}};
            o_motion <= 1'b0;
        end else begin
            if (i_sof)
                o_motion <= 1'b0;

            if (ds_hit) begin
                o_motion <= prev_map[ds_addr] ^ i_bit;
                prev_map[ds_addr] <= i_bit;
            end else if (i_vld) begin
                o_motion <= 1'b0;
            end
        end
    end
endmodule
