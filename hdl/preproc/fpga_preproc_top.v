`timescale 1ns / 1ps

module fpga_preproc_top (
    input               clk,
    input               rst_n,
    input               i_vld,
    input               i_sof,
    input       [11:0]  i_x,
    input       [10:0]  i_y,
    input       [7:0]   i_r,
    input       [7:0]   i_g,
    input       [7:0]   i_b,
    output  reg         o_valid,
    output  reg         o_edge,
    output  reg         o_thresh,
    output  reg         o_motion,
    output  reg [1:0]   o_color
);
    wire [7:0] y_in = (i_r >> 2) + (i_g >> 1) + (i_b >> 2);

    wire [7:0] y_med;
    wire [7:0] sobel_mag;
    wire       thr_flag;
    wire [7:0] thr_mean;
    wire [1:0] color_class;
    wire       motion_flag;

    median3x3 u_median (
        .clk    (clk),
        .rst_n  (rst_n),
        .i_vld  (i_vld),
        .i_pix  (y_in),
        .o_pix  (y_med)
    );

    sobel3x3 u_sobel (
        .clk    (clk),
        .rst_n  (rst_n),
        .i_vld  (i_vld),
        .i_pix  (y_med),
        .o_mag  (sobel_mag)
    );

    adaptive_thresh u_thr (
        .clk    (clk),
        .rst_n  (rst_n),
        .i_vld  (i_vld),
        .i_pix  (y_med),
        .o_flag (thr_flag),
        .o_mean (thr_mean)
    );

    color_mask u_color (
        .i_r            (i_r),
        .i_g            (i_g),
        .i_b            (i_b),
        .o_color_class  (color_class)
    );

    motion_diff #(
        .DS_W(320),
        .DS_H(180)
    ) u_motion (
        .clk        (clk),
        .rst_n      (rst_n),
        .i_vld      (i_vld),
        .i_sof      (i_sof),
        .i_x        (i_x),
        .i_y        (i_y),
        .i_bit      (thr_flag),
        .o_motion   (motion_flag)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_valid <= 1'b0;
            o_edge <= 1'b0;
            o_thresh <= 1'b0;
            o_motion <= 1'b0;
            o_color <= 2'b00;
        end else begin
            o_valid <= i_vld;
            o_edge <= (sobel_mag > 8'd24);
            o_thresh <= thr_flag;
            o_motion <= motion_flag;
            o_color <= color_class;
        end
    end
endmodule
