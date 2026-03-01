`timescale 1ns / 1ps

module median3x3 (
    input               clk,
    input               rst_n,
    input               i_vld,
    input       [7:0]   i_pix,
    output  reg [7:0]   o_pix
);
    reg [7:0] d0;
    reg [7:0] d1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            d0 <= 8'd0;
            d1 <= 8'd0;
            o_pix <= 8'd0;
        end else if (i_vld) begin
            d1 <= d0;
            d0 <= i_pix;
            /* Lightweight denoise proxy: 1x3 median-like approximation. */
            if ((i_pix >= d0 && i_pix <= d1) || (i_pix <= d0 && i_pix >= d1))
                o_pix <= i_pix;
            else if ((d0 >= i_pix && d0 <= d1) || (d0 <= i_pix && d0 >= d1))
                o_pix <= d0;
            else
                o_pix <= d1;
        end
    end
endmodule
