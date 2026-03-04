`timescale 1ns / 1ps

/*
 * Thresholded USM-lite on luma proxy.
 * Uses cross-channel contrast as a cheap edge proxy to avoid line buffers.
 */
module usm_luma (
    input       [7:0]   i_r,
    input       [7:0]   i_g,
    input       [7:0]   i_b,
    input       [7:0]   i_y,
    input               i_en,
    input       [7:0]   i_gain_q4_4,
    input       [7:0]   i_thr,
    input       [7:0]   i_limit,
    output  reg [7:0]   o_y
);
    integer drg;
    integer dgb;
    integer edge;
    integer boost;
    integer y_tmp;

    always @(*) begin
        if (!i_en) begin
            o_y = i_y;
        end else begin
            drg = (i_r >= i_g) ? (i_r - i_g) : (i_g - i_r);
            dgb = (i_g >= i_b) ? (i_g - i_b) : (i_b - i_g);
            edge = (drg + dgb) >>> 1;
            y_tmp = i_y;

            if (edge >= i_thr) begin
                boost = ((edge - i_thr) * i_gain_q4_4) >>> 4;
                if (boost > i_limit)
                    boost = i_limit;
                y_tmp = y_tmp + boost;
            end

            if (y_tmp < 0)
                o_y = 8'd0;
            else if (y_tmp > 255)
                o_y = 8'd255;
            else
                o_y = y_tmp[7:0];
        end
    end
endmodule

