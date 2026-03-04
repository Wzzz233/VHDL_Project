`timescale 1ns / 1ps

/*
 * CLAHE-lite approximation for streaming FPGA OCR pre-enhancement.
 * This block is intentionally lightweight: it applies a bounded local
 * contrast expansion around mid-gray controlled by strength and clip.
 */
module clahe_lite (
    input       [7:0]   i_y,
    input               i_en,
    input       [7:0]   i_strength,
    input       [7:0]   i_clip,
    output  reg [7:0]   o_y
);
    integer delta;
    integer gain_q8;
    integer y_tmp;
    integer y_min;
    integer y_max;

    always @(*) begin
        if (!i_en) begin
            o_y = i_y;
        end else begin
            delta = $signed({1'b0, i_y}) - 128;
            gain_q8 = 256 + i_strength;
            y_tmp = 128 + ((delta * gain_q8) >>> 8);
            y_min = $signed({1'b0, i_y}) - i_clip;
            y_max = $signed({1'b0, i_y}) + i_clip;
            if (y_tmp < y_min)
                y_tmp = y_min;
            if (y_tmp > y_max)
                y_tmp = y_max;
            if (y_tmp < 0)
                o_y = 8'd0;
            else if (y_tmp > 255)
                o_y = 8'd255;
            else
                o_y = y_tmp[7:0];
        end
    end
endmodule

