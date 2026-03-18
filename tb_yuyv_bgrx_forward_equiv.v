`timescale 1ns / 1ps

module tb_yuyv_bgrx_forward_equiv;

localparam integer WORDS_PER_LINE = 16;
localparam integer LINE_COUNT = 4;
localparam integer TOTAL_WORDS = WORDS_PER_LINE * LINE_COUNT;

reg  [15:0] src_words [0:TOTAL_WORDS-1];
reg  [31:0] out_pixels [0:TOTAL_WORDS-1];
integer out_count;
integer mismatch_count;
integer word_idx;
integer line_idx;
integer seed;

reg        pair_half;
reg [15:0] pair_word0;
reg        pending_valid;
reg [31:0] pending_pixel;

function [7:0] clip_to_u8_s11;
    input signed [10:0] value;
begin
    if (value < 11'sd0)
        clip_to_u8_s11 = 8'd0;
    else if (value > 11'sd255)
        clip_to_u8_s11 = 8'hFF;
    else
        clip_to_u8_s11 = value[7:0];
end
endfunction

function [63:0] pack_2pix_yuv_to_bgrx;
    input [7:0] y0;
    input [7:0] y1;
    input [7:0] u8;
    input [7:0] v8;
    input [7:0] a0;
    input [7:0] a1;
    reg signed [8:0]  du_s;
    reg signed [8:0]  dv_s;
    reg signed [18:0] rv_mul;
    reg signed [18:0] bu_mul;
    reg signed [18:0] gu_mul;
    reg signed [18:0] gv_mul;
    reg signed [18:0] g_uv_mul;
    reg signed [10:0] rv_term;
    reg signed [10:0] bu_term;
    reg signed [10:0] g_uv_term;
    reg signed [10:0] y0_s;
    reg signed [10:0] y1_s;
    reg signed [10:0] r0_calc;
    reg signed [10:0] g0_calc;
    reg signed [10:0] b0_calc;
    reg signed [10:0] r1_calc;
    reg signed [10:0] g1_calc;
    reg signed [10:0] b1_calc;
begin
    du_s = $signed({1'b0, u8}) - 9'sd128;
    dv_s = $signed({1'b0, v8}) - 9'sd128;

    rv_mul = 10'sd359 * dv_s;
    bu_mul = 10'sd454 * du_s;
    gu_mul = 8'sd88 * du_s;
    gv_mul = 9'sd183 * dv_s;
    g_uv_mul = gu_mul + gv_mul;

    rv_term = rv_mul >>> 8;
    bu_term = bu_mul >>> 8;
    g_uv_term = g_uv_mul >>> 8;

    y0_s = $signed({3'b000, y0});
    y1_s = $signed({3'b000, y1});

    r0_calc = y0_s + rv_term;
    g0_calc = y0_s - g_uv_term;
    b0_calc = y0_s + bu_term;

    r1_calc = y1_s + rv_term;
    g1_calc = y1_s - g_uv_term;
    b1_calc = y1_s + bu_term;

    pack_2pix_yuv_to_bgrx = {
        a1, clip_to_u8_s11(r1_calc), clip_to_u8_s11(g1_calc), clip_to_u8_s11(b1_calc),
        a0, clip_to_u8_s11(r0_calc), clip_to_u8_s11(g0_calc), clip_to_u8_s11(b0_calc)
    };
end
endfunction

function [63:0] pack_2pix_yuv_words_to_bgrx;
    input [15:0] w0;
    input [15:0] w1;
    input        order_uyvy;
    input [7:0]  a0;
    input [7:0]  a1;
    reg [7:0] y0;
    reg [7:0] y1;
    reg [7:0] u0;
    reg [7:0] v0;
begin
    if (!order_uyvy) begin
        y0 = w0[15:8];
        u0 = w0[7:0];
        y1 = w1[15:8];
        v0 = w1[7:0];
    end else begin
        u0 = w0[15:8];
        y0 = w0[7:0];
        v0 = w1[15:8];
        y1 = w1[7:0];
    end
    pack_2pix_yuv_words_to_bgrx = pack_2pix_yuv_to_bgrx(y0, y1, u0, v0, a0, a1);
end
endfunction

function [127:0] pack_4pix_yuyv_to_bgrx;
    input [15:0] w0;
    input [15:0] w1;
    input [15:0] w2;
    input [15:0] w3;
    input        order_uyvy;
    input [7:0]  a0;
    input [7:0]  a1;
    input [7:0]  a2;
    input [7:0]  a3;
    reg [7:0] y0;
    reg [7:0] y1;
    reg [7:0] y2;
    reg [7:0] y3;
    reg [7:0] u0;
    reg [7:0] v0;
    reg [7:0] u1;
    reg [7:0] v1;
    reg [63:0] pix01;
    reg [63:0] pix23;
begin
    if (!order_uyvy) begin
        y0 = w0[15:8];
        u0 = w0[7:0];
        y1 = w1[15:8];
        v0 = w1[7:0];
        y2 = w2[15:8];
        u1 = w2[7:0];
        y3 = w3[15:8];
        v1 = w3[7:0];
    end else begin
        u0 = w0[15:8];
        y0 = w0[7:0];
        v0 = w1[15:8];
        y1 = w1[7:0];
        u1 = w2[15:8];
        y2 = w2[7:0];
        v1 = w3[15:8];
        y3 = w3[7:0];
    end

    pix01 = pack_2pix_yuv_to_bgrx(y0, y1, u0, v0, a0, a1);
    pix23 = pack_2pix_yuv_to_bgrx(y2, y3, u1, v1, a2, a3);
    pack_4pix_yuyv_to_bgrx = {pix23, pix01};
end
endfunction

task automatic emit_pixel;
    input [31:0] pix;
begin
    if (out_count >= TOTAL_WORDS) begin
        $display("ERROR: output pixel overflow");
        $fatal;
    end
    out_pixels[out_count] = pix;
    out_count = out_count + 1;
end
endtask

task automatic feed_word;
    input [15:0] w;
    input        order_uyvy;
    input        line_start;
    input        frame_start;
    reg [63:0] pix01;
begin
    if (frame_start || line_start)
        pair_half = 1'b0;

    if (pending_valid) begin
        emit_pixel(pending_pixel);
        pending_valid = 1'b0;
    end

    if (!pair_half) begin
        pair_word0 = w;
        pair_half = 1'b1;
    end else begin
        pix01 = pack_2pix_yuv_words_to_bgrx(pair_word0, w, order_uyvy, 8'h00, 8'h00);
        emit_pixel(pix01[31:0]);
        pending_pixel = pix01[63:32];
        pending_valid = 1'b1;
        pair_half = 1'b0;
    end
end
endtask

task automatic flush_line;
begin
    if (pending_valid) begin
        emit_pixel(pending_pixel);
        pending_valid = 1'b0;
    end
    pair_half = 1'b0;
end
endtask

task automatic run_case;
    input order_uyvy;
    reg [127:0] ref_beat;
    reg [127:0] out_beat;
    integer line;
    integer col;
    integer idx;
begin
    out_count = 0;
    mismatch_count = 0;
    pair_half = 1'b0;
    pair_word0 = 16'd0;
    pending_valid = 1'b0;
    pending_pixel = 32'd0;

    for (line = 0; line < LINE_COUNT; line = line + 1) begin
        for (col = 0; col < WORDS_PER_LINE; col = col + 1) begin
            idx = line * WORDS_PER_LINE + col;
            feed_word(src_words[idx], order_uyvy, (col == 0), (line == 0 && col == 0));
        end
        flush_line();
    end

    if (out_count != TOTAL_WORDS) begin
        $display("ERROR: output pixel count mismatch, got=%0d expect=%0d", out_count, TOTAL_WORDS);
        $fatal;
    end

    for (idx = 0; idx < TOTAL_WORDS; idx = idx + 4) begin
        ref_beat = pack_4pix_yuyv_to_bgrx(
            src_words[idx + 0], src_words[idx + 1], src_words[idx + 2], src_words[idx + 3],
            order_uyvy, 8'h00, 8'h00, 8'h00, 8'h00
        );
        out_beat = {out_pixels[idx + 3], out_pixels[idx + 2], out_pixels[idx + 1], out_pixels[idx + 0]};
        if (ref_beat !== out_beat) begin
            mismatch_count = mismatch_count + 1;
            $display("MISMATCH order=%0d beat=%0d ref=%h out=%h", order_uyvy, idx >> 2, ref_beat, out_beat);
        end
    end

    if (mismatch_count != 0) begin
        $display("ERROR: found %0d beat mismatches for order_uyvy=%0d", mismatch_count, order_uyvy);
        $fatal;
    end
end
endtask

initial begin
    if ((WORDS_PER_LINE % 2) != 0 || (WORDS_PER_LINE % 4) != 0) begin
        $display("ERROR: WORDS_PER_LINE must be even and 4-word aligned");
        $fatal;
    end

    seed = 32'h42AB_C001;
    for (word_idx = 0; word_idx < TOTAL_WORDS; word_idx = word_idx + 1)
        src_words[word_idx] = $random(seed);

    run_case(1'b0);
    run_case(1'b1);

    $display("PASS: camera-domain 2-word stream conversion matches legacy 4-word beat packing.");
    $finish;
end

endmodule

