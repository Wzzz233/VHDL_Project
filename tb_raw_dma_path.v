`timescale 1ns / 1ps

module raw_dma_pending_case #(
    parameter USE_PENDING = 1'b1,
    parameter [2:0] RAW_BOOTSTRAP_WORDS = 3'd1
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       start_pulse,
    input  wire       frame_data_ready_allow,
    input  wire       payload_fire_allow,
    output reg        payload_fire,
    output reg  [7:0] payload_word_id,
    output reg [31:0] payload_count,
    output reg [31:0] mismatch_count,
    output reg [31:0] drop_count
);

reg        dma_session_active_r;
reg        dma_expand_phase_r;
reg        raw_req_pending_r;
reg [17:0] frame_src_req_count_r;
reg [2:0]  frame_src_bootstrap_count_r;
reg        raw_startup_active_r;
reg        raw_start_word_valid_r;
reg [7:0]  raw_start_word_id_r;
reg        req_pipe0_valid_r;
reg        req_pipe1_valid_r;
reg [7:0]  req_pipe0_word_r;
reg [7:0]  req_pipe1_word_r;
reg [7:0]  next_req_word_r;
reg        out_pair_active_valid_r;
reg        out_pair_next_valid_r;
reg [7:0]  out_pair_active_word_r;
reg [7:0]  out_pair_next_word_r;
reg [7:0]  expected_word_id_r;
reg        expected_hi_phase_r;

wire       frame_rd_data_valid = req_pipe1_valid_r;
wire [7:0] frame_rd_word_id = req_pipe1_word_r;
wire       raw_start_bootstrap_req = raw_startup_active_r &&
                                     (frame_src_bootstrap_count_r < RAW_BOOTSTRAP_WORDS);
wire       raw_stream_ready = raw_startup_active_r ? raw_start_word_valid_r : out_pair_active_valid_r;
wire       frame_output_step = dma_session_active_r && payload_fire_allow && raw_stream_ready;
wire       out_pair_pop = frame_output_step && dma_expand_phase_r;
wire       raw_capture_fire = dma_session_active_r && frame_rd_data_valid;
wire       raw_start_capture_now = raw_startup_active_r && raw_capture_fire && !raw_start_word_valid_r;
wire       raw_startup_use_word = raw_startup_active_r &&
                                  (raw_start_word_valid_r || raw_start_capture_now);
wire       raw_startup_done = raw_startup_use_word && out_pair_pop;
wire       raw_output_need = frame_output_step && !dma_expand_phase_r;
wire       raw_queue_has_room = !out_pair_active_valid_r || !out_pair_next_valid_r || out_pair_pop;
wire       raw_req_need = raw_start_bootstrap_req ||
                          (USE_PENDING && raw_req_pending_r) ||
                          raw_output_need;
wire       frame_rd_req_en_raw = dma_session_active_r &&
                                 frame_data_ready_allow &&
                                 raw_queue_has_room &&
                                 (frame_src_req_count_r < 18'd32) &&
                                 raw_req_need;
wire [7:0] raw_selected_word = raw_startup_use_word
    ? (raw_start_word_valid_r ? raw_start_word_id_r : frame_rd_word_id)
    : out_pair_active_word_r;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        dma_session_active_r <= 1'b0;
        dma_expand_phase_r <= 1'b0;
        raw_req_pending_r <= 1'b0;
        frame_src_req_count_r <= 18'd0;
        frame_src_bootstrap_count_r <= 3'd0;
        raw_startup_active_r <= 1'b0;
        raw_start_word_valid_r <= 1'b0;
        raw_start_word_id_r <= 8'd0;
        req_pipe0_valid_r <= 1'b0;
        req_pipe1_valid_r <= 1'b0;
        req_pipe0_word_r <= 8'd0;
        req_pipe1_word_r <= 8'd0;
        next_req_word_r <= 8'd0;
        out_pair_active_valid_r <= 1'b0;
        out_pair_next_valid_r <= 1'b0;
        out_pair_active_word_r <= 8'd0;
        out_pair_next_word_r <= 8'd0;
        expected_word_id_r <= 8'd0;
        expected_hi_phase_r <= 1'b0;
        payload_fire <= 1'b0;
        payload_word_id <= 8'd0;
        payload_count <= 32'd0;
        mismatch_count <= 32'd0;
        drop_count <= 32'd0;
    end else begin
        req_pipe1_valid_r <= req_pipe0_valid_r;
        req_pipe1_word_r <= req_pipe0_word_r;
        req_pipe0_valid_r <= 1'b0;
        payload_fire <= 1'b0;

        if (start_pulse && !dma_session_active_r) begin
            dma_session_active_r <= 1'b1;
            dma_expand_phase_r <= 1'b0;
            raw_req_pending_r <= 1'b0;
            frame_src_req_count_r <= 18'd0;
            frame_src_bootstrap_count_r <= 3'd0;
            raw_startup_active_r <= 1'b1;
            raw_start_word_valid_r <= 1'b0;
            raw_start_word_id_r <= 8'd0;
            req_pipe0_valid_r <= 1'b0;
            req_pipe1_valid_r <= 1'b0;
            req_pipe0_word_r <= 8'd0;
            req_pipe1_word_r <= 8'd0;
            next_req_word_r <= 8'd0;
            out_pair_active_valid_r <= 1'b0;
            out_pair_next_valid_r <= 1'b0;
            out_pair_active_word_r <= 8'd0;
            out_pair_next_word_r <= 8'd0;
            expected_word_id_r <= 8'd0;
            expected_hi_phase_r <= 1'b0;
            payload_count <= 32'd0;
            mismatch_count <= 32'd0;
            drop_count <= 32'd0;
        end else if (dma_session_active_r) begin
            if (USE_PENDING) begin
                if (frame_rd_req_en_raw)
                    raw_req_pending_r <= 1'b0;
                else if (raw_output_need)
                    raw_req_pending_r <= 1'b1;
            end

            if (frame_rd_req_en_raw) begin
                frame_src_req_count_r <= frame_src_req_count_r + 18'd1;
                if (frame_src_bootstrap_count_r != RAW_BOOTSTRAP_WORDS)
                    frame_src_bootstrap_count_r <= frame_src_bootstrap_count_r + 3'd1;
                req_pipe0_valid_r <= 1'b1;
                req_pipe0_word_r <= next_req_word_r;
                next_req_word_r <= next_req_word_r + 8'd1;
            end

            if (frame_output_step) begin
                payload_fire <= 1'b1;
                payload_word_id <= raw_selected_word;
                payload_count <= payload_count + 32'd1;

                if (raw_selected_word != expected_word_id_r)
                    mismatch_count <= mismatch_count + 32'd1;

                if (expected_hi_phase_r)
                    expected_word_id_r <= expected_word_id_r + 8'd1;
                expected_hi_phase_r <= ~expected_hi_phase_r;
                dma_expand_phase_r <= ~dma_expand_phase_r;
            end

            if (raw_startup_done)
                raw_startup_active_r <= 1'b0;

            if (out_pair_pop) begin
                if (out_pair_next_valid_r) begin
                    out_pair_active_valid_r <= 1'b1;
                    out_pair_active_word_r <= out_pair_next_word_r;
                    out_pair_next_valid_r <= 1'b0;
                end else begin
                    out_pair_active_valid_r <= 1'b0;
                end
            end

            if (raw_capture_fire && raw_startup_active_r && !raw_start_word_valid_r) begin
                raw_start_word_id_r <= frame_rd_word_id;
                raw_start_word_valid_r <= 1'b1;
            end

            if (raw_capture_fire) begin
                if (!out_pair_active_valid_r || (out_pair_pop && !out_pair_next_valid_r)) begin
                    out_pair_active_valid_r <= 1'b1;
                    out_pair_active_word_r <= frame_rd_word_id;
                end else if (!out_pair_next_valid_r || out_pair_pop) begin
                    out_pair_next_valid_r <= 1'b1;
                    out_pair_next_word_r <= frame_rd_word_id;
                end else begin
                    drop_count <= drop_count + 32'd1;
                end
            end
        end
    end
end

endmodule

module tb_raw_dma_path;

reg clk;
reg rst_n;
reg start_pulse;
reg frame_data_ready_allow;
reg payload_fire_allow;
integer cycle;

wire        bad_payload_fire;
wire [7:0]  bad_payload_word_id;
wire [31:0] bad_payload_count;
wire [31:0] bad_mismatch_count;
wire [31:0] bad_drop_count;

wire        good_payload_fire;
wire [7:0]  good_payload_word_id;
wire [31:0] good_payload_count;
wire [31:0] good_mismatch_count;
wire [31:0] good_drop_count;

raw_dma_pending_case #(
    .USE_PENDING(1'b0),
    .RAW_BOOTSTRAP_WORDS(3'd2)
) u_bad (
    .clk(clk),
    .rst_n(rst_n),
    .start_pulse(start_pulse),
    .frame_data_ready_allow(frame_data_ready_allow),
    .payload_fire_allow(payload_fire_allow),
    .payload_fire(bad_payload_fire),
    .payload_word_id(bad_payload_word_id),
    .payload_count(bad_payload_count),
    .mismatch_count(bad_mismatch_count),
    .drop_count(bad_drop_count)
);

raw_dma_pending_case #(
    .USE_PENDING(1'b1),
    .RAW_BOOTSTRAP_WORDS(3'd2)
) u_good (
    .clk(clk),
    .rst_n(rst_n),
    .start_pulse(start_pulse),
    .frame_data_ready_allow(frame_data_ready_allow),
    .payload_fire_allow(payload_fire_allow),
    .payload_fire(good_payload_fire),
    .payload_word_id(good_payload_word_id),
    .payload_count(good_payload_count),
    .mismatch_count(good_mismatch_count),
    .drop_count(good_drop_count)
);

always #5 clk = ~clk;

always @(posedge clk) begin
    if (rst_n) begin
        cycle = cycle + 1;
        if (bad_payload_fire)
            $display("C%0d BAD  beat word=%0d count=%0d mismatch=%0d drops=%0d",
                     cycle, bad_payload_word_id, bad_payload_count, bad_mismatch_count, bad_drop_count);
        if (good_payload_fire)
            $display("C%0d GOOD beat word=%0d count=%0d mismatch=%0d drops=%0d",
                     cycle, good_payload_word_id, good_payload_count, good_mismatch_count, good_drop_count);
    end
end

reg [7:0] first_beat_expected [0:3];
reg [2:0] first_beat_idx;
reg       first_beat_fail;
initial begin
    first_beat_expected[0] = 8'd0;
    first_beat_expected[1] = 8'd0;
    first_beat_expected[2] = 8'd1;
    first_beat_expected[3] = 8'd1;
    first_beat_idx = 3'd0;
    first_beat_fail = 1'b0;
end
always @(posedge clk) begin
    if (rst_n && good_payload_fire && first_beat_idx < 3'd4) begin
        if (good_payload_word_id !== first_beat_expected[first_beat_idx]) begin
            $display("FIRST-BEAT ERROR: beat %0d expected word %0d got %0d",
                     first_beat_idx, first_beat_expected[first_beat_idx], good_payload_word_id);
            first_beat_fail = 1'b1;
        end
        first_beat_idx = first_beat_idx + 3'd1;
    end
end

initial begin
    clk = 1'b0;
    rst_n = 1'b0;
    start_pulse = 1'b0;
    frame_data_ready_allow = 1'b0;
    payload_fire_allow = 1'b0;
    cycle = 0;

    #12;
    rst_n = 1'b1;
    #8;
    start_pulse = 1'b1;
    frame_data_ready_allow = 1'b1;
    #10;
    start_pulse = 1'b0;
    payload_fire_allow = 1'b1;

    repeat (6) @(posedge clk);
    frame_data_ready_allow = 1'b0;
    repeat (5) @(posedge clk);
    frame_data_ready_allow = 1'b1;

    repeat (4) @(posedge clk);
    payload_fire_allow = 1'b0;
    repeat (4) @(posedge clk);
    payload_fire_allow = 1'b1;

    repeat (24) @(posedge clk);

    $display("SUMMARY bad_count=%0d bad_mismatch=%0d bad_drops=%0d good_count=%0d good_mismatch=%0d good_drops=%0d",
             bad_payload_count, bad_mismatch_count, bad_drop_count,
             good_payload_count, good_mismatch_count, good_drop_count);

    if (good_mismatch_count != 0) begin
        $display("ERROR: pending raw request model still misordered payload words");
        $fatal;
    end

    if (good_drop_count != 0) begin
        $display("ERROR: pending raw request model still overflowed the 2-deep raw queue");
        $fatal;
    end

    if (good_payload_count < 12) begin
        $display("ERROR: pending raw request model did not sustain enough payload beats");
        $fatal;
    end

    if (bad_payload_count >= good_payload_count) begin
        $display("ERROR: non-pending raw request model did not show the expected progress loss");
        $fatal;
    end

    if (first_beat_fail) begin
        $display("ERROR: startup-to-steady transition word ordering was incorrect");
        $fatal;
    end

    $display("PASS: pending raw request model preserved ordered queue output and resumed after ready stalls.");
    $finish;
end

endmodule
