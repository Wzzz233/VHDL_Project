`timescale 1ns / 1ps

module raw_dma_path_case #(
    parameter BUFFERED_RAW_SOURCE = 1'b0,
    parameter STEADY_READY_FROM_RD = 1'b1,
    parameter HOLD_UPDATE_ON_LOW_OUTPUT = 1'b0,
    parameter [2:0] RAW_BOOTSTRAP_WORDS = 3'd1
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       start_pulse,
    input  wire       frame_rd_data_ready_in,
    input  wire       payload_fire_allow,
    output reg        payload_fire,
    output reg  [7:0] payload_word_id,
    output reg        mismatch,
    output reg [31:0] mismatch_count
);

reg        dma_session_active_r;
reg        dma_expand_phase_r;
reg [17:0] frame_src_req_count_r;
reg [2:0]  frame_src_bootstrap_count_r;
reg        raw_startup_active_r;
reg        raw_start_word_valid_r;
reg [7:0]  raw_start_word_id_r;
reg [7:0]  frame_rd_hold_word_id_r;
reg [7:0]  req_word_counter_r;
reg        req_pipe0_valid_r;
reg        req_pipe1_valid_r;
reg [7:0]  req_pipe0_word_r;
reg [7:0]  req_pipe1_word_r;
reg        out_pair_active_valid_r;
reg        out_pair_next_valid_r;
reg [7:0]  out_pair_active_word_r;
reg [7:0]  out_pair_next_word_r;
reg [7:0]  expected_word_id_r;
reg        expected_hi_phase_r;
reg        session_started_r;

wire dma_expand_mode = 1'b1;
wire frame_rd_data_valid = req_pipe1_valid_r;
wire [7:0] frame_rd_word_id = req_pipe1_word_r;
wire raw_start_bootstrap_req = raw_startup_active_r &&
                               (frame_src_bootstrap_count_r < RAW_BOOTSTRAP_WORDS);
wire raw_stream_ready = (dma_expand_mode && raw_startup_active_r)
    ? raw_start_word_valid_r
    : (STEADY_READY_FROM_RD ? frame_rd_data_ready_in : 1'b1);
wire frame_output_step = dma_session_active_r && payload_fire_allow && raw_stream_ready;
wire frame_rd_req_en_raw = dma_session_active_r &&
                           frame_rd_data_ready_in &&
                           (frame_src_req_count_r < 18'd32) &&
                           (raw_start_bootstrap_req ||
                            (frame_output_step && ~dma_expand_phase_r));
wire raw_capture_fire = dma_session_active_r && frame_rd_data_valid;
wire raw_start_capture_now = raw_startup_active_r && raw_capture_fire && !raw_start_word_valid_r;
wire raw_startup_use_word = raw_startup_active_r &&
                            (raw_start_word_valid_r || raw_start_capture_now);
wire out_pair_pop = frame_output_step && dma_expand_phase_r;
wire raw_startup_done = raw_startup_use_word && out_pair_pop;
wire raw_hold_update_fire = HOLD_UPDATE_ON_LOW_OUTPUT
    ? (frame_output_step && ~dma_expand_phase_r)
    : raw_capture_fire;
wire [7:0] raw_start_src_word_id = raw_start_word_valid_r ? raw_start_word_id_r : frame_rd_word_id;
wire [7:0] raw_lo_word_id = raw_startup_use_word ? raw_start_src_word_id : frame_rd_word_id;
wire [7:0] raw_hi_word_id = raw_startup_use_word ? raw_start_src_word_id : frame_rd_hold_word_id_r;
wire [7:0] selected_payload_word = BUFFERED_RAW_SOURCE
    ? out_pair_active_word_r
    : (dma_expand_phase_r ? raw_hi_word_id : raw_lo_word_id);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        dma_session_active_r <= 1'b0;
        dma_expand_phase_r <= 1'b0;
        frame_src_req_count_r <= 18'd0;
        frame_src_bootstrap_count_r <= 3'd0;
        raw_startup_active_r <= 1'b0;
        raw_start_word_valid_r <= 1'b0;
        raw_start_word_id_r <= 8'd0;
        frame_rd_hold_word_id_r <= 8'd0;
        req_word_counter_r <= 8'd0;
        req_pipe0_valid_r <= 1'b0;
        req_pipe1_valid_r <= 1'b0;
        req_pipe0_word_r <= 8'd0;
        req_pipe1_word_r <= 8'd0;
        out_pair_active_valid_r <= 1'b0;
        out_pair_next_valid_r <= 1'b0;
        out_pair_active_word_r <= 8'd0;
        out_pair_next_word_r <= 8'd0;
        expected_word_id_r <= 8'd0;
        expected_hi_phase_r <= 1'b0;
        session_started_r <= 1'b0;
        payload_fire <= 1'b0;
        payload_word_id <= 8'd0;
        mismatch <= 1'b0;
        mismatch_count <= 32'd0;
    end else begin
        req_pipe1_valid_r <= req_pipe0_valid_r;
        req_pipe1_word_r <= req_pipe0_word_r;
        req_pipe0_valid_r <= 1'b0;
        payload_fire <= 1'b0;
        mismatch <= 1'b0;

        if (start_pulse && !dma_session_active_r) begin
            dma_session_active_r <= 1'b1;
            dma_expand_phase_r <= 1'b0;
            frame_src_req_count_r <= 18'd0;
            frame_src_bootstrap_count_r <= 3'd0;
            raw_startup_active_r <= 1'b1;
            raw_start_word_valid_r <= 1'b0;
            raw_start_word_id_r <= 8'd0;
            frame_rd_hold_word_id_r <= 8'd0;
            req_word_counter_r <= 8'd0;
            req_pipe0_valid_r <= 1'b0;
            req_pipe1_valid_r <= 1'b0;
            out_pair_active_valid_r <= 1'b0;
            out_pair_next_valid_r <= 1'b0;
            out_pair_active_word_r <= 8'd0;
            out_pair_next_word_r <= 8'd0;
            expected_word_id_r <= 8'd0;
            expected_hi_phase_r <= 1'b0;
            session_started_r <= 1'b0;
        end else if (dma_session_active_r) begin
            if (frame_output_step) begin
                payload_fire <= 1'b1;
                payload_word_id <= selected_payload_word;
                if (session_started_r) begin
                    if (selected_payload_word != expected_word_id_r) begin
                        mismatch <= 1'b1;
                        mismatch_count <= mismatch_count + 32'd1;
                    end
                end else begin
                    session_started_r <= 1'b1;
                end

                if (expected_hi_phase_r)
                    expected_word_id_r <= expected_word_id_r + 8'd1;
                expected_hi_phase_r <= ~expected_hi_phase_r;

                dma_expand_phase_r <= ~dma_expand_phase_r;
            end

            if (out_pair_pop) begin
                if (out_pair_next_valid_r) begin
                    out_pair_active_valid_r <= 1'b1;
                    out_pair_active_word_r <= out_pair_next_word_r;
                    out_pair_next_valid_r <= 1'b0;
                end else begin
                    out_pair_active_valid_r <= 1'b0;
                end
            end

            if (raw_hold_update_fire)
                frame_rd_hold_word_id_r <= frame_rd_word_id;

            if (raw_capture_fire) begin
                if (raw_startup_active_r && !raw_start_word_valid_r) begin
                    raw_start_word_id_r <= frame_rd_word_id;
                    raw_start_word_valid_r <= 1'b1;
                end

                if (!out_pair_active_valid_r || (out_pair_pop && !out_pair_next_valid_r)) begin
                    out_pair_active_valid_r <= 1'b1;
                    out_pair_active_word_r <= frame_rd_word_id;
                end else if (!out_pair_next_valid_r || out_pair_pop) begin
                    out_pair_next_valid_r <= 1'b1;
                    out_pair_next_word_r <= frame_rd_word_id;
                end
            end

            if (raw_startup_done)
                raw_startup_active_r <= 1'b0;

            if (frame_rd_req_en_raw) begin
                frame_src_req_count_r <= frame_src_req_count_r + 18'd1;
                if (frame_src_bootstrap_count_r != RAW_BOOTSTRAP_WORDS)
                    frame_src_bootstrap_count_r <= frame_src_bootstrap_count_r + 3'd1;
                req_pipe0_valid_r <= 1'b1;
                req_pipe0_word_r <= req_word_counter_r;
                req_word_counter_r <= req_word_counter_r + 8'd1;
            end
        end
    end
end

endmodule

module tb_raw_dma_path;

reg clk;
reg rst_n;
reg start_pulse;
reg frame_rd_data_ready_in;
reg payload_fire_allow;
integer cycle;

wire       bad_payload_fire;
wire [7:0] bad_payload_word_id;
wire       bad_mismatch;
wire [31:0] bad_mismatch_count;

wire       good_payload_fire;
wire [7:0] good_payload_word_id;
wire       good_mismatch;
wire [31:0] good_mismatch_count;

raw_dma_path_case #(
    .BUFFERED_RAW_SOURCE(1'b1),
    .STEADY_READY_FROM_RD(1'b0)
) u_bad (
    .clk(clk),
    .rst_n(rst_n),
    .start_pulse(start_pulse),
    .frame_rd_data_ready_in(frame_rd_data_ready_in),
    .payload_fire_allow(payload_fire_allow),
    .payload_fire(bad_payload_fire),
    .payload_word_id(bad_payload_word_id),
    .mismatch(bad_mismatch),
    .mismatch_count(bad_mismatch_count)
);

raw_dma_path_case #(
    .BUFFERED_RAW_SOURCE(1'b0),
    .STEADY_READY_FROM_RD(1'b1),
    .HOLD_UPDATE_ON_LOW_OUTPUT(1'b1)
) u_good (
    .clk(clk),
    .rst_n(rst_n),
    .start_pulse(start_pulse),
    .frame_rd_data_ready_in(frame_rd_data_ready_in),
    .payload_fire_allow(payload_fire_allow),
    .payload_fire(good_payload_fire),
    .payload_word_id(good_payload_word_id),
    .mismatch(good_mismatch),
    .mismatch_count(good_mismatch_count)
);

always #5 clk = ~clk;

always @(posedge clk) begin
    if (rst_n) begin
        cycle = cycle + 1;
        if (bad_payload_fire)
            $display("C%0d BAD  beat word=%0d mismatches=%0d", cycle, bad_payload_word_id, bad_mismatch_count);
        if (good_payload_fire)
            $display("C%0d GOOD beat word=%0d mismatches=%0d", cycle, good_payload_word_id, good_mismatch_count);
    end
end

initial begin
    clk = 1'b0;
    rst_n = 1'b0;
    start_pulse = 1'b0;
    frame_rd_data_ready_in = 1'b1;
    payload_fire_allow = 1'b0;
    cycle = 0;

    #12;
    rst_n = 1'b1;
    #8;
    start_pulse = 1'b1;
    #10;
    start_pulse = 1'b0;
    payload_fire_allow = 1'b1;

    repeat (4) @(posedge clk);
    frame_rd_data_ready_in = 1'b0;
    repeat (2) @(posedge clk);
    frame_rd_data_ready_in = 1'b1;

    repeat (5) @(posedge clk);
    payload_fire_allow = 1'b0;
    repeat (2) @(posedge clk);
    payload_fire_allow = 1'b1;

    repeat (4) @(posedge clk);
    frame_rd_data_ready_in = 1'b0;
    repeat (3) @(posedge clk);
    frame_rd_data_ready_in = 1'b1;

    repeat (16) @(posedge clk);

    $display("SUMMARY bad_mismatch_count=%0d good_mismatch_count=%0d",
             bad_mismatch_count, good_mismatch_count);

    if (good_mismatch_count != 0) begin
        $display("ERROR: proposed raw fast path still emitted a misordered payload sequence");
        $fatal;
    end

    if (bad_mismatch_count == 0) begin
        $display("ERROR: regression stimulus did not expose the buffered raw path hazard");
        $fatal;
    end

    $display("PASS: direct raw fast path preserved payload order while buffered raw path misordered words under stalls.");
    $finish;
end

endmodule
