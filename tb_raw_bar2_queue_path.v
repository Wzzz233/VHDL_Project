`timescale 1ns / 1ps

module raw_bar2_queue_path_case #(
    parameter [2:0] RAW_BOOTSTRAP_WORDS = 3'd2
) (
    input  wire clk,
    input  wire rst_n,
    input  wire start_pulse,
    input  wire frame_data_ready_allow,
    input  wire tx_hold,
    output reg  rd_started,
    output reg  [31:0] beat_count,
    output reg  [31:0] mismatch_count,
    output reg  [31:0] queue_phase_mismatch_count,
    output reg  [31:0] pending_timeout_count,
    output reg  [31:0] invalid_queue_read_count
) ;

reg        dma_session_active_r;
reg        dma_expand_phase_r;
reg        raw_req_pending_r;
reg [17:0] frame_src_req_count_r;
reg [2:0]  frame_src_bootstrap_count_r;
reg        raw_startup_active_r;
reg        raw_start_word_valid_r;
reg [7:0]  raw_start_word_r;
reg        req_pipe0_valid_r;
reg        req_pipe1_valid_r;
reg [7:0]  req_pipe0_word_r;
reg [7:0]  req_pipe1_word_r;
reg [7:0]  next_req_word_r;
reg        out_pair_active_valid_r;
reg        out_pair_next_valid_r;
reg [7:0]  out_pair_active_word_r;
reg [7:0]  out_pair_next_word_r;
reg [11:0] bar_rd_addr_d_r;
reg        bar_rd_clk_en_d_r;
reg        low_phase_pending_r;
reg [7:0]  low_phase_word_r;
reg [4:0]  pending_age_r;
reg        rd_start_pulse_r;
reg        prev_output_valid_r;
reg [7:0]  prev_output_word_r;

wire [7:0] consumer_word = raw_startup_active_r ? raw_start_word_r : out_pair_active_word_r;
wire       raw_bootstrap_ready = raw_start_word_valid_r;
wire       raw_stream_ready = raw_startup_active_r ? raw_bootstrap_ready
                                                   : out_pair_active_valid_r;
wire       bar_rd_clk_en;
wire [11:0] bar_rd_addr;
wire       bar2_addr_step = bar_rd_clk_en &&
                            ((bar_rd_addr != bar_rd_addr_d_r) || (~bar_rd_clk_en_d_r));
wire       out_pair_pop = !raw_startup_active_r && bar2_addr_step && dma_expand_phase_r;
wire       raw_output_need = bar2_addr_step && !dma_expand_phase_r;
wire       raw_queue_has_room = !out_pair_active_valid_r || !out_pair_next_valid_r || out_pair_pop;
wire       raw_start_bootstrap_req = raw_startup_active_r &&
                                     (frame_src_bootstrap_count_r < RAW_BOOTSTRAP_WORDS);
wire       raw_req_need = raw_start_bootstrap_req || raw_req_pending_r || raw_output_need;
wire       frame_rd_req_en_raw = dma_session_active_r &&
                                 frame_data_ready_allow &&
                                 raw_queue_has_room &&
                                 (frame_src_req_count_r < 18'd64) &&
                                 raw_req_need;
wire       frame_rd_data_valid = req_pipe1_valid_r;
wire [7:0] frame_rd_word_id = req_pipe1_word_r;
wire       raw_capture_fire = dma_session_active_r && frame_rd_data_valid;
wire       raw_start_capture_now = raw_startup_active_r && !raw_start_word_valid_r && raw_capture_fire;
wire       raw_queue_capture_fire = raw_capture_fire && (!raw_startup_active_r || raw_start_word_valid_r);
wire       raw_startup_done = raw_startup_active_r &&
                              raw_bootstrap_ready &&
                              bar2_addr_step &&
                              dma_expand_phase_r;

wire [7:0] active_word_or_invalid = raw_stream_ready ? consumer_word : 8'hEE;
wire [127:0] bar_rd_data = {16{active_word_or_invalid}};

wire       rd_data_valid;
wire [127:0] rd_data;
wire       rd_last_data;
wire       beat_fire = rd_data_valid && !tx_hold;

ips2l_pcie_dma_rd_ctrl #(
    .ADDR_WIDTH(12)
) dut (
    .clk(clk),
    .rst_n(rst_n),
    .i_rd_en(rd_start_pulse_r),
    .i_rd_length(10'd64),
    .i_rd_addr(64'd0),
    .i_tx_hold(tx_hold),
    .i_tlp_tx(1'b1),
    .o_rd_ram_hold(),
    .o_gen_tlp_start(rd_data_valid),
    .o_rd_data(rd_data),
    .o_last_data(rd_last_data),
    .o_bar_rd_clk_en(bar_rd_clk_en),
    .o_bar_rd_addr(bar_rd_addr),
    .i_bar_rd_data(bar_rd_data)
);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        dma_session_active_r <= 1'b0;
        dma_expand_phase_r <= 1'b0;
        raw_req_pending_r <= 1'b0;
        frame_src_req_count_r <= 18'd0;
        frame_src_bootstrap_count_r <= 3'd0;
        raw_startup_active_r <= 1'b0;
        raw_start_word_valid_r <= 1'b0;
        raw_start_word_r <= 8'd0;
        req_pipe0_valid_r <= 1'b0;
        req_pipe1_valid_r <= 1'b0;
        req_pipe0_word_r <= 8'd0;
        req_pipe1_word_r <= 8'd0;
        next_req_word_r <= 8'd0;
        out_pair_active_valid_r <= 1'b0;
        out_pair_next_valid_r <= 1'b0;
        out_pair_active_word_r <= 8'd0;
        out_pair_next_word_r <= 8'd0;
        bar_rd_addr_d_r <= 12'd0;
        bar_rd_clk_en_d_r <= 1'b0;
        low_phase_pending_r <= 1'b0;
        low_phase_word_r <= 8'd0;
        pending_age_r <= 5'd0;
        rd_start_pulse_r <= 1'b0;
        prev_output_valid_r <= 1'b0;
        prev_output_word_r <= 8'd0;
        rd_started <= 1'b0;
        beat_count <= 32'd0;
        mismatch_count <= 32'd0;
        queue_phase_mismatch_count <= 32'd0;
        pending_timeout_count <= 32'd0;
        invalid_queue_read_count <= 32'd0;
    end else begin
        bar_rd_addr_d_r <= bar_rd_addr;
        bar_rd_clk_en_d_r <= bar_rd_clk_en;
        req_pipe1_valid_r <= req_pipe0_valid_r;
        req_pipe1_word_r <= req_pipe0_word_r;
        req_pipe0_valid_r <= 1'b0;
        rd_start_pulse_r <= 1'b0;

        if (start_pulse && !dma_session_active_r) begin
            dma_session_active_r <= 1'b1;
            dma_expand_phase_r <= 1'b0;
            raw_req_pending_r <= 1'b0;
            frame_src_req_count_r <= 18'd0;
            frame_src_bootstrap_count_r <= 3'd0;
            raw_startup_active_r <= 1'b1;
            raw_start_word_valid_r <= 1'b0;
            raw_start_word_r <= 8'd0;
            req_pipe0_valid_r <= 1'b0;
            req_pipe1_valid_r <= 1'b0;
            req_pipe0_word_r <= 8'd0;
            req_pipe1_word_r <= 8'd0;
            next_req_word_r <= 8'd0;
            out_pair_active_valid_r <= 1'b0;
            out_pair_next_valid_r <= 1'b0;
            out_pair_active_word_r <= 8'd0;
            out_pair_next_word_r <= 8'd0;
            low_phase_pending_r <= 1'b0;
            low_phase_word_r <= 8'd0;
            pending_age_r <= 5'd0;
            rd_started <= 1'b0;
            prev_output_valid_r <= 1'b0;
            prev_output_word_r <= 8'd0;
            beat_count <= 32'd0;
            mismatch_count <= 32'd0;
            queue_phase_mismatch_count <= 32'd0;
            pending_timeout_count <= 32'd0;
            invalid_queue_read_count <= 32'd0;
        end else if (dma_session_active_r) begin
            if (!rd_started && raw_bootstrap_ready) begin
                rd_start_pulse_r <= 1'b1;
                rd_started <= 1'b1;
            end

            if (frame_rd_req_en_raw) begin
                frame_src_req_count_r <= frame_src_req_count_r + 18'd1;
                if (frame_src_bootstrap_count_r != RAW_BOOTSTRAP_WORDS)
                    frame_src_bootstrap_count_r <= frame_src_bootstrap_count_r + 3'd1;
                req_pipe0_valid_r <= 1'b1;
                req_pipe0_word_r <= next_req_word_r;
                next_req_word_r <= next_req_word_r + 8'd1;
            end

            if (raw_req_pending_r) begin
                if (pending_age_r != 5'd31)
                    pending_age_r <= pending_age_r + 5'd1;
                if (pending_age_r >= 5'd12)
                    pending_timeout_count <= pending_timeout_count + 32'd1;
            end else begin
                pending_age_r <= 5'd0;
            end

            if (raw_req_pending_r && frame_rd_req_en_raw)
                raw_req_pending_r <= 1'b0;
            else if (raw_output_need && !frame_rd_req_en_raw)
                raw_req_pending_r <= 1'b1;

            if (raw_start_capture_now) begin
                raw_start_word_valid_r <= 1'b1;
                raw_start_word_r <= frame_rd_word_id;
            end

            if (bar2_addr_step) begin
                if (dma_expand_phase_r && low_phase_pending_r) begin
                    if (consumer_word != low_phase_word_r)
                        queue_phase_mismatch_count <= queue_phase_mismatch_count + 32'd1;
                    low_phase_pending_r <= 1'b0;
                end else if (!dma_expand_phase_r) begin
                    low_phase_pending_r <= 1'b1;
                    low_phase_word_r <= consumer_word;
                end
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

            if (raw_queue_capture_fire) begin
                if (!out_pair_active_valid_r || (out_pair_pop && !out_pair_next_valid_r)) begin
                    out_pair_active_valid_r <= 1'b1;
                    out_pair_active_word_r <= frame_rd_word_id;
                end else if (!out_pair_next_valid_r || out_pair_pop) begin
                    out_pair_next_valid_r <= 1'b1;
                    out_pair_next_word_r <= frame_rd_word_id;
                end
            end

            if (beat_fire) begin
                beat_count <= beat_count + 32'd1;
                if (rd_data[7:0] == 8'hEE)
                    invalid_queue_read_count <= invalid_queue_read_count + 32'd1;
                if (!prev_output_valid_r) begin
                    if (rd_data[7:0] != 8'd0)
                        mismatch_count <= mismatch_count + 32'd1;
                end else if ((rd_data[7:0] != prev_output_word_r) &&
                             (rd_data[7:0] != (prev_output_word_r + 8'd1))) begin
                    mismatch_count <= mismatch_count + 32'd1;
                end
                prev_output_valid_r <= 1'b1;
                prev_output_word_r <= rd_data[7:0];
            end
        end
    end
end

endmodule

module tb_raw_bar2_queue_path;

reg clk;
reg rst_n;
reg start_pulse;
reg frame_data_ready_allow;
reg tx_hold;
integer timeout_cycles;

wire        rd_started;
wire [31:0] beat_count;
wire [31:0] mismatch_count;
wire [31:0] queue_phase_mismatch_count;
wire [31:0] pending_timeout_count;
wire [31:0] invalid_queue_read_count;

raw_bar2_queue_path_case u_case (
    .clk(clk),
    .rst_n(rst_n),
    .start_pulse(start_pulse),
    .frame_data_ready_allow(frame_data_ready_allow),
    .tx_hold(tx_hold),
    .rd_started(rd_started),
    .beat_count(beat_count),
    .mismatch_count(mismatch_count),
    .queue_phase_mismatch_count(queue_phase_mismatch_count),
    .pending_timeout_count(pending_timeout_count),
    .invalid_queue_read_count(invalid_queue_read_count)
);

always #5 clk = ~clk;

initial begin
    clk = 1'b0;
    rst_n = 1'b0;
    start_pulse = 1'b0;
    frame_data_ready_allow = 1'b0;
    tx_hold = 1'b0;

    #20;
    rst_n = 1'b1;
    frame_data_ready_allow = 1'b1;
    #10;
    start_pulse = 1'b1;
    #10;
    start_pulse = 1'b0;

    timeout_cycles = 0;
    while (!rd_started && timeout_cycles < 200) begin
        @(posedge clk);
        timeout_cycles = timeout_cycles + 1;
    end
    if (!rd_started) begin
        $display("ERROR: timeout waiting for BAR2 read start after raw bootstrap");
        $fatal;
    end

    wait (beat_count >= 32'd4);
    tx_hold = 1'b1;
    repeat (3) @(posedge clk);
    tx_hold = 1'b0;

    wait (beat_count >= 32'd8);
    frame_data_ready_allow = 1'b0;
    repeat (4) @(posedge clk);
    frame_data_ready_allow = 1'b1;

    wait (beat_count >= 32'd12);
    tx_hold = 1'b1;
    repeat (2) @(posedge clk);
    tx_hold = 1'b0;

    timeout_cycles = 0;
    while (beat_count < 32'd16 && timeout_cycles < 400) begin
        @(posedge clk);
        timeout_cycles = timeout_cycles + 1;
    end
    if (beat_count < 32'd16) begin
        $display("ERROR: timeout waiting for all BAR2 queue beats");
        $fatal;
    end
    repeat (4) @(posedge clk);

    $display("SUMMARY beats=%0d mismatches=%0d queue_phase=%0d pending_timeout=%0d invalid_reads=%0d",
             beat_count, mismatch_count, queue_phase_mismatch_count,
             pending_timeout_count, invalid_queue_read_count);

    if (beat_count != 32'd16) begin
        $display("ERROR: expected exactly 16 beats from 8 source words");
        $fatal;
    end

    if (mismatch_count != 32'd0) begin
        $display("ERROR: BAR2 consumer observed non-monotonic queue beat ordering");
        $fatal;
    end

    if (queue_phase_mismatch_count != 32'd0) begin
        $display("ERROR: queue head changed before the matching high beat retired");
        $fatal;
    end

    if (pending_timeout_count != 32'd0) begin
        $display("ERROR: raw_req_pending did not resolve within the allowed window");
        $fatal;
    end

    if (invalid_queue_read_count != 32'd0) begin
        $display("ERROR: BAR2 reader sampled queue data before bootstrap completed");
        $fatal;
    end

    $display("PASS: queue-only raw BAR2 path preserved stable queue pairing and monotonic consumer ordering through backpressure.");
    $finish;
end

endmodule
