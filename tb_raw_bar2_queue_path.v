`timescale 1ns / 1ps

module raw_bar2_queue_path_case (
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
);

reg        dma_session_active_r;
reg [17:0] frame_src_req_count_r;
reg [7:0]  next_req_word_r;
reg        req_pipe0_valid_r;
reg        req_pipe1_valid_r;
reg [7:0]  req_pipe0_word_r;
reg [7:0]  req_pipe1_word_r;
reg        src_word_valid_r;
reg [7:0]  src_word_r;
reg [11:0] bar_rd_addr_d_r;
reg        bar_rd_clk_en_d_r;
reg        rd_start_pulse_r;
reg [4:0]  pending_age_r;
reg        prev_output_valid_r;
reg [7:0]  prev_output_word_r;

wire       bar_rd_clk_en;
wire [11:0] bar_rd_addr;
wire       bar2_addr_step = bar_rd_clk_en &&
                            ((bar_rd_addr != bar_rd_addr_d_r) || (~bar_rd_clk_en_d_r));
wire       bootstrap_fetch = dma_session_active_r && !rd_started && frame_data_ready_allow &&
                             (frame_src_req_count_r < 18'd4);
wire       stream_fetch = dma_session_active_r && rd_started && bar2_addr_step && frame_data_ready_allow;
wire       frame_rd_fetch_en = bootstrap_fetch || stream_fetch;
wire [7:0] active_word_or_invalid = src_word_valid_r ? src_word_r : 8'hEE;
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
        frame_src_req_count_r <= 18'd0;
        next_req_word_r <= 8'd0;
        req_pipe0_valid_r <= 1'b0;
        req_pipe1_valid_r <= 1'b0;
        req_pipe0_word_r <= 8'd0;
        req_pipe1_word_r <= 8'd0;
        src_word_valid_r <= 1'b0;
        src_word_r <= 8'd0;
        bar_rd_addr_d_r <= 12'd0;
        bar_rd_clk_en_d_r <= 1'b0;
        rd_start_pulse_r <= 1'b0;
        pending_age_r <= 5'd0;
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
            frame_src_req_count_r <= 18'd0;
            next_req_word_r <= 8'd0;
            req_pipe0_valid_r <= 1'b0;
            req_pipe1_valid_r <= 1'b0;
            req_pipe0_word_r <= 8'd0;
            req_pipe1_word_r <= 8'd0;
            src_word_valid_r <= 1'b0;
            src_word_r <= 8'd0;
            pending_age_r <= 5'd0;
            prev_output_valid_r <= 1'b0;
            prev_output_word_r <= 8'd0;
            rd_started <= 1'b0;
            beat_count <= 32'd0;
            mismatch_count <= 32'd0;
            queue_phase_mismatch_count <= 32'd0;
            pending_timeout_count <= 32'd0;
            invalid_queue_read_count <= 32'd0;
        end else if (dma_session_active_r) begin
            if (!rd_started && req_pipe1_valid_r) begin
                rd_start_pulse_r <= 1'b1;
                rd_started <= 1'b1;
            end

            if (frame_rd_fetch_en) begin
                frame_src_req_count_r <= frame_src_req_count_r + 18'd1;
                req_pipe0_valid_r <= 1'b1;
                req_pipe0_word_r <= next_req_word_r;
                next_req_word_r <= next_req_word_r + 8'd1;
            end

            if (req_pipe1_valid_r) begin
                src_word_valid_r <= 1'b1;
                src_word_r <= req_pipe1_word_r;
            end

            if (rd_started && bar2_addr_step && !frame_data_ready_allow) begin
                if (pending_age_r != 5'd31)
                    pending_age_r <= pending_age_r + 5'd1;
                if (pending_age_r >= 5'd12)
                    pending_timeout_count <= pending_timeout_count + 32'd1;
            end else begin
                pending_age_r <= 5'd0;
            end

            if (rd_started && bar2_addr_step && bar_rd_clk_en_d_r) begin
                if (bar_rd_addr != (bar_rd_addr_d_r + 12'd1))
                    queue_phase_mismatch_count <= queue_phase_mismatch_count + 32'd1;
            end

            if (beat_fire) begin
                beat_count <= beat_count + 32'd1;
                if (rd_data[7:0] == 8'hEE)
                    invalid_queue_read_count <= invalid_queue_read_count + 32'd1;
                if (prev_output_valid_r &&
                    (rd_data[7:0] != prev_output_word_r) &&
                    (rd_data[7:0] != (prev_output_word_r + 8'd1)))
                    mismatch_count <= mismatch_count + 32'd1;
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
        $display("ERROR: timeout waiting for BAR2 read start in linear mode");
        $fatal;
    end

    wait (beat_count >= 32'd4);
    tx_hold = 1'b1;
    repeat (3) @(posedge clk);
    tx_hold = 1'b0;

    wait (beat_count >= 32'd8);
    frame_data_ready_allow = 1'b0;
    repeat (3) @(posedge clk);
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
        $display("ERROR: timeout waiting for all BAR2 beats in linear mode");
        $fatal;
    end
    repeat (4) @(posedge clk);

    $display("SUMMARY beats=%0d mismatches=%0d addr_jumps=%0d pending_timeout=%0d invalid_reads=%0d",
             beat_count, mismatch_count, queue_phase_mismatch_count,
             pending_timeout_count, invalid_queue_read_count);

    if (beat_count != 32'd16) begin
        $display("ERROR: expected exactly 16 beats from linear BAR2 stream");
        $fatal;
    end

    if (mismatch_count != 32'd0) begin
        $display("ERROR: BAR2 consumer observed non-monotonic beat ordering in linear mode");
        $fatal;
    end

    if (queue_phase_mismatch_count != 32'd0) begin
        $display("ERROR: BAR2 address steps were not strictly monotonic");
        $fatal;
    end

    if (pending_timeout_count != 32'd0) begin
        $display("ERROR: frame read fetch requests stalled too long in linear mode");
        $fatal;
    end

    if (invalid_queue_read_count != 32'd0) begin
        $display("ERROR: BAR2 reader consumed data before source stream became valid");
        $fatal;
    end

    $display("PASS: linear BAR2 read model kept monotonic source ordering through backpressure.");
    $finish;
end

endmodule
