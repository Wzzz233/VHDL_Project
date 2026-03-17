`timescale 1ns / 1ps

module tb_cmos_8_16bit_yuyv;

logic       pclk;
logic       rst_n;
logic       de_i;
logic [7:0] pdata_i;
logic       vs_i;

logic       pos_pixel_clk;
logic       pos_de_o;
logic       pos_pix_vld_o;
logic       pos_vs_o;
logic [15:0] pos_pdata_o;

logic       neg_pixel_clk;
logic       neg_de_o;
logic       neg_pix_vld_o;
logic       neg_vs_o;
logic [15:0] neg_pdata_o;

logic [15:0] expected [0:7];
int expected_count;

int pos_idx;
int neg_idx;
int pos_words;
int neg_words;
int pos_mismatch;
int neg_mismatch;
bit monitor_en;

byte unsigned line1 [0:7];
byte unsigned line2 [0:4];
byte unsigned line3 [0:3];

cmos_8_16bit #(
    .CAPTURE_ON_NEGEDGE(1'b0)
) dut_posedge (
    .pclk      (pclk),
    .rst_n     (rst_n),
    .de_i      (de_i),
    .pdata_i   (pdata_i),
    .vs_i      (vs_i),
    .pixel_clk (pos_pixel_clk),
    .de_o      (pos_de_o),
    .pix_vld_o (pos_pix_vld_o),
    .vs_o      (pos_vs_o),
    .pdata_o   (pos_pdata_o)
);

cmos_8_16bit #(
    .CAPTURE_ON_NEGEDGE(1'b1)
) dut_negedge (
    .pclk      (pclk),
    .rst_n     (rst_n),
    .de_i      (de_i),
    .pdata_i   (pdata_i),
    .vs_i      (vs_i),
    .pixel_clk (neg_pixel_clk),
    .de_o      (neg_de_o),
    .pix_vld_o (neg_pix_vld_o),
    .vs_o      (neg_vs_o),
    .pdata_o   (neg_pdata_o)
);

always #5 pclk = ~pclk;

task automatic reset_scoreboard;
begin
    pos_idx = 0;
    neg_idx = 0;
    pos_words = 0;
    neg_words = 0;
    pos_mismatch = 0;
    neg_mismatch = 0;
end
endtask

task automatic drive_line_on_negedge(input byte unsigned data_line[]);
begin
    @(negedge pclk);
    #1 de_i <= 1'b1;
    pdata_i <= data_line[0];
    for (int i = 1; i < data_line.size(); i++) begin
        @(negedge pclk);
        #1 pdata_i <= data_line[i];
    end
    @(negedge pclk);
    #1 de_i <= 1'b0;
    repeat (2) @(negedge pclk);
end
endtask

task automatic drive_line_on_posedge(input byte unsigned data_line[]);
begin
    @(posedge pclk);
    #1 de_i <= 1'b1;
    pdata_i <= data_line[0];
    for (int i = 1; i < data_line.size(); i++) begin
        @(posedge pclk);
        #1 pdata_i <= data_line[i];
    end
    @(posedge pclk);
    #1 de_i <= 1'b0;
    repeat (2) @(posedge pclk);
end
endtask

task automatic run_case_sensor_updates_on_negedge;
begin
    $display("CASE-A: sensor updates on negedge (expected CAPTURE_ON_NEGEDGE=0 to pass)");
    reset_scoreboard();
    monitor_en = 1'b1;

    @(negedge pclk);
    #1 vs_i <= 1'b1;
    repeat (2) @(negedge pclk);

    drive_line_on_negedge(line1);
    drive_line_on_negedge(line2); // odd-byte line: last byte should be dropped
    drive_line_on_negedge(line3);

    @(negedge pclk);
    #1 vs_i <= 1'b0;
    repeat (8) @(posedge pclk);
    monitor_en = 1'b0;

    $display("CASE-A RESULT: pos_words=%0d pos_mismatch=%0d neg_words=%0d neg_mismatch=%0d",
             pos_words, pos_mismatch, neg_words, neg_mismatch);

    if (pos_words != expected_count || pos_mismatch != 0) begin
        $display("ERROR: CASE-A expected posedge-capture DUT to match all expected YUYV words");
        $fatal;
    end
end
endtask

task automatic run_case_sensor_updates_on_posedge;
begin
    $display("CASE-B: sensor updates on posedge (expected CAPTURE_ON_NEGEDGE=1 to pass)");
    reset_scoreboard();
    monitor_en = 1'b1;

    @(posedge pclk);
    #1 vs_i <= 1'b1;
    repeat (2) @(posedge pclk);

    drive_line_on_posedge(line1);
    drive_line_on_posedge(line2); // odd-byte line: last byte should be dropped
    drive_line_on_posedge(line3);

    @(posedge pclk);
    #1 vs_i <= 1'b0;
    repeat (8) @(posedge pclk);
    monitor_en = 1'b0;

    $display("CASE-B RESULT: pos_words=%0d pos_mismatch=%0d neg_words=%0d neg_mismatch=%0d",
             pos_words, pos_mismatch, neg_words, neg_mismatch);

    if (neg_words != expected_count || neg_mismatch != 0) begin
        $display("ERROR: CASE-B expected negedge-capture DUT to match all expected YUYV words");
        $fatal;
    end
end
endtask

always @(posedge pclk) begin
    if (monitor_en && pos_pix_vld_o) begin
        pos_words <= pos_words + 1;
        if (pos_idx >= expected_count || pos_pdata_o !== expected[pos_idx])
            pos_mismatch <= pos_mismatch + 1;
        pos_idx <= pos_idx + 1;
    end

    if (monitor_en && neg_pix_vld_o) begin
        neg_words <= neg_words + 1;
        if (neg_idx >= expected_count || neg_pdata_o !== expected[neg_idx])
            neg_mismatch <= neg_mismatch + 1;
        neg_idx <= neg_idx + 1;
    end
end

initial begin
    pclk = 1'b0;
    rst_n = 1'b0;
    de_i = 1'b0;
    pdata_i = 8'h00;
    vs_i = 1'b0;
    monitor_en = 1'b0;
    reset_scoreboard();

    line1 = '{8'h10, 8'hA0, 8'h20, 8'hB0, 8'h30, 8'hA1, 8'h40, 8'hB1};
    line2 = '{8'h11, 8'hC0, 8'h21, 8'hD0, 8'h31}; // odd bytes (drop last one)
    line3 = '{8'h12, 8'hC1, 8'h22, 8'hD1};

    expected[0] = 16'h10A0;
    expected[1] = 16'h20B0;
    expected[2] = 16'h30A1;
    expected[3] = 16'h40B1;
    expected[4] = 16'h11C0;
    expected[5] = 16'h21D0;
    expected[6] = 16'h12C1;
    expected[7] = 16'h22D1;
    expected_count = 8;

    repeat (5) @(posedge pclk);
    rst_n = 1'b1;
    repeat (4) @(posedge pclk);

    run_case_sensor_updates_on_negedge();
    repeat (6) @(posedge pclk);
    run_case_sensor_updates_on_posedge();

    $display("PASS: cmos_8_16bit YUYV byte-phase behavior validated for both sampling-edge configurations.");
    $finish;
end

endmodule
