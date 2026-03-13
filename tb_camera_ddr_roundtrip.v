`timescale 1ns / 1ps

module wr_fram_buf (
    input      [31:0]  a_wr_data,
    input      [11:0]  a_addr,
    input              a_wr_en,
    input              a_clk,
    input              a_rst,
    input      [9:0]   b_addr,
    output reg [127:0] b_rd_data,
    input              b_clk,
    input              b_rst
);

reg [31:0] mem [0:4095];
integer idx;
integer base;

always @(posedge a_clk) begin
    if (a_rst) begin
        for (idx = 0; idx < 4096; idx = idx + 1)
            mem[idx] <= 32'd0;
    end else if (a_wr_en) begin
        mem[a_addr] <= a_wr_data;
    end
end

always @(posedge b_clk) begin
    if (b_rst) begin
        b_rd_data <= 128'd0;
    end else begin
        base = {b_addr, 2'b00};
        b_rd_data <= {
            mem[base + 3],
            mem[base + 2],
            mem[base + 1],
            mem[base + 0]
        };
    end
end

endmodule

module rd_fram_buf (
    input      [127:0] a_wr_data,
    input      [9:0]   a_addr,
    output reg [127:0] a_rd_data,
    input              a_wr_en,
    input              a_clk,
    input              a_rst,
    input      [9:0]   b_addr,
    input      [127:0] b_wr_data,
    output reg [127:0] b_rd_data,
    input              b_wr_en,
    input              b_clk,
    input              b_rst
);

reg [127:0] mem [0:1023];
integer idx;

always @(posedge a_clk) begin
    if (a_rst) begin
        for (idx = 0; idx < 1024; idx = idx + 1)
            mem[idx] <= 128'd0;
        a_rd_data <= 128'd0;
    end else begin
        if (a_wr_en)
            mem[a_addr] <= a_wr_data;
        a_rd_data <= mem[a_addr];
    end
end

always @(posedge b_clk) begin
    if (b_rst) begin
        b_rd_data <= 128'd0;
    end else begin
        if (b_wr_en)
            mem[b_addr] <= b_wr_data;
        b_rd_data <= mem[b_addr];
    end
end

endmodule

module tb_camera_ddr_roundtrip;

localparam integer H_NUM = 128;
localparam integer V_NUM = 8;
localparam integer CHECK_WORDS = 8;
localparam integer CTRL_ADDR_WIDTH = 16;
localparam integer LEN_WIDTH = 16;
localparam integer DQ_WIDTH = 16;
localparam integer LINE_ADDR_WIDTH = 10;
localparam integer DDR_BEAT_ADDR_INC = 8;

reg         pclk;
reg         ddr_clk;
reg         vout_clk;
reg         rst_n;
reg         de_i;
reg         vs_i;
reg  [7:0]  pdata_i;
reg         rd_fsync;
reg         rd_en;

wire        pixel_clk;
wire        de_o;
wire        pix_vld_o;
wire        vs_o;
wire [15:0] pdata_o;

wire                        ddr_wreq;
wire [CTRL_ADDR_WIDTH-1:0]  ddr_waddr;
wire [LEN_WIDTH-1:0]        ddr_wr_len;
wire                        ddr_wrdy;
wire                        ddr_wdone;
wire [DQ_WIDTH*8-1:0]       ddr_wdata;
wire                        ddr_wdata_req;
wire [7:0]                  frame_wcnt;

wire                        vout_de;
wire [127:0]                vout_data;
wire                        rd_data_ready;
wire                        ddr_rreq;
wire [CTRL_ADDR_WIDTH-1:0]  ddr_raddr;
wire [LEN_WIDTH-1:0]        ddr_rd_len;
wire                        ddr_rrdy;
wire                        ddr_rdone;
wire [DQ_WIDTH*8-1:0]       ddr_rdata;
wire                        ddr_rdata_en;

reg                         wr_busy;
reg [CTRL_ADDR_WIDTH-1:0]   wr_addr_cur;
integer                     wr_beats_left;
reg                         wr_done_r;
reg                         wr_data_req_r;

reg                         rd_busy;
reg [CTRL_ADDR_WIDTH-1:0]   rd_addr_cur;
integer                     rd_beats_left;
reg                         rd_done_r;
reg                         rd_data_en_r;
reg [DQ_WIDTH*8-1:0]        rd_data_r;

reg [127:0] ddr_mem [0:2047];

integer i;
integer word_count;
integer timeout_cnt;

function [15:0] pixel_word;
    input integer idx;
begin
    pixel_word = 16'h1200 + idx[15:0];
end
endfunction

function [127:0] expected_word;
    input integer word_idx;
    integer base;
begin
    base = word_idx * 8;
    expected_word = {
        pixel_word(base + 7),
        pixel_word(base + 6),
        pixel_word(base + 5),
        pixel_word(base + 4),
        pixel_word(base + 3),
        pixel_word(base + 2),
        pixel_word(base + 1),
        pixel_word(base + 0)
    };
end
endfunction

task drive_byte;
    input [7:0] byte_val;
    input       de_val;
    input       vs_val;
begin
    @(negedge pclk);
    pdata_i <= byte_val;
    de_i <= de_val;
    vs_i <= vs_val;
end
endtask

task drive_pixel;
    input [15:0] pix;
begin
    drive_byte(pix[15:8], 1'b1, 1'b1);
    drive_byte(pix[7:0],  1'b1, 1'b1);
end
endtask

task drive_frame;
    integer y;
    integer x;
    integer pix_idx;
begin
    drive_byte(8'h00, 1'b0, 1'b1);
    for (y = 0; y < V_NUM; y = y + 1) begin
        for (x = 0; x < H_NUM; x = x + 1) begin
            pix_idx = y * H_NUM + x;
            drive_pixel(pixel_word(pix_idx));
        end
        drive_byte(8'h00, 1'b0, 1'b1);
        drive_byte(8'h00, 1'b0, 1'b1);
    end
    drive_byte(8'h00, 1'b0, 1'b0);
    drive_byte(8'h00, 1'b0, 1'b0);
    drive_byte(8'h00, 1'b0, 1'b0);
    drive_byte(8'h00, 1'b0, 1'b0);
end
endtask

assign ddr_wrdy = 1'b1;
assign ddr_rrdy = 1'b1;
assign ddr_wdone = wr_done_r;
assign ddr_wdata_req = wr_data_req_r;
assign ddr_rdone = rd_done_r;
assign ddr_rdata_en = rd_data_en_r;
assign ddr_rdata = rd_data_r;

cmos_8_16bit u_cmos_8_16bit (
    .pclk      (pclk),
    .rst_n     (rst_n),
    .de_i      (de_i),
    .pdata_i   (pdata_i),
    .vs_i      (vs_i),
    .pixel_clk (pixel_clk),
    .de_o      (de_o),
    .pix_vld_o (pix_vld_o),
    .vs_o      (vs_o),
    .pdata_o   (pdata_o)
);

wr_buf #(
    .ADDR_WIDTH      (CTRL_ADDR_WIDTH),
    .ADDR_OFFSET     (0),
    .H_NUM           (H_NUM),
    .V_NUM           (V_NUM),
    .DQ_WIDTH        (DQ_WIDTH),
    .LEN_WIDTH       (LEN_WIDTH),
    .PIX_WIDTH       (16),
    .LINE_ADDR_WIDTH (LINE_ADDR_WIDTH),
    .FRAME_CNT_WIDTH (8)
) u_wr_buf (
    .ddr_clk         (ddr_clk),
    .ddr_rstn        (rst_n),
    .wr_clk          (pclk),
    .wr_fsync        (vs_o),
    .wr_en           (de_o),
    .wr_data_vld     (pix_vld_o),
    .wr_data         (pdata_o),
    .rd_bac          (1'b0),
    .ddr_wreq        (ddr_wreq),
    .ddr_waddr       (ddr_waddr),
    .ddr_wr_len      (ddr_wr_len),
    .ddr_wrdy        (ddr_wrdy),
    .ddr_wdone       (ddr_wdone),
    .ddr_wdata       (ddr_wdata),
    .ddr_wdata_req   (ddr_wdata_req),
    .frame_wcnt      (frame_wcnt),
    .frame_wirq      ()
);

rd_buf #(
    .ADDR_WIDTH      (CTRL_ADDR_WIDTH),
    .ADDR_OFFSET     (0),
    .H_NUM           (H_NUM),
    .V_NUM           (V_NUM),
    .DQ_WIDTH        (DQ_WIDTH),
    .LEN_WIDTH       (LEN_WIDTH),
    .PIX_WIDTH       (16),
    .LINE_ADDR_WIDTH (LINE_ADDR_WIDTH),
    .FRAME_CNT_WIDTH (8)
) u_rd_buf (
    .ddr_clk         (ddr_clk),
    .ddr_rstn        (rst_n),
    .vout_clk        (vout_clk),
    .rd_fsync        (rd_fsync),
    .rd_en           (rd_en),
    .vout_de         (vout_de),
    .vout_data       (vout_data),
    .o_data_ready    (rd_data_ready),
    .init_done       (rst_n),
    .i_wr_frame_idx  (2'd2),
    .ddr_rreq        (ddr_rreq),
    .ddr_raddr       (ddr_raddr),
    .ddr_rd_len      (ddr_rd_len),
    .ddr_rrdy        (ddr_rrdy),
    .ddr_rdone       (ddr_rdone),
    .ddr_rdata       (ddr_rdata),
    .ddr_rdata_en    (ddr_rdata_en)
);

always #5 pclk = ~pclk;
always #4 ddr_clk = ~ddr_clk;
always #4 vout_clk = ~vout_clk;

always @(posedge ddr_clk or negedge rst_n) begin
    if (!rst_n) begin
        wr_busy <= 1'b0;
        wr_addr_cur <= {CTRL_ADDR_WIDTH{1'b0}};
        wr_beats_left <= 0;
        wr_done_r <= 1'b0;
        wr_data_req_r <= 1'b0;
        rd_busy <= 1'b0;
        rd_addr_cur <= {CTRL_ADDR_WIDTH{1'b0}};
        rd_beats_left <= 0;
        rd_done_r <= 1'b0;
        rd_data_en_r <= 1'b0;
        rd_data_r <= {DQ_WIDTH*8{1'b0}};
        for (i = 0; i < 2048; i = i + 1)
            ddr_mem[i] <= 128'd0;
    end else begin
        wr_done_r <= 1'b0;
        wr_data_req_r <= 1'b0;
        rd_done_r <= 1'b0;
        rd_data_en_r <= 1'b0;
        rd_data_r <= {DQ_WIDTH*8{1'b0}};

        if (!wr_busy && ddr_wreq) begin
            wr_busy <= 1'b1;
            wr_addr_cur <= ddr_waddr;
            wr_beats_left <= ddr_wr_len;
        end else if (wr_busy) begin
            wr_data_req_r <= 1'b1;
            ddr_mem[wr_addr_cur[CTRL_ADDR_WIDTH-1:3]] <= ddr_wdata;
            wr_addr_cur <= wr_addr_cur + DDR_BEAT_ADDR_INC;
            if (wr_beats_left <= 1) begin
                wr_busy <= 1'b0;
                wr_beats_left <= 0;
                wr_done_r <= 1'b1;
            end else begin
                wr_beats_left <= wr_beats_left - 1;
            end
        end

        if (!rd_busy && ddr_rreq) begin
            rd_busy <= 1'b1;
            rd_addr_cur <= ddr_raddr;
            rd_beats_left <= ddr_rd_len;
        end else if (rd_busy) begin
            rd_data_en_r <= 1'b1;
            rd_data_r <= ddr_mem[rd_addr_cur[CTRL_ADDR_WIDTH-1:3]];
            rd_addr_cur <= rd_addr_cur + DDR_BEAT_ADDR_INC;
            if (rd_beats_left <= 1) begin
                rd_busy <= 1'b0;
                rd_beats_left <= 0;
                rd_done_r <= 1'b1;
            end else begin
                rd_beats_left <= rd_beats_left - 1;
            end
        end
    end
end

initial begin
    pclk = 1'b0;
    ddr_clk = 1'b0;
    vout_clk = 1'b0;
    rst_n = 1'b0;
    de_i = 1'b0;
    vs_i = 1'b0;
    pdata_i = 8'd0;
    rd_fsync = 1'b0;
    rd_en = 1'b0;
    word_count = 0;
    timeout_cnt = 0;

    #40;
    rst_n = 1'b1;

    drive_frame();

    repeat (16) @(posedge vout_clk);
    rd_fsync <= 1'b1;
    @(posedge vout_clk);
    rd_fsync <= 1'b0;

    while (!rd_data_ready) begin
        @(posedge vout_clk);
        timeout_cnt = timeout_cnt + 1;
        if (timeout_cnt > 4000) begin
            $display("ERROR: rd_data_ready did not assert");
            $fatal;
        end
    end

    rd_en <= 1'b1;
    while (word_count < CHECK_WORDS) begin
        @(posedge vout_clk);
        if (vout_de) begin
            if ((word_count == 0) && (vout_data == 128'd0)) begin
                // rd_fram_buf is synchronous; the first vout_de can still carry the reset word.
            end else begin
                if (vout_data !== expected_word(word_count)) begin
                    $display("ERROR: word%0d mismatch got=%032x exp=%032x",
                             word_count, vout_data, expected_word(word_count));
                    $fatal;
                end
                word_count = word_count + 1;
            end
        end
    end

    rd_en <= 1'b0;
    repeat (8) @(posedge vout_clk);
    $display("PASS: DVP 8-bit -> BGR565 16-bit -> DDR write/read returned the expected first %0d words.", CHECK_WORDS);
    $finish;
end

endmodule
