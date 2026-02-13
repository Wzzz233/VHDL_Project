`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Meyesemi
// Engineer: Nill
// 
// Create Date: 15/03/23 15:02:21
// Design Name: 
// Module Name: rd_buf
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: Modified for PCIe 128-bit Zero Copy
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
`define UD #1
module rd_buf #(
    parameter                     ADDR_WIDTH      = 6'd27,
    parameter                     ADDR_OFFSET     = 32'h0000_0000,
    parameter                     H_NUM           = 12'd1920,
    parameter                     V_NUM           = 12'd1080,
    parameter                     DQ_WIDTH        = 12'd32,
    parameter                     LEN_WIDTH       = 12'd16,
    parameter                     PIX_WIDTH       = 12'd24,
    parameter                     LINE_ADDR_WIDTH = 16'd19,
    parameter                     FRAME_CNT_WIDTH = 16'd8
)  (
    input                         ddr_clk,
    input                         ddr_rstn,
    
    input                         vout_clk,
    input                         rd_fsync,
    input                         rd_en,
    output                        vout_de,
    output [127:0]                vout_data, // Changed to 128-bit
    
    input                         init_done,
    input                         i_wr_frame_bit,
    
    output                        ddr_rreq,
    output [ADDR_WIDTH- 1'b1 : 0] ddr_raddr,
    output [LEN_WIDTH- 1'b1 : 0]  ddr_rd_len,
    input                         ddr_rrdy,
    input                         ddr_rdone,
    
    input [8*DQ_WIDTH- 1'b1 : 0]  ddr_rdata,
    input                         ddr_rdata_en 
);
    localparam SIM            = 1'b0;
    localparam RAM_WIDTH      = 128;
    localparam DDR_DATA_WIDTH = DQ_WIDTH * 8;
    localparam WR_LINE_NUM    = H_NUM * PIX_WIDTH / RAM_WIDTH;
    localparam RD_LINE_NUM    = WR_LINE_NUM * RAM_WIDTH / DDR_DATA_WIDTH;
    // Keep the same address unit conversion as the reference design.
    localparam DDR_ADDR_OFFSET = RD_LINE_NUM * DDR_DATA_WIDTH / DQ_WIDTH;
    
    //===========================================================================
    reg       rd_fsync_1d;
    reg       rd_en_1d,rd_en_2d;
    wire      rd_rst;
    reg       ddr_rstn_1d,ddr_rstn_2d;
    always @(posedge vout_clk)
    begin
        rd_fsync_1d <= rd_fsync;
        rd_en_1d <= rd_en; 
        rd_en_2d <= rd_en_1d;
        ddr_rstn_1d <= ddr_rstn;
        ddr_rstn_2d <= ddr_rstn_1d;
    end 
    assign rd_rst = ~rd_fsync_1d &rd_fsync;
    
    //===========================================================================
    reg      wr_fsync_1d,wr_fsync_2d,wr_fsync_3d;
    wire     wr_rst;
    
    reg      wr_trig;
    reg [11:0] wr_line;
    reg      ddr_rdone_d;
    wire     ddr_rdone_rise;
    always @(posedge ddr_clk)
    begin
        wr_fsync_1d <= rd_fsync;
        wr_fsync_2d <= wr_fsync_1d;
        wr_fsync_3d <= wr_fsync_2d;

        if(~ddr_rstn)
            wr_trig <= 1'b0;
        else
            // wr_line starts from 1, so use "< V_NUM" to avoid one extra line.
            wr_trig <= wr_rst || (ddr_rdone_rise && (wr_line < V_NUM));
    end 
    
    always @(posedge ddr_clk)
    begin
        if(~ddr_rstn)
            ddr_rdone_d <= 1'b0;
        else
            ddr_rdone_d <= ddr_rdone;
    end
    
    assign ddr_rdone_rise = ddr_rdone & ~ddr_rdone_d;

    always @(posedge ddr_clk)
    begin
        if(wr_rst || (~ddr_rstn))
            wr_line <= 12'd1;
        else if(wr_trig)
            wr_line <= wr_line + 12'd1;
    end 
    
    assign wr_rst = ~wr_fsync_3d && wr_fsync_2d;
    
    //==========================================================================
    reg locked_frame_bit;
    always @(posedge ddr_clk)
    begin 
        if(~ddr_rstn)
            locked_frame_bit <= 1'b0;
        else if(wr_rst)
            // Lock to the opposite bank so read side uses a completed frame.
            locked_frame_bit <= ~i_wr_frame_bit;
        else
            locked_frame_bit <= locked_frame_bit;
    end 

    reg [LINE_ADDR_WIDTH - 1'b1 :0] wr_cnt;
    always @(posedge ddr_clk)
    begin 
        if(wr_rst)
            wr_cnt <= 10'd0;
        else if(ddr_rdone_rise)
            wr_cnt <= wr_cnt + DDR_ADDR_OFFSET;
        else
            wr_cnt <= wr_cnt;
    end 
    
    assign ddr_rreq = wr_trig;
    assign ddr_raddr = {locked_frame_bit,wr_cnt} + ADDR_OFFSET;
    assign ddr_rd_len = RD_LINE_NUM;
    
    reg  [ 9:0]           wr_addr;
    reg  [ 9:0]           rd_addr; // Changed to 10-bit for 128-bit width
    wire [127:0]          rd_data; // Changed to 128-bit
    
    //===========================================================================
    always @(posedge ddr_clk)
    begin
        if(wr_rst)
            wr_addr <= (SIM == 1'b1) ? 10'd360 : 10'd0;
        else if(ddr_rdata_en)
            wr_addr <= wr_addr + 10'd1;
        else
            wr_addr <= wr_addr;
    end 

    rd_fram_buf rd_fram_buf (
        .a_wr_data   (  ddr_rdata       ),// input [127:0]            
        .a_addr      (  wr_addr         ),// input [9:0]              
        .a_rd_data   (                  ),
        .a_wr_en     (  ddr_rdata_en    ),// input                    
        .a_clk       (  ddr_clk         ),// input                    
        .a_rst       (  ~ddr_rstn       ),// input                    
        .b_addr      (  rd_addr         ),// input [9:0]             
        .b_wr_data   (  128'd0          ),
        .b_rd_data   (  rd_data         ),// output [127:0]            
        .b_wr_en     (  1'b0            ),
        .b_clk       (  vout_clk        ),// input                    
        .b_rst       (  ~ddr_rstn_2d    ) // input                    
    );
    
    // Simple read logic for 128-bit
    always @(posedge vout_clk)
    begin
        if(rd_rst)
            rd_addr <= 'd0;
        else if(rd_en)
            rd_addr <= rd_addr + 1'b1;
        else
            rd_addr <= rd_addr;
    end 
    
    assign vout_de = rd_en_2d;
    assign vout_data = rd_data;

endmodule
