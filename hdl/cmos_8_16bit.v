`timescale 1ns / 1ps

// Convert OV5640 8-bit bus to 16-bit RGB565 stream.
// No device-specific clock primitives are used.
module cmos_8_16bit #(
    parameter CAPTURE_ON_NEGEDGE = 1'b0
) (
    input               pclk,
    input               rst_n,
    input               de_i,
    input       [7:0]   pdata_i,
    input               vs_i,
    output reg          pixel_clk,
    output reg          de_o,
    output reg          pix_vld_o,
    output reg  [15:0]  pdata_o
);

reg [7:0]   pdata_cap;
reg         de_cap;
reg         vs_cap;
wire [7:0]  pdata_src;
wire        de_src;
wire        vs_src;
reg         de_src_d;
reg         vs_src_d;
reg         byte_phase;
reg [7:0]   byte_hi;

wire line_start = (~de_src_d) & de_src;
wire frame_start = (~vs_src_d) & vs_src;

generate
    if (CAPTURE_ON_NEGEDGE) begin : g_negedge_capture
        always @(negedge pclk or negedge rst_n) begin
            if (!rst_n) begin
                pdata_cap <= 8'd0;
                de_cap    <= 1'b0;
                vs_cap    <= 1'b0;
            end else begin
                pdata_cap <= pdata_i;
                de_cap    <= de_i;
                vs_cap    <= vs_i;
            end
        end

        assign pdata_src = pdata_cap;
        assign de_src = de_cap;
        assign vs_src = vs_cap;
    end else begin : g_posedge_direct
        assign pdata_src = pdata_i;
        assign de_src = de_i;
        assign vs_src = vs_i;
    end
endgenerate

always @(posedge pclk or negedge rst_n) begin
    if (!rst_n) begin
        pixel_clk  <= 1'b0;
        de_src_d   <= 1'b0;
        vs_src_d   <= 1'b0;
        byte_phase <= 1'b0;
        byte_hi    <= 8'd0;
        de_o       <= 1'b0;
        pix_vld_o  <= 1'b0;
        pdata_o    <= 16'd0;
    end else begin
        pixel_clk <= ~pixel_clk;
        de_src_d <= de_src;
        vs_src_d <= vs_src;
        de_o <= de_src;
        pix_vld_o <= 1'b0;

        if (frame_start) begin
            byte_phase <= 1'b0;
        end else if (!de_src) begin
            byte_phase <= 1'b0;
        end else if (line_start || !byte_phase) begin
            byte_hi <= pdata_src;
            byte_phase <= 1'b1;
        end else begin
            pdata_o <= {byte_hi, pdata_src};
            pix_vld_o <= 1'b1;
            byte_phase <= 1'b0;
        end
    end
end

endmodule
