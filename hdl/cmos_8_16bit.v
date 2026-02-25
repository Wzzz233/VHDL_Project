`timescale 1ns / 1ps

// Convert OV5640 8-bit bus to 16-bit RGB565 stream.
// No device-specific clock primitives are used.
module cmos_8_16bit (
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

reg         vs_i_d;
reg         byte_phase;
reg [7:0]   byte_hi;

always @(posedge pclk or negedge rst_n) begin
    if (!rst_n) begin
        pixel_clk  <= 1'b0;
        vs_i_d     <= 1'b0;
        byte_phase <= 1'b0;
        byte_hi    <= 8'd0;
        de_o       <= 1'b0;
        pix_vld_o  <= 1'b0;
        pdata_o    <= 16'd0;
    end else begin
        pixel_clk <= ~pixel_clk;
        vs_i_d <= vs_i;
        de_o <= de_i;
        pix_vld_o <= 1'b0;

        if ((~vs_i_d) && vs_i) begin
            byte_phase <= 1'b0;
        end else if (!de_i) begin
            byte_phase <= 1'b0;
        end else if (!byte_phase) begin
            byte_hi <= pdata_i;
            byte_phase <= 1'b1;
        end else begin
            pdata_o <= {byte_hi, pdata_i};
            pix_vld_o <= 1'b1;
            byte_phase <= 1'b0;
        end
    end
end

endmodule
