`timescale 1ns / 1ps

// Compatibility wrapper for the legacy OV5640 register sequencer.
module i2c_com(
    input               clock_i2c,
    input               camera_rstn,
    output              ack,
    input      [31:0]   i2c_data,
    input               start,
    output              tr_end,
    output              i2c_sclk,
    inout               i2c_sdat
);

wire iic_busy;
wire iic_byte_over_unused;
wire [7:0] iic_data_out_unused;
wire iic_sda_out;
wire iic_sda_oe;
reg  iic_busy_d;

assign ack = 1'b0;
assign i2c_sdat = iic_sda_oe ? iic_sda_out : 1'bz;
assign tr_end = iic_busy_d & ~iic_busy;

always @(posedge clock_i2c or negedge camera_rstn) begin
    if (!camera_rstn)
        iic_busy_d <= 1'b0;
    else
        iic_busy_d <= iic_busy;
end

iic_dri #(
    .CLK_FRE(20_000),
    .IIC_FREQ(10_000),
    .T_WR(1),
    .ADDR_BYTE(2),
    .LEN_WIDTH(3),
    .DATA_BYTE(1)
) u_iic_dri (
    .clk(clock_i2c),
    .rstn(camera_rstn),
    .pluse(start),
    .device_id(i2c_data[31:24]),
    .w_r(1'b1),
    .byte_len(4'd1),
    .addr(i2c_data[23:8]),
    .data_in(i2c_data[7:0]),
    .busy(iic_busy),
    .byte_over(iic_byte_over_unused),
    .data_out(iic_data_out_unused),
    .scl(i2c_sclk),
    .sda_in(i2c_sdat),
    .sda_out(iic_sda_out),
    .sda_out_en(iic_sda_oe)
);

endmodule
