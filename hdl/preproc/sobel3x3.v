`timescale 1ns / 1ps

module sobel3x3 (
    input               clk,
    input               rst_n,
    input               i_vld,
    input       [7:0]   i_pix,
    output  reg [7:0]   o_mag
);
    reg [7:0] p0;
    reg [7:0] p1;
    reg [7:0] p2;
    reg [8:0] gx_abs;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            p0 <= 8'd0;
            p1 <= 8'd0;
            p2 <= 8'd0;
            o_mag <= 8'd0;
            gx_abs <= 9'd0;
        end else if (i_vld) begin
            p2 <= p1;
            p1 <= p0;
            p0 <= i_pix;

            if (p2 >= i_pix)
                gx_abs <= {1'b0, p2} - {1'b0, i_pix};
            else
                gx_abs <= {1'b0, i_pix} - {1'b0, p2};

            o_mag <= gx_abs[8] ? 8'hFF : gx_abs[7:0];
        end
    end
endmodule
