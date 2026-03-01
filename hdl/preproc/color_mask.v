`timescale 1ns / 1ps

module color_mask (
    input       [7:0]   i_r,
    input       [7:0]   i_g,
    input       [7:0]   i_b,
    output  reg [1:0]   o_color_class
);
    wire [8:0] r9 = {1'b0, i_r};
    wire [8:0] g9 = {1'b0, i_g};
    wire [8:0] b9 = {1'b0, i_b};

    always @(*) begin
        if ((i_r > 8'd150) && (r9 > (g9 + 9'd24)) && (r9 > (b9 + 9'd24)))
            o_color_class = 2'b11;    // red
        else if ((i_b > 8'd110) && (b9 > (g9 + 9'd16)) && (b9 > (r9 + 9'd16)))
            o_color_class = 2'b01;    // blue
        else if ((i_g > 8'd90) && (i_r > 8'd70) && (i_b < 8'd140))
            o_color_class = 2'b10;    // yellow/green
        else
            o_color_class = 2'b00;    // none
    end
endmodule
