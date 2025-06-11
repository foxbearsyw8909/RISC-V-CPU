module Mux_3to1 (
    input [31:0] mux_in1,
    input [31:0] mux_in2,
    input [31:0] mux_in3,

    input [1:0] sel,

    output [31:0] mux_out);

    assign mux_out = (sel == 2'b01) ? mux_in1 :
                    (sel == 2'b10) ? mux_in2 :
                    (sel == 2'b11) ? mux_in3 : 32'b0;

   
endmodule
