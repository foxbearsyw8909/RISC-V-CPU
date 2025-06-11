module Mux_2to1_CU (
    input [6:0] mux_in0,
    input [6:0] mux_in1,
    input sel,

    output [6:0] mux_out);

    assign mux_out = sel ? mux_in1 : mux_in0;
   
endmodule
