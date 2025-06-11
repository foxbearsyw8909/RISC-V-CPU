module mux (
   input [31:0] mux_in1,
   input [31:0] mux_in2,
   input sel,

   output [31:0] mux_o);

   assign mux_o = sel ? mux_in1 : mux_in2;
   
endmodule
