module add (
  input [31:0] adder_in1,
  input [31:0] adder_in2,

  output [31:0] adder_o);

  assign adder_o = (adder_in1 + adder_in2);

endmodule

