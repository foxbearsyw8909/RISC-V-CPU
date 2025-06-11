module Add (
  input [31:0] adder_in1,
  input [31:0] adder_in2,

  output [31:0] adder_out);

  assign adder_out = (adder_in1 + adder_in2);

endmodule

