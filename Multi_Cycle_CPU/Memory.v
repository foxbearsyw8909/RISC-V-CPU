module Memory #(parameter MEM_DEPTH = 16384) (input reset,
                                              input clk,
                                              input [31:0] addr,    // address of the memory
                                              input [31:0] din,     // data to be written
                                              input mem_read,       // is read signal driven?
                                              input mem_write,      // is write signal driven?
                                              output [31:0] dout);  // output of the data memory at addr
  integer i;
  // Memory
  reg [31:0] mem[0: MEM_DEPTH - 1];
  // Do not touch mem_addr
  wire [31:0] mem_addr;
  assign mem_addr = addr >> 2;

  // Asynchrnously read data from the memory
  assign dout = (mem_read) ? mem[mem_addr] : 32'b0;

  always @(posedge clk) begin
    // Initialize data memory (do not touch)
    if (reset) begin
      for (i = 0; i < MEM_DEPTH; i = i + 1)
        // DO NOT TOUCH COMMENT BELOW
        /* verilator lint_off BLKSEQ */
        mem[i] = 32'b0;
        /* verilator lint_on BLKSEQ */
        // DO NOT TOUCH COMMENT ABOVE
      // Provide path of the file including instructions with binary format
      $readmemh("/home/ymlee/CSED311-lab3-/Lab3/student_tb/non-controlflow_mem.txt", mem);
    end

    // Synchronously write data to the memory
    else begin
      if (mem_write) mem[mem_addr] <= din;
      else mem[mem_addr] <= mem[mem_addr];
    end
  end
endmodule
