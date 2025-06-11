// Submit this file with other files you created.
// Do not touch port declarations of the module 'cpu'.

// Guidelines
// 1. It is highly recommened to `define opcodes and something useful.
// 2. You can modify the module.
// (e.g., port declarations, remove modules, define new modules, ...)
// 3. You might need to describe combinational logics to drive them into the module (e.g., mux, and, or, ...)
// 4. `include files if required
`include "opcodes.v"
`include "alu_opcodes.v"

module cpu(input reset,                     // positive reset signal
           input clk,                       // clock signal
           output is_halted,                // Whehther to finish simulation
           output [31:0] print_reg [0:31]); // TO PRINT REGISTER VALUES IN TESTBENCH (YOU SHOULD NOT USE THIS)
  /***** Wire declarations *****/
  wire [31:0] inst; // 32-bit instruction
  wire [31:0] next_pc;
  wire [31:0] current_pc;
  wire [31:0] imm; // generated 32-bit immediate value

  // wires for register file
  wire [31:0] rs1_dout;
  wire [31:0] rs2_dout;

  // wires for alu contorl unit & alu
  wire [3:0] alu_op;
  wire alu_bcond;
  wire [31:0] alu_result;

  // wire for data memory
  wire [31:0] mem_dout;

  // wires for control signals
  wire is_jal; // JAL
  wire is_jalr; // JALR (== PCSrc2)
  wire branch; // Branch
  wire mem_read; // MemRead
  wire mem_to_reg; //MemtoReg
  wire mem_write; // MemWrite
  wire alu_src; // ALUSrc
  wire write_enable; // RegWrite
  wire pc_to_reg; // PCtoReg
  wire is_ecall; // ECALL

  // wires for mux outputs
  wire [31:0] mux_alu_out;
  wire [31:0] mux_WD_out;
  wire [31:0] mux_WB_out;
  wire [31:0] mux_PC1_out;

  //wire for adders
  wire [31:0] add_PC_4_out;
  wire [31:0] add_PC_imm_out;

  /***** Register declarations *****/

  // ---------- Update program counter ----------
  // PC must be updated on the rising edge (positive edge) of the clock.

  pc pc(
    .reset(reset),       // input (Use reset to initialize PC. Initial value must be 0)
    .clk(clk),         // input
    .next_pc(next_pc),     // input
    .current_pc(current_pc)   // output
  );
  
  // ---------- Instruction Memory ----------
  instruction_memory imem(
    .reset(reset),   // input
    .clk(clk),     // input
    .addr(current_pc),    // input
    .dout(inst)     // output
  );

  // ---------- Register File ----------
  register_file reg_file (
    .reset (reset),        // input
    .clk (clk),          // input
    .rs1 (inst[19:15]),          // input, source register 1
    .rs2 (inst[24:20]),          // input, source register 2
    .rd (inst[11:7]),           // input, destination register
    .rd_din (mux_WD_out),       // input, input data for rd
    .write_enable (write_enable), // input, RegWrite signal
    .is_ecall (is_ecall),     // input, ECALL signal
    .rs1_dout (rs1_dout),     // output, output of rs1
    .rs2_dout (rs2_dout),     // output, output of rs2
    .print_reg (print_reg),  //DO NOT TOUCH THIS (X use)
    .is_halted(is_halted)     // output
  );

  // ---------- Control Unit ----------
  control_unit ctrl_unit (
    .part_of_inst(inst[6:0]),  // input
    .is_jal(is_jal),        // output
    .is_jalr(is_jalr),       // output
    .branch(branch),        // output
    .mem_read(mem_read),      // output
    .mem_to_reg(mem_to_reg),    // output
    .mem_write(mem_write),     // output
    .alu_src(alu_src),       // output
    .write_enable(write_enable),  // output
    .pc_to_reg(pc_to_reg),     // output
    .is_ecall(is_ecall)       // output (ecall inst)
  );

  // ---------- Immediate Generator ----------
  immediate_generator imm_gen(
    .part_of_inst(inst),  // input
    .imm_gen_out(imm)    // output
  );

  // ---------- ALU Control Unit ----------
  alu_control_unit alu_ctrl_unit (
    .part_of_inst(inst),  // input
    .alu_op(alu_op)         // output
  );

  // ---------- ALU ----------
  alu alu (
    .alu_op(alu_op),      // input
    .alu_in_1(rs1_dout),    // input
    .alu_in_2(mux_alu_out),    // input
    .alu_result(alu_result),  // output
    .alu_bcond(alu_bcond)    // output
  );

  // ---------- Data Memory ----------
  data_memory dmem(
    .reset (reset),      // input
    .clk (clk),        // input
    .addr (alu_result),       // input
    .din (rs2_dout),        // input
    .mem_read (mem_read),   // input (read signal)
    .mem_write (mem_write),  // input (write signal)
    .dout (mem_dout)        // output
  );



  //----------- we added -------------
  //------- Muxes and Adders -----------
  mux mux_WD(  //mux for write data in register file
    .mux_in1 (add_PC_4_out), // PCtoReg == 1
    .mux_in2 (mux_WB_out), // PCtoReg == 0
    .sel (pc_to_reg), // PCtoReg
    .mux_o (mux_WD_out) // output
  );

  mux mux_alu(  //mux for ALU
    .mux_in1 (imm), // ALUSrc == 1
    .mux_in2 (rs2_dout), // ALUSrc == 0
    .sel (alu_src), //ALUSrc
    .mux_o (mux_alu_out) // output
  );

  mux mux_WB( //mux for write back (after data memory)
    .mux_in1 (mem_dout), // MemtoReg == 1
    .mux_in2 (alu_result), // MemtoReg == 0
    .sel (mem_to_reg), // MemtoReg
    .mux_o (mux_WB_out) // output
  );

  mux mux_PC1( //mux for calculate pc1
    .mux_in1 (add_PC_imm_out), // PCSrc1 == 1
    .mux_in2 (add_PC_4_out), // PCSrc1 == 0
    .sel (is_jal || (branch && alu_bcond)), // PCSrc1
    .mux_o (mux_PC1_out) // output
  );

  mux mux_PC2( //mux for calculate pc2
    .mux_in1 (alu_result), // PCSrc2 == 1
    .mux_in2 (mux_PC1_out), // PCSrc2 == 0
    .sel (is_jalr), // PCSrc2
    .mux_o (next_pc) // output (to PC)
  );

  add add_PC_4 ( // PC <- PC + 4
    .adder_in1 (current_pc),
    .adder_in2 (32'd4),
    .adder_o (add_PC_4_out) // output
  );

  add add_PC_imm ( // PC <- PC + imm
    .adder_in1 (current_pc),
    .adder_in2 (imm),
    .adder_o (add_PC_imm_out) // output
  );

endmodule
