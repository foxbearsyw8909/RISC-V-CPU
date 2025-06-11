// Submit this file with other files you created.
// Do not touch port declarations of the module 'CPU'.

// Guidelines
// 1. It is highly recommened to `define opcodes and something useful.
// 2. You can modify the module.
// (e.g., port declarations, remove modules, define new modules, ...)
// 3. You might need to describe combinational logics to drive them into the module (e.g., mux, and, or, ...)
// 4. `include files if required
module cpu(input reset,       // positive reset signal
           input clk,         // clock signal
           output is_halted,
           output [31:0]print_reg[0:31]
           ); // Whehther to finish simulation
  /***** Wire declarations *****/
  wire [31:0] next_pc;
  wire [31:0] current_pc;

  // wires for Memory
  wire [31:0] mem_addr;
  wire [31:0] mem_dout;

  // wires for RegisterFile
  wire [31:0] rd_din;
  wire [31:0] rs1_dout;
  wire [31:0] rs2_dout;

  // wire for ImmediateGenerator
  wire [31:0] imm;

  // wires for ALU and ALUControlUnit
  wire [31:0] alu_result;
  wire alu_bcond;
  wire [3:0] alu_control_out;

  // wires for MUX
  wire [31:0] mux_ALU_A_out;
  wire [31:0] mux_ALU_B_out;

  // wires for control signals
  wire PCWriteCond;
  wire PCWrite;
  wire IorD;
  wire MemRead;
  wire MemWrite;
  wire MemtoReg;
  wire IRWrite;
  wire PCSource; // 2-bit signal in the textbook
  wire [1:0] ALUOp;
  wire [1:0] ALUSrcB;
  wire ALUSrcA;
  wire RegWrite;
  wire is_ecall;
  
  /***** Register declarations *****/
  reg [31:0] IR; // instruction register
  reg [31:0] MDR; // memory data register
  reg [31:0] A; // Read 1 data register
  reg [31:0] B; // Read 2 data register
  reg [31:0] ALUOut; // ALU output register
  // Do not modify and use registers declared above.

  wire [31:0] IR_out_wire;
  wire [31:0] MDR_out_wire;
  wire [31:0] A_out_wire;
  wire [31:0] B_out_wire;
  wire [31:0] ALUOut_out_wire;
  wire [31:0] read_rs1;

  assign IR_out_wire = IR;
  assign MDR_out_wire = MDR;
  assign A_out_wire = A;
  assign B_out_wire = B;
  assign ALUOut_out_wire = ALUOut;

  // wires for State Register
  wire [3:0] current_state;
  wire [3:0] next_state;

  StateRegister state_reg (
      .clk(clk),
      .reset(reset),
      .next_state(next_state),
      .current_state(current_state)
  );

  // ---------- Update program counter ----------
  // PC must be updated on the rising edge (positive edge) of the clock.
  PC pc(
    .reset(reset),       // input (Use reset to initialize PC. Initial value must be 0)
    .clk(clk),         // input
    .next_pc(next_pc),     // input
    .pc_update(PCWrite || (PCWriteCond && alu_bcond)),   // input
    .current_pc(current_pc)   // output
  );

  // ---------- Register File ----------
  RegisterFile reg_file(
    .reset(reset),        // input
    .clk(clk),          // input
    .rs1(read_rs1[19:15]),          // input*
    .rs2(IR_out_wire[24:20]),          // input
    .rd(IR_out_wire[11:7]),           // input
    .rd_din(rd_din),       // input
    .write_enable(RegWrite),    // input
    .rs1_dout(rs1_dout),     // output
    .rs2_dout(rs2_dout),      // output
    .print_reg(print_reg)     // output (TO PRINT REGISTER VALUES IN TESTBENCH)
  );

  // ---------- Memory ----------
  Memory memory(
    .reset(reset),        // input
    .clk(clk),          // input
    .addr(mem_addr),         // input
    .din(B_out_wire),          // input (Write data)
    .mem_read(MemRead),     // input
    .mem_write(MemWrite),    // input
    .dout(mem_dout)   // output
  );

  // ---------- Control Unit ----------
  ControlUnit ctrl_unit(
    .part_of_inst(IR_out_wire[6:0]),  // input
    .current_state(current_state),   // input
    .PCWriteCond(PCWriteCond),    // output
    .PCWrite(PCWrite),       // output
    .IorD(IorD),        // output
    .MemRead(MemRead),      // output
    .MemWrite(MemWrite),    // output
    .MemtoReg(MemtoReg),     // output
    .IRWrite(IRWrite),       // output
    .PCSource(PCSource),     // output
    .ALUOp(ALUOp),     // output
    .ALUSrcB(ALUSrcB),   // output
    .ALUSrcA(ALUSrcA),   // output
    .RegWrite(RegWrite),    // output
    .is_ecall(is_ecall)       // output (ecall inst)
  );

  // ---------- Microcode Controller ----------
  MicrocodeController microcode_ctrl(
    .clk(clk),      // input
    .reset(reset),      // input
    .part_of_inst(IR_out_wire[6:0]),      // input
    .current_state(current_state),      // input
    .next_state(next_state)       //output
  );

  // ---------- Immediate Generator ----------
  ImmediateGenerator imm_gen(
    .part_of_inst(IR_out_wire),  // input
    .imm_gen_out(imm)    // output
  );

  // ---------- ALU Control Unit ----------
  ALUControlUnit alu_ctrl_unit(
    .part_of_inst(IR_out_wire),  // input
    .ALUOp(ALUOp),    // input
    .alu_op(alu_control_out)    // output
  );

  // ---------- ALU ----------
  ALU alu(
    .alu_op(alu_control_out),      // input
    .alu_in_1(mux_ALU_A_out),    // input  
    .alu_in_2(mux_ALU_B_out),    // input
    .alu_result(alu_result),  // output
    .alu_bcond(alu_bcond)     // output
  );

  // ---------- Mux 2 to 1 ----------
  Mux_2to1 mux_Reg(
    .mux_in0(ALUOut_out_wire),   // input
    .mux_in1(MDR_out_wire),   // input
    .sel(MemtoReg),   //input
    .mux_out(rd_din)    // output
  );

  Mux_2to1 mux_Mem(
    .mux_in0(current_pc),   // input
    .mux_in1(ALUOut_out_wire),   // input
    .sel(IorD),   //input
    .mux_out(mem_addr)    // output
  );

  Mux_2to1 mux_ALU_A(
    .mux_in0(current_pc),   // input
    .mux_in1(A_out_wire),   // input
    .sel(ALUSrcA),   //input
    .mux_out(mux_ALU_A_out)    // output
  );

  Mux_2to1 mux_PC(
    .mux_in0(alu_result),   // input
    .mux_in1(ALUOut_out_wire),   // input
    .sel(PCSource),   //input
    .mux_out(next_pc)    // output
  );

  // ---------- Mux 3 to 1 ----------
  Mux_3to1 mux_ALU_B(
    .mux_in0(B_out_wire),   // input
    .mux_in1(32'd4),   // input
    .mux_in2(imm),   // input
    .sel(ALUSrcB),   //input
    .mux_out(mux_ALU_B_out)    // output
  );

  always @(posedge clk) begin
    if (reset) begin
      IR <= 0;
      MDR <= 0;
      A <= 0;
      B <= 0;
      ALUOut <= 0;
    end

    else begin
      if(IRWrite) begin
        IR <= mem_dout;
        MDR <= mem_dout;
      end
      else begin
      MDR <= mem_dout;
      end
      A <= rs1_dout;
      B <= rs2_dout;
      ALUOut <= alu_result;
    end
  end
  
  assign read_rs1 = (is_ecall) ? 32'd17 << 15 : IR;

  assign is_halted = (is_ecall == 1'b1 && rs1_dout == 32'd10);

endmodule
