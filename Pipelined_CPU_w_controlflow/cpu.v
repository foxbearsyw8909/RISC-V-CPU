// Submit this file with other files you created.
// Do not touch port declarations of the module 'CPU'.

// Guidelines
// 1. It is highly recommened to `define opcodes and something useful.
// 2. You can modify modules (except InstMemory, DataMemory, and RegisterFile)
// (e.g., port declarations, remove modules, define new modules, ...)
// 3. You might need to describe combinational logics to drive them into the module (e.g., mux, and, or, ...)
// 4. `include files if required
`include "opcodes.v"
`include "alu_opcodes.v"

module cpu(input reset,       // positive reset signal
           input clk,         // clock signal
           output is_halted, // Whehther to finish simulation
           output [31:0]print_reg[0:31]); // Whehther to finish simulation
  /***** Wire declarations *****/
  // wires for Instruction Memory
  wire [31:0] inst; // 32-bit instruction

  // wires for PC
  wire [31:0] next_pc;
  wire [31:0] current_pc;

  // wire for Branch Predictor
  wire [31:0] pred_pc;
  wire btb_hit;
  wire [31:0] btb_target_pc;
  wire taken;
  //wire [31:0] real_pc;

  // wires for Hazard Detection Unit
  wire Hazard;    // input signal for mux_CU
  wire PCWrite;
  wire IF_ID_Write;

  // wires for Registers
  wire [31:0] rs1_dout;
  wire [31:0] rf_rs1_dout;
  wire [31:0] rs2_dout;

  // wires for Immediate Generator
  wire [31:0] imm;

  // wires for Control Unit signals
  wire branch;
  wire is_jalr;
  wire is_jal;
  wire MemRead;
  wire MemtoReg;
  wire MemWrite;
  wire ALUSrc;
  wire RegWrite;
  wire PCtoReg;
  wire is_ecall;
  
  wire is_flush;

  wire [9:0] TotalControl;

  // wires for ALU & ALUcontrol
  wire [3:0] alu_op;    // output of ALUControlUnit
  wire [31:0] alu_result;
  wire bcond;

  // wires for Forwarding unit
  wire [1:0] ForwardA;
  wire [1:0] ForwardB;

  // wire for Data Memory
  wire [31:0] mem_dout;

  // wires for muxes
  wire [31:0] mux_WB_out;
  wire [31:0] mux_Jump_WB_out;
  wire [31:0] mux_ALU_out;
  wire [31:0] mux_PC_out;
  wire [4:0] mux_rs1_out;
  wire [9:0] mux_CU_out;

  wire [31:0] mux_ALU_A_out;
  wire [31:0] mux_ALU_B_out;
  wire [31:0] mux_ALU_ForwardB_out;

  wire [1:0] PCSrc;
  wire [31:0] real_pc;

  // wires for adders
  wire [31:0] adder_PC_imm_out;
  wire [31:0] adder_PC_4_out;

  /***** Register declarations *****/
  // You need to modify the width of registers
  // In addition, 
  // 1. You might need other pipeline registers that are not described below
  // 2. You might not need registers described below
  /***** IF/ID pipeline registers *****/
  reg [31:0] IF_ID_inst;           // will be used in ID stage
  reg [31:0] IF_ID_PC;    // added
  reg [31:0] IF_ID_pred_pc;   // added

  /***** ID/EX pipeline registers *****/
  // From the control unit
  // reg ID_EX_alu_op;         // will be used in EX stage (not used in Lab4)
  reg ID_EX_alu_src;        // will be used in EX stage
  reg ID_EX_mem_write;      // will be used in MEM stage
  reg ID_EX_mem_read;       // will be used in MEM stage
  reg ID_EX_mem_to_reg;     // will be used in WB stage
  reg ID_EX_reg_write;      // will be used in WB stage
  reg ID_EX_is_halted;    // added
  reg ID_EX_branch;   // added
  reg ID_EX_is_jalr;    // added
  reg ID_EX_is_jal;   // added
  // From others
  reg [31:0] ID_EX_PC;
  reg [31:0] ID_EX_rs1_data;
  reg [31:0] ID_EX_rs2_data;
  reg [31:0] ID_EX_imm;
  reg [31:0] ID_EX_ALU_ctrl_unit_input;
  reg [4:0] ID_EX_rd_idx;   // transfers inst[11:7]
  reg [4:0] ID_EX_rs1_idx;    // added
  reg [4:0] ID_EX_rs2_idx;    // added
  reg [31:0] ID_EX_pred_pc;   // added
  

  /***** EX/MEM pipeline registers *****/
  // From the control unit
  reg EX_MEM_mem_write;     // will be used in MEM stage
  reg EX_MEM_mem_read;      // will be used in MEM stage
  reg EX_MEM_mem_to_reg;    // will be used in WB stage
  reg EX_MEM_reg_write;     // will be used in WB stage
  reg EX_MEM_bcond;   // added
  reg EX_MEM_is_halted;    // added 
  reg EX_MEM_is_jal;    // added
  reg EX_MEM_is_jalr;   // added
  // From others
  reg [31:0] EX_MEM_alu_out;
  reg [31:0] EX_MEM_dmem_data;
  reg [4:0] EX_MEM_rd_idx;
  reg [31:0] EX_MEM_PC_imm;    // added
  reg [31:0] EX_MEM_PC;   // added

  /***** MEM/WB pipeline registers *****/
  // From the control unitS
  reg MEM_WB_mem_to_reg;    // will be used in WB stage
  reg MEM_WB_reg_write;     // will be used in WB stage
  reg MEM_WB_is_halted;    // added
  reg MEM_WB_is_jal;    // added
  reg MEM_WB_is_jalr;   // added
  // From others
  reg [31:0] MEM_WB_mem_to_reg_src_1;
  reg [31:0] MEM_WB_mem_to_reg_src_2;
  reg [4:0] MEM_WB_rd_idx;    // added
  reg [31:0] MEM_WB_PC_4;    // added


  // ---------- Update program counter ----------
  // PC must be updated on the rising edge (positive edge) of the clock.
  PC pc(
    .reset(reset),       // input (Use reset to initialize PC. Initial value must be 0)
    .clk(clk),         // input
    .PCWrite(PCWrite),    // input
    .next_pc(next_pc),     // input
    .current_pc(current_pc)   // output
  );

  // ---------- Branch Target Buffer & Gshare Branch Predictor ----------
  BTB btb(
    .current_pc(current_pc),   // input
    .btb_hit(btb_hit),    // output
    .btb_target_pc(btb_target_pc),   // output

    // updating BTB at EX stage (due to misprediction)
    .clk(clk),    // input
    .reset(reset),    // input
    .update(ID_EX_branch && is_flush),    // input, Jump instr. 제외
    .update_pc(ID_EX_PC),   // input
    .real_target_pc(real_pc)    // input
  );

  BranchPredictor gshare(
    .current_pc(current_pc),    // input
    .taken(taken),   // output

    // updating PHT at EX stage (due to misprediction)
    .clk(clk),    // input
    .reset(reset),    // input
    .update(ID_EX_branch && is_flush),    // input, Jump instr. 제외
    .Hazard(Hazard),    // input
    .real_taken(ID_EX_branch && bcond)   // input
  );

  assign pred_pc = (btb_hit && taken) ? btb_target_pc : (current_pc + 32'd4); // Gshare, 2-bit global
  // assign pred_pc = (btb_hit) ? btb_target_pc : (current_pc + 32'd4); // always-taken
  assign next_pc = (is_flush) ? real_pc : pred_pc;

  assign is_flush = ((ID_EX_pred_pc != real_pc) && ID_EX_branch) || ID_EX_is_jal || ID_EX_is_jalr; // (PCSrc != 2'b00) // PCSrc == 0일 때 pred_pc 사용 -> X
  assign PCSrc = {(ID_EX_is_jal || (ID_EX_branch && bcond)), ID_EX_is_jalr};  // {PCSrc1, PCSrc2} = {(is_jal || (branch && bcond)), is_jalr}

  // ---------- Instruction Memory ----------
  InstMemory imem(
    .reset(reset),   // input
    .clk(clk),     // input
    .addr(current_pc),    // input
    .dout(inst)     // output
  );

  // Update IF/ID pipeline registers here
  always @(posedge clk) begin
    if (reset || is_flush) begin
      IF_ID_inst <= 32'b0;
      IF_ID_PC <= 32'b0;
      IF_ID_pred_pc <= 32'b0;
    end
    else begin
      if (IF_ID_Write) begin
        IF_ID_inst <= inst;
        IF_ID_PC <= current_pc;
        IF_ID_pred_pc <= pred_pc;
      end
      else begin
        IF_ID_inst <= IF_ID_inst;
        IF_ID_PC <= IF_ID_PC;
        IF_ID_pred_pc <= IF_ID_pred_pc;
      end
    end
  end

  // ---------- Hazard Detection Unit ----------
  HazardDetectionUnit HD(
    .IF_ID_inst(IF_ID_inst),    // input
    .ID_EX_rd_idx(ID_EX_rd_idx),    // input
    .ID_EX_mem_read(ID_EX_mem_read),    // input
    .ID_EX_reg_write(ID_EX_reg_write),    // input
    .IF_ID_Write(IF_ID_Write),   // output
    .Hazard(Hazard),    // output
    .PCWrite(PCWrite)   // output
  );

  // ---------- Register File ----------
  RegisterFile reg_file (
    .reset (reset),        // input
    .clk (clk),          // input
    .rs1 (mux_rs1_out),          // input
    .rs2 (IF_ID_inst[24:20]),          // input
    .rd (MEM_WB_rd_idx),           // input
    .rd_din (mux_Jump_WB_out),       // input
    .write_enable (MEM_WB_reg_write),    // input
    .rs1_dout (rf_rs1_dout),     // output
    .rs2_dout (rs2_dout),      // output
    .print_reg(print_reg)
  );

  // ---------- Control Unit ----------
  ControlUnit ctrl_unit (
    .is_jal(is_jal),    // input
    .is_jalr(is_jalr),    // input
    .branch(branch),    // input
    .part_of_inst(IF_ID_inst[6:0]),  // input
    .mem_read(MemRead),      // output
    .mem_to_reg(MemtoReg),    // output
    .mem_write(MemWrite),     // output
    .alu_src(ALUSrc),       // output
    .write_enable(RegWrite),  // output
    .pc_to_reg(PCtoReg),     // output
    .is_ecall(is_ecall)       // output (ecall inst)
  );

  // 10-bit control signal: MSB <- (branch, is_jalr, is_jal, is_ecall, PCtoReg, RegWrite, ALUSrc, MemWrite, MemtoReg, MemRead) -> LSB
  assign TotalControl = {branch, is_jalr, is_jal, is_ecall, PCtoReg, RegWrite, ALUSrc, MemWrite, MemtoReg, MemRead};

  // ---------- Immediate Generator ----------
  ImmediateGenerator imm_gen(
    .part_of_inst(IF_ID_inst),  // input
    .imm_gen_out(imm)    // output
  );

  // Update ID/EX pipeline registers here
  always @(posedge clk) begin
    if (reset || is_flush) begin
      ID_EX_branch <= 0;
      ID_EX_is_jalr <= 0;
      ID_EX_is_jal <= 0;
      ID_EX_is_halted <= 0;
      ID_EX_reg_write <= 0;
      ID_EX_alu_src <= 0;
      ID_EX_mem_write <= 0;
      ID_EX_mem_to_reg <= 0;
      ID_EX_mem_read <= 0;

      ID_EX_PC <= 0;
      ID_EX_rs1_data <= 0;
      ID_EX_rs2_data <= 0;
      ID_EX_imm <= 0;
      ID_EX_ALU_ctrl_unit_input <= 0;
      ID_EX_rd_idx <= 0;
      ID_EX_rs1_idx <= 0;
      ID_EX_rs2_idx <= 0;
      ID_EX_pred_pc <= 0;
    end
    else begin
      ID_EX_branch <= mux_CU_out[9];
      ID_EX_is_jalr <= mux_CU_out[8];
      ID_EX_is_jal <= mux_CU_out[7];
      ID_EX_is_halted <= (rs1_dout == 32'd10) && mux_CU_out[6];
      ID_EX_reg_write <= mux_CU_out[4];
      ID_EX_alu_src <= mux_CU_out[3];
      ID_EX_mem_write <= mux_CU_out[2];
      ID_EX_mem_to_reg <= mux_CU_out[1];
      ID_EX_mem_read <= mux_CU_out[0];

      ID_EX_PC <= IF_ID_PC;
      ID_EX_rs1_data <= rs1_dout;
      ID_EX_rs2_data <= rs2_dout;
      ID_EX_imm <= imm;
      ID_EX_ALU_ctrl_unit_input <= IF_ID_inst;
      ID_EX_rd_idx <= IF_ID_inst[11:7];
      ID_EX_rs1_idx <= IF_ID_inst[19:15];
      ID_EX_rs2_idx <= IF_ID_inst[24:20];
      ID_EX_pred_pc <= IF_ID_pred_pc;
    end
  end

  // ---------- ALU Control Unit ----------
  ALUControlUnit alu_ctrl_unit (
    .part_of_inst(ID_EX_ALU_ctrl_unit_input),  // input
    .alu_op(alu_op)         // output
  );

  // ---------- ALU ----------
  ALU alu (
    .alu_op(alu_op),      // input
    .alu_in_1(mux_ALU_A_out),    // input  
    .alu_in_2(mux_ALU_out),    // input
    .alu_result(alu_result),  // output
    .alu_bcond(bcond)     // output
  );

  // ---------- Forwarding Unit ----------
  ForwardingUnit forwarding_unit(
    .ID_EX_rs1(ID_EX_rs1_idx),    // input
    .ID_EX_rs2(ID_EX_rs2_idx),    // input
    .EX_MEM_rd(EX_MEM_rd_idx),    // input
    .MEM_WB_rd(MEM_WB_rd_idx),    // input
    .EX_MEM_reg_write(EX_MEM_reg_write),    // input
    .MEM_WB_reg_write(MEM_WB_reg_write),    // input
    .ForwardA(ForwardA),    // output
    .ForwardB(ForwardB)   // output
  );

  // Update EX/MEM pipeline registers here
  always @(posedge clk) begin
    if (reset) begin
      EX_MEM_mem_write <= 0;
      EX_MEM_mem_read <= 0;
      EX_MEM_mem_to_reg <= 0;
      EX_MEM_reg_write <= 0;
      EX_MEM_bcond <= 0;
      EX_MEM_is_halted <= 0;
      EX_MEM_is_jal <= 0;
      EX_MEM_is_jalr <= 0;

      EX_MEM_PC_imm <= 0;
      EX_MEM_alu_out <= 0;
      EX_MEM_dmem_data <= 0;
      EX_MEM_rd_idx <= 0;
      EX_MEM_PC <= 0;
    end
    else begin
      EX_MEM_mem_write <= ID_EX_mem_write;
      EX_MEM_mem_read <= ID_EX_mem_read;
      EX_MEM_mem_to_reg <= ID_EX_mem_to_reg;
      EX_MEM_reg_write <= ID_EX_reg_write;
      EX_MEM_bcond <= bcond;
      EX_MEM_is_halted <= ID_EX_is_halted;
      EX_MEM_is_jal <= ID_EX_is_jal;
      EX_MEM_is_jalr <= ID_EX_is_jalr;

      EX_MEM_PC_imm <= adder_PC_imm_out;
      EX_MEM_alu_out <= alu_result;
      EX_MEM_dmem_data <= mux_ALU_B_out;
      EX_MEM_rd_idx <= ID_EX_rd_idx;
      EX_MEM_PC <= ID_EX_PC;
    end
  end

  // ---------- Data Memory ----------
  DataMemory dmem(
    .reset (reset),      // input
    .clk (clk),        // input
    .addr (EX_MEM_alu_out),       // input
    .din (EX_MEM_dmem_data),        // input
    .mem_read (EX_MEM_mem_read),   // input
    .mem_write (EX_MEM_mem_write),  // input
    .dout (mem_dout)        // output
  );

  // Update MEM/WB pipeline registers here
  always @(posedge clk) begin
    if (reset) begin
      MEM_WB_mem_to_reg <= 0;
      MEM_WB_reg_write <= 0;
      MEM_WB_is_halted <= 0;
      MEM_WB_is_jal <= 0;
      MEM_WB_is_jalr <= 0;

      MEM_WB_mem_to_reg_src_1 <= 0;
      MEM_WB_mem_to_reg_src_2 <= 0;
      MEM_WB_rd_idx <= 0;
      MEM_WB_PC_4 <= 0;
    end
    else begin
      MEM_WB_mem_to_reg <= EX_MEM_mem_to_reg;
      MEM_WB_reg_write <= EX_MEM_reg_write;
      MEM_WB_is_halted <= EX_MEM_is_halted;
      MEM_WB_is_jal <= EX_MEM_is_jal;
      MEM_WB_is_jalr <= EX_MEM_is_jalr;

      MEM_WB_mem_to_reg_src_1 <= mem_dout;
      MEM_WB_mem_to_reg_src_2 <= EX_MEM_alu_out;
      MEM_WB_rd_idx <= EX_MEM_rd_idx;
      MEM_WB_PC_4 <= adder_PC_4_out;
    end
  end

  assign is_halted = MEM_WB_is_halted;

  // ---------- Muxes ---------- 
  Mux_2to1 mux_WB(
    .mux_in0(MEM_WB_mem_to_reg_src_2),    // input
    .mux_in1(MEM_WB_mem_to_reg_src_1),    // input
    .sel(MEM_WB_mem_to_reg),   // input
    .mux_out(mux_WB_out)    // output
  );

  Mux_2to1 mux_Jump_WB(
    .mux_in0(mux_WB_out),   // input
    .mux_in1(MEM_WB_PC_4),    // input
    .sel(MEM_WB_is_jal || MEM_WB_is_jalr),   // input
    .mux_out(mux_Jump_WB_out)   // output
  );

  Mux_2to1 mux_ALU(
    .mux_in0(mux_ALU_B_out),   // input
    .mux_in1(ID_EX_imm),    // input
    .sel(ID_EX_alu_src),   // input
    .mux_out(mux_ALU_out)   // output
  );
  
  Mux_4to1 mux_PC(
    .mux_in0(ID_EX_PC + 32'd4),   // input 
    .mux_in1(alu_result),   // input 
    .mux_in2(adder_PC_imm_out),  // input
    .mux_in3(32'b0),
    .sel(PCSrc),   // input
    .mux_out(real_pc)    // output
  );

  Mux_2to1_rs1 mux_rs1(
    .mux_in0(IF_ID_inst[19:15]),   // input
    .mux_in1(5'd17),    // input
    .sel(mux_CU_out[6]),   // input
    .mux_out(mux_rs1_out)   // output
  );

  Mux_2to1_CU mux_CU(
    .mux_in0(TotalControl),   // input
    .mux_in1(10'b0),   // input
    .sel(Hazard),   // input
    .mux_out(mux_CU_out)    // output
  );

  Mux_2to1 mux_rs1_dout(
    .mux_in0(rf_rs1_dout),   // input
    .mux_in1(EX_MEM_alu_out),   // input
    .sel(mux_CU_out[6] && (EX_MEM_rd_idx == 5'd17)),   // input
    .mux_out(rs1_dout)    // output
  );

  Mux_4to1 mux_ALU_ForwardA(
    .mux_in0(32'b0),
    .mux_in1(EX_MEM_alu_out),   // input
    .mux_in2(mux_WB_out),   // input
    .mux_in3(ID_EX_rs1_data),   // input
    .sel(ForwardA),   // input
    .mux_out(mux_ALU_A_out)    // output
  );

  Mux_4to1 mux_ALU_ForwardB(
    .mux_in0(32'b0),
    .mux_in1(EX_MEM_alu_out),   // input
    .mux_in2(mux_WB_out),   // input
    .mux_in3(ID_EX_rs2_data),   // input
    .sel(ForwardB),   // input
    .mux_out(mux_ALU_ForwardB_out)    // output
  );

  Mux_2to1 mux_rs1_imm(
    .mux_in0(mux_ALU_ForwardB_out),   // input
    .mux_in1(ID_EX_imm),   // input
    .sel(ID_EX_is_jalr),   // input
    .mux_out(mux_ALU_B_out)    // output
  );


  // ---------- Adder ---------- 
  
  Add adder_PC_4(
    .adder_in1(EX_MEM_PC),   // input
    .adder_in2(32'd4),    // input
    .adder_out(adder_PC_4_out)    // output
  );
  
  Add adder_PC_imm(
    .adder_in1(ID_EX_PC),   // input
    .adder_in2(ID_EX_imm),    // input
    .adder_out(adder_PC_imm_out)    // output
  );
  
  
endmodule
