`include "opcodes.v"
`include "alu_opcodes.v"
module ALUControlUnit (input [31:0] part_of_inst,
                       output reg [3:0] alu_op);

   wire [14:0] _unused = {part_of_inst[24:15], 
                          part_of_inst[11:7]};

   always @(*) begin
      //alu_op = 4'b1111;
      //------- R-type -------
      if(part_of_inst[6:0]==`ARITHMETIC) begin
         //funct7 check
         if (part_of_inst[31:25] == `FUNCT7_SUB) begin
            alu_op = `ALU_SUB;
         end
         else if (part_of_inst[31:25] == `FUNCT7_OTHERS) begin
            //funct3 check
            if (part_of_inst[14:12] == `FUNCT3_ADD) alu_op = `ALU_ADD; 
            else if (part_of_inst[14:12] == `FUNCT3_SLL) alu_op = `ALU_SLL;
            else if (part_of_inst[14:12] == `FUNCT3_XOR) alu_op = `ALU_XOR;
            else if (part_of_inst[14:12] == `FUNCT3_SRL) alu_op = `ALU_SRL;
            else if (part_of_inst[14:12] == `FUNCT3_OR) alu_op = `ALU_OR;
            else if (part_of_inst[14:12] == `FUNCT3_AND) alu_op = `ALU_AND;
            else alu_op = `ALU_ZERO;
         end
         else begin
            alu_op = `ALU_ZERO;
         end
      end
      //------- I-type -------
      else if (part_of_inst[6:0]==`ARITHMETIC_IMM ) begin
         if (part_of_inst[14:12] == `FUNCT3_ADD) alu_op = `ALU_ADD; 
         else if (part_of_inst[14:12] == `FUNCT3_SLL) alu_op = `ALU_SLL;
         else if (part_of_inst[14:12] == `FUNCT3_XOR) alu_op = `ALU_XOR;
         else if (part_of_inst[14:12] == `FUNCT3_SRL) alu_op = `ALU_SRL;
         else if (part_of_inst[14:12] == `FUNCT3_OR) alu_op = `ALU_OR;
         else if (part_of_inst[14:12] == `FUNCT3_AND) alu_op = `ALU_AND;
         else alu_op = `ALU_ZERO;
      end
      //-------B-Type-------
      else if (part_of_inst[6:0]==`BRANCH) begin
         if (part_of_inst[14:12] == `FUNCT3_BEQ) alu_op = `ALU_BEQ; 
         else if (part_of_inst[14:12] == `FUNCT3_BNE) alu_op = `ALU_BNE;
         else if (part_of_inst[14:12] == `FUNCT3_BLT) alu_op = `ALU_BLT;
         else if (part_of_inst[14:12] == `FUNCT3_BGE) alu_op = `ALU_BGE;
         else alu_op = `ALU_ZERO;
      end
      //----Operations that needs to calculate byte adresses----
      else if ((part_of_inst[6:0] == `LOAD) || (part_of_inst[6:0] == `STORE) || (part_of_inst[6:0] == `JALR)) begin
         alu_op = `ALU_ADD;
      end
      // When part_of_inst[6:0] is `JAL
      else begin 
         alu_op = `ALU_ZERO;
      end
   end
endmodule
