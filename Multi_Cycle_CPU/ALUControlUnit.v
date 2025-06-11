`include "opcodes.v"
`include "alu_opcodes.v"
module ALUControlUnit (
    input [31:0] part_of_inst,
    input [1:0] ALUOp,

    output reg [3:0] alu_op);

    wire [21:0] _unused = { part_of_inst[24:15], 
                            part_of_inst[11:0]};

always @(*) begin
    //alu_op = `ALU_ZERO;
    case(ALUOp)
        2'b00: //LW or SW or JALR or JAL
            alu_op = `ALU_ADD;
        2'b01: //BRANCH (BEQ)
            if (part_of_inst[14:12] == `FUNCT3_BEQ) alu_op = `ALU_BEQ; 
            else if (part_of_inst[14:12] == `FUNCT3_BNE) alu_op = `ALU_BNE;
            else if (part_of_inst[14:12] == `FUNCT3_BLT) alu_op = `ALU_BLT;
            else if (part_of_inst[14:12] == `FUNCT3_BGE) alu_op = `ALU_BGE;
            else alu_op = `ALU_ZERO;
        
        2'b10: //R/I-type
            if (part_of_inst[31:25] == `FUNCT7_SUB) alu_op = `ALU_SUB;
            else begin
                //funct3 check
                if (part_of_inst[14:12] == `FUNCT3_ADD) alu_op = `ALU_ADD; 
                else if (part_of_inst[14:12] == `FUNCT3_SLL) alu_op = `ALU_SLL;
                else if (part_of_inst[14:12] == `FUNCT3_XOR) alu_op = `ALU_XOR;
                else if (part_of_inst[14:12] == `FUNCT3_SRL) alu_op = `ALU_SRL;
                else if (part_of_inst[14:12] == `FUNCT3_OR) alu_op = `ALU_OR;
                else if (part_of_inst[14:12] == `FUNCT3_AND) alu_op = `ALU_AND;
                else alu_op = `ALU_ZERO;
            end

        2'b11: alu_op = `ALU_ZERO;
    endcase
end
endmodule
