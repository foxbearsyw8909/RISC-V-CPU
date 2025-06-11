`include "opcodes.v"
`include "state_codes.v"

module ControlUnit (
    input [6:0] part_of_inst,  
    input [3:0] current_state,

    output reg PCWriteCond,     
    output reg PCWrite,
    output reg IorD,
    output reg MemRead,
    output reg MemWrite,
    output reg MemtoReg,
    output reg IRWrite,
    output reg PCSource,
    output reg [1:0] ALUOp,
    output reg ALUSrcA,
    output reg [1:0] ALUSrcB,
    output reg RegWrite,
    output reg is_ecall
);
// Combinational control logic that produces control signals
always @(*) begin
       // signal initialization
        PCWriteCond = 1'b0;
        PCWrite = 1'b0;
        IorD = 1'b0;
        MemRead = 1'b0;
        MemWrite = 1'b0;
        MemtoReg = 1'b0;
        IRWrite = 1'b0;
        PCSource = 1'b0;
        ALUOp = 2'b00;
        ALUSrcA = 1'b0;
        ALUSrcB = 2'b00;
        RegWrite = 1'b0;
        is_ecall = 1'b0;

    case (current_state)
        `IF_PC : begin
            MemRead = 1'b1;
            IorD = 1'b0; // I
            IRWrite = 1'b1;
            ALUSrcA = 1'b0;
            ALUSrcB = 2'b01; 
            ALUOp = 2'b00; // PC+4  
        end

        `ID_REG_FETCH : begin
            if (part_of_inst == `ECALL) begin
                is_ecall = 1'b1;
                PCWrite = 1'b1;
                PCSource = 1'b1;
            end
            if (part_of_inst == `JAL | part_of_inst == `JALR) begin
                ALUSrcA = 1'b0;
                ALUSrcB = 2'b01; 
                ALUOp = 2'b00; // PC+4
            end
            else if (part_of_inst == `BRANCH) begin
                ALUSrcA = 1'b0;
                ALUSrcB = 2'b10; 
                ALUOp = 2'b00; // PC+imm
                PCWrite = 1'b1;
                PCSource = 1'b1; // PC<-ALUOut(PC+4)
            end
            else begin
                PCWrite = 1'b1;
                PCSource = 1'b1;
            end
        end

        `EX_ECALL : ;

        `EX_LD_SD : begin
            ALUSrcA = 1'b1;
            ALUSrcB = 2'b10;
        end
        `MEM_READ : begin
            MemRead = 1'b1;
            IorD = 1'b1; // D
        end
        `WB_LD : begin
            RegWrite = 1'b1;
            MemtoReg = 1'b1;
        end
        `MEM_WRITE : begin
            MemWrite = 1'b1;
            IorD = 1'b1;
        end

        `EX_R : begin // R-Type ALU: GPR[rs1] (OP) GPR[rs2]
            ALUSrcA = 1'b1;
            ALUSrcB = 2'b00;
            ALUOp = 2'b10;
        end 
        `EX_IMM : begin // I-Type ALU : GPR[rs1] (OP) imm
            ALUSrcA = 1'b1;
            ALUSrcB = 2'b10;
            ALUOp = 2'b10;
        end
        `WB_R_I: begin
            RegWrite = 1'b1;
            MemtoReg = 1'b0;
        end

        `EX_BRANCH_COND : begin
            ALUSrcA = 1'b1;
            ALUSrcB = 2'b00;
            ALUOp = 2'b01; // branch condition
            PCWriteCond = 1'b1;
            PCSource = 1'b1; // PC<-ALUOut(PC+imm)
        end

        `EX_WB_JAL : begin
            MemtoReg = 1'b0;
            RegWrite = 1'b1; // WB
            ALUSrcA = 1'b0;
            ALUSrcB = 2'b10;
            ALUOp = 2'b00; // PC+imm
            PCSource = 1'b0;
            PCWrite = 1'b1;
        end

        `EX_WB_JALR : begin
            MemtoReg = 1'b0;
            RegWrite = 1'b1; // WB
            ALUSrcA = 1'b1;
            ALUSrcB = 2'b10;
            ALUOp = 2'b00; // GPR[rs1]+imm
            PCSource = 1'b0;
            PCWrite = 1'b1;
        end
        default : ;
    endcase
end
endmodule
