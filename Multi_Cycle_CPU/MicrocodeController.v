`include "opcodes.v"
`include "state_codes.v"

module MicrocodeController (
    input clk,
    input reset,
    input [6:0] part_of_inst,
    input [3:0] current_state,
    output reg [3:0] next_state
);
    always @(*) begin
        next_state = current_state; // 기본값 추가로 latch 방지
        case (current_state)
            `IF_PC : next_state = `ID_REG_FETCH;

            `ID_REG_FETCH : begin
                if (part_of_inst == `LOAD || part_of_inst == `STORE) begin
                    next_state = `EX_LD_SD;
                end 
                else if (part_of_inst == `ARITHMETIC) begin
                    next_state = `EX_R;
                end 
                else if (part_of_inst == `ARITHMETIC_IMM) begin
                    next_state = `EX_IMM;
                end
                else if (part_of_inst == `BRANCH) begin
                    next_state = `EX_BRANCH_COND;
                end 
                else if (part_of_inst == `JAL) begin
                    next_state = `EX_WB_JAL;
                end
                else if (part_of_inst == `JALR) begin
                    next_state = `EX_WB_JALR;
                end
                else if (part_of_inst == `ECALL) begin
                    next_state = `EX_ECALL;
                end
                else ;
            end
            `EX_ECALL : begin
                next_state = `IF_PC;
            end
            
            `EX_LD_SD : begin
                if(part_of_inst == `LOAD) begin
                    next_state = `MEM_READ;
                end 
                else if(part_of_inst == `STORE) begin
                    next_state = `MEM_WRITE;
                end
                else ;
            end
            `MEM_READ : begin
                next_state = `WB_LD;
            end
            
            `EX_R,
            `EX_IMM : begin
                next_state = `WB_R_I;
            end

            `WB_LD,
            `MEM_WRITE,
            `WB_R_I,
            `EX_BRANCH_COND,
            `EX_WB_JAL,
            `EX_WB_JALR : begin // completion of each instruction
                next_state = `IF_PC;
            end
            default : ;
        endcase
    end
endmodule
