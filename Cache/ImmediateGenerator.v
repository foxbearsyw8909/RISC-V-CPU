`include "opcodes.v"
module ImmediateGenerator (input wire [31:0] part_of_inst,
                            output reg [31:0] imm_gen_out); // sign-extended immediate value
always @(*) begin
    case(part_of_inst[6:0])
        `ARITHMETIC_IMM, `LOAD, `JALR: // I-type, imm: part_of_inst[20:31]
            imm_gen_out = {{20{part_of_inst[31]}}, part_of_inst[31:20]};

        `STORE: // S-type, imm[4:0]: part_of_inst[7:11], imm[11:5]: part_of_inst[25:31]
            imm_gen_out = {{20{part_of_inst[31]}}, part_of_inst[31:25], part_of_inst[11:7]};

        `JAL: // UJ-type, imm[10:1]: part_of_inst[21:30], imm[11]: part_of_inst[20], imm[19:12]: part_of_inst[12:19], imm[20]: part_of_inst[31]
            imm_gen_out = {{12{part_of_inst[31]}}, part_of_inst[19:12], part_of_inst[20], part_of_inst[30:21], 1'b0};

        `BRANCH: // SB-type, imm[4:1]: part_of_inst[8:11], imm[10:5]: part_of_inst[25:30], imm[11]: part_of_inst[7], imm[12]: part_of_inst[31]
            imm_gen_out = {{20{part_of_inst[31]}}, part_of_inst[7], part_of_inst[30:25], part_of_inst[11:8], 1'b0};

        default: imm_gen_out = 32'b0; // ARITHMETIC, ECALL
    endcase
end

endmodule
