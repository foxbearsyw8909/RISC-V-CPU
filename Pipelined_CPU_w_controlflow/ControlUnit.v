`include "opcodes.v"

module ControlUnit (
    input [6:0] part_of_inst,  

    output reg is_jal,
    output reg is_jalr,
    output reg branch,
    output reg mem_read,
    output reg mem_to_reg,
    output reg mem_write,
    output reg alu_src,
    output reg write_enable,
    output reg pc_to_reg,
    output reg is_ecall
    );

   assign is_jal = (part_of_inst == `JAL);
   assign is_jalr = (part_of_inst == `JALR);
   assign branch = (part_of_inst == `BRANCH);
   assign mem_read = (part_of_inst == `LOAD);
   assign mem_to_reg = (part_of_inst == `LOAD);
   assign mem_write = (part_of_inst == `STORE);
   assign alu_src = (part_of_inst != `ARITHMETIC) && (part_of_inst != `BRANCH) && (part_of_inst != `ECALL);
   assign write_enable = (part_of_inst != `STORE) && (part_of_inst != `BRANCH) && (part_of_inst != `ECALL);
   assign pc_to_reg = (part_of_inst == `JAL) || (part_of_inst == `JALR);
   assign is_ecall = (part_of_inst == `ECALL);


endmodule
