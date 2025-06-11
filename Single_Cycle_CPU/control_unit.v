`include "opcodes.v"
module control_unit (
   input [6:0] part_of_inst,

   output is_jal,
   output is_jalr,
   output branch,
   output mem_read,
   output mem_to_reg,
   output mem_write,
   output alu_src,
   output write_enable,
   output pc_to_reg,
   output is_ecall);

   assign is_jal = (part_of_inst == `JAL);
   assign is_jalr = (part_of_inst == `JALR);
   assign branch = (part_of_inst == `BRANCH);
   assign mem_read = (part_of_inst == `LOAD);
   assign mem_to_reg = (part_of_inst == `LOAD);
   assign mem_write = (part_of_inst == `STORE);
   assign alu_src = (part_of_inst != `ARITHMETIC) && (part_of_inst != `BRANCH);
   assign write_enable = (part_of_inst != `STORE) && (part_of_inst != `BRANCH) && (part_of_inst != `ECALL);
   assign pc_to_reg = (part_of_inst == `JAL) || (part_of_inst == `JALR);
   assign is_ecall = (part_of_inst == `ECALL);


endmodule
