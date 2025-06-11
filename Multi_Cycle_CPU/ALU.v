`include "alu_opcodes.v"
module ALU (input [3:0] alu_op,
            input [31:0] alu_in_1,
            input [31:0] alu_in_2,
           
            output reg [31:0] alu_result,
            output reg alu_bcond);

   always @(*) begin
      alu_result = 32'b0;
      alu_bcond = 1'b0;
      case(alu_op)
         `ALU_ADD: begin
            alu_result = (alu_in_1 + alu_in_2) ;
         end
         `ALU_SUB: begin
            alu_result = (alu_in_1 - alu_in_2) ;
         end
         `ALU_SLL: begin
            alu_result = (alu_in_1 << alu_in_2) ;
         end
         `ALU_XOR: begin
            alu_result = (alu_in_1 ^ alu_in_2) ;
         end
         `ALU_SRL: begin
            alu_result = (alu_in_1 >> alu_in_2) ;
         end
         `ALU_OR: begin
            alu_result = (alu_in_1 | alu_in_2) ;
         end  
         `ALU_AND: begin
            alu_result = (alu_in_1 & alu_in_2) ; 
         end

         //Branch condition
         `ALU_BEQ: begin
            alu_bcond = (alu_in_1 == alu_in_2) ;
         end
         `ALU_BNE: begin
            alu_bcond = (alu_in_1 != alu_in_2) ; 
         end
         `ALU_BLT: begin
            alu_bcond = ($signed(alu_in_1) < $signed(alu_in_2));
         end
         `ALU_BGE: begin
            alu_bcond = ($signed(alu_in_1) >= $signed(alu_in_2));
         end

         `ALU_ZERO: begin
            alu_result = 32'b0;
            alu_bcond = 1'b0;
         end
         default: begin
            alu_result = 32'b0;
            alu_bcond = 1'b0;
         end
      endcase            
   end

endmodule
