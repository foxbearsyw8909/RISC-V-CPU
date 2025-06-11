`include "opcodes.v"
module HazardDetectionUnit( input [31:0] IF_ID_inst,
                            input [4:0] ID_EX_rd_idx,
                            input ID_EX_mem_read,
                            input ID_EX_reg_write,
                            output reg Hazard,
                            output reg PCWrite,
                            output reg IF_ID_Write);
    
    reg use_rs1;
    reg use_rs2;

    always @(*) begin
        if (IF_ID_inst[6:0] == `ARITHMETIC_IMM | IF_ID_inst[6:0] == `LOAD | IF_ID_inst[6:0] == `JALR) begin
            use_rs1 = 1'b1;
            use_rs2 = 1'b0;
        end
        else if (IF_ID_inst[6:0] == `JAL | IF_ID_inst[6:0] == `ECALL) begin
            use_rs1 = 1'b0;
            use_rs2 = 1'b0;
        end
        else begin // R, SD, JALR, BRANCH
            use_rs1 = 1'b1;
            use_rs2 = 1'b1;
        end

        if ((IF_ID_inst[6:0] == `ECALL) && (ID_EX_rd_idx == 5'd17)) begin //&& ID_EX_reg_write
            Hazard = 1'b1;
            PCWrite = 1'b0;
            IF_ID_Write = 1'b0;
        end
        else if ((((IF_ID_inst[19:15] == ID_EX_rd_idx) && use_rs1) || ((IF_ID_inst[24:20] == ID_EX_rd_idx) && use_rs2)) && ID_EX_mem_read) begin
            Hazard = 1'b1;
            PCWrite = 1'b0;
            IF_ID_Write = 1'b0;
        end
        else begin
            Hazard = 1'b0;
            PCWrite = 1'b1;
            IF_ID_Write = 1'b1;
        end
    end

    
endmodule
