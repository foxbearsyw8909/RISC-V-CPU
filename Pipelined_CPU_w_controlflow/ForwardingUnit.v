module ForwardingUnit(input [4:0] ID_EX_rs1,
                      input [4:0] ID_EX_rs2,
                      input [4:0] EX_MEM_rd,
                      input [4:0] MEM_WB_rd,
                      input EX_MEM_reg_write,
                      input MEM_WB_reg_write,
                      output reg [1:0] ForwardA,
                      output reg [1:0] ForwardB
                      );
    always @(*) begin
        ForwardA = 2'b00;
        ForwardB = 2'b00;

        // ForwardA
       if (ID_EX_rs1 != 0 && ID_EX_rs1 == EX_MEM_rd && EX_MEM_reg_write) begin // dist  = 1
            ForwardA = 2'b01;
       end
       else if (ID_EX_rs1 != 0 && ID_EX_rs1 == MEM_WB_rd && MEM_WB_reg_write) begin  // dist = 2
            ForwardA = 2'b10;
       end
       else begin // dist >= 3
            ForwardA = 2'b11;
       end
        
        // ForwardB
       if (ID_EX_rs2 != 0 && ID_EX_rs2 == EX_MEM_rd && EX_MEM_reg_write) begin // dist  = 1
            ForwardB = 2'b01;
       end
       else if (ID_EX_rs2 != 0 && ID_EX_rs2 == MEM_WB_rd && MEM_WB_reg_write) begin  // dist = 2
            ForwardB = 2'b10;
       end
       else begin // dist >= 3
            ForwardB = 2'b11;
       end

    end
endmodule
