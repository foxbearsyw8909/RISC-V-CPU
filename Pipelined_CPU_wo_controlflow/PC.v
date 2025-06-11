module PC (input reset,
           input clk,
           input [31:0] next_pc,
           input PCWrite,
           output reg [31:0] current_pc);


    always @(posedge clk) begin
        if (reset) begin
                current_pc <= 32'b0;
        end
        else begin
            if (PCWrite) begin
                current_pc <= next_pc;
            end
            else begin
                current_pc <= current_pc;
            end
        end
    end
   
endmodule
