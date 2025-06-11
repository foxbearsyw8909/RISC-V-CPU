module BTB (input [31:0] current_pc,
            input clk,
            input reset,
            input update,
            input [31:0] update_pc,
            input [31:0] real_target_pc,
            output btb_hit,
            output [31:0] btb_target_pc);

    reg [24:0] tag_table [0:31];
    reg [31:0] target_table [0:31];
    reg valid_table [0:31];

    wire [4:0] idx; // 32 entries
    wire [24:0] tag;
    assign idx = current_pc[6:2];
    assign tag = current_pc[31:7];

    assign btb_hit = valid_table[idx] && (tag_table[idx] == tag);
    assign btb_target_pc = target_table[idx];

    // initializing or updating BTB table (including valid & tag tables)
    integer i;
    always @(posedge clk) begin
        if (reset) begin
            for (i = 0; i < 32; i = i + 1) begin
                valid_table[i] <= 1'b0;
                tag_table[i] <= 25'b0;
                target_table[i] <= 32'b0;
            end
        end
        else begin
            if (update) begin
                tag_table[update_pc[6:2]] <= update_pc[31:7];
                target_table[update_pc[6:2]] <= real_target_pc;
                valid_table[update_pc[6:2]] <= 1'b1;
            end
            else ;
        end
    end

endmodule
