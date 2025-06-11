module BranchPredictor (input [31:0] current_pc,
                        input clk,
                        input reset,
                        input update,
                        input Hazard,
                        input real_taken,
                        output taken);

    reg [4:0] bhsr;
    reg [1:0] pht [0:31];

    wire [4:0] current_pc_idx;
    assign current_pc_idx = current_pc[6:2]; // 32 entries

    wire [4:0] pht_idx;
    assign pht_idx = bhsr ^ current_pc_idx; // for Gshare
    // assign pht_idx = current_pc_idx; // for 2-bit global

    assign taken = (pht[pht_idx] >= 2'b10);

    // initializing or updating BHSR & PHT
    reg [4:0] ID_pht_idx;
    reg [4:0] update_pht_idx; // EX_pht_idx
    integer i;

    always @(posedge clk) begin
        if (reset) begin
            bhsr <= 5'b0; // for Gshare
            for (i = 0; i < 32; i = i + 1) pht[i] <= 2'b01; // try 2'b00 ~ 2'b11
        end
        else begin
            if (!Hazard) begin
                ID_pht_idx <= pht_idx;
                update_pht_idx <= ID_pht_idx;
            end
            if (update) begin
                case(pht[update_pht_idx])
                    2'b00: pht[update_pht_idx] <= real_taken ? 2'b01 : 2'b00;
                    2'b01: pht[update_pht_idx] <= real_taken ? 2'b10 : 2'b00;
                    2'b10: pht[update_pht_idx] <= real_taken ? 2'b11 : 2'b01;
                    2'b11: pht[update_pht_idx] <= real_taken ? 2'b11 : 2'b10;
                endcase
                bhsr <= {bhsr[3:0], real_taken}; // for Gshare
            end
            else ;
        end
    end


 endmodule
