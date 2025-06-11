`include "state_codes.v"
module StateRegister (
    input clk,
    input reset,
    input [3:0] next_state,
    output reg [3:0] current_state
);

    always @(posedge clk) begin
        if (reset)
            current_state <= `IF_PC;
        else
            current_state <= next_state;
    end

endmodule
