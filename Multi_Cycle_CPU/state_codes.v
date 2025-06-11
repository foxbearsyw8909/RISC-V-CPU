// 4-bit codes for microcoding in ControlUnit & MicrocodeController
`define IF_PC 4'b0001

`define ID_REG_FETCH 4'b0010

`define MEM_READ 4'b0011
`define MEM_WRITE 4'b0100

`define WB_LD 4'b0101
`define WB_R_I 4'b0110

`define EX_R 4'b0111
`define EX_IMM 4'b1000
`define EX_BRANCH_COND 4'b1001
`define EX_LD_SD 4'b1010

`define EX_WB_JAL 4'b1011
`define EX_WB_JALR 4'b1100

`define EX_ECALL 4'b1101
