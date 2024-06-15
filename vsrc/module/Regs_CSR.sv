`ifdef VERILATOR
`include "include/common.sv"
`endif


module Regs_CSR
    import common::*;
(
    input             clk,          // 时钟信号
    input             reset,        // 复位信号
    input      [11:0] rw_CSR,       // 需要读写的寄存器
    output reg [63:0] readData_CSR  // 读取得到的数据
);

    reg [1:0] mode;
    mstatus_t mstatus;
    reg [63:0] mie;
    reg [63:0] mtvec;
    reg [63:0] mscratch;
    reg [63:0] mepc;
    reg [63:0] mcause;
    reg [63:0] mtval;
    reg [63:0] mip;
    satp_t satp;

    always @(*) begin  // 读取CSR
        case (rw_CSR)
            12'h180: readData_CSR = satp;
            12'h300: readData_CSR = mstatus;
            12'h304: readData_CSR = mie;
            12'h305: readData_CSR = mtvec;
            12'h340: readData_CSR = mscratch;
            12'h341: readData_CSR = mepc;
            12'h342: readData_CSR = mcause;
            12'h343: readData_CSR = mtval;
            12'h344: readData_CSR = mip;

            12'b0000_0000_0000: readData_CSR = mtvec;
            12'b0011_0000_0010: readData_CSR = mepc;
            default: readData_CSR = 64'b0;
        endcase
    end

endmodule
