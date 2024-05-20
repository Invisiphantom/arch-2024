`ifdef VERILATOR
`include "include/common.sv"
`endif


module Regs_CSR
    import common::*;
(
    input             clk,          // 时钟信号
    input             reset,        // 复位信号
    input             excep,        // 是否为csrrx指令
    input      [ 2:0] funct3,       // 指令的funct3字段
    input      [11:0] rw_CSR,       // 需要读写的寄存器
    output reg [63:0] readData_CSR, // 读取得到的数据

    input         [63:0] writeCSR_rs1,   // 需要计算并写入的数据
    input         [ 4:0] writeCSR_zimm,  // 需要计算并写入的数据
    output csrs_t        csrGroup
);

    reg [ 1:0] mode;
    reg [63:0] mstatus;
    reg [63:0] mie;
    reg [63:0] mtvec;
    reg [63:0] mscratch;
    reg [63:0] mepc;
    reg [63:0] mcause;
    reg [63:0] mtval;
    reg [63:0] mip;

    always @(*) begin  // 读取CSR
        case (rw_CSR)
            12'h300: readData_CSR = mstatus;
            12'h304: readData_CSR = mie;
            12'h305: readData_CSR = mtvec;
            12'h340: readData_CSR = mscratch;
            12'h341: readData_CSR = mepc;
            12'h342: readData_CSR = mcause;
            12'h343: readData_CSR = mtval;
            12'h344: readData_CSR = mip;
            12'h302: readData_CSR = mepc;  // mret
            default: readData_CSR = 64'b0;
        endcase
    end


    reg [63:0] temp;
    always @(posedge clk) begin
        if (reset) begin  // 复位赋值mode为3
            mode = 2'b11;
            mstatus = 64'b0;
            mie = 64'b0;
            mtvec = 64'b0;
            mscratch = 64'b0;
            mepc = 64'b0;
            mcause = 64'b0;
            mtval = 64'b0;
            mip = 64'b0;
        end else if (excep) begin
            case (funct3)
                3'b001:  temp = writeCSR_rs1;  // csrrw
                3'b010:  temp = readData_CSR | writeCSR_rs1;  // csrrs
                3'b011:  temp = readData_CSR & ~writeCSR_rs1;  // csrrc
                3'b101:  temp = {59'b0, writeCSR_zimm};  // csrrwi
                3'b110:  temp = readData_CSR | {59'b0, writeCSR_zimm};  // csrrsi
                3'b111:  temp = readData_CSR & ~{59'b0, writeCSR_zimm};  // csrrci
                default: temp = 64'h0;
            endcase

            case (rw_CSR)
                12'h300: mstatus = temp;
                12'h304: mie = temp;
                12'h305: mtvec = temp;
                12'h340: mscratch = temp;
                12'h341: mepc = temp;
                12'h342: mcause = temp;
                12'h343: mtval = temp;
                12'h344: mip = temp;
                12'h302: begin  // mret
                    mode = mstatus[12:11];  // MPP
                    mstatus[3] = mstatus[7];  // MIE = MPIE
                    mstatus[7] = 1'b1;  // MPIE = 1
                    mstatus[12:11] = 2'b0;  // MPP = 0
                end
                12'h000: begin  // ecall
                end
                default: ;
            endcase
        end
    end












    assign csrGroup.mode = mode;
    assign csrGroup.mstatus = mstatus;
    assign csrGroup.mie = mie;
    assign csrGroup.mtvec = mtvec;
    assign csrGroup.mscratch = mscratch;
    assign csrGroup.mepc = mepc;
    assign csrGroup.mcause = mcause;
    assign csrGroup.mtval = mtval;
    assign csrGroup.mip = mip;
endmodule
