`ifdef VERILATOR
`include "include/common.sv"
`endif

module Regs
    import common::*;
(
    input clk,   // 时钟信号
    input reset, // 复位信号

    input  [ 4:0] readReg1,     // 读取的寄存器1
    input  [ 4:0] readReg2,     // 读取的寄存器2
    output [63:0] readData1_R,  // 读取的数据1
    output [63:0] readData2_R,  // 读取的数据2

    input                regWrite,     // 是否需要写回
    input         [ 4:0] writeReg,     // 写入的寄存器
    input         [63:0] writeData_R,  // 写入的数据
    output regs_t        regGroup      // 外接测试模块的寄存器组
);

    reg [63:0] Register[0:31];

    assign readData1_R = Register[readReg1];
    assign readData2_R = Register[readReg2];

    always @(*) begin
        if (reset) for (integer i = 0; i < 32; i = i + 1) Register[i] = 64'b0;
        else if (regWrite && writeReg != 5'b00000) begin  // x0寄存器不可写
            Register[writeReg] = writeData_R;
        end else Register[0] = 64'h0;
    end

    assign regGroup.x0  = Register[0];
    assign regGroup.ra  = Register[1];
    assign regGroup.sp  = Register[2];
    assign regGroup.gp  = Register[3];
    assign regGroup.tp  = Register[4];
    assign regGroup.t0  = Register[5];
    assign regGroup.t1  = Register[6];
    assign regGroup.t2  = Register[7];
    assign regGroup.s0  = Register[8];
    assign regGroup.s1  = Register[9];
    assign regGroup.a0  = Register[10];
    assign regGroup.a1  = Register[11];
    assign regGroup.a2  = Register[12];
    assign regGroup.a3  = Register[13];
    assign regGroup.a4  = Register[14];
    assign regGroup.a5  = Register[15];
    assign regGroup.a6  = Register[16];
    assign regGroup.a7  = Register[17];
    assign regGroup.s2  = Register[18];
    assign regGroup.s3  = Register[19];
    assign regGroup.s4  = Register[20];
    assign regGroup.s5  = Register[21];
    assign regGroup.s6  = Register[22];
    assign regGroup.s7  = Register[23];
    assign regGroup.s8  = Register[24];
    assign regGroup.s9  = Register[25];
    assign regGroup.s10 = Register[26];
    assign regGroup.s11 = Register[27];
    assign regGroup.t3  = Register[28];
    assign regGroup.t4  = Register[29];
    assign regGroup.t5  = Register[30];
    assign regGroup.t6  = Register[31];
endmodule
