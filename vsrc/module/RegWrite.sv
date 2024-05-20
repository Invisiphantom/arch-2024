module RegWrite (
    input memRead,  // store
    input jump,     // jal
    input jumpReg,  // jalr
    input excep,    // excep
    input [63:0] aluResult,  // ALU运算结果
    input [63:0] readData_M, // 从内存读出的数据
    input [63:0] PCaddress,  // PC地址
    input [63:0] readData_CSR, // CSR寄存器数据
    output reg [63:0] writeData_R // 需要写入寄存器的数据
);

    always @(*) begin // 纯组合逻辑
        if (jump | jumpReg) writeData_R = PCaddress + 4; // 跳转指令需保存PC+4
        else if (memRead) writeData_R = readData_M; // 将内存数据加载到寄存器
        else if (excep) writeData_R = readData_CSR; // 将CSR寄存器数据写入寄存器
        else writeData_R = aluResult; // 将ALU运算结果写入寄存器
    end
endmodule
