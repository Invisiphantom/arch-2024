module Control (
    input  [6:0] Opcode_7,  // instruction[6:0]
    output       regWrite,  // write back
    output       aluSrc,    // 0:rs2 1:imm
    output [1:0] aluOp_2,   // 00:add 01:sub 10:R-type 11:I-type
    output       aluSext,   // 0:normal 1:sign extend
    output       memRead,   // load
    output       memWrite,  // store
    output       branch,    // branch
    output       jump,      // jal
    output       jumpReg,   // jalr
    output       lui,       // lui
    output       auipc,     // auipc
    output       excep      // excep
);

    reg [12:0] controlSignal;
    assign {{regWrite}, {aluSrc, aluOp_2[1:0], aluSext}, memRead, memWrite, branch, jump, jumpReg, lui, auipc, excep} = controlSignal[12:0];
    always @(*) begin
        case (Opcode_7)
            7'b0110011: controlSignal = 13'b1_0100_00000000;  // R-type
            7'b0111011: controlSignal = 13'b1_0101_00000000;  // R-type (+)
            7'b0010011: controlSignal = 13'b1_1110_00000000;  // I-type
            7'b0011011: controlSignal = 13'b1_1111_00000000;  // I-type (+)
            7'b0000011: controlSignal = 13'b1_1000_10000000;  // load

            7'b0100011: controlSignal = 13'b0_1000_01000000;  // store
            7'b1100011: controlSignal = 13'b0_0010_00100000;  // branch

            7'b1101111: controlSignal = 13'b1_xxx0_00010000;  // jal
            7'b1100111: controlSignal = 13'b1_1000_00001000;  // jalr
            7'b0110111: controlSignal = 13'b1_1000_00000100;  // lui
            7'b0010111: controlSignal = 13'b1_1000_00000010;  // auipc
            7'b1110011: controlSignal = 13'b1_0000_00000001;  // excep
            default: begin
                controlSignal = 13'b0;
            end
        endcase
    end
endmodule
