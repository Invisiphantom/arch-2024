module ALUControl (
    input      [1:0] aluOp_2,      // 00:add 01:sub 10:R-type 11:I-type
    input            funct7_25,    // instruction[25]  RV64M
    input            funct7_30,    // instruction[30]
    input      [2:0] funct3,       // instruction[14:12]
    output reg [4:0] aluControl_5  // ALU需要执行的运算
);

    always @(*) begin
        case (aluOp_2)
            2'b00: aluControl_5 = 5'b00000;  // add
            2'b01: aluControl_5 = 5'b01000;  // sub
            2'b10:  // R-type
            case ({
                funct7_25, funct3, funct7_30
            })
                5'b0_000_0: aluControl_5 = 5'b00000;  // add
                5'b0_000_1: aluControl_5 = 5'b01000;  // sub
                5'b0_100_0: aluControl_5 = 5'b00100;  // xor
                5'b0_110_0: aluControl_5 = 5'b00110;  // or
                5'b0_111_0: aluControl_5 = 5'b00111;  // and

                5'b0_001_0: aluControl_5 = 5'b00001;  // sll
                5'b0_101_0: aluControl_5 = 5'b00101;  // srl
                5'b0_101_1: aluControl_5 = 5'b01101;  // sra

                5'b0_010_0: aluControl_5 = 5'b00010;  // slt
                5'b0_011_0: aluControl_5 = 5'b00011;  // sltu


                5'b1_000_0: aluControl_5 = 5'b10000;  // mul
                5'b1_001_0: aluControl_5 = 5'b10001;  // mulh
                5'b1_010_0: aluControl_5 = 5'b10010;  // mulhsu
                5'b1_011_0: aluControl_5 = 5'b10011;  // mulhu

                5'b1_100_0: aluControl_5 = 5'b10100;  // div
                5'b1_101_0: aluControl_5 = 5'b10101;  // divu

                5'b1_110_0: aluControl_5 = 5'b10110;  // rem
                5'b1_111_0: aluControl_5 = 5'b10111;  // remu

                default: aluControl_5 = 5'bxxxxx;
            endcase
            2'b11:  // I-type
            case ({
                funct3
            })
                3'b000: aluControl_5 = 5'b00000;  // addi
                3'b100: aluControl_5 = 5'b00100;  // xori
                3'b110: aluControl_5 = 5'b00110;  // ori
                3'b111: aluControl_5 = 5'b00111;  // andi

                3'b001: aluControl_5 = 5'b00001;  // slli imm[5:0]
                3'b101: begin
                    if (funct7_30 == 1'b0) aluControl_5 = 5'b00101;  // srli imm[5:0]
                    else aluControl_5 = 5'b01101;  // srai imm[5:0]
                end

                3'b010:  aluControl_5 = 5'b00010;  // slti
                3'b011:  aluControl_5 = 5'b00011;  // sltiu
                default: aluControl_5 = 5'bxxxxx;
            endcase
        endcase
    end
endmodule
