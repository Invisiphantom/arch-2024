`ifdef VERILATOR
`include "include/common.sv"
`endif


module ALU
    import common::*;
(
    input clk,   // 时钟信号
    input reset, // 复位信号

    input      [ 4:0] aluControl_5,  // ALU需要执行的运算
    input             aluSext,       // 是否进行符号扩展
    input      [63:0] aluA_in,       // ALU的数据端口A
    input      [63:0] aluB_in,       // ALU的数据端口B
    output     [63:0] aluResult,     // ALU的运算结果
    output            zero,          // 结果为零
    output            s_less,        // 有符号小于
    output            u_less,        // 无符号小于
    output reg        alu_ok
);

    wire [63:0] aluA;
    wire [63:0] aluB;
    assign aluA = aluSext ? {32'b0, aluA_in[31:0]} : aluA_in;
    assign aluB = aluSext ? {32'b0, aluB_in[31:0]} : aluB_in;

    assign zero = (aluResult == 64'b0);
    assign s_less = ($signed(aluA) < $signed(aluB));
    assign u_less = (aluA < aluB);
    assign aluResult = aluControl_5[4] == 0 ? calResult : mulResult;



    reg  [63:0] calRes;
    wire [63:0] calResult;
    assign calResult = (aluSext == 1'b1) ? {{32{calRes[31]}}, calRes[31:0]} : calRes;
    always @(*) begin
        case (aluControl_5[3:0])
            4'b0000: calRes = aluA + aluB;  // add
            4'b1000: calRes = aluA - aluB;  // sub
            4'b0100: calRes = aluA ^ aluB;  // xor
            4'b0110: calRes = aluA | aluB;  // or
            4'b0111: calRes = aluA & aluB;  // and
            4'b0001: begin
                if (aluSext == 1'b0) calRes = aluA << aluB[5:0];  // sll imm[5:0]
                else calRes = aluA << aluB[4:0];  // sllw imm[4:0]
            end
            4'b0101: begin
                if (aluSext == 1'b0) calRes = aluA >> aluB[5:0];  // srl imm[5:0]
                else calRes = {32'b0, aluA[31:0] >> aluB[4:0]};  // srlw imm[4:0]
            end
            4'b1101: begin
                if (aluSext == 1'b0) calRes = $signed(aluA) >>> aluB[5:0];  // sra imm[5:0]
                else calRes = {32'b0, $signed(aluA[31:0]) >>> aluB[4:0]};  // sraw imm[4:0]
            end
            4'b0010: calRes = {63'b0, s_less};  // slt
            4'b0011: calRes = {63'b0, u_less};  // sltu
            default: calRes = 64'b0;
        endcase
    end



    reg state;  // 当前乘除法状态机状态
    reg [63:0] mulRes;  // 计算得到的乘除法结果
    wire [63:0] mulResult;  // 经过符号位扩展后的乘除法结果
    assign mulResult = (aluSext == 1'b1) ? {{32{mulRes[31]}}, mulRes[31:0]} : mulRes;
    always @(posedge clk) begin
        if (reset) begin
            state  = 1'b0;
            alu_ok = 1'b0;
            mulRes = 64'b0;
        end else if (alu_ok) alu_ok = 1'b0;
        else if (aluControl_5[4] == 1'b1)
            case (aluControl_5[3:0])

            
                4'b0000, 4'b0001, 4'b0010, 4'b0011: begin  // Booth-Base4乘法器
                    reg [ 65:0] X;  // 符号位扩展的aluA
                    reg [ 65:0] X_n;  // -X
                    reg [ 65:0] X_2;  // X << 1
                    reg [ 65:0] X_2n;  // -X << 1
                    reg [ 66:0] Y_reg;  // Y[65:-1]

                    reg [ 65:0] Acc;  // 累加器
                    reg [  6:0] count;  // 当前处于的位数
                    reg [127:0] rawResult;  // 截成128位乘法结果

                    case (state)
                        1'b0: begin  // 初始化阶段
                            case (aluControl_5[3:0])
                                4'b0000: begin  // mul
                                    X = {aluA[63], aluA[63], aluA};  // $signed(aluA)
                                    Y_reg = {aluB[63], aluB[63], aluB, 1'b0};  // $signed(aluB)
                                end
                                4'b0001: begin  // mulh
                                    X = {aluA[63], aluA[63], aluA};  // $signed(aluA)
                                    Y_reg = {aluB[63], aluB[63], aluB, 1'b0};  // $signed(aluB)
                                end
                                4'b0010: begin  // mulhsu
                                    X = {aluA[63], aluA[63], aluA};  // $signed(aluA)
                                    Y_reg = {1'b0, 1'b0, aluB, 1'b0};  // $unsigned(aluB)
                                end
                                4'b0011: begin  // mulhu
                                    X = {1'b0, 1'b0, aluA};  // $unsigned(aluA)
                                    Y_reg = {1'b0, 1'b0, aluB, 1'b0};  // $unsigned(aluB)
                                end
                                default: ;
                            endcase
                            X_n   = -X;
                            X_2   = X << 1;
                            X_2n  = X_n << 1;

                            Acc   = 66'b0;
                            count = 7'b0;  // 从Y[0]开始计算
                            state = 1'b1;  // 进入循环移位累加阶段
                        end
                        1'b1: begin  // 循环移位累加阶段
                            case (Y_reg[2:0])
                                3'b001:  Acc = Acc + X;
                                3'b010:  Acc = Acc + X;
                                3'b011:  Acc = Acc + X_2;
                                3'b101:  Acc = Acc + X_n;
                                3'b110:  Acc = Acc + X_n;
                                3'b100:  Acc = Acc + X_2n;
                                default: ;
                            endcase
                            count = count + 2;  // 当前计算的位数+2
                            {Acc, Y_reg} = $signed({Acc, Y_reg}) >>> 2;
                            if (count == 7'b1000010) begin
                                state = 1'b0;  // 初始化阶段
                                alu_ok = 1'b1;  // 乘法完成
                                rawResult = {Acc[61:0], Y_reg[66:1]};
                            end
                        end
                    endcase

                    case (aluControl_5[3:0])
                        4'b0000: mulRes = rawResult[63:0];  // mul
                        4'b0001, 4'b0010, 4'b0011: mulRes = rawResult[127:64];  // mulh, mulhsu, mulhu
                        default: mulRes = 64'b0;
                    endcase
                end


                4'b0100, 4'b0101, 4'b0110, 4'b0111: begin  // 除法器
                    reg [  6:0] count;  // 当前计算的位数
                    reg [127:0] X_reg;  // 被除数
                    reg [127:0] Y_reg;  // 除数
                    reg [ 63:0] Q_reg;  // 商

                    case (state)
                        1'b0: begin  // 初始化
                            case (aluControl_5[3:0])
                                4'b0100, 4'b0110: begin  // div, rem
                                    if (aluSext == 1'b0) begin  // 64bits
                                        // 将被除数与除数取绝对值
                                        X_reg[127:64] = 64'b0;
                                        X_reg[63:0]   = ($signed(aluA) < 0) ? -aluA : aluA;
                                        Y_reg[63:0]   = 64'b0;
                                        Y_reg[127:64] = ($signed(aluB) < 0) ? -aluB : aluB;
                                    end else begin  // 32bits
                                        // 将被除数与除数取绝对值
                                        X_reg[127:32] = 96'b0;
                                        X_reg[31:0] = ($signed(aluA[31:0]) < 0) ? -aluA[31:0] : aluA[31:0];
                                        {Y_reg[127:96], Y_reg[63:0]} = 96'b0;
                                        Y_reg[95:64] = ($signed(aluB[31:0]) < 0) ? -aluB[31:0] : aluB[31:0];
                                    end
                                end
                                4'b0101, 4'b0111: begin  // divu, remu
                                    X_reg = {64'b0, aluA};
                                    Y_reg = {aluB, 64'b0};
                                end
                                default: ;
                            endcase
                            count = 0;
                            Q_reg = 64'b0;
                            state = 1'b1;
                        end

                        1'b1: begin  // 循环移位相减
                            Y_reg = Y_reg >> 1;
                            Q_reg = Q_reg << 1;
                            if (X_reg >= Y_reg) begin
                                X_reg = X_reg - Y_reg;
                                Q_reg = Q_reg + 1;
                            end
                            count = count + 1;

                            // 64次循环运算结束, 开始处理结果的符号
                            if (count == 7'b100_0000) begin
                                case (aluControl_5[3:0])
                                    4'b0100: begin  // div (商符号 被除数^除数)
                                        if (aluSext == 1'b0) begin  // 64bits
                                            if (aluB == 64'b0) mulRes = {64{1'b1}};  // div 0
                                            else mulRes = (($signed(aluA) < 0) ^ ($signed(aluB) < 0)) ? -Q_reg : Q_reg;
                                        end else begin  // 32bits
                                            if (aluB[31:0] == 32'b0) mulRes[31:0] = {32{1'b1}};  // div 0
                                            else mulRes[31:0] = (($signed(aluA[31:0]) < 0) ^ ($signed(aluB[31:0]) < 0)) ? -Q_reg[31:0] : Q_reg[31:0];
                                        end
                                    end
                                    4'b0101: begin  // divu
                                        if (aluB == 64'b0) mulRes = {64{1'b1}};  // divu 0
                                        mulRes = Q_reg;
                                    end
                                    4'b0110: begin  // rem (余数符号 保持与被除数相同)
                                        if (aluSext == 1'b0) mulRes = ($signed(aluA) < 0) ? -X_reg[63:0] : X_reg[63:0];
                                        else mulRes[31:0] = ($signed(aluA[31:0]) < 0) ? -X_reg[31:0] : X_reg[31:0];
                                    end
                                    4'b0111: mulRes = X_reg[63:0];  // remu
                                    default: mulRes = 64'b0;
                                endcase

                                state  = 1'b0;  // 初始化阶段
                                alu_ok = 1'b1;  // 除法完成
                            end
                        end
                    endcase
                end
                default: mulRes = 64'b0;
            endcase
    end


endmodule
