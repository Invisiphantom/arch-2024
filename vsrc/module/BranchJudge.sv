module BranchJudge (
    input            branch,  // 当前是否为分支指令
    input            zero,    // 结果为零
    input            s_less,  // 有符号小于
    input            u_less,  // 无符号小于
    input      [2:0] funct3,  // instruction[14:12]
    output reg       cnd      // 是否执行分支跳转
);
    /*
 beq : funct3==000 && zero==1      bne : funct3==001 && zero==0
 blt : funct3==100 && s_less==1    bge : funct3==101 && s_less==0
 bltu: funct3==110 && u_less==1    bgeu: funct3==111 && u_less==0
*/

    always @(*) begin // 纯组合逻辑
        if (branch == 1'b1)
            case (funct3[2:1])
                2'b00:   cnd = funct3[0] ^ zero;
                2'b10:   cnd = funct3[0] ^ s_less;
                2'b11:   cnd = funct3[0] ^ u_less;
                default: cnd = 1'bx;
            endcase
        else cnd = 1'b0;
    end
endmodule
