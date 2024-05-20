`ifndef __CORE_SV
`define __CORE_SV

`ifdef VERILATOR
`include "include/common.sv"
`include "module/Control.sv"
`include "module/Regs.sv"
`include "module/Regs_CSR.sv"
`include "module/ImmGen.sv"
`include "module/ALUControl.sv"
`include "module/ALU.sv"
`include "module/BranchJudge.sv"
`include "module/DataMem.sv"
`include "module/RegWrite.sv"
`endif


module core
    import common::*;
(
    input  logic       clk,
    input  logic       reset,
    output ibus_req_t  ireq,
    input  ibus_resp_t iresp,
    output dbus_req_t  dreq,
    input  dbus_resp_t dresp,
    input  logic       trint,
    input  logic       swint,
    input  logic       exint
);

    reg [63:0] A1_IF_PCaddress;
    always @(posedge clk) begin
        if (reset) begin  // 复位赋值PCINIT
            A1_IF_PCaddress <= PCINIT;
        end else begin
            if (MEM_wait | EXE_wait) A1_IF_PCaddress <= A1_IF_PCaddress;
            else if (IF_jump) A1_IF_PCaddress <= A1_IF_PCaddress + IF_imm;
            else if (IF_jumpReg) A1_IF_PCaddress <= IF_readData1_R + IF_imm;
            else if (EXE_cnd) A1_IF_PCaddress <= EXE_PCaddress + EXE_imm;
            else if (iresp.data_ok) A1_IF_PCaddress <= A1_IF_PCaddress + 4;
            else A1_IF_PCaddress <= A1_IF_PCaddress;
        end
    end

    assign ireq.valid = 1'b1;
    assign ireq.addr  = A1_IF_PCaddress;

    wire [31:0] A1_IF_instruction;
    assign A1_IF_instruction = (MEM_wait | EXE_wait | ~iresp.data_ok) ? 32'b0 : iresp.data;  // 取得指令




    wire [1:0] IF_aluOp_2;
    wire IF_regWrite, IF_aluSrc, IF_aluSext, IF_memRead, IF_memWrite;
    wire IF_branch, IF_jump, IF_jumpReg, IF_lui, IF_auipc, IF_excep;
    Control u_Control (
        .Opcode_7(A1_IF_instruction[6:0]),
        .regWrite(IF_regWrite),
        .aluSrc  (IF_aluSrc),
        .aluOp_2 (IF_aluOp_2),
        .aluSext (IF_aluSext),
        .memRead (IF_memRead),
        .memWrite(IF_memWrite),
        .branch  (IF_branch),
        .jump    (IF_jump),
        .jumpReg (IF_jumpReg),
        .lui     (IF_lui),
        .auipc   (IF_auipc),
        .excep   (IF_excep)
    );

    wire [4:0] IF_aluControl_5;
    ALUControl u_ALUControl (
        .aluOp_2     (IF_aluOp_2),
        .funct7_25   (A1_IF_instruction[25]),
        .funct7_30   (A1_IF_instruction[30]),
        .funct3      (A1_IF_instruction[14:12]),
        .aluControl_5(IF_aluControl_5)
    );





    wire [63:0] IF_readData1_R;  // 数据x[rs1]
    wire [63:0] IF_readData2_R;  // 数据x[rs2]
    regs_t regGroup;  // 外接寄存器组用于commit
    Regs u_Regs (
        .clk        (clk),
        .reset      (reset),
        .readReg1   (A1_IF_instruction[19:15]),
        .readReg2   (A1_IF_instruction[24:20]),
        .readData1_R(IF_readData1_R),
        .readData2_R(IF_readData2_R),

        .regWrite   (WB_regWrite),
        .writeReg   (A4_WB_instruction[11:7]),
        .writeData_R(WB_writeData_R),
        .regGroup   (regGroup)
    );


    wire [63:0] IF_readData_CSR;  // 数据CSRs[csr]
    csrs_t csrGroup;  // 外接寄存器组用于commit
    Regs_CSR u_Regs_CSR (
        .clk   (clk),
        .reset (reset),
        .excep (IF_excep),
        .funct3(A1_IF_instruction[14:12]),
        .rw_CSR(A1_IF_instruction[31:20]),

        .writeCSR_rs1 (IF_readData1_R),
        .writeCSR_zimm(A1_IF_instruction[19:15]),
        .readData_CSR (IF_readData_CSR),
        .csrGroup     (csrGroup)
    );


    wire [63:0] IF_imm;
    ImmGen u_ImmGen (
        .instruction(A1_IF_instruction),
        .imm        (IF_imm)
    );




    // IF_EXE
    reg [63:0] EXE_PCaddress;
    reg [31:0] A2_EXE_instruction;
    reg EXE_regWrite, EXE_aluSrc, EXE_aluSext, EXE_memRead, EXE_memWrite;
    reg EXE_branch, EXE_jump, EXE_jumpReg, EXE_lui, EXE_auipc, EXE_excep;
    reg [ 4:0] EXE_aluControl_5;
    reg [63:0] EXE_readData1_R_old;
    reg [63:0] EXE_readData2_R_old;
    reg [63:0] EXE_readData_CSR;
    reg [63:0] EXE_imm;
    always @(posedge clk) begin
        if (reset) begin
            EXE_PCaddress       <= 64'b0;
            A2_EXE_instruction  <= 32'b0;
            EXE_regWrite        <= 1'b0;
            EXE_aluSrc          <= 1'b0;
            EXE_aluSext         <= 1'b0;
            EXE_memRead         <= 1'b0;
            EXE_memWrite        <= 1'b0;
            EXE_branch          <= 1'b0;
            EXE_jump            <= 1'b0;
            EXE_jumpReg         <= 1'b0;
            EXE_lui             <= 1'b0;
            EXE_auipc           <= 1'b0;
            EXE_excep           <= 1'b0;
            EXE_aluControl_5    <= 5'b0;
            EXE_readData1_R_old <= 64'b0;
            EXE_readData2_R_old <= 64'b0;
            EXE_readData_CSR    <= 64'b0;
            EXE_imm             <= 64'b0;
        end else if (~(MEM_wait | EXE_wait)) begin
            EXE_PCaddress       <= A1_IF_PCaddress;
            A2_EXE_instruction  <= A1_IF_instruction;
            EXE_regWrite        <= IF_regWrite;
            EXE_aluSrc          <= IF_aluSrc;
            EXE_aluSext         <= IF_aluSext;
            EXE_memRead         <= IF_memRead;
            EXE_memWrite        <= IF_memWrite;
            EXE_branch          <= IF_branch;
            EXE_jump            <= IF_jump;
            EXE_jumpReg         <= IF_jumpReg;
            EXE_lui             <= IF_lui;
            EXE_auipc           <= IF_auipc;
            EXE_excep           <= IF_excep;
            EXE_aluControl_5    <= IF_aluControl_5;
            EXE_readData1_R_old <= IF_readData1_R;
            EXE_readData2_R_old <= IF_readData2_R;
            EXE_readData_CSR    <= IF_readData_CSR;
            EXE_imm             <= IF_imm;
        end else begin
            EXE_PCaddress       <= EXE_PCaddress;
            A2_EXE_instruction  <= A2_EXE_instruction;
            EXE_regWrite        <= EXE_regWrite;
            EXE_aluSrc          <= EXE_aluSrc;
            EXE_aluSext         <= EXE_aluSext;
            EXE_memRead         <= EXE_memRead;
            EXE_memWrite        <= EXE_memWrite;
            EXE_branch          <= EXE_branch;
            EXE_jump            <= EXE_jump;
            EXE_jumpReg         <= EXE_jumpReg;
            EXE_lui             <= EXE_lui;
            EXE_auipc           <= EXE_auipc;
            EXE_excep           <= EXE_excep;
            EXE_aluControl_5    <= EXE_aluControl_5;
            EXE_readData1_R_old <= EXE_readData1_R_old;
            EXE_readData2_R_old <= EXE_readData2_R_old;
            EXE_readData_CSR    <= EXE_readData_CSR;
            EXE_imm             <= EXE_imm;
        end
    end

    // --------------------------------------------------


    wire alu_ok;
    reg  EXE_wait;
    always @(posedge clk) begin
        if (reset) EXE_wait <= 1'b0;
        else if (IF_aluControl_5[4] == 1'b1) EXE_wait <= 1'b1;  // EXE阶段开始
        else if (alu_ok) EXE_wait <= 1'b0;  // EXE阶段结束
        else EXE_wait <= EXE_wait;  // 信号保持
    end



    wire [4:0] EXE_rs1;
    wire [4:0] EXE_rs2;
    wire [4:0] MEM_rd;
    wire [4:0] WB_rd;
    assign EXE_rs1 = A2_EXE_instruction[19:15];
    assign EXE_rs2 = A2_EXE_instruction[24:20];
    assign MEM_rd  = A3_MEM_instruction[11:7];
    assign WB_rd   = A4_WB_instruction[11:7];

    reg [63:0] aluA;
    always @(*) begin
        if (EXE_lui == 1'b1) aluA = 64'b0;  // lui [rd=0+imm]
        else if (EXE_auipc == 1'b1) aluA = EXE_PCaddress;  // auipc [rd=PC+imm]
        else begin  // rs1
            aluA = EXE_readData1_R_old;
            if (WB_regWrite & (EXE_rs1 != 5'b0) & (EXE_rs1 == WB_rd)) aluA = WB_writeData_R;
            if (MEM_regWrite & (EXE_rs1 != 5'b0) & (EXE_rs1 == MEM_rd))
                if (~MEM_memRead)  // 非load指令
                    if (MEM_jump | MEM_jumpReg) aluA = MEM_PCaddress + 4;
                    else aluA = MEM_aluResult;  // RI-type, lui, auipc
        end
    end

    reg [63:0] aluB;
    always @(*) begin
        if (EXE_aluSrc == 1'b1) aluB = EXE_imm;  // imm
        else begin  // rs2
            aluB = EXE_readData2_R_old;
            if (WB_regWrite & (EXE_rs2 != 5'b0) & (EXE_rs2 == WB_rd)) aluB = WB_writeData_R;
            if (MEM_regWrite & (EXE_rs2 != 5'b0) & (EXE_rs2 == MEM_rd))
                if (~MEM_memRead)  // 非load指令
                    if (MEM_jump | MEM_jumpReg) aluB = MEM_PCaddress + 4;
                    else aluB = MEM_aluResult;  // RI-type, lui, auipc
        end
    end


    wire [63:0] EXE_aluResult;
    wire zero, s_less, u_less;
    ALU u_ALU (
        .clk         (clk),
        .reset       (reset),
        .aluControl_5(EXE_aluControl_5),
        .aluSext     (EXE_aluSext),
        .aluA_in     (aluA),
        .aluB_in     (aluB),
        .aluResult   (EXE_aluResult),
        .zero        (zero),
        .s_less      (s_less),
        .u_less      (u_less),
        .alu_ok      (alu_ok)
    );

    wire EXE_cnd;
    BranchJudge u_BranchJudge (
        .branch(EXE_branch),
        .zero  (zero),
        .s_less(s_less),
        .u_less(u_less),
        .funct3(A2_EXE_instruction[14:12]),
        .cnd   (EXE_cnd)
    );


    // 消除WB-MEM数据冒险
    reg [63:0] EXE_readData2_R;
    always @(*) begin
        if (EXE_memWrite & WB_regWrite & (A4_WB_instruction[11:7] == A2_EXE_instruction[24:20]) & (A4_WB_instruction[11:7] != 5'b0)) EXE_readData2_R = WB_writeData_R;
        else EXE_readData2_R = EXE_readData2_R_old;
    end


    // EXE_MEM
    reg [63:0] MEM_PCaddress;
    reg [31:0] A3_MEM_instruction;
    reg MEM_regWrite, MEM_memRead, MEM_memWrite, MEM_jump, MEM_jumpReg, MEM_excep;
    reg [63:0] MEM_readData2_R;
    reg [63:0] MEM_readData_CSR;
    reg [63:0] MEM_aluResult;
    always @(posedge clk) begin
        if (reset) begin
            MEM_PCaddress      <= 64'b0;
            A3_MEM_instruction <= 32'b0;
            MEM_regWrite       <= 1'b0;
            MEM_memRead        <= 1'b0;
            MEM_memWrite       <= 1'b0;
            MEM_jump           <= 1'b0;
            MEM_jumpReg        <= 1'b0;
            MEM_excep          <= 1'b0;
            MEM_readData2_R    <= 64'b0;
            MEM_readData_CSR   <= 64'b0;
            MEM_aluResult      <= 64'b0;
        end else if (~(MEM_wait | EXE_wait)) begin
            MEM_PCaddress <= EXE_PCaddress;
            A3_MEM_instruction <= A2_EXE_instruction;
            MEM_regWrite <= EXE_regWrite;
            MEM_memRead <= EXE_memRead;
            MEM_memWrite <= EXE_memWrite;
            MEM_jump <= EXE_jump;
            MEM_jumpReg <= EXE_jumpReg;
            MEM_excep <= EXE_excep;
            MEM_readData2_R <= EXE_readData2_R;
            MEM_readData_CSR <= EXE_readData_CSR;
            MEM_aluResult <= EXE_aluResult;
        end else begin
            MEM_PCaddress <= MEM_PCaddress;
            A3_MEM_instruction <= A3_MEM_instruction;
            MEM_regWrite <= MEM_regWrite;
            MEM_memRead <= MEM_memRead;
            MEM_memWrite <= MEM_memWrite;
            MEM_jump <= MEM_jump;
            MEM_jumpReg <= MEM_jumpReg;
            MEM_excep <= MEM_excep;
            MEM_readData2_R <= MEM_readData2_R;
            MEM_readData_CSR <= MEM_readData_CSR;
            MEM_aluResult <= MEM_aluResult;
        end
    end

    // --------------------------------------------------


    reg  MEM_run;
    wire MEM_wait;
    assign MEM_wait = MEM_run & ~dresp.data_ok;
    always @(posedge clk) begin
        if (reset) MEM_run <= 1'b0;
        else if (EXE_memRead | EXE_memWrite) MEM_run <= 1'b1;  // MEM阶段开始
        else if (dresp.data_ok) MEM_run <= 1'b0;  // MEM阶段结束
        else MEM_run <= MEM_run;  // 信号保持
    end


    // 消除WB-MEM数据冒险
    reg [63:0] writeData_M;
    always @(*) begin
        if (MEM_memWrite & WB_regWrite & (A4_WB_instruction[11:7] == A3_MEM_instruction[24:20]) & (A4_WB_instruction[11:7] != 5'b0)) writeData_M = WB_writeData_R;
        else writeData_M = MEM_readData2_R;
    end


    wire [63:0] MEM_readData_M;
    DataMem u_DataMem (
        .clk        (clk),
        .memRead    (MEM_memRead),
        .memWrite   (MEM_memWrite),
        .funct3     (A3_MEM_instruction[14:12]),
        .memAddr    (MEM_aluResult),
        .writeData_M(writeData_M),
        .readData_M (MEM_readData_M),
        .dreq       (dreq),
        .dresp      (dresp)
    );



    // MEM_WB
    reg [63:0] WB_PCaddress;
    reg [31:0] A4_WB_instruction;
    reg WB_regWrite, WB_memRead, WB_jump, WB_jumpReg, WB_excep;
    reg [63:0] WB_readData_CSR;
    reg [63:0] WB_aluResult;
    reg [63:0] WB_readData_M;
    always @(posedge clk) begin
        if (reset) begin
            WB_PCaddress      <= 64'b0;
            A4_WB_instruction <= 32'b0;
            WB_regWrite       <= 1'b0;
            WB_memRead        <= 1'b0;
            WB_jump           <= 1'b0;
            WB_jumpReg        <= 1'b0;
            WB_excep          <= 1'b0;
            WB_readData_CSR   <= 64'b0;
            WB_aluResult      <= 64'b0;
            WB_readData_M     <= 64'b0;
        end else if (~(MEM_wait | EXE_wait)) begin
            WB_PCaddress      <= MEM_PCaddress;
            A4_WB_instruction <= A3_MEM_instruction;
            WB_regWrite       <= MEM_regWrite;
            WB_memRead        <= MEM_memRead;
            WB_jump           <= MEM_jump;
            WB_jumpReg        <= MEM_jumpReg;
            WB_excep          <= MEM_excep;
            WB_readData_CSR   <= MEM_readData_CSR;
            WB_aluResult      <= MEM_aluResult;
            WB_readData_M     <= MEM_readData_M;
        end else begin
            WB_PCaddress      <= WB_PCaddress;
            A4_WB_instruction <= A4_WB_instruction;
            WB_regWrite       <= WB_regWrite;
            WB_memRead        <= WB_memRead;
            WB_jump           <= WB_jump;
            WB_jumpReg        <= WB_jumpReg;
            WB_excep          <= WB_excep;
            WB_readData_CSR   <= WB_readData_CSR;
            WB_aluResult      <= WB_aluResult;
            WB_readData_M     <= WB_readData_M;
        end
    end

    // --------------------------------------------------

    wire [63:0] WB_writeData_R;
    RegWrite u_RegWrite (
        .memRead     (WB_memRead),
        .jump        (WB_jump),
        .jumpReg     (WB_jumpReg),
        .excep       (WB_excep),
        .aluResult   (WB_aluResult),
        .readData_M  (WB_readData_M),
        .PCaddress   (WB_PCaddress),
        .readData_CSR(WB_readData_CSR),
        .writeData_R (WB_writeData_R)
    );





`ifdef VERILATOR
    DifftestInstrCommit DifftestInstrCommit (
        .clock   (clk),
        .coreid  (0),
        .index   (0),
        .valid   ((|A4_WB_instruction) & ~(MEM_wait | EXE_wait)),
        .pc      (WB_PCaddress),
        .instr   (A4_WB_instruction),
        .skip    (WB_memRead & (WB_aluResult[31] == 1'b0)),
        .isRVC   (0),
        .scFailed(0),
        .wen     (WB_regWrite),                                    // 这条指令是否写入寄存器
        .wdest   ({3'b0, A4_WB_instruction[11:7]}),                // 写入哪个通用寄存器
        .wdata   (WB_writeData_R)                                  // 写入的数据
    );

    DifftestArchIntRegState DifftestArchIntRegState (
        .clock (clk),
        .coreid(0),
        .gpr_0 (regGroup.x0),
        .gpr_1 (regGroup.ra),
        .gpr_2 (regGroup.sp),
        .gpr_3 (regGroup.gp),
        .gpr_4 (regGroup.tp),
        .gpr_5 (regGroup.t0),
        .gpr_6 (regGroup.t1),
        .gpr_7 (regGroup.t2),
        .gpr_8 (regGroup.s0),
        .gpr_9 (regGroup.s1),
        .gpr_10(regGroup.a0),
        .gpr_11(regGroup.a1),
        .gpr_12(regGroup.a2),
        .gpr_13(regGroup.a3),
        .gpr_14(regGroup.a4),
        .gpr_15(regGroup.a5),
        .gpr_16(regGroup.a6),
        .gpr_17(regGroup.a7),
        .gpr_18(regGroup.s2),
        .gpr_19(regGroup.s3),
        .gpr_20(regGroup.s4),
        .gpr_21(regGroup.s5),
        .gpr_22(regGroup.s6),
        .gpr_23(regGroup.s7),
        .gpr_24(regGroup.s8),
        .gpr_25(regGroup.s9),
        .gpr_26(regGroup.s10),
        .gpr_27(regGroup.s11),
        .gpr_28(regGroup.t3),
        .gpr_29(regGroup.t4),
        .gpr_30(regGroup.t5),
        .gpr_31(regGroup.t6)
    );

    DifftestTrapEvent DifftestTrapEvent (
        .clock   (clk),
        .coreid  (0),
        .valid   (0),
        .code    (0),
        .pc      (0),
        .cycleCnt(0),
        .instrCnt(0)
    );

    DifftestCSRState DifftestCSRState (
        .clock         (clk),
        .coreid        (0),
        .priviledgeMode(csrGroup.mode),
        .mstatus       (csrGroup.mstatus),
        .sstatus       (csrGroup.mstatus & 64'h800000030001e000),
        .mepc          (csrGroup.mepc),
        .sepc          (0),
        .mtval         (csrGroup.mtval),
        .stval         (0),
        .mtvec         (csrGroup.mtvec),
        .stvec         (0),
        .mcause        (csrGroup.mcause),
        .scause        (0),
        .satp          (0),
        .mip           (csrGroup.mip),
        .mie           (csrGroup.mie),
        .mscratch      (csrGroup.mscratch),
        .sscratch      (0),
        .mideleg       (0),
        .medeleg       (0)
    );
`endif
endmodule
`endif
