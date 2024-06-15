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

    wire A0_clk;
    assign A0_clk = clk;


    reg [63:0] A1_IF_PCaddress;
    always @(posedge clk) begin
        if (reset) begin  // 复位赋值PCINIT
            A1_IF_PCaddress <= PCINIT;
        end else begin
            if (MEM_wait | EXE_wait) A1_IF_PCaddress <= A1_IF_PCaddress;
            else if (IF_mret) A1_IF_PCaddress <= u_Regs_CSR.mepc;
            else if (IF_ecall) A1_IF_PCaddress <= u_Regs_CSR.mtvec;
            else if (IF_jump) A1_IF_PCaddress <= A1_IF_PCaddress + IF_imm;
            else if (IF_jumpReg) A1_IF_PCaddress <= IF_readData1_R + IF_imm;
            else if (EXE_cnd) A1_IF_PCaddress <= EXE_PCaddress + EXE_imm;
            else if (iresp.data_ok) A1_IF_PCaddress <= A1_IF_PCaddress + 4;
        end
    end


    wire [63:0] A1_IF_PCaddress_;
    assign A1_IF_PCaddress_ = ireq.addr;


    reg  IF_PC_mmu_run;
    wire IF_PC_mmu_wait;
    wire IF_PC_mmu_ok;
    assign IF_PC_mmu_wait = IF_PC_mmu_run & ~IF_PC_mmu_ok;  // 异步拉低




    always @(posedge clk) begin
        if (reset) begin
            IF_PC_mmu_run = 1'b0;
            ireq.valid = 1'b1;
        end else begin
            if (iresp.data_ok) begin  // PC地址发送改变
                ireq.valid = 1'b0;  // 关闭ibus
                IF_PC_mmu_run = 1'b1;  // 开始地址转换
            end else if (IF_PC_mmu_ok == 1'b1) begin
                ireq.valid = 1'b1;  // 开启ibus
                IF_PC_mmu_run = 1'b0;  // 结束地址转换
            end
        end
    end


    wire [31:0] A1_IF_instruction;
    assign A1_IF_instruction = (MEM_wait | EXE_wait | ~iresp.data_ok) ? 32'b0 : iresp.data;  // 取得指令


    wire [1:0] IF_aluOp_2;
    wire IF_regWrite, IF_aluSrc, IF_aluSext, IF_memRead, IF_memWrite;
    wire IF_branch, IF_jump, IF_jumpReg, IF_lui, IF_auipc, IF_csrrx;
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
        .csrrx   (IF_csrrx)
    );

    wire IF_ecall, IF_mret;
    assign IF_ecall = (A1_IF_instruction == 32'b0000_0000_0000_00000_000_00000_1110011);
    assign IF_mret  = (A1_IF_instruction == 32'b0011_0000_0010_00000_000_00000_1110011);


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
        .regGroup   (regGroup)
    );


    wire [63:0] IF_readData_CSR;  // 数据CSRs[csr]
    Regs_CSR u_Regs_CSR (
        .clk         (clk),
        .reset       (reset),
        .rw_CSR      (A1_IF_instruction[31:20]),
        .readData_CSR(IF_readData_CSR)
    );


    wire [63:0] IF_imm;
    ImmGen u_ImmGen (
        .instruction(A1_IF_instruction),
        .imm        (IF_imm)
    );



    reg  [63:0] temp;
    wire [ 4:0] IF_writeCSR_zimm;
    assign IF_writeCSR_zimm = A1_IF_instruction[19:15];
    always @(posedge clk) begin
        if (reset) begin
            u_Regs_CSR.mode = M_Mode;
            u_Regs_CSR.mstatus = 64'b0;
            u_Regs_CSR.mie = 64'b0;
            u_Regs_CSR.mtvec = 64'b0;
            u_Regs_CSR.mscratch = 64'b0;
            u_Regs_CSR.mepc = 64'b0;
            u_Regs_CSR.mcause = 64'b0;
            u_Regs_CSR.mtval = 64'b0;
            u_Regs_CSR.mip = 64'b0;
            u_Regs_CSR.satp = 64'b0;
        end else if (IF_csrrx) begin  // 在EXE阶段的上升沿触发
            case (A1_IF_instruction[14:12])
                3'b001:  temp = IF_readData1_R;  // csrrw
                3'b010:  temp = IF_readData_CSR | IF_readData1_R;  // csrrs
                3'b011:  temp = IF_readData_CSR & ~IF_readData1_R;  // csrrc
                3'b101:  temp = {59'b0, IF_writeCSR_zimm};  // csrrwi
                3'b110:  temp = IF_readData_CSR | {59'b0, IF_writeCSR_zimm};  // csrrsi
                3'b111:  temp = IF_readData_CSR & ~{59'b0, IF_writeCSR_zimm};  // csrrci
                default: temp = 64'h0;
            endcase
            case (A1_IF_instruction[31:20])
                12'h300: u_Regs_CSR.mstatus = temp;
                12'h304: u_Regs_CSR.mie = temp;
                12'h305: u_Regs_CSR.mtvec = temp;
                12'h340: u_Regs_CSR.mscratch = temp;
                12'h341: u_Regs_CSR.mepc = temp;
                12'h342: u_Regs_CSR.mcause = temp;
                12'h343: u_Regs_CSR.mtval = temp;
                12'h344: u_Regs_CSR.mip = temp;
                12'h180: u_Regs_CSR.satp = temp;

                12'b0000_0000_0000: begin  // ecall
                    u_Regs_CSR.mepc = A1_IF_PCaddress;
                    u_Regs_CSR.mcause[63] = 1'b0;
                    u_Regs_CSR.mcause[3:0] = 4'b1000;
                    u_Regs_CSR.mstatus.mpie = u_Regs_CSR.mstatus.mie;
                    u_Regs_CSR.mstatus.mie = 1'b0;
                    u_Regs_CSR.mstatus.mpp = u_Regs_CSR.mode;
                    u_Regs_CSR.mode = M_Mode;
                end
                12'b0011_0000_0010: begin  // mret
                    u_Regs_CSR.mstatus.mie = u_Regs_CSR.mstatus.mpie;
                    u_Regs_CSR.mstatus.mpie = 1'b1;
                    u_Regs_CSR.mode = u_Regs_CSR.mstatus.mpp;
                    u_Regs_CSR.mstatus.mpp = U_Mode;
                end
                default: ;
            endcase
        end
    end





    // IF_EXE
    reg [63:0] EXE_PCaddress;
    reg [31:0] A2_EXE_instruction;
    reg EXE_regWrite, EXE_aluSrc, EXE_aluSext, EXE_memRead, EXE_memWrite;
    reg EXE_branch, EXE_jump, EXE_jumpReg, EXE_lui, EXE_auipc, EXE_csrrx;
    reg [ 4:0] EXE_aluControl_5;
    reg [63:0] EXE_readData1_R;
    reg [63:0] EXE_readData2_R;
    reg [63:0] EXE_readData_CSR;
    reg [63:0] EXE_imm;
    always @(posedge clk) begin
        if (reset) begin
            EXE_PCaddress      <= 64'b0;
            A2_EXE_instruction <= 32'b0;
            EXE_regWrite       <= 1'b0;
            EXE_aluSrc         <= 1'b0;
            EXE_aluSext        <= 1'b0;
            EXE_memRead        <= 1'b0;
            EXE_memWrite       <= 1'b0;
            EXE_branch         <= 1'b0;
            EXE_jump           <= 1'b0;
            EXE_jumpReg        <= 1'b0;
            EXE_lui            <= 1'b0;
            EXE_auipc          <= 1'b0;
            EXE_csrrx          <= 1'b0;
            EXE_aluControl_5   <= 5'b0;
            EXE_readData1_R    <= 64'b0;
            EXE_readData2_R    <= 64'b0;
            EXE_readData_CSR   <= 64'b0;
            EXE_imm            <= 64'b0;
        end else if (~(MEM_wait | EXE_wait)) begin
            EXE_PCaddress      <= A1_IF_PCaddress;
            A2_EXE_instruction <= A1_IF_instruction;
            EXE_regWrite       <= IF_regWrite;
            EXE_aluSrc         <= IF_aluSrc;
            EXE_aluSext        <= IF_aluSext;
            EXE_memRead        <= IF_memRead;
            EXE_memWrite       <= IF_memWrite;
            EXE_branch         <= IF_branch;
            EXE_jump           <= IF_jump;
            EXE_jumpReg        <= IF_jumpReg;
            EXE_lui            <= IF_lui;
            EXE_auipc          <= IF_auipc;
            EXE_csrrx          <= IF_csrrx;
            EXE_aluControl_5   <= IF_aluControl_5;
            EXE_readData1_R    <= IF_readData1_R;
            EXE_readData2_R    <= IF_readData2_R;
            EXE_readData_CSR   <= IF_readData_CSR;
            EXE_imm            <= IF_imm;
        end
    end

    // --------------------------------------------------


    reg  EXE_run;
    wire EXE_wait;  // 等待EXE阶段完成
    assign EXE_wait = EXE_run & ~alu_ok;  // 异步拉低
    always @(posedge clk) begin
        if (reset) EXE_run <= 1'b0;  // 初始化信号
        else if (IF_aluControl_5[4] == 1'b1) EXE_run <= 1'b1;  // EXE阶段开始
        else if (alu_ok) EXE_run <= 1'b0;  // EXE阶段结束
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
    always @(*) begin  // 处理WB-EXE数据冒险
        if (EXE_lui == 1'b1) aluA = 64'b0;  // lui [rd=0+imm]
        else if (EXE_auipc == 1'b1) aluA = EXE_PCaddress;  // auipc [rd=PC+imm]
        else begin  // rs1
            aluA = EXE_readData1_R;
            if (WB_regWrite & (EXE_rs1 != 5'b0) & (EXE_rs1 == WB_rd)) aluA = WB_writeData_R;
            if (MEM_regWrite & (EXE_rs1 != 5'b0) & (EXE_rs1 == MEM_rd))
                if (~MEM_memRead)  // 非load指令
                    if (MEM_jump | MEM_jumpReg) aluA = MEM_PCaddress + 4;
                    else aluA = MEM_aluResult;  // RI-type, lui, auipc
        end
    end

    reg [63:0] aluB;
    always @(*) begin  // 处理WB-EXE数据冒险
        if (EXE_aluSrc == 1'b1) aluB = EXE_imm;  // imm
        else begin  // rs2
            aluB = EXE_readData2_R;
            if (WB_regWrite & (EXE_rs2 != 5'b0) & (EXE_rs2 == WB_rd)) aluB = WB_writeData_R;
            if (MEM_regWrite & (EXE_rs2 != 5'b0) & (EXE_rs2 == MEM_rd))
                if (~MEM_memRead)  // 非load指令
                    if (MEM_jump | MEM_jumpReg) aluB = MEM_PCaddress + 4;
                    else aluB = MEM_aluResult;  // RI-type, lui, auipc
        end
    end

    wire alu_ok;
    wire zero, s_less, u_less;
    wire [63:0] EXE_aluResult;
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



    // EXE_MEM
    reg [63:0] MEM_PCaddress;
    reg [31:0] A3_MEM_instruction;
    reg MEM_regWrite, MEM_memRead, MEM_memWrite, MEM_jump, MEM_jumpReg, MEM_csrrx;
    reg [63:0] MEM_readData1_R;
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
            MEM_csrrx          <= 1'b0;
            MEM_readData1_R    <= 64'b0;
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
            MEM_csrrx <= EXE_csrrx;
            // 处理WB-MEM数据冒险
            MEM_readData1_R <= EXE_readData1_R;
            if (EXE_memWrite && WB_regWrite && A4_WB_instruction[11:7] == A2_EXE_instruction[24:20] && A4_WB_instruction[11:7] != 5'b0) MEM_readData2_R <= WB_writeData_R;
            else MEM_readData2_R <= EXE_readData2_R;
            MEM_readData_CSR <= EXE_readData_CSR;
            MEM_aluResult <= EXE_aluResult;
        end
    end

    // --------------------------------------------------


    reg  MEM_run;
    wire MEM_wait;  // 等待MEM阶段完成
    assign MEM_wait = MEM_run & ~dresp.data_ok;  // 异步拉低
    always @(posedge clk) begin
        if (reset) MEM_run <= 1'b0;
        else if (EXE_memRead | EXE_memWrite) MEM_run <= 1'b1;  // MEM阶段开始
        else if (dresp.data_ok) MEM_run <= 1'b0;  // MEM阶段结束
    end


    // 处理WB-MEM数据冒险
    reg [63:0] writeData_M;
    always @(*) begin
        if (MEM_memWrite && WB_regWrite && A4_WB_instruction[11:7] == A3_MEM_instruction[24:20] && A4_WB_instruction[11:7] != 5'b0) writeData_M = WB_writeData_R;
        else writeData_M = MEM_readData2_R;
    end


    wire [63:0] MEM_readData_M;
    DataMem u_DataMem (
        .clk  (clk),
        .reset(reset),
        .memRead    (MEM_memRead),
        .memWrite   (MEM_memWrite),
        .funct3     (A3_MEM_instruction[14:12]),
        .memAddr    (MEM_aluResult),
        .writeData_M(writeData_M),
        .readData_M (MEM_readData_M),
        // MMU虚拟地址转换
        .mmu_wait(IF_PC_mmu_wait),
        .virAddr (A1_IF_PCaddress),
        .mode    (u_Regs_CSR.mode),
        .satp    (u_Regs_CSR.satp),
        .mmu_ok  (IF_PC_mmu_ok),
        .phyAddr (ireq.addr),
        .dreq (dreq),
        .dresp(dresp)
    );





    // 计算需要写回的数据
    wire [63:0] MEM_writeData_R;
    RegWrite u_RegWrite (
        .memRead     (MEM_memRead),
        .jump        (MEM_jump),
        .jumpReg     (MEM_jumpReg),
        .csrrx       (MEM_csrrx),
        .aluResult   (MEM_aluResult),
        .readData_M  (MEM_readData_M),
        .PCaddress   (MEM_PCaddress),
        .readData_CSR(MEM_readData_CSR),
        .writeData_R (MEM_writeData_R)
    );

    wire MEM_rd;
    assign MEM_rd = A3_MEM_instruction[11:7];
    always @(posedge clk) begin  // 在WB阶段的上升沿触发
        if (reset) for (integer i = 0; i < 32; i = i + 1) u_Regs.Register[i] = 64'b0;
        else if (MEM_regWrite && MEM_rd != 5'b00000 && ~(MEM_wait | EXE_wait)) begin  // x0寄存器不可写
            u_Regs.Register[MEM_rd] = MEM_writeData_R;
        end else u_Regs.Register[0] = 64'h0;
    end









    // MEM_WB
    reg [63:0] WB_PCaddress;
    reg [63:0] WB_aluResult;
    reg [63:0] WB_writeData_R;
    reg [31:0] A4_WB_instruction;
    reg WB_regWrite, WB_memRead;
    always @(posedge clk) begin
        if (reset) begin
            WB_PCaddress      <= 64'b0;
            A4_WB_instruction <= 32'b0;
            WB_regWrite       <= 1'b0;
            WB_memRead        <= 1'b0;
            WB_aluResult      <= 64'b0;
        end else if (~(MEM_wait | EXE_wait)) begin
            WB_PCaddress      <= MEM_PCaddress;
            A4_WB_instruction <= A3_MEM_instruction;
            WB_regWrite       <= MEM_regWrite;
            WB_memRead        <= MEM_memRead;
            WB_aluResult      <= MEM_aluResult;
            WB_writeData_R    <= MEM_writeData_R;
        end
    end






`ifdef VERILATOR


    wire commit_valid, commit_skip;
    assign commit_valid = (|A4_WB_instruction) & ~(MEM_wait | EXE_wait);
    assign commit_skip  = WB_memRead & (WB_aluResult[31] == 1'b0);
    DifftestInstrCommit DifftestInstrCommit (
        .clock   (clk),
        .coreid  (0),
        .index   (0),
        .valid   (commit_valid),
        .pc      (WB_PCaddress),
        .instr   (A4_WB_instruction),
        .skip    (commit_skip),
        .isRVC   (0),
        .scFailed(0),
        .wen     (WB_regWrite),                      // 这条指令是否写入寄存器
        .wdest   ({3'b0, A4_WB_instruction[11:7]}),  // 写入哪个通用寄存器
        .wdata   (WB_writeData_R)                    // 写入的数据
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
        .priviledgeMode(u_Regs_CSR.mode),
        .mstatus       (u_Regs_CSR.mstatus),
        .sstatus       (u_Regs_CSR.mstatus & 64'h800000030001e000),
        .mepc          (u_Regs_CSR.mepc),
        .sepc          (0),
        .mtval         (u_Regs_CSR.mtval),
        .stval         (0),
        .mtvec         (u_Regs_CSR.mtvec),
        .stvec         (0),
        .mcause        (u_Regs_CSR.mcause),
        .scause        (0),
        .satp          (u_Regs_CSR.satp),
        .mip           (u_Regs_CSR.mip),
        .mie           (u_Regs_CSR.mie),
        .mscratch      (u_Regs_CSR.mscratch),
        .sscratch      (0),
        .mideleg       (0),
        .medeleg       (0)
    );




`endif
endmodule
`endif
