`ifdef VERILATOR
`include "include/common.sv"
`endif

module DataMem
    import common::*;
(
    input clk,   // 时钟信号
    input reset, // 复位信号

    input             memRead,      // 是否需要读内存
    input             memWrite,     // 是否需要写内存
    input      [ 2:0] funct3,       // 0__:sign 1__:unsign  (b:00 h:01 w:10 d:11)
    input      [63:0] memAddr,      // 虚拟内存地址
    input      [63:0] writeData_M,  // 写入的数据
    output reg [63:0] readData_M,   // 读得的数据

    input                mmu_wait,  // 是否需要地址转换
    input         [63:0] virAddr,   // 虚拟地址(Sv39)
    input         [ 1:0] mode,      // 当前特权级
    input  satp_t        satp,      // satp寄存器
    output reg           mmu_ok,    // 地址转换完成
    output reg    [63:0] phyAddr,   // 物理地址

    output dbus_req_t  dreq,  // dBUS请求
    input  dbus_resp_t dresp  // dBUS响应
);


    always @(*) begin
        case (funct3)  // 获取数据
            3'b000: begin  // lb
                case (memAddr[2:0])
                    3'b000:  readData_M = {{56{dresp.data[7]}}, dresp.data[7:0]};
                    3'b001:  readData_M = {{56{dresp.data[15]}}, dresp.data[15:8]};
                    3'b010:  readData_M = {{56{dresp.data[23]}}, dresp.data[23:16]};
                    3'b011:  readData_M = {{56{dresp.data[31]}}, dresp.data[31:24]};
                    3'b100:  readData_M = {{56{dresp.data[39]}}, dresp.data[39:32]};
                    3'b101:  readData_M = {{56{dresp.data[47]}}, dresp.data[47:40]};
                    3'b110:  readData_M = {{56{dresp.data[55]}}, dresp.data[55:48]};
                    3'b111:  readData_M = {{56{dresp.data[63]}}, dresp.data[63:56]};
                    default: ;
                endcase
            end
            3'b001: begin  // lh
                case (memAddr[2:1])
                    2'b00:   readData_M = {{48{dresp.data[15]}}, dresp.data[15:0]};
                    2'b01:   readData_M = {{48{dresp.data[31]}}, dresp.data[31:16]};
                    2'b10:   readData_M = {{48{dresp.data[47]}}, dresp.data[47:32]};
                    2'b11:   readData_M = {{48{dresp.data[63]}}, dresp.data[63:48]};
                    default: ;
                endcase
            end
            3'b010: begin  // lw
                case (memAddr[2])
                    1'b0: readData_M = {{32{dresp.data[31]}}, dresp.data[31:0]};
                    1'b1: readData_M = {{32{dresp.data[63]}}, dresp.data[63:32]};
                    default: ;
                endcase
            end
            3'b011:  readData_M = dresp.data;  // ld
            3'b100: begin  // lbu
                case (memAddr[2:0])
                    3'b000:  readData_M = {56'b0, dresp.data[7:0]};
                    3'b001:  readData_M = {56'b0, dresp.data[15:8]};
                    3'b010:  readData_M = {56'b0, dresp.data[23:16]};
                    3'b011:  readData_M = {56'b0, dresp.data[31:24]};
                    3'b100:  readData_M = {56'b0, dresp.data[39:32]};
                    3'b101:  readData_M = {56'b0, dresp.data[47:40]};
                    3'b110:  readData_M = {56'b0, dresp.data[55:48]};
                    3'b111:  readData_M = {56'b0, dresp.data[63:56]};
                    default: ;
                endcase
            end
            3'b101: begin  // lhu
                case (memAddr[2:1])
                    2'b00:   readData_M = {48'b0, dresp.data[15:0]};
                    2'b01:   readData_M = {48'b0, dresp.data[31:16]};
                    2'b10:   readData_M = {48'b0, dresp.data[47:32]};
                    2'b11:   readData_M = {48'b0, dresp.data[63:48]};
                    default: ;
                endcase
            end
            3'b110: begin  // lwu
                case (memAddr[2])
                    1'b0: readData_M = {32'b0, dresp.data[31:0]};
                    1'b1: readData_M = {32'b0, dresp.data[63:32]};
                    default: ;
                endcase
            end
            default: ;
        endcase
    end


    reg bubble;
    reg [1:0] state;  // 三级MMU状态机
    wire [11:0] Phy_offset;  // 物理地址偏移
    wire [8:0] L2_offset, L1_offset, L0_offset;
    Sv39_entry_t L2_entry, L1_entry, L0_entry;
    assign L2_offset  = virAddr[38:30];  // 9bit
    assign L1_offset  = virAddr[29:21];  // 9bit
    assign L0_offset  = virAddr[20:12];  // 9bit
    assign Phy_offset = virAddr[11:0];  // 12bit

    always @(posedge clk) begin
        if (reset) begin
            bubble = 1'b0;
            state = 2'b00;
            mmu_ok = 1'b0;
            phyAddr = 64'b0;
            L2_entry = 64'b0;
            L1_entry = 64'b0;
            L0_entry = 64'b0;
        end else if (mmu_ok) mmu_ok = 1'b0;
        else if (memRead) begin  // 如果需要读内存
            if (~dresp.data_ok) begin  // 发起读请求
                dreq.valid  = 1'b1;  // 读使能
                dreq.addr   = memAddr;  // 内存地址
                dreq.strobe = 8'b0;  // 无需有效位
                case (funct3)
                    3'b000:  dreq.size = MSIZE1;  // lb
                    3'b001:  dreq.size = MSIZE2;  // lh
                    3'b010:  dreq.size = MSIZE4;  // lw
                    3'b011:  dreq.size = MSIZE8;  // ld
                    3'b100:  dreq.size = MSIZE1;  // lbu
                    3'b101:  dreq.size = MSIZE2;  // lhu
                    3'b110:  dreq.size = MSIZE4;  // lwu
                    default: ;
                endcase
            end else dreq.valid = 1'b0;  // 读完成, 关闭读使能




        end else if (memWrite) begin  // 如果需要写内存
            if (~dresp.data_ok) begin  // 发起写请求
                dreq.valid = 1'b1;  // 写使能
                dreq.addr  = memAddr;  // 内存地址
                case (funct3)
                    3'b000: begin  // sb
                        dreq.size = MSIZE1;
                        case (memAddr[2:0])
                            3'b000:  dreq.strobe = 8'b0000_0001;
                            3'b001:  dreq.strobe = 8'b0000_0010;
                            3'b010:  dreq.strobe = 8'b0000_0100;
                            3'b011:  dreq.strobe = 8'b0000_1000;
                            3'b100:  dreq.strobe = 8'b0001_0000;
                            3'b101:  dreq.strobe = 8'b0010_0000;
                            3'b110:  dreq.strobe = 8'b0100_0000;
                            3'b111:  dreq.strobe = 8'b1000_0000;
                            default: ;
                        endcase
                        case (memAddr[2:0])
                            3'b000:  dreq.data[7:0] = writeData_M[7:0];
                            3'b001:  dreq.data[15:8] = writeData_M[7:0];
                            3'b010:  dreq.data[23:16] = writeData_M[7:0];
                            3'b011:  dreq.data[31:24] = writeData_M[7:0];
                            3'b100:  dreq.data[39:32] = writeData_M[7:0];
                            3'b101:  dreq.data[47:40] = writeData_M[7:0];
                            3'b110:  dreq.data[55:48] = writeData_M[7:0];
                            3'b111:  dreq.data[63:56] = writeData_M[7:0];
                            default: ;
                        endcase
                    end
                    3'b001: begin  // sh
                        dreq.size = MSIZE2;
                        case (memAddr[2:0])
                            3'b000:  dreq.strobe = 8'b0000_0011;
                            3'b010:  dreq.strobe = 8'b0000_1100;
                            3'b100:  dreq.strobe = 8'b0011_0000;
                            3'b110:  dreq.strobe = 8'b1100_0000;
                            default: ;
                        endcase
                        case (memAddr[2:0])
                            3'b000:  dreq.data[15:0] = writeData_M[15:0];
                            3'b010:  dreq.data[31:16] = writeData_M[15:0];
                            3'b100:  dreq.data[47:32] = writeData_M[15:0];
                            3'b110:  dreq.data[63:48] = writeData_M[15:0];
                            default: ;
                        endcase
                    end
                    3'b010: begin  // sw
                        dreq.size = MSIZE4;
                        case (memAddr[2:0])
                            3'b000:  dreq.strobe = 8'b0000_1111;
                            3'b100:  dreq.strobe = 8'b1111_0000;
                            default: ;
                        endcase
                        case (memAddr[2:0])
                            3'b000:  dreq.data[31:0] = writeData_M[31:0];
                            3'b100:  dreq.data[63:32] = writeData_M[31:0];
                            default: ;
                        endcase
                    end
                    3'b011: begin  // sd
                        dreq.size   = MSIZE8;
                        dreq.strobe = 8'b1111_1111;
                        dreq.data   = writeData_M;
                    end
                    default: ;
                endcase
            end else dreq.valid = 1'b0;  // 写完成, 关闭写使能




        end else if (mmu_wait) begin
            if (mode == M_Mode || satp.mode == SATP_bare) begin
                if (bubble == 1'b0) bubble = 1'b1;  // 延迟一个周期
                else begin
                    bubble  = 1'b0;
                    mmu_ok  = 1'b1;
                    phyAddr = virAddr;
                end
            end else
                case (state)
                    2'b00: begin  // L2_cache
                        if (~dresp.data_ok) begin
                            dreq.valid  = 1'b1;
                            dreq.strobe = 8'b0;
                            dreq.size   = MSIZE8;
                            dreq.addr   = {8'b0, satp.ppn, L2_offset, 3'b0};
                        end else begin
                            dreq.valid = 1'b0;
                            L2_entry   = dresp.data;
                            if (L2_entry[3:1] == 3'b000) state = 2'b01;
                            else begin  // 找到叶结点
                                mmu_ok  = 1'b1;
                                state   = 2'b00;  // 重置状态机
                                phyAddr = {8'b0, L2_entry.ppn, Phy_offset};
                            end
                        end
                    end

                    2'b01: begin  // L1_cache
                        if (~dresp.data_ok) begin
                            dreq.valid  = 1'b1;
                            dreq.strobe = 8'b0;
                            dreq.size   = MSIZE8;
                            dreq.addr   = {8'b0, L2_entry.ppn, L1_offset, 3'b0};
                        end else begin
                            dreq.valid = 1'b0;
                            L1_entry   = dresp.data;
                            if (L1_entry[3:1] == 3'b000) state = 2'b10;
                            else begin  // 找到叶结点
                                mmu_ok  = 1'b1;
                                state   = 2'b00;  // 重置状态机
                                phyAddr = {8'b0, L1_entry.ppn, Phy_offset};
                            end
                        end
                    end

                    2'b10: begin  // L0_cache
                        if (~dresp.data_ok) begin
                            dreq.valid  = 1'b1;
                            dreq.strobe = 8'b0;
                            dreq.size   = MSIZE8;
                            dreq.addr   = {8'b0, L1_entry.ppn, L0_offset, 3'b0};
                        end else begin
                            dreq.valid = 1'b0;
                            L0_entry = dresp.data;
                            mmu_ok = 1'b1;
                            state = 2'b00;
                            phyAddr = {8'b0, L0_entry.ppn, Phy_offset};
                        end
                    end
                    default: ;
                endcase
        end
    end
endmodule
