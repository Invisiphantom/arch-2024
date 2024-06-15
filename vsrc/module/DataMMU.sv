`ifdef VERILATOR
`include "include/common.sv"
`endif

module DataMMU
    import common::*;
(
    input       clk,    // 时钟信号
    input       reset,  // 复位信号
    input [1:0] mode,   // 当前特权级

    input        [63:0] virAddr,  // 虚拟地址(Sv39)
    input satp_t        satp,     // satp寄存器
    input               mmu_wait, // 是否需要地址转换

    output reg                mmu_ok,
    output reg         [63:0] phyAddr,
    output dbus_req_t         dreq,     // dBUS请求
    input  dbus_resp_t        dresp     // dBUS响应
);

    wire [11:0] Phy_offset;
    wire [8:0] L2_offset, L1_offset, L0_offset;
    assign L2_offset  = virAddr[38:30];  // 9bit
    assign L1_offset  = virAddr[29:21];  // 9bit
    assign L0_offset  = virAddr[20:12];  // 9bit
    assign Phy_offset = virAddr[11:0];  // 12bit

    reg bubble;
    reg [1:0] state;
    Sv39_entry_t L2_entry, L1_entry, L0_entry;

    always @(posedge clk) begin
        if (reset) begin
            bubble  = 1'b0;
            state   = 2'b00;
            mmu_ok  = 1'b0;
            phyAddr = 64'b0;
            L2_entry = 64'b0;
            L1_entry = 64'b0;
            L0_entry = 64'b0;
        end else if (mmu_ok) mmu_ok = 1'b0;
        else if (mmu_wait) begin

            if (mode == M_Mode || satp.mode == SATP_bare) begin
                if (bubble == 1'b0) bubble = 1'b1;
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
