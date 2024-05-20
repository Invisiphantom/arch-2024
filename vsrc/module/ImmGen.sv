module ImmGen (
    input      [31:0] instruction,  // 指令
    output reg [63:0] imm           // 立即数
);
    /*
          |31         25 |24   20|19  12|11          7|6  0|
 I-type   |        imm[11:0]     |      |             |    |
 S-type   |  imm[11:5]   |       |      |  imm[4:0]   |    |
 B-type   | imm[12|10:5] |       |      | imm[4:1|11] |    |
 J-type   |    imm[20|10:1|11|19:12]    |             |    |
 U-type   |         imm[31:12]          |             |    |
*/

    wire [6:0] opcode_7;
    assign opcode_7 = instruction[6:0];
    always @(*) begin // 纯组合逻辑
        case (opcode_7)
            7'b0010011: imm = {{(64 - 12) {instruction[31]}}, instruction[31:20]};  // I-type
            7'b0011011: imm = {{(64 - 12) {instruction[31]}}, instruction[31:20]};  // I-type (w)
            7'b0000011: imm = {{(64 - 12) {instruction[31]}}, instruction[31:20]};  // Load I-type
            7'b0100011: imm = {{(64 - 12) {instruction[31]}}, instruction[31:25], instruction[11:7]};  // Store S-type
            7'b1100011: imm = {{(64 - 13) {instruction[31]}}, instruction[31], instruction[7], instruction[30:25], instruction[11:8], 1'b0};  // Branch B-type
            7'b1101111: imm = {{(64 - 21) {instruction[31]}}, instruction[31], instruction[19:12], instruction[20], instruction[30:21], 1'b0}; // jal J-type
            7'b1100111: imm = {{(64 - 12) {instruction[31]}}, instruction[31:20]};  // jalr I-type
            7'b0110111: imm = {{(64 - 32) {instruction[31]}}, instruction[31:12], 12'b0};  // lui   U-type
            7'b0010111: imm = {{(64 - 32) {instruction[31]}}, instruction[31:12], 12'b0};  // auipc U-type
            default:    imm = 64'b0;
        endcase
    end
endmodule
