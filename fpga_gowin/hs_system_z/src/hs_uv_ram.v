// -----------------------------------------------------------------------------
// hs_uv_ram.v – bộ nhớ u, v: 19 200 × 26 bit ({v, u}, Q6.6), 1 cổng ghi + 1 cổng đọc đồng bộ
//
// CHIA TAY THEO ĐÚNG KHỐI BSRAM để Gowin không thể suy ra cấu hình tốn khối:
//   3 lát theo chiều rộng (9 + 9 + 8 bit) × 10 dãy theo chiều sâu (2 048 ô mỗi dãy)
//   → 30 mảng 2 048 × ≤ 9 bit, mỗi mảng vừa ĐÚNG 1 khối BSRAM 18 Kbit (cấu hình 2K × 9) = 30 khối.
// Địa chỉ: dãy = addr[14:11], ô trong dãy = addr[10:0]. Đọc: mọi dãy đọc cùng ô, chọn dãy bằng
// số dãy đã thanh ghi → dữ liệu ra sau 1 chu kỳ, giống hệt bản cũ.
// -----------------------------------------------------------------------------
`timescale 1ns / 1ps
module hs_uv_ram #(
    parameter integer DEPTH = 19200,
    parameter integer W     = 26,
    parameter integer AW    = 15
)(
    input  wire          clk,
    input  wire          we,
    input  wire [AW-1:0] waddr,
    input  wire [W-1:0]  wdata,
    input  wire          re,
    input  wire [AW-1:0] raddr,
    output wire [W-1:0]  rdata
);
    localparam integer BD = 2048;                         // độ sâu một khối 2K × 9
    localparam integer NB = (DEPTH + BD - 1) / BD;        // 10 dãy
    localparam integer OW = 11;                           // bit địa chỉ trong dãy

    wire [AW-OW-1:0] wb = waddr[AW-1:OW];
    wire [AW-OW-1:0] rb = raddr[AW-1:OW];
    wire [OW-1:0]    wo = waddr[OW-1:0];
    wire [OW-1:0]    ro = raddr[OW-1:0];
    reg  [AW-OW-1:0] rb_q;
    always @(posedge clk) if (re) rb_q <= rb;

    wire [W-1:0] bank_q [0:NB-1];
    genvar b;
    generate
        for (b = 0; b < NB; b = b + 1) begin : bank
            reg [8:0] m0 [0:BD-1] /* synthesis syn_ramstyle = "block_ram" */;
            reg [8:0] m1 [0:BD-1] /* synthesis syn_ramstyle = "block_ram" */;
            reg [7:0] m2 [0:BD-1] /* synthesis syn_ramstyle = "block_ram" */;
            reg [8:0] q0, q1;
            reg [7:0] q2;
            always @(posedge clk) begin
                if (we && wb == b) begin
                    m0[wo] <= wdata[8:0];
                    m1[wo] <= wdata[17:9];
                    m2[wo] <= wdata[25:18];
                end
                if (re) begin
                    q0 <= m0[ro];
                    q1 <= m1[ro];
                    q2 <= m2[ro];
                end
            end
            assign bank_q[b] = {q2, q1, q0};
        end
    endgenerate
    assign rdata = bank_q[rb_q];
endmodule
