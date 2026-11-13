// -----------------------------------------------------------------------------
// hs_div_pipe.v – bộ chia không dấu pipeline (chia khôi phục, 1 bit thương / tầng)
//
//   quo = num / den  (lấy phần nguyên), với điều kiện thương < 2^QW
//
// Dùng cho a = Ix4/D16, b = Iy4/D16 (đặc tả 0.4 mục 4.3): num = |n|·2^18 + D16/2 (28 bit),
// den = D16 (21 bit). Vì D16 ≥ 64 + n² nên thương ≤ 16 384 < 2^15 → chỉ cần QW = 15 tầng
// (không phải 28): mỗi tầng một phép trừ (DW + k) bit. Độ trễ QW chu kỳ, 1 kết quả / chu kỳ.
// ce = 0 giữ nguyên toàn bộ pipeline (dừng theo áp lực ngược).
// -----------------------------------------------------------------------------
`timescale 1ns / 1ps
module hs_div_pipe #(
    parameter integer NW = 28,
    parameter integer DW = 21,
    parameter integer QW = 15
)(
    input  wire          clk,
    input  wire          ce,
    input  wire [NW-1:0] num,
    input  wire [DW-1:0] den,
    output wire [QW-1:0] quo
);
    reg [NW-1:0] rem_r [0:QW];
    reg [DW-1:0] den_r [0:QW] /* synthesis syn_srlstyle = "registers" */;   // chuỗi trễ thuần: giữ là flip-flop
    reg [QW-1:0] q_r   [0:QW];

    always @(*) begin
        rem_r[0] = num;
        den_r[0] = den;
        q_r[0]   = {QW{1'b0}};
    end

    genvar i;
    generate
        for (i = 0; i < QW; i = i + 1) begin : st
            localparam integer K = QW - 1 - i;                 // bit thương tầng này quyết định
            wire [NW+QW:0] sh  = {{(NW+QW+1-DW){1'b0}}, den_r[i]} << K;
            wire           ge  = {{(QW+1){1'b0}}, rem_r[i]} >= sh;
            wire [NW+QW:0] dif = {{(QW+1){1'b0}}, rem_r[i]} - sh;
            always @(posedge clk)
                if (ce) begin
                    rem_r[i+1] <= ge ? dif[NW-1:0] : rem_r[i];
                    den_r[i+1] <= den_r[i];
                    q_r[i+1]   <= q_r[i] | (ge ? ({{(QW-1){1'b0}}, 1'b1} << K) : {QW{1'b0}});
                end
        end
    endgenerate

    assign quo = q_r[QW];
endmodule
