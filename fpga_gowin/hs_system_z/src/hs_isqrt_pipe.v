// -----------------------------------------------------------------------------
// hs_isqrt_pipe.v – căn bậc hai nguyên pipeline: root = floor(√x), x 28 bit → root 14 bit
// Thuật toán từng chữ số (2 bit vào / 1 bit ra mỗi tầng), 14 tầng, 1 kết quả / chu kỳ.
// Kèm đường trễ cho dữ liệu đi kèm (SBW bit) để khớp đúng điểm ảnh.
// -----------------------------------------------------------------------------
`timescale 1ns / 1ps
module hs_isqrt_pipe #(
    parameter integer SBW = 32
)(
    input  wire           clk,
    input  wire           in_valid,
    input  wire [27:0]    x,
    input  wire [SBW-1:0] sb_in,
    output wire           out_valid,
    output wire [13:0]    root,
    output wire [SBW-1:0] sb_out
);
    localparam integer N = 14;
    reg [27:0]    xr  [0:N];
    reg [15:0]    rem [0:N];
    reg [13:0]    rt  [0:N];
    reg           vl  [0:N] /* synthesis syn_srlstyle = "registers" */;
    reg [SBW-1:0] sb  [0:N] /* synthesis syn_srlstyle = "registers" */;
    always @(*) begin
        xr[0] = x; rem[0] = 16'd0; rt[0] = 14'd0; vl[0] = in_valid; sb[0] = sb_in;
    end
    genvar i;
    generate
        for (i = 0; i < N; i = i + 1) begin : st
            wire [17:0] r2  = {rem[i], xr[i][27:26]};              // đưa 2 bit tiếp theo vào
            wire [17:0] tr  = {2'b0, rt[i], 2'b01};                 // (root << 2) | 1
            wire        ok  = r2 >= tr;
            always @(posedge clk) begin
                xr[i+1]  <= xr[i] << 2;
                rem[i+1] <= ok ? (r2 - tr) : r2;
                rt[i+1]  <= {rt[i][12:0], ok};
                vl[i+1]  <= vl[i];
                sb[i+1]  <= sb[i];
            end
        end
    endgenerate
    assign out_valid = vl[N];
    assign root      = rt[N];
    assign sb_out    = sb[N];
endmodule
