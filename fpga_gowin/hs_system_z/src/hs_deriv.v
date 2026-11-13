// -----------------------------------------------------------------------------
// hs_deriv.v – khối DERIV: hai ảnh 8 bit (đã Gauss) → gói 64 bit / điểm ảnh
//
//   Ix4 = (a01−a00)+(a11−a10)+(b01−b00)+(b11−b10)    (11 bit có dấu, chính xác)
//   Iy4 = (a10−a00)+(a11−a01)+(b10−b00)+(b11−b01)
//   It4 = (b00−a00)+(b01−a01)+(b10−a10)+(b11−a11)
//   D16 = ALPHA2_16 + Ix4² + Iy4²                   (21 bit không dấu)
//   a   = sat15(sign(Ix4)·((|Ix4|·2^18 + D16/2) / D16)),  b tương tự với Iy4
//   gói = {1'b0, b[14:0], a[14:0], It4[10:0], Iy4[10:0], Ix4[10:0]}  = một dòng deriv.mem
//
// Biên phải / dưới: replicate (a01 = a00 ở x = 159, hàng y+1 = hàng y ở y = 119).
//
// Vào: luồng điểm ảnh theo hàng, in_data = {i2[7:0], i1[7:0]}, 19 200 nhịp sau mỗi start.
// Ra : luồng gói theo hàng (19 200 gói), có áp lực ngược out_ready.
// Cách làm: 2 bộ đệm hàng (SSRAM 2 × 160 × 16 bit). Nạp hàng y+1 rồi quét hàng y (161 nhịp);
// ~2 chu kỳ / điểm ⇒ ≈ 0,4 ms ở 100 MHz. Hai bộ chia chạy song song (a và b).
// -----------------------------------------------------------------------------
`timescale 1ns / 1ps
module hs_deriv (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,          // xung: bắt đầu một cặp khung
    // luồng ảnh vào
    input  wire        in_valid,
    output wire        in_ready,
    input  wire [15:0] in_data,        // {i2, i1}
    // luồng gói ra
    output wire        out_valid,
    input  wire        out_ready,
    output wire [63:0] out_data,
    output reg         done,           // xung sau gói cuối
    output reg         busy
);
`undef HS_PARAMS_VH
`include "hs_params.vh"

    localparam integer DIVQ = 15;                         // số tầng bộ chia
    localparam integer LAT  = 2 + DIVQ + 1;               // tầng A, B, chia, đóng gói

    // ---------------------------------------------------------------- bộ đệm hàng
    reg [15:0] rb0 [0:IMG_W-1] /* synthesis syn_ramstyle = "distributed_ram" */;
    reg [15:0] rb1 [0:IMG_W-1] /* synthesis syn_ramstyle = "distributed_ram" */;

    localparam [1:0] S_IDLE = 2'd0, S_LOAD = 2'd1, S_SCAN = 2'd2, S_DRAIN = 2'd3;
    reg [1:0]  st;
    reg [7:0]  ld_c;            // cột đang nạp
    reg [6:0]  ld_row;          // hàng đang nạp
    reg [6:0]  y;               // hàng đang quét
    reg [7:0]  sc;              // bước quét 0 … 160
    reg [14:0] n_out;           // số gói đã ra

    // ---------------------------------------------------------------- dừng pipeline
    reg  [LAT-1:0] vld;                                   // valid từng tầng
    wire ce = !(vld[LAT-1] && !out_ready);

    // ---------------------------------------------------------------- nạp hàng
    assign in_ready = (st == S_LOAD);
    wire ld_fire = in_valid && in_ready;
    always @(posedge clk) if (ld_fire) begin
        if (ld_row[0]) rb1[ld_c] <= in_data; else rb0[ld_c] <= in_data;
    end

    // ---------------------------------------------------------------- đọc cột (đọc không đồng bộ)
    wire [7:0]  rc    = (sc > IMG_W - 1) ? IMG_W - 1 : sc;
    wire [15:0] r0    = rb0[rc];
    wire [15:0] r1    = rb1[rc];
    wire        last_row = (y == IMG_H - 1);
    wire [15:0] cur   = y[0] ? r1 : r0;                    // hàng y
    wire [15:0] nxt   = last_row ? cur : (y[0] ? r0 : r1); // hàng y+1 (replicate ở hàng cuối)
    reg  [15:0] pcur, pnxt;                                // cột trước
    wire        scan_fire = (st == S_SCAN) && ce;
    wire        emit      = scan_fire && (sc != 0);        // ra điểm (y, sc−1)

    // ---------------------------------------------------------------- FSM
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_IDLE; ld_c <= 0; ld_row <= 0; y <= 0; sc <= 0; busy <= 1'b0;
            pcur <= 0; pnxt <= 0;
        end else begin
            case (st)
            S_IDLE: if (start) begin
                st <= S_LOAD; ld_c <= 0; ld_row <= 0; y <= 0; busy <= 1'b1;
            end
            S_LOAD: if (ld_fire) begin
                if (ld_c == IMG_W - 1) begin
                    ld_c <= 0;
                    if (ld_row == 0) ld_row <= 1;            // cần cả hàng 0 và 1 trước khi quét
                    else begin st <= S_SCAN; sc <= 0; end
                end else ld_c <= ld_c + 1'b1;
            end
            S_SCAN: if (ce) begin
                pcur <= cur; pnxt <= nxt;
                if (sc == IMG_W) begin                       // xong hàng y
                    if (last_row) st <= S_DRAIN;
                    else begin
                        y <= y + 1'b1;
                        if (y + 1 == IMG_H - 1) begin st <= S_SCAN; sc <= 0; end   // hàng cuối: không nạp thêm
                        else begin st <= S_LOAD; ld_row <= y + 2'd2; end
                    end
                    sc <= 0;
                end else sc <= sc + 1'b1;
            end
            S_DRAIN: if (done) begin st <= S_IDLE; busy <= 1'b0; end
            endcase
        end
    end

    // ---------------------------------------------------------------- tầng A: đạo hàm
    wire signed [10:0] a00 = {3'b0, pcur[7:0]},  a01 = {3'b0, cur[7:0]};
    wire signed [10:0] a10 = {3'b0, pnxt[7:0]},  a11 = {3'b0, nxt[7:0]};
    wire signed [10:0] b00 = {3'b0, pcur[15:8]}, b01 = {3'b0, cur[15:8]};
    wire signed [10:0] b10 = {3'b0, pnxt[15:8]}, b11 = {3'b0, nxt[15:8]};
    reg  signed [10:0] gx_a, gy_a, gt_a;
    always @(posedge clk) if (ce) begin
        gx_a <= (a01 - a00) + (a11 - a10) + (b01 - b00) + (b11 - b10);
        gy_a <= (a10 - a00) + (a11 - a01) + (b10 - b00) + (b11 - b01);
        gt_a <= (b00 - a00) + (b01 - a01) + (b10 - a10) + (b11 - a11);
    end

    // ---------------------------------------------------------------- tầng B: D16 và tử số
    reg  [20:0] d16_b;
    reg  [27:0] nx_b, ny_b;
    reg  signed [10:0] gx_b, gy_b, gt_b;
    wire [9:0] agx = gx_a[10] ? -gx_a : gx_a;               // |Ix4| ≤ 1020
    wire [9:0] agy = gy_a[10] ? -gy_a : gy_a;
    wire [20:0] d16_w = ALPHA2_16 + agx * agx + agy * agy;
    always @(posedge clk) if (ce) begin
        d16_b <= d16_w;
        nx_b  <= ({18'd0, agx} << COEF_FRAC) + (d16_w >> 1);
        ny_b  <= ({18'd0, agy} << COEF_FRAC) + (d16_w >> 1);
        gx_b <= gx_a; gy_b <= gy_a; gt_b <= gt_a;
    end

    // ---------------------------------------------------------------- chia (DIVQ tầng)
    wire [DIVQ-1:0] qa, qb;
    hs_div_pipe #(.NW(28), .DW(21), .QW(DIVQ)) u_diva (.clk(clk), .ce(ce), .num(nx_b), .den(d16_b), .quo(qa));
    hs_div_pipe #(.NW(28), .DW(21), .QW(DIVQ)) u_divb (.clk(clk), .ce(ce), .num(ny_b), .den(d16_b), .quo(qb));
    // thanh ghi trễ: giữ là flip-flop, không để Gowin đổi thành BSRAM (mỗi chuỗi trễ sẽ tốn 1 khối)
    reg signed [10:0] gx_d [0:DIVQ-1] /* synthesis syn_srlstyle = "registers" */;
    reg signed [10:0] gy_d [0:DIVQ-1] /* synthesis syn_srlstyle = "registers" */;
    reg signed [10:0] gt_d [0:DIVQ-1] /* synthesis syn_srlstyle = "registers" */;
    integer k;
    always @(posedge clk) if (ce) begin
        gx_d[0] <= gx_b; gy_d[0] <= gy_b; gt_d[0] <= gt_b;
        for (k = 1; k < DIVQ; k = k + 1) begin
            gx_d[k] <= gx_d[k-1]; gy_d[k] <= gy_d[k-1]; gt_d[k] <= gt_d[k-1];
        end
    end

    // ---------------------------------------------------------------- đóng gói: dấu + bão hòa 15 bit
    function [14:0] coef;                                  // sat15: [−16 384, 16 383], q ≤ 16 384
        input [DIVQ-1:0] q;
        input            neg;
        begin
            if (neg) coef = (q > 16384) ? 15'h4000 : (~q + 1'b1);   // −q (−16 384 biểu diễn được)
            else     coef = (q > 16383) ? 15'd16383 : q;
        end
    endfunction
    reg [63:0] pkt;
    always @(posedge clk) if (ce)
        pkt <= {1'b0, coef(qb, gy_d[DIVQ-1][10]), coef(qa, gx_d[DIVQ-1][10]),
                gt_d[DIVQ-1], gy_d[DIVQ-1], gx_d[DIVQ-1]};

    // ---------------------------------------------------------------- valid
    always @(posedge clk or negedge rst_n)
        if (!rst_n) vld <= 0;
        else if (ce) vld <= {vld[LAT-2:0], emit};

    assign out_valid = vld[LAT-1];
    assign out_data  = pkt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin n_out <= 0; done <= 1'b0; end
        else begin
            done <= 1'b0;
            if (start) n_out <= 0;
            else if (out_valid && out_ready) begin
                if (n_out == IMG_W * IMG_H - 1) begin n_out <= 0; done <= 1'b1; end
                else n_out <= n_out + 1'b1;
            end
        end
    end
endmodule
