// -----------------------------------------------------------------------------
// hs_iter.v – khối HS_ITER: N_ITER vòng Jacobi Horn–Schunck, u, v Q6.6 (13 bit) – đặc tả 0.4 mục 5
//
//   ū = ((2·Σcạnh + Σgóc)·AVG_MUL + 2^16) >>> 17          (trung bình 3×3, 1/6 cạnh, 1/12 góc)
//   N = Ix4·ū + Iy4·v̄ + (It4 <<< 6)                        (25 bit)
//   u' = sat13(ū − ((a·N + 2^17) >>> 18)),  v' tương tự với b
//
// Vào : luồng gói DERIV 64 bit theo hàng, phát lại N_ITER lần (19 200 × N_ITER gói, từ SDRAM).
// Ra  : luồng (u', v') theo hàng của VÒNG CUỐI cho khối FEAT; done khi xong.
//
// Jacobi tại chỗ: u, v nằm trong MỘT bộ nhớ (hs_uv_ram, 30 BSRAM). Bốn bộ đệm hàng (SSRAM,
// 4 × 160 × 26 bit) giữ giá trị CŨ của hàng y−1, y, y+1; trong lúc quét hàng y, bộ nạp trước
// đọc hàng y+2 (cũ) từ uv_ram vào bộ đệm thứ tư. Hàng r luôn ở bộ đệm r mod 4 (120 chia hết
// cho 4 nên đúng cả khi qua vòng mới: hàng 118, 119 nạp sẵn hàng 0, 1 của vòng sau).
// Vòng đầu (first_iter): cửa sổ = 0, không cần xóa uv_ram.
//
// Pipeline sau khi nhận gói (P1…P8, không dừng): tổng cạnh/góc → ×AVG_MUL → ū → Ix·ū, Iy·v̄ → N
// → a·N, b·N → trừ, bão hòa → ghi uv_ram. Nhận tối đa 1 gói / chu kỳ; thực tế bị giới hạn bởi
// tốc độ SDRAM (2 từ / gói).
// -----------------------------------------------------------------------------
`timescale 1ns / 1ps
module hs_iter #(
    parameter integer NIT = 0                 // 0 = lấy N_ITER trong hs_params.vh
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    input  wire        in_valid,
    output wire        in_ready,
    input  wire [63:0] in_data,               // gói DERIV
    output reg         out_valid,              // (u', v') vòng cuối, theo hàng
    output reg  signed [12:0] out_u,
    output reg  signed [12:0] out_v,
    output reg         done,
    output reg         busy,
    output reg  [31:0] n_sat,                  // số lần bão hòa u/v (gỡ lỗi)
    output reg  [6:0]  iter,                   // vòng hiện tại
    output wire        dbg_we,                 // gỡ lỗi: lệnh ghi uv_ram (theo thứ tự quét)
    output wire [25:0] dbg_wd
);
`undef HS_PARAMS_VH
`include "hs_params.vh"

    localparam integer NI   = (NIT > 0) ? NIT : N_ITER;
    localparam integer W    = IMG_W, H = IMG_H;
    localparam integer UVB  = 2 * UV_W;                     // 26
    localparam signed [12:0] UMAX = 13'sd4095, UMIN = -13'sd4096;

    // ================================================================ uv_ram
    reg         ram_we;
    reg  [14:0] ram_wa;
    reg  [25:0] ram_wd;
    reg         ram_re;
    reg  [14:0] ram_ra;
    wire [25:0] ram_rd;
    assign dbg_we = ram_we;
    assign dbg_wd = ram_wd;
    hs_uv_ram u_ram (.clk(clk), .we(ram_we), .waddr(ram_wa), .wdata(ram_wd),
                     .re(ram_re), .raddr(ram_ra), .rdata(ram_rd));

    // ================================================================ 4 bộ đệm hàng
    reg [25:0] rb0 [0:W-1] /* synthesis syn_ramstyle = "distributed_ram" */;
    reg [25:0] rb1 [0:W-1] /* synthesis syn_ramstyle = "distributed_ram" */;
    reg [25:0] rb2 [0:W-1] /* synthesis syn_ramstyle = "distributed_ram" */;
    reg [25:0] rb3 [0:W-1] /* synthesis syn_ramstyle = "distributed_ram" */;

    // ---------------------------------------------------------------- bộ nạp trước
    reg        pf_busy, pf_wv;
    reg [7:0]  pf_c, pf_wc;
    reg [6:0]  pf_row;
    reg [1:0]  pf_buf, pf_wbuf;
    reg        pf_go;                 // xung yêu cầu
    reg [6:0]  pf_go_row;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pf_busy <= 1'b0; pf_wv <= 1'b0; pf_c <= 0; pf_wc <= 0; pf_row <= 0; pf_buf <= 0; pf_wbuf <= 0;
            ram_re <= 1'b0; ram_ra <= 0;
        end else begin
            ram_re <= 1'b0;
            pf_wv  <= 1'b0;
            if (pf_go) begin
                pf_busy <= 1'b1; pf_c <= 0; pf_row <= pf_go_row; pf_buf <= pf_go_row[1:0];
            end else if (pf_busy) begin
                ram_re  <= 1'b1;
                ram_ra  <= pf_row * W + pf_c;
                pf_wv   <= 1'b1; pf_wc <= pf_c; pf_wbuf <= pf_buf;
                if (pf_c == W - 1) pf_busy <= 1'b0;
                pf_c <= pf_c + 1'b1;
            end
        end
    end
    // dữ liệu đọc về sau 1 chu kỳ (ram_re ở chu kỳ trước) → ghi bộ đệm
    reg        pf_wv2;
    reg [7:0]  pf_wc2;
    reg [1:0]  pf_wbuf2;
    always @(posedge clk) begin
        pf_wv2 <= pf_wv; pf_wc2 <= pf_wc; pf_wbuf2 <= pf_wbuf;
    end
    wire pf_idle = !pf_go && !pf_busy && !pf_wv && !pf_wv2;
    always @(posedge clk) if (pf_wv2) begin
        case (pf_wbuf2)
        2'd0: rb0[pf_wc2] <= ram_rd;
        2'd1: rb1[pf_wc2] <= ram_rd;
        2'd2: rb2[pf_wc2] <= ram_rd;
        default: rb3[pf_wc2] <= ram_rd;
        endcase
    end

    // ================================================================ FSM quét
    localparam [2:0] S_IDLE = 3'd0, S_ROWWAIT = 3'd1, S_PRIME = 3'd2, S_ROW = 3'd3, S_DRAIN = 3'd4;
    reg [2:0]  st;
    reg [6:0]  y;
    reg [7:0]  x;
    reg        first_iter, last_iter;
    reg [3:0]  drain_cnt;

    // đọc không đồng bộ cả 4 bộ đệm ở cột rc
    wire [7:0]  rc  = (st == S_PRIME) ? 8'd0 : ((x >= W - 1) ? W - 1 : x + 1'b1);
    wire [25:0] q0 = rb0[rc], q1 = rb1[rc], q2 = rb2[rc], q3 = rb3[rc];
    function [25:0] pick; input [1:0] i; input [25:0] a0, a1, a2, a3;
        begin case (i) 2'd0: pick = a0; 2'd1: pick = a1; 2'd2: pick = a2; default: pick = a3; endcase end
    endfunction
    wire [1:0]  iu = (y == 0)     ? y[1:0] : y[1:0] - 2'd1;
    wire [1:0]  im = y[1:0];
    wire [1:0]  id = (y == H - 1) ? y[1:0] : y[1:0] + 2'd1;
    wire [25:0] cu_r = pick(iu, q0, q1, q2, q3);              // cột rc của hàng y−1, y, y+1
    wire [25:0] cm_r = pick(im, q0, q1, q2, q3);
    wire [25:0] cd_r = pick(id, q0, q1, q2, q3);
    // thanh ghi cửa sổ: cột x−1 (L) và x (C) của 3 hàng
    reg  [25:0] lu, lm, ld, cu, cm, cd;

    assign in_ready = (st == S_ROW);
    wire fire = in_valid && in_ready;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_IDLE; y <= 0; x <= 0; iter <= 0; first_iter <= 1'b0; last_iter <= 1'b0;
            busy <= 1'b0; pf_go <= 1'b0; pf_go_row <= 0; drain_cnt <= 0;
            lu <= 0; lm <= 0; ld <= 0; cu <= 0; cm <= 0; cd <= 0;
        end else begin
            pf_go <= 1'b0;
            case (st)
            S_IDLE: if (start) begin
                busy <= 1'b1; y <= 0; iter <= 0; first_iter <= 1'b1; last_iter <= (NI == 1);
                st <= S_ROWWAIT;
            end
            S_ROWWAIT: if (pf_idle && !pf_go) begin
                // nạp trước hàng y+2 (hoặc hàng 0, 1 của vòng sau); vòng cuối thì không cần
                if (y + 2 < H)        begin pf_go <= 1'b1; pf_go_row <= y + 2'd2; end
                else if (!last_iter)  begin pf_go <= 1'b1; pf_go_row <= y + 2'd2 - H; end
                st <= S_PRIME;
            end
            S_PRIME: begin                                     // nạp cột 0 vào C
                cu <= cu_r; cm <= cm_r; cd <= cd_r;
                lu <= cu_r; lm <= cm_r; ld <= cd_r;
                x  <= 0;
                st <= S_ROW;
            end
            S_ROW: if (fire) begin
                lu <= cu; lm <= cm; ld <= cd;
                cu <= cu_r; cm <= cm_r; cd <= cd_r;            // cột x+1 (replicate ở x = 159)
                if (x == W - 1) begin
                    if (y == H - 1) begin
                        y <= 0;
                        if (last_iter) begin st <= S_DRAIN; drain_cnt <= 0; end
                        else begin
                            iter <= iter + 1'b1; first_iter <= 1'b0; last_iter <= (iter + 2 == NI);
                            st <= S_ROWWAIT;
                        end
                    end else begin y <= y + 1'b1; st <= S_ROWWAIT; end
                end else x <= x + 1'b1;
            end
            S_DRAIN: begin                                     // chờ pipeline ghi xong
                drain_cnt <= drain_cnt + 1'b1;
                if (drain_cnt == 4'd12) begin st <= S_IDLE; busy <= 1'b0; end
            end
            default: st <= S_IDLE;
            endcase
        end
    end

    // ================================================================ pipeline tính
    // cửa sổ của điểm (y, x): trái = L (hoặc C ở x = 0), giữa = C, phải = cột vừa đọc (hoặc C ở x = 159)
    wire        xl0 = (x == 0), xr0 = (x == W - 1);
    wire [25:0] wl_u = xl0 ? cu : lu, wl_m = xl0 ? cm : lm, wl_d = xl0 ? cd : ld;
    wire [25:0] wr_u = xr0 ? cu : cu_r, wr_m = xr0 ? cm : cm_r, wr_d = xr0 ? cd : cd_r;
    // tách u (13 bit thấp) và v (13 bit cao)
    `define U(w) $signed(w[12:0])
    `define V(w) $signed(w[25:13])

    // P1: tổng cạnh E và góc C (16 bit có dấu)
    reg               v1;
    reg signed [15:0] eu1, cu1, ev1, cv1;
    reg        [63:0] pk1;
    reg        [14:0] ad1;
    reg               last1;
    always @(posedge clk) begin
        v1 <= fire;
        if (fire) begin
            if (first_iter) begin eu1 <= 0; cu1 <= 0; ev1 <= 0; cv1 <= 0; end
            else begin
                eu1 <= `U(cu) + `U(cd) + `U(wl_m) + `U(wr_m);
                cu1 <= `U(wl_u) + `U(wr_u) + `U(wl_d) + `U(wr_d);
                ev1 <= `V(cu) + `V(cd) + `V(wl_m) + `V(wr_m);
                cv1 <= `V(wl_u) + `V(wr_u) + `V(wl_d) + `V(wr_d);
            end
            pk1 <= in_data; ad1 <= y * W + x; last1 <= last_iter;
        end
    end
    // P2: (2E + C) · AVG_MUL  (17 × 15 → 32 bit)
    reg               v2, last2;
    reg signed [31:0] su2, sv2;
    reg        [63:0] pk2;
    reg        [14:0] ad2;
    always @(posedge clk) begin
        v2 <= v1; pk2 <= pk1; ad2 <= ad1; last2 <= last1;
        su2 <= ($signed({eu1, 1'b0}) + cu1) * $signed(32'sd10923);
        sv2 <= ($signed({ev1, 1'b0}) + cv1) * $signed(32'sd10923);
    end
    // P3: ū = (· + 2^16) >>> 17
    reg               v3, last3;
    reg signed [12:0] au3, av3;
    reg        [63:0] pk3;
    reg        [14:0] ad3;
    wire signed [31:0] ru = (su2 + 32'sd65536) >>> AVG_SHIFT;
    wire signed [31:0] rv = (sv2 + 32'sd65536) >>> AVG_SHIFT;
    always @(posedge clk) begin
        v3 <= v2; pk3 <= pk2; ad3 <= ad2; last3 <= last2;
        au3 <= ru[12:0]; av3 <= rv[12:0];
    end
    // P4: Ix4·ū, Iy4·v̄ (11 × 13 → 24 bit)
    reg               v4, last4;
    reg signed [23:0] mx4, my4;
    reg signed [12:0] au4, av4;
    reg signed [10:0] it4;
    reg signed [14:0] a4, b4;
    reg        [14:0] ad4;
    always @(posedge clk) begin
        v4 <= v3; ad4 <= ad3; last4 <= last3; au4 <= au3; av4 <= av3;
        mx4 <= $signed(pk3[10:0])  * au3;
        my4 <= $signed(pk3[21:11]) * av3;
        it4 <= $signed(pk3[32:22]);
        a4  <= $signed(pk3[47:33]); b4 <= $signed(pk3[62:48]);
    end
    // P5: N = Ix4·ū + Iy4·v̄ + (It4 <<< 6)   (25 bit)
    reg               v5, last5;
    reg signed [24:0] n5;
    reg signed [12:0] au5, av5;
    reg signed [14:0] a5, b5;
    reg        [14:0] ad5;
    always @(posedge clk) begin
        v5 <= v4; ad5 <= ad4; last5 <= last4; au5 <= au4; av5 <= av4; a5 <= a4; b5 <= b4;
        n5 <= mx4 + my4 + ($signed({{8{it4[10]}}, it4}) <<< UV_FRAC);
    end
    // P6: a·N, b·N (15 × 25 → 40 bit)
    reg               v6, last6;
    reg signed [39:0] pa6, pb6;
    reg signed [12:0] au6, av6;
    reg        [14:0] ad6;
    always @(posedge clk) begin
        v6 <= v5; ad6 <= ad5; last6 <= last5; au6 <= au5; av6 <= av5;
        pa6 <= a5 * n5;
        pb6 <= b5 * n5;
    end
    // P7: u' = sat13(ū − ((a·N + 2^17) >>> 18))
    reg               v7, last7;
    reg signed [12:0] u7, w7;
    reg        [14:0] ad7;
    reg        [1:0]  sat7;
    wire signed [39:0] du = (pa6 + 40'sd131072) >>> COEF_FRAC;
    wire signed [39:0] dv = (pb6 + 40'sd131072) >>> COEF_FRAC;
    wire signed [39:0] nu = {{27{au6[12]}}, au6} - du;
    wire signed [39:0] nv = {{27{av6[12]}}, av6} - dv;
    always @(posedge clk) begin
        v7 <= v6; ad7 <= ad6; last7 <= last6;
        u7 <= (nu > UMAX) ? UMAX : (nu < UMIN) ? UMIN : nu[12:0];
        w7 <= (nv > UMAX) ? UMAX : (nv < UMIN) ? UMIN : nv[12:0];
        sat7 <= {(nv > UMAX) || (nv < UMIN), (nu > UMAX) || (nu < UMIN)};
    end
    // P8: ghi uv_ram; vòng cuối → FEAT
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ram_we <= 1'b0; out_valid <= 1'b0; n_sat <= 0; out_u <= 0; out_v <= 0; ram_wa <= 0; ram_wd <= 0;
        end else begin
            ram_we    <= v7;
            ram_wa    <= ad7;
            ram_wd    <= {w7, u7};
            out_valid <= v7 && last7;
            out_u     <= u7;
            out_v     <= w7;
            if (start && !busy) n_sat <= 0;
            else if (v7) n_sat <= n_sat + sat7[0] + sat7[1];
        end
    end
    `undef U
    `undef V

    always @(posedge clk or negedge rst_n)
        if (!rst_n) done <= 1'b0;
        else done <= (st == S_DRAIN) && (drain_cnt == 4'd12);
endmodule
