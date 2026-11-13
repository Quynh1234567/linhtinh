// -----------------------------------------------------------------------------
// hs_feat.v – khối FEAT: luồng (u, v) Q6.6 của vòng cuối → 48 giá trị z Q5.10 cho OC-SVM
//
//   |v| = (isqrt(4·(u² + v²)) + 1) >> 1             (MAG_ROUND = 1; 0: isqrt(u² + v²))
//   S[3c+0..2] = Σu, Σv, Σ|v| trong ô c = (y/30)·4 + x/40   (24 bit có dấu)
//   mean = sat16((S·CELL_MUL + 2^19) >>> 20)                 → Q5.10
//   g    = sign(mean) · LUT[ |m| < 2048 ? |m|>>1 : 1024 + ((|m|−2048)>>6) ]
//   z    = sat16(((g − MEAN[d])·INV[d] + 2^(INV_FRAC−1)) >>> INV_FRAC)
//
// Thứ tự 48 giá trị: ô theo hàng, mỗi ô [u, v, |v|] – đúng thứ tự mô hình đã train.
// Pha 1 chạy cùng luồng điểm ảnh (1 điểm / chu kỳ); pha 2 tuần tự 48 giá trị (~7 chu kỳ mỗi giá trị).
// Ghi z trực tiếp vào ocsvm_core qua x_we / x_addr / x_data; res_* để testbench so từng tầng.
// -----------------------------------------------------------------------------
`timescale 1ns / 1ps
module hs_feat #(
    parameter LUT_FILE  = "log_lut.mem",
    parameter MEAN_FILE = "scaler_mean_q.mem",
    parameter INV_FILE  = "scaler_inv_q.mem"
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               start,          // xóa bộ cộng dồn (trước vòng cuối)
    input  wire               in_valid,
    input  wire signed [12:0] in_u,
    input  wire signed [12:0] in_v,
    // gỡ lỗi / kiểm thử
    output wire               mag_valid,
    output wire [12:0]        mag,
    output reg                res_valid,      // một nhịp cho mỗi d = 0 … 47
    output reg  [5:0]         res_d,
    output reg  signed [23:0] res_sum,
    output reg  signed [15:0] res_mean,
    output reg  signed [15:0] res_log,
    output reg  signed [15:0] res_z,
    // tới ocsvm_core
    output reg                x_we,
    output reg  [7:0]         x_addr,
    output reg  [15:0]        x_data,
    output reg                done
);
`undef HS_PARAMS_VH
`include "hs_params.vh"

    // ---------------------------------------------------------------- bảng
    reg [LOG_OUT_W-1:0] lut  [0:LOG_LUT_SIZE-1];
    reg [15:0]          smean[0:DIM-1] /* synthesis syn_romstyle = "distributed_rom" */;   // 48 × 16: không tốn BSRAM
    reg [15:0]          sinv [0:DIM-1] /* synthesis syn_romstyle = "distributed_rom" */;
    initial begin
        $readmemh(LUT_FILE, lut);
        $readmemh(MEAN_FILE, smean);
        $readmemh(INV_FILE, sinv);
    end

    // ---------------------------------------------------------------- vị trí điểm ảnh, ô
    reg [7:0]  px;
    reg [6:0]  py;
    reg [1:0]  cx, cy;
    reg [5:0]  cxc;                    // đếm trong ô theo x (0…39)
    reg [4:0]  cyc_;                   // đếm trong ô theo y (0…29)
    reg [14:0] n_in;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin px <= 0; py <= 0; cx <= 0; cy <= 0; cxc <= 0; cyc_ <= 0; n_in <= 0; end
        else if (start) begin px <= 0; py <= 0; cx <= 0; cy <= 0; cxc <= 0; cyc_ <= 0; n_in <= 0; end
        else if (in_valid) begin
            n_in <= n_in + 1'b1;
            if (px == IMG_W - 1) begin
                px <= 0; cx <= 0; cxc <= 0;
                py <= py + 1'b1;
                if (cyc_ == CELL_H - 1) begin cyc_ <= 0; cy <= cy + 1'b1; end
                else cyc_ <= cyc_ + 1'b1;
            end else begin
                px <= px + 1'b1;
                if (cxc == CELL_W - 1) begin cxc <= 0; cx <= cx + 1'b1; end
                else cxc <= cxc + 1'b1;
            end
        end
    end
    wire [3:0] cell_id = {cy, cx};      // ('cell' là từ khóa Verilog)

    // ---------------------------------------------------------------- |v|
    reg               q_v;
    reg        [27:0] q_x;
    reg        [3:0]  q_sb;            // chỉ còn số ô cho nhánh |v|
    wire signed [25:0] uu = in_u * in_u, vv = in_v * in_v;
    always @(posedge clk) begin
        q_v  <= in_valid;
        q_x  <= (MAG_ROUND != 0) ? ({2'b0, uu} + {2'b0, vv}) << 2 : ({2'b0, uu} + {2'b0, vv});
        q_sb <= cell_id;
    end
    wire        s_v;
    wire [13:0] s_root;
    wire [3:0]  s_sb;
    hs_isqrt_pipe #(.SBW(4)) u_sqrt (.clk(clk), .in_valid(q_v), .x(q_x), .sb_in(q_sb),
                                      .out_valid(s_v), .root(s_root), .sb_out(s_sb));
    wire [13:0] mag_w = (MAG_ROUND != 0) ? ((s_root + 1'b1) >> 1) : s_root;
    assign mag_valid = s_v;
    assign mag       = mag_w[12:0];

    // ---------------------------------------------------------------- cộng dồn 48 tổng
    // 48 bộ cộng dồn viết thành 48 thanh ghi RIÊNG (generate), mỗi thanh ghi có điều kiện bật theo ô cố định.
    // Bản cũ ghi 3 phần tử của một mảng ở 3 địa chỉ biến đổi trong cùng chu kỳ (acc[base], acc[base+1],
    // acc[base+2]); mô phỏng đúng nhưng trên board, thống kê v (địa chỉ base+1) sai ở vài ô → tránh kiểu
    // "bộ nhớ nhiều cổng ghi" để tổng hợp không thể hiểu khác.
    reg [14:0] n_acc;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) n_acc <= 0;
        else if (start) begin
            n_acc <= 0;
        end else if (s_v) n_acc <= n_acc + 1'b1;
    end
    // Σu, Σv cộng NGAY tại đầu vào (không cần đi qua 14 tầng căn bậc hai); chỉ Σ|v| cộng ở đầu ra isqrt.
    // Thứ tự cộng không ảnh hưởng tổng nên kết quả giữ nguyên từng bit.
    wire [3:0]             s_cell = s_sb[3:0];
    wire signed [SUM_W-1:0] inc_u = {{(SUM_W-13){in_u[12]}}, in_u};
    wire signed [SUM_W-1:0] inc_v = {{(SUM_W-13){in_v[12]}}, in_v};
    wire signed [SUM_W-1:0] inc_m = {{(SUM_W-13){1'b0}}, mag_w[12:0]};
    wire [SUM_W*DIM-1:0]   acc_flat;
    genvar gk;
    generate
        for (gk = 0; gk < DIM; gk = gk + 1) begin : accg
            localparam integer GC = gk / 3;        // ô
            localparam integer GS = gk % 3;        // 0: Σu · 1: Σv · 2: Σ|v|
            reg signed [SUM_W-1:0] a;
            wire signed [SUM_W-1:0] inc = (GS == 0) ? inc_u : (GS == 1) ? inc_v : inc_m;
            wire                    en  = (GS == 2) ? (s_v && (s_cell == GC))          // |v|: sau isqrt
                                                    : (in_valid && (cell_id == GC));    // u, v: đầu vào
            always @(posedge clk)
                if (start) a <= {SUM_W{1'b0}};
                else if (en) a <= a + inc;
            assign acc_flat[gk*SUM_W +: SUM_W] = a;
        end
    endgenerate

    // ---------------------------------------------------------------- pha 2: 48 giá trị tuần tự
    localparam [2:0] F_IDLE = 3'd0, F_MUL = 3'd1, F_MEAN = 3'd2, F_ADDR = 3'd3, F_LUT = 3'd4,
                     F_ZMUL = 3'd5, F_OUT = 3'd6, F_DONE = 3'd7;
    reg [2:0]  fs;
    reg        armed;                   // start → chờ đủ 19 200 điểm → tính 48 giá trị một lần
    always @(posedge clk or negedge rst_n)
        if (!rst_n) armed <= 1'b0;
        else if (start) armed <= 1'b1;
        else if (fs == F_IDLE && n_acc == IMG_W * IMG_H) armed <= 1'b0;
    reg [5:0]  d;
    wire signed [SUM_W-1:0] acc_rd = acc_flat[d*SUM_W +: SUM_W];
    reg signed [SUM_W-1:0] s_d;
    reg signed [39:0] prod;
    reg signed [15:0] mean_d, g_d;
    reg        [10:0] laddr;
    reg               lneg, lzero;
    reg        [11:0] lq;
    reg signed [33:0] zp;
    wire signed [39:0] mr  = (prod + (40'sd1 <<< (CELL_SHIFT - 1))) >>> CELL_SHIFT;
    wire        [16:0] amx = mean_d[15] ? -{mean_d[15], mean_d} : {1'b0, mean_d};
    wire signed [33:0] zr  = (zp + (34'sd1 <<< (INV_FRAC - 1))) >>> INV_FRAC;
    // μ và 1/σ đọc ROM MỘT lần vào thanh ghi ở F_MUL, rồi mới mở rộng dấu từ thanh ghi.
    // Bản cũ viết $signed({smean[d][15], smean[d]}) (chọn bit trên phần tử mảng chỉ số biến):
    // mô phỏng đúng nhưng Gowin tổng hợp mất bit dấu → μ âm thành 65536-|μ| → z bão hòa ±32768
    // (đúng 6 giá trị d = 4,13,16,25,28,37 có μ < 0 sai trên board).
    reg        [15:0] mu_r, inv_r;
    wire signed [16:0] mu17 = {mu_r[15], mu_r};
    wire signed [16:0] g17  = {g_d[15], g_d};
    wire signed [16:0] gm   = g17 - mu17;
    wire signed [17:0] inv18 = {2'b00, inv_r};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fs <= F_IDLE; d <= 0; done <= 1'b0; res_valid <= 1'b0; x_we <= 1'b0; x_addr <= 0; x_data <= 0;
            res_d <= 0; res_sum <= 0; res_mean <= 0; res_log <= 0; res_z <= 0;
        end else begin
            done <= 1'b0; res_valid <= 1'b0; x_we <= 1'b0;
            case (fs)
            F_IDLE: if (armed && !start && n_acc == IMG_W * IMG_H) begin fs <= F_MUL; d <= 0; end
            F_MUL: begin
                s_d  <= acc_rd;
                prod <= acc_rd * CELL_MUL;
                mu_r  <= smean[d];
                inv_r <= sinv[d];
                fs   <= F_MEAN;
            end
            F_MEAN: begin                                         // sat16
                mean_d <= (mr > 40'sd32767) ? 16'sd32767 : (mr < -40'sd32768) ? -16'sd32768 : mr[15:0];
                fs <= F_ADDR;
            end
            F_ADDR: begin
                lneg  <= mean_d[15];
                lzero <= (mean_d == 0);
                if (amx < LOG_A_LIMIT) laddr <= amx >> LOG_A_STEP_SH;
                else laddr <= ((1 << LOG_A_BITS) + ((amx - LOG_A_LIMIT) >> LOG_B_STEP_SH) > LOG_LUT_SIZE - 1)
                              ? LOG_LUT_SIZE - 1 : (1 << LOG_A_BITS) + ((amx - LOG_A_LIMIT) >> LOG_B_STEP_SH);
                fs <= F_LUT;
            end
            F_LUT: begin
                lq <= lut[laddr];
                fs <= F_ZMUL;
            end
            F_ZMUL: begin
                g_d <= lzero ? 16'sd0 : (lneg ? -$signed({4'b0, lq}) : $signed({4'b0, lq}));
                fs  <= F_OUT;
            end
            F_OUT: begin
                zp <= gm * inv18;
                fs <= F_DONE;
            end
            F_DONE: begin
                res_valid <= 1'b1; res_d <= d; res_sum <= s_d; res_mean <= mean_d; res_log <= g_d;
                res_z  <= (zr > 34'sd32767) ? 16'sd32767 : (zr < -34'sd32768) ? -16'sd32768 : zr[15:0];
                x_we   <= 1'b1; x_addr <= {2'b0, d};
                x_data <= (zr > 34'sd32767) ? 16'h7FFF : (zr < -34'sd32768) ? 16'h8000 : zr[15:0];
                if (d == DIM - 1) begin fs <= F_IDLE; done <= 1'b1; end
                else begin d <= d + 1'b1; fs <= F_MUL; end
            end
            endcase
        end
    end
endmodule