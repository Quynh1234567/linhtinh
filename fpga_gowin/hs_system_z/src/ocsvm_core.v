// -----------------------------------------------------------------------------
// ocsvm_core.v – lõi tính hàm quyết định One-Class SVM, khớp từng bit với Golden Model
//
//   f_q = Σ_i α_q[i] · LUT[(Σ_j (x_j − sv_ij)² · GAMMA_Q) >> LUT_SHIFT] − RHO_Q
//   anomaly = (f_q < THR_Q)            (DECISION_MODE = 0)
//
// Kiến trúc: P làn PE song song; mỗi SV chia thành G = DIM/P nhóm, mỗi chu kỳ một nhóm.
//   S0 địa chỉ ROM SV → S1 dữ liệu SV → S2 hiệu (x − sv) → S3 bình phương → S4 cây cộng
//   → S5 cộng dồn D_i (ra 1 giá trị mỗi G chu kỳ)
//   → R1 t = D_i·γ → R2 địa chỉ LUT + kiểm tra tràn, địa chỉ α → R3 đọc LUT, α
//   → R4 α·k → R5 cộng dồn f (khởi tạo −RHO_Q)
// Độ trễ ≈ N_SV·G + 11 chu kỳ. Hằng số lấy từ ocsvm_params.vh (sinh bởi 04_export_fpga.py).
// ROM SV dùng file sv_lanes.mem (sinh bởi scripts/15_export_rtl_mem.py --lanes P):
//   từ thứ i·G + g chứa phần tử g·P … g·P+P−1 của SV i, phần tử nhỏ nhất ở bit thấp.
// -----------------------------------------------------------------------------
module ocsvm_core #(
    parameter integer P          = 8,
    parameter         SV_FILE    = "sv_lanes.mem",
    parameter         ALPHA_FILE = "alpha.mem",
    parameter         LUT_FILE   = "exp_lut.mem"
)(
    input  wire                     clk,
    input  wire                     rst_n,
    // ghi véc-tơ đầu vào x (đã chuẩn hóa, Q5.10) – chỉ ghi khi không busy
    input  wire                     x_we,
    input  wire [7:0]               x_addr,
    input  wire [15:0]              x_data,
    // điều khiển
    input  wire                     start,
    output reg                      busy,
    output reg                      done,           // xung 1 chu kỳ
    output reg  signed [63:0]       f_out,          // dùng ACC_W bit thấp
    output reg                      anomaly
);
`undef OCSVM_PARAMS_VH   // file .vh co include guard: bo de MOI module deu nhan duoc localparam
`include "ocsvm_params.vh"

    // ---------------------------------------------------------------- tham số dẫn xuất
    localparam integer G      = DIM / P;                       // số nhóm mỗi SV
    localparam integer NWORD  = N_SV * G;                      // số từ ROM SV
    localparam integer AW     = (NWORD > 1) ? $clog2(NWORD) : 1;
    localparam integer GW     = (G > 1) ? $clog2(G) : 1;
    localparam integer SW     = (N_SV > 1) ? $clog2(N_SV) : 1;
    localparam integer SQ_W   = 2 * DIFF_W;                    // bình phương (không dấu)
    localparam integer SUM_W  = SQ_W + ((P > 1) ? $clog2(P) : 0);
    localparam integer HI_W   = T_W - LUT_SHIFT - LUT_ADDR_W;  // các bit trên địa chỉ LUT (tràn)
    localparam [GAMMA_W-1:0] GAMMA_U = GAMMA_Q;                 // γ không dấu đúng GAMMA_W bit

    // ---------------------------------------------------------------- bộ nhớ
    reg [P*DATA_W-1:0]      sv_rom   [0:NWORD-1];
    reg [COEF_W-1:0]        alpha_rom[0:N_SV-1] /* synthesis syn_romstyle = "distributed_rom" */;
    reg [LUT_W-1:0]         lut_rom  [0:LUT_SIZE-1];
    reg signed [DATA_W-1:0] xmem     [0:DIM-1] /* synthesis syn_ramstyle = "registers" */;  // 8 cổng đọc song song
    initial begin
        $readmemh(SV_FILE, sv_rom);
        $readmemh(ALPHA_FILE, alpha_rom);
        $readmemh(LUT_FILE, lut_rom);
    end

    always @(posedge clk)
        if (x_we && !busy) xmem[x_addr] <= x_data[DATA_W-1:0];

    // synthesis translate_off
    initial if (DIM % P != 0) $display("LOI: DIM = %0d khong chia het cho P = %0d", DIM, P);
    // synthesis translate_on

    // ---------------------------------------------------------------- S0: phát địa chỉ
    reg          issuing;
    reg [AW-1:0] addr;
    reg [GW-1:0] g_cnt;
    reg [SW-1:0] i_cnt;
    reg [AW-1:0] a_addr;          // S0 → ROM
    reg          a_v, a_first, a_last, a_svlast;
    reg [GW-1:0] a_g;
    reg          acc_clear;       // xung khởi tạo bộ cộng dồn f

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            issuing <= 1'b0; addr <= 0; g_cnt <= 0; i_cnt <= 0; busy <= 1'b0;
            a_v <= 1'b0; a_first <= 1'b0; a_last <= 1'b0; a_svlast <= 1'b0; a_g <= 0; a_addr <= 0;
            acc_clear <= 1'b0;
        end else begin
            acc_clear <= 1'b0;
            a_v <= 1'b0;
            if (start && !busy) begin
                busy <= 1'b1; issuing <= 1'b1; addr <= 0; g_cnt <= 0; i_cnt <= 0;
                acc_clear <= 1'b1;
            end else if (issuing) begin
                a_v      <= 1'b1;
                a_addr   <= addr;
                a_g      <= g_cnt;
                a_first  <= (g_cnt == 0);
                a_last   <= (g_cnt == G - 1);
                a_svlast <= (g_cnt == G - 1) && (i_cnt == N_SV - 1);
                addr <= addr + 1'b1;
                if (g_cnt == G - 1) begin
                    g_cnt <= 0;
                    if (i_cnt == N_SV - 1) issuing <= 1'b0;
                    else i_cnt <= i_cnt + 1'b1;
                end else g_cnt <= g_cnt + 1'b1;
            end
            if (done) busy <= 1'b0;
        end
    end

    // ---------------------------------------------------------------- S1: đọc ROM SV (đồng bộ)
    reg [P*DATA_W-1:0] sv_word;
    always @(posedge clk) sv_word <= sv_rom[a_addr];
    reg          b_v, b_first, b_last, b_svlast;
    reg [GW-1:0] b_g;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin b_v <= 1'b0; b_first <= 1'b0; b_last <= 1'b0; b_svlast <= 1'b0; b_g <= 0; end
        else begin b_v <= a_v; b_first <= a_first; b_last <= a_last; b_svlast <= a_svlast; b_g <= a_g; end

    // ---------------------------------------------------------------- S2: hiệu x − sv (P làn)
    reg signed [DIFF_W-1:0] diff [0:P-1];
    reg          c_v, c_first, c_last, c_svlast;
    // Mỗi làn đọc phần tử xmem ra một wire RIÊNG rồi mới lấy bit dấu từ wire đó.
    // Không viết xmem[b_g*P+l][DATA_W-1] (chọn bit trên phần tử mảng chỉ số biến): Gowin tổng hợp
    // mất bit dấu ở kiểu này (đã gặp ở hs_feat: μ âm → z bão hòa) dù mô phỏng vẫn đúng.
    genvar gl;
    generate
        for (gl = 0; gl < P; gl = gl + 1) begin : lane
            wire        [DATA_W-1:0] xw  = xmem[b_g * P + gl];
            wire        [DATA_W-1:0] sw  = sv_word[gl*DATA_W +: DATA_W];
            wire signed [DATA_W:0]   x_s = {xw[DATA_W-1], xw};
            wire signed [DATA_W:0]   s_s = {sw[DATA_W-1], sw};
            always @(posedge clk) diff[gl] <= x_s - s_s;
        end
    endgenerate
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin c_v <= 1'b0; c_first <= 1'b0; c_last <= 1'b0; c_svlast <= 1'b0; end
        else begin c_v <= b_v; c_first <= b_first; c_last <= b_last; c_svlast <= b_svlast; end

    // ---------------------------------------------------------------- S3: bình phương
    reg [SQ_W-1:0] sq [0:P-1];
    reg            d_v, d_first, d_last, d_svlast;
    integer m;
    always @(posedge clk) begin
        for (m = 0; m < P; m = m + 1)
            sq[m] <= diff[m] * diff[m];         // toán hạng có dấu, mở rộng lên SQ_W theo vế trái
    end
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin d_v <= 1'b0; d_first <= 1'b0; d_last <= 1'b0; d_svlast <= 1'b0; end
        else begin d_v <= c_v; d_first <= c_first; d_last <= c_last; d_svlast <= c_svlast; end

    // ---------------------------------------------------------------- S4: cây cộng P → 1
    reg [SUM_W-1:0] tree_sum;
    reg [SUM_W-1:0] tree_comb;
    reg             e_v, e_first, e_last, e_svlast;
    integer n;
    always @* begin
        tree_comb = {SUM_W{1'b0}};
        for (n = 0; n < P; n = n + 1) tree_comb = tree_comb + sq[n];
    end
    always @(posedge clk) tree_sum <= tree_comb;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin e_v <= 1'b0; e_first <= 1'b0; e_last <= 1'b0; e_svlast <= 1'b0; end
        else begin e_v <= d_v; e_first <= d_first; e_last <= d_last; e_svlast <= d_svlast; end

    // ---------------------------------------------------------------- S5: cộng dồn D_i
    reg [DIST_W-1:0] dist_acc, dist_out;
    reg              f_v, f_svlast;
    wire [DIST_W-1:0] dist_next = (e_first ? {DIST_W{1'b0}} : dist_acc) + tree_sum;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin dist_acc <= 0; dist_out <= 0; f_v <= 1'b0; f_svlast <= 1'b0; end
        else begin
            f_v <= 1'b0;
            if (e_v) begin
                dist_acc <= dist_next;
                if (e_last) begin dist_out <= dist_next; f_v <= 1'b1; f_svlast <= e_svlast; end
            end
        end
    end

    // ---------------------------------------------------------------- R1: t = D_i · γ
    reg [T_W-1:0] t_r;
    reg           g_v, g_svlast;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin t_r <= 0; g_v <= 1'b0; g_svlast <= 1'b0; end
        else begin
            g_v <= f_v; g_svlast <= f_svlast;
            if (f_v) t_r <= dist_out * GAMMA_U;
        end
    end

    // ---------------------------------------------------------------- R2: địa chỉ LUT, tràn, địa chỉ α
    reg [LUT_ADDR_W-1:0] lut_addr;
    reg                  ovf;
    wire                 ovf_w;                     // t >= LUT_SIZE << LUT_SHIFT  →  k = 0
    generate
        if (HI_W > 0) begin : g_ovf
            assign ovf_w = |t_r[T_W-1 : LUT_SHIFT + LUT_ADDR_W];
        end else begin : g_noovf
            assign ovf_w = 1'b0;
        end
    endgenerate
    reg [SW-1:0]         alpha_addr, alpha_cnt;
    reg                  h_v, h_svlast;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin lut_addr <= 0; ovf <= 1'b0; alpha_addr <= 0; alpha_cnt <= 0; h_v <= 1'b0; h_svlast <= 1'b0; end
        else begin
            h_v <= g_v; h_svlast <= g_svlast;
            if (acc_clear) alpha_cnt <= 0;
            if (g_v) begin
                lut_addr   <= t_r[LUT_SHIFT +: LUT_ADDR_W];
                ovf        <= ovf_w;
                alpha_addr <= alpha_cnt;
                alpha_cnt  <= alpha_cnt + 1'b1;
            end
        end
    end

    // ---------------------------------------------------------------- R3: đọc LUT và α (đồng bộ)
    reg [LUT_W-1:0]  lut_q;
    reg [COEF_W-1:0] alpha_q;
    always @(posedge clk) begin
        lut_q   <= lut_rom[lut_addr];
        alpha_q <= alpha_rom[alpha_addr];
    end
    reg i_v, i_svlast, i_ovf;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin i_v <= 1'b0; i_svlast <= 1'b0; i_ovf <= 1'b0; end
        else begin i_v <= h_v; i_svlast <= h_svlast; i_ovf <= ovf; end

    // ---------------------------------------------------------------- R4: α · k
    wire [LUT_W-1:0]         k_val = i_ovf ? {LUT_W{1'b0}} : lut_q;
    reg  signed [PROD_W-1:0] prod;
    reg                      j_v, j_svlast;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin prod <= 0; j_v <= 1'b0; j_svlast <= 1'b0; end
        else begin
            j_v <= i_v; j_svlast <= i_svlast;
            if (i_v) prod <= $signed(alpha_q) * $signed({1'b0, k_val});
        end
    end

    // ---------------------------------------------------------------- R5: cộng dồn f, quyết định
    reg  signed [ACC_W-1:0] acc;
    wire signed [ACC_W-1:0] acc_next = acc + {{(ACC_W-PROD_W){prod[PROD_W-1]}}, prod};
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin acc <= 0; done <= 1'b0; f_out <= 0; anomaly <= 1'b0; end
        else begin
            done <= 1'b0;
            if (acc_clear) acc <= -RHO_Q;
            else if (j_v) begin
                acc <= acc_next;
                if (j_svlast) begin
                    f_out   <= {{(64-ACC_W){acc_next[ACC_W-1]}}, acc_next};
                    anomaly <= (DECISION_MODE == 1) ? ((acc_next < 0 ? -acc_next : acc_next) > THR_Q)
                                                    : (acc_next < THR_Q);
                    done    <= 1'b1;
                end
            end
        end
    end
endmodule
