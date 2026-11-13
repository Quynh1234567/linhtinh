// -----------------------------------------------------------------------------
// hs_system_top.v – HỆ THỐNG DEMO 1 trên Tang Nano 20K
//
//   ESP32-S3 ──SPI (A5|cờ|chunk|seq + ảnh)──► spi_frame_rx ──► SDRAM 64 Mbit (4 slot ảnh)
//                                                                  │ đọc lại mỗi vòng
//   frame manager ──start(slot trước, slot sau)──► hs_engine_sd ◄──┘ (DERIV → HS × N → FEAT)
//                                                       │ 48 z
//                                                       ▼
//                                                  ocsvm_core ──► LED + dòng UART (BL616 → PC)
//
// Khởi động: tự kiểm tra SDRAM (8 192 byte trên 4 bank) → "SD E=0000 W=0800 K=36" (E = số từ sai).
// Mỗi cặp ảnh: "HS N=… Q=seq C=chu kỳ HS T=chu kỳ từ lúc nhận đủ ảnh tới kết quả F=f_q A=cờ S=bão hòa
//               Z=Σz D=số cặp bị bỏ R=số khung nhận" (hex). Thời gian = chu kỳ / K MHz.
//   DUMP_Z = 1 (chỉ dùng cho phát lại IITB, scripts/34_fpga_replay.py): nối thêm " ZV=" + 48 z × 4 hex
//   (z0 trước, 16 bit bù 2) vào cuối dòng HS để PC so TỪNG phần tử. DUMP_Z = 0: giống hệt bản demo.
// LED (sáng = 0): 0 đảo mỗi kết quả · 1 bất thường · 2 đảo mỗi khung nhận · 3 lỗi SDRAM · 4 đang tính · 5 nhịp 1 Hz
// -----------------------------------------------------------------------------
`timescale 1ns / 1ps
module hs_system_top #(
    parameter integer USE_PLL    = 1,               // 1: 54 MHz qua rPLL · 0: 27 MHz trực tiếp
    parameter integer CLK_HZ     = 54_000_000,      // phải khớp USE_PLL
    parameter integer BAUD       = 115_200,
    parameter integer NIT        = 0,               // 0 = N_ITER của hs_params.vh
    parameter integer EARLY_CAPTURE = 1,
    parameter integer DUMP_Z     = 0,               // 1: in thêm 48 z mỗi kết quả (bản kiểm chứng)
    parameter         LUT_FILE   = "log_lut.mem",
    parameter         MEAN_FILE  = "scaler_mean_q.mem",
    parameter         INV_FILE   = "scaler_inv_q.mem",
    parameter         SV_FILE    = "sv_lanes.mem",
    parameter         ALPHA_FILE = "alpha.mem",
    parameter         EXP_FILE   = "exp_lut.mem"
)(
    input  wire        clk,                         // thạch anh 27 MHz (chân 4)
    input  wire        spi_sck,
    input  wire        spi_mosi,
    input  wire        spi_cs_n,
    output wire        uart_tx,
    output wire [5:0]  led,
    // SDRAM tích hợp trong GW2AR-18 (tên cổng cố định của Gowin, không cần khai trong .cst)
    output wire        O_sdram_clk,
    output wire        O_sdram_cke,
    output wire        O_sdram_cs_n,
    output wire        O_sdram_cas_n,
    output wire        O_sdram_ras_n,
    output wire        O_sdram_wen_n,
    inout  wire [31:0] IO_sdram_dq,
    output wire [10:0] O_sdram_addr,
    output wire [1:0]  O_sdram_ba,
    output wire [3:0]  O_sdram_dqm
);
    // ================================================================ xung nhịp, reset
    wire sclk, sclkp, lock;
    hs_pll #(.USE_PLL(USE_PLL)) u_pll (.clkin(clk), .clkout(sclk), .clkoutp(sclkp), .lock(lock));
    reg [7:0] por = 8'd0;
    always @(posedge sclk) if (!lock) por <= 0; else if (!por[7]) por <= por + 1'b1;
    wire rstn = por[7];

    // ================================================================ SDRAM
    wire        c_rd, c_wr, c_ref, c_busy, c_dready;
    wire [22:0] c_addr;
    wire [7:0]  c_din;
    wire [31:0] c_dout32;
    sdram #(.FREQ(CLK_HZ)) u_sdram (
        .SDRAM_DQ(IO_sdram_dq), .SDRAM_A(O_sdram_addr), .SDRAM_BA(O_sdram_ba), .SDRAM_nCS(O_sdram_cs_n),
        .SDRAM_nWE(O_sdram_wen_n), .SDRAM_nRAS(O_sdram_ras_n), .SDRAM_nCAS(O_sdram_cas_n),
        .SDRAM_CLK(O_sdram_clk), .SDRAM_CKE(O_sdram_cke), .SDRAM_DQM(O_sdram_dqm),
        .clk(sclk), .clk_sdram(sclkp), .resetn(rstn), .rd(c_rd), .wr(c_wr), .refresh(c_ref),
        .addr(c_addr), .din(c_din), .dout(), .dout32(c_dout32), .data_ready(c_dready), .busy(c_busy));

    wire        w_valid, w_pop, r_req, r_ack, r_valid, sd_ready;
    wire [22:0] w_addr, r_addr;
    wire [7:0]  w_data;
    wire [31:0] r_data;
    sdram_arb #(.EARLY_CAPTURE(EARLY_CAPTURE)) u_arb (.clk(sclk), .rstn(rstn),
        .c_rd(c_rd), .c_wr(c_wr), .c_ref(c_ref), .c_addr(c_addr), .c_din(c_din),
        .c_busy(c_busy), .c_dready(c_dready), .c_dout32(c_dout32),
        .w_valid(w_valid), .w_addr(w_addr), .w_data(w_data), .w_pop(w_pop),
        .r_req(r_req), .r_addr(r_addr), .r_ack(r_ack), .r_valid(r_valid), .r_data(r_data), .ready(sd_ready));

    // ================================================================ tự kiểm tra SDRAM
    localparam integer NT = 8192;                      // byte (4 bank × 2 KB)
    reg  [1:0]  tst;                                   // 0 chờ · 1 ghi · 2 đọc · 3 xong
    reg  [13:0] tk, tr_req, tr_got;
    reg  [15:0] sd_err;
    reg         t_rreq;
    wire [15:0] sd_words = NT / 4;
    function [22:0] taddr; input [13:0] k; taddr = {k[12:11], 10'b0100000000, k[10:0]}; endfunction
    function [7:0]  tpat;  input [13:0] k; tpat  = k[7:0] ^ {k[12:8], 3'b101} ^ 8'h5A; endfunction
    wire [13:0] tg = {tr_got[11:0], 2'b00};
    wire [31:0] t_exp = {tpat(tg + 14'd3), tpat(tg + 14'd2), tpat(tg + 14'd1), tpat(tg)};
    always @(posedge sclk or negedge rstn) begin
        if (!rstn) begin tst <= 0; tk <= 0; tr_req <= 0; tr_got <= 0; sd_err <= 0; t_rreq <= 1'b0; end
        else case (tst)
            2'd0: if (sd_ready) tst <= 2'd1;
            2'd1: if (w_pop) begin if (tk == NT - 1) tst <= 2'd2; tk <= tk + 1'b1; end
            2'd2: begin
                if (t_rreq && r_ack) begin t_rreq <= 1'b0; tr_req <= tr_req + 1'b1; end
                else if (!t_rreq && tr_req < NT / 4) t_rreq <= 1'b1;
                if (r_valid) begin
                    if (r_data != t_exp) sd_err <= sd_err + 1'b1;
                    tr_got <= tr_got + 1'b1;
                    if (tr_got == NT / 4 - 1) tst <= 2'd3;
                end
            end
            default: ;
        endcase
    end
    wire testing = (tst != 2'd3);

    // ================================================================ nhận ảnh SPI
    wire        f_done, f_reset;
    wire [1:0]  f_slot, wslot;
    wire [7:0]  f_seq;
    wire [15:0] n_bad;
    wire        s_wvalid;
    wire [22:0] s_waddr;
    wire [7:0]  s_wdata;
    spi_frame_rx u_spi (.clk(sclk), .rstn(rstn & ~testing), .spi_sck(spi_sck), .spi_mosi(spi_mosi), .spi_cs_n(spi_cs_n),
        .wslot(wslot), .w_valid(s_wvalid), .w_addr(s_waddr), .w_data(s_wdata), .w_pop(w_pop & ~testing),
        .frame_done(f_done), .done_slot(f_slot), .done_seq(f_seq), .done_reset(f_reset), .n_bad(n_bad));
    assign w_valid = testing ? (tst == 2'd1) : s_wvalid;
    assign w_addr  = testing ? taddr(tk) : s_waddr;
    assign w_data  = testing ? tpat(tk) : s_wdata;

    // ================================================================ quản lý khung (4 slot × 32 KB)
    reg        have_prev, eng_on;
    reg  [1:0] prev, ea, eb;
    reg  [7:0] e_seq, n_drop;
    reg [15:0] n_rx;
    reg        e_start;
    reg [31:0] lat;
    function ok_slot; input [1:0] s; input hp; input [1:0] p; input on; input [1:0] a, b;
        ok_slot = !(hp && s == p) && !(on && (s == a || s == b));
    endfunction
    assign wslot = ok_slot(2'd0, have_prev, prev, eng_on, ea, eb) ? 2'd0 :
                   ok_slot(2'd1, have_prev, prev, eng_on, ea, eb) ? 2'd1 :
                   ok_slot(2'd2, have_prev, prev, eng_on, ea, eb) ? 2'd2 : 2'd3;
    wire svm_done;
    always @(posedge sclk or negedge rstn) begin
        if (!rstn) begin
            have_prev <= 1'b0; eng_on <= 1'b0; prev <= 0; ea <= 0; eb <= 0; e_seq <= 0; n_drop <= 0; n_rx <= 0;
            e_start <= 1'b0; lat <= 0;
        end else begin
            e_start <= 1'b0;
            if (eng_on) lat <= lat + 1'b1;
            if (svm_done) eng_on <= 1'b0;
            if (f_done) begin
                n_rx <= n_rx + 1'b1;
                if (f_reset || !have_prev) begin prev <= f_slot; have_prev <= 1'b1; end   // khung đầu chuỗi mới
                else if (!eng_on) begin
                    ea <= prev; eb <= f_slot; e_seq <= f_seq; eng_on <= 1'b1; e_start <= 1'b1; lat <= 0;
                    prev <= f_slot;
                end else begin n_drop <= n_drop + 1'b1; prev <= f_slot; end
            end
        end
    end

    // ================================================================ HS + OC-SVM
    wire        e_rreq, x_we, svm_start, e_busy, anomaly, svm_busy;
    wire [22:0] e_raddr;
    wire [7:0]  x_addr;
    wire [15:0] x_data;
    wire [31:0] n_sat, cycles;
    wire signed [63:0] f_out;
    hs_engine_sd #(.NIT(NIT), .LUT_FILE(LUT_FILE), .MEAN_FILE(MEAN_FILE), .INV_FILE(INV_FILE)) u_eng (
        .clk(sclk), .rstn(rstn), .start(e_start), .base_a({6'd0, ea, 15'd0}), .base_b({6'd0, eb, 15'd0}),
        .r_req(e_rreq), .r_addr(e_raddr), .r_ack(r_ack & ~testing), .r_valid(r_valid & ~testing), .r_data(r_data),
        .x_we(x_we), .x_addr(x_addr), .x_data(x_data), .svm_start(svm_start), .busy(e_busy), .n_sat(n_sat), .cycles(cycles));
    assign r_req  = testing ? t_rreq : e_rreq;
    assign r_addr = testing ? taddr({tr_req[11:0], 2'b00}) : e_raddr;
    ocsvm_core #(.P(8), .SV_FILE(SV_FILE), .ALPHA_FILE(ALPHA_FILE), .LUT_FILE(EXP_FILE)) u_svm (
        .clk(sclk), .rst_n(rstn), .x_we(x_we), .x_addr(x_addr), .x_data(x_data),
        .start(svm_start), .busy(svm_busy), .done(svm_done), .f_out(f_out), .anomaly(anomaly));
    reg [15:0] zsum;
    always @(posedge sclk) if (e_start) zsum <= 0; else if (x_we) zsum <= zsum + x_data;
    // 48 z của cặp vừa tính (chỉ khi DUMP_Z). Cặp sau ghi đè sau ≈ 57 ms, dòng UART gửi xong trong ≈ 25 ms.
    reg [15:0] zbuf [0:47];
    always @(posedge sclk) if (DUMP_Z != 0 && x_we && x_addr < 8'd48) zbuf[x_addr[5:0]] <= x_data;

    // ================================================================ UART
    reg  [15:0] n_res;
    reg  [7:0]  q_seq, q_drop;
    reg  [31:0] q_cyc, q_lat, q_f, q_sat;
    reg  [15:0] q_z;
    reg  [3:0]  q_a4;
    reg         r_anom;
    wire [7:0]  clk_mhz = CLK_HZ / 1_000_000;
    localparam integer CI_W = (DUMP_Z != 0) ? 9 : 7;
    reg  [CI_W-1:0] ci;
    reg  [1:0]  um;                                    // 0 rỗi · 1 gửi SD · 2 gửi HS
    reg         sd_pend, hs_pend, send, wait_busy;
    wire        tx_busy;
    function [7:0] hx; input [3:0] v; hx = (v < 10) ? (8'h30 + v) : (8'h41 + v - 10); endfunction
    reg [7:0] ch_hs, ch_sd;
    localparam integer ZV0 = 85;                       // ký tự hex đầu tiên của 48 z
    localparam integer N_HS = (DUMP_Z != 0) ? ZV0 + 192 + 2 : 83, N_SD = 23;
    always @(*) begin
        case (ci)
        0: ch_hs = "H";
        1: ch_hs = "S";
        2: ch_hs = " ";
        3: ch_hs = "N";
        4: ch_hs = "=";
        5: ch_hs = hx(n_res[15:12]);
        6: ch_hs = hx(n_res[11:8]);
        7: ch_hs = hx(n_res[7:4]);
        8: ch_hs = hx(n_res[3:0]);
        9: ch_hs = " ";
        10: ch_hs = "Q";
        11: ch_hs = "=";
        12: ch_hs = hx(q_seq[7:4]);
        13: ch_hs = hx(q_seq[3:0]);
        14: ch_hs = " ";
        15: ch_hs = "C";
        16: ch_hs = "=";
        17: ch_hs = hx(q_cyc[31:28]);
        18: ch_hs = hx(q_cyc[27:24]);
        19: ch_hs = hx(q_cyc[23:20]);
        20: ch_hs = hx(q_cyc[19:16]);
        21: ch_hs = hx(q_cyc[15:12]);
        22: ch_hs = hx(q_cyc[11:8]);
        23: ch_hs = hx(q_cyc[7:4]);
        24: ch_hs = hx(q_cyc[3:0]);
        25: ch_hs = " ";
        26: ch_hs = "T";
        27: ch_hs = "=";
        28: ch_hs = hx(q_lat[31:28]);
        29: ch_hs = hx(q_lat[27:24]);
        30: ch_hs = hx(q_lat[23:20]);
        31: ch_hs = hx(q_lat[19:16]);
        32: ch_hs = hx(q_lat[15:12]);
        33: ch_hs = hx(q_lat[11:8]);
        34: ch_hs = hx(q_lat[7:4]);
        35: ch_hs = hx(q_lat[3:0]);
        36: ch_hs = " ";
        37: ch_hs = "F";
        38: ch_hs = "=";
        39: ch_hs = hx(q_f[31:28]);
        40: ch_hs = hx(q_f[27:24]);
        41: ch_hs = hx(q_f[23:20]);
        42: ch_hs = hx(q_f[19:16]);
        43: ch_hs = hx(q_f[15:12]);
        44: ch_hs = hx(q_f[11:8]);
        45: ch_hs = hx(q_f[7:4]);
        46: ch_hs = hx(q_f[3:0]);
        47: ch_hs = " ";
        48: ch_hs = "A";
        49: ch_hs = "=";
        50: ch_hs = hx(q_a4[3:0]);
        51: ch_hs = " ";
        52: ch_hs = "S";
        53: ch_hs = "=";
        54: ch_hs = hx(q_sat[31:28]);
        55: ch_hs = hx(q_sat[27:24]);
        56: ch_hs = hx(q_sat[23:20]);
        57: ch_hs = hx(q_sat[19:16]);
        58: ch_hs = hx(q_sat[15:12]);
        59: ch_hs = hx(q_sat[11:8]);
        60: ch_hs = hx(q_sat[7:4]);
        61: ch_hs = hx(q_sat[3:0]);
        62: ch_hs = " ";
        63: ch_hs = "Z";
        64: ch_hs = "=";
        65: ch_hs = hx(q_z[15:12]);
        66: ch_hs = hx(q_z[11:8]);
        67: ch_hs = hx(q_z[7:4]);
        68: ch_hs = hx(q_z[3:0]);
        69: ch_hs = " ";
        70: ch_hs = "D";
        71: ch_hs = "=";
        72: ch_hs = hx(q_drop[7:4]);
        73: ch_hs = hx(q_drop[3:0]);
        74: ch_hs = " ";
        75: ch_hs = "R";
        76: ch_hs = "=";
        77: ch_hs = hx(n_rx[15:12]);
        78: ch_hs = hx(n_rx[11:8]);
        79: ch_hs = hx(n_rx[7:4]);
        80: ch_hs = hx(n_rx[3:0]);
        81: ch_hs = (DUMP_Z != 0) ? " " : 8'h0D;
        82: ch_hs = (DUMP_Z != 0) ? "Z" : 8'h0A;
        83: ch_hs = "V";
        84: ch_hs = "=";
        ZV0 + 192:     ch_hs = 8'h0D;
        ZV0 + 192 + 1: ch_hs = 8'h0A;
        default: ch_hs = 8'h0A;
        endcase
    end
    // ký tự hex của 48 z: đọc zbuf qua 2 tầng thanh ghi (ci giữ nguyên ~470 chu kỳ mỗi ký tự,
    // nên dữ liệu luôn ổn định trước khi uart_tx chốt) – không thêm đường tổ hợp dài vào 54 MHz
    wire [8:0] ci9 = ci;                               // mở rộng 0 (CI_W = 7 hoặc 9)
    wire [8:0] kz  = ci9 - ZV0;
    reg  [15:0] zw_r;
    reg  [1:0]  zn_r;
    reg         zon_r;
    wire in_zv = (DUMP_Z != 0) && (um == 2'd2) && (ci9 >= ZV0) && (ci9 < ZV0 + 192);   // kz[7:2] ∈ 0…47
    always @(posedge sclk) begin
        if (in_zv) zw_r <= zbuf[kz[7:2]];              // chỉ đọc trong vùng ZV=: không bao giờ ra ngoài 0…47
        zn_r  <= kz[1:0];
        zon_r <= in_zv;
    end
    wire [3:0] z_nib = (zn_r == 2'd0) ? zw_r[15:12] : (zn_r == 2'd1) ? zw_r[11:8] :
                       (zn_r == 2'd2) ? zw_r[7:4]   : zw_r[3:0];
    always @(*) begin
        case (ci)
        0: ch_sd = "S";
        1: ch_sd = "D";
        2: ch_sd = " ";
        3: ch_sd = "E";
        4: ch_sd = "=";
        5: ch_sd = hx(sd_err[15:12]);
        6: ch_sd = hx(sd_err[11:8]);
        7: ch_sd = hx(sd_err[7:4]);
        8: ch_sd = hx(sd_err[3:0]);
        9: ch_sd = " ";
        10: ch_sd = "W";
        11: ch_sd = "=";
        12: ch_sd = hx(sd_words[15:12]);
        13: ch_sd = hx(sd_words[11:8]);
        14: ch_sd = hx(sd_words[7:4]);
        15: ch_sd = hx(sd_words[3:0]);
        16: ch_sd = " ";
        17: ch_sd = "K";
        18: ch_sd = "=";
        19: ch_sd = hx(clk_mhz[7:4]);
        20: ch_sd = hx(clk_mhz[3:0]);
        21: ch_sd = 8'h0D;
        22: ch_sd = 8'h0A;
        default: ch_sd = 8'h0A;
        endcase
    end

    wire [7:0] ch = (um == 2'd1) ? ch_sd : ch_hs;
    reg  [7:0] ch_r;                                   // DUMP_Z: ký tự qua thanh ghi (xem trên)
    always @(posedge sclk) ch_r <= zon_r ? hx(z_nib) : ch;
    wire [7:0] ch_tx = (DUMP_Z != 0) ? ch_r : ch;      // DUMP_Z = 0: y hệt bản demo
    uart_tx #(.CLKS_PER_BIT(CLK_HZ / BAUD)) u_tx (.clk(sclk), .rst_n(rstn), .data(ch_tx), .send(send), .tx(uart_tx), .busy(tx_busy));
    reg tst_done_d;
    always @(posedge sclk or negedge rstn) begin
        if (!rstn) begin
            n_res <= 0; q_seq <= 0; q_drop <= 0; q_cyc <= 0; q_lat <= 0; q_f <= 0; q_sat <= 0; q_z <= 0; q_a4 <= 0; r_anom <= 1'b0;
            ci <= 0; um <= 0; sd_pend <= 1'b0; hs_pend <= 1'b0; send <= 1'b0; wait_busy <= 1'b0; tst_done_d <= 1'b0;
        end else begin
            send <= 1'b0;
            tst_done_d <= !testing;
            if (!testing && !tst_done_d) sd_pend <= 1'b1;
            if (svm_done) begin
                n_res <= n_res + 1'b1; q_seq <= e_seq; q_cyc <= cycles; q_lat <= lat; q_f <= f_out[31:0];
                q_a4 <= {3'b0, anomaly}; q_sat <= n_sat; q_z <= zsum; q_drop <= n_drop; r_anom <= anomaly;
                hs_pend <= 1'b1;
            end
            case (um)
            2'd0: if (sd_pend) begin um <= 2'd1; ci <= 0; sd_pend <= 1'b0; end
                  else if (hs_pend) begin um <= 2'd2; ci <= 0; hs_pend <= 1'b0; end
            default: begin
                if (!wait_busy) begin
                    if (!tx_busy && !send) begin send <= 1'b1; wait_busy <= 1'b1; end
                end else if (tx_busy) begin
                    wait_busy <= 1'b0;
                    if (ci == ((um == 2'd1) ? N_SD : N_HS) - 1) um <= 2'd0;
                    else ci <= ci + 1'b1;
                end
            end
            endcase
        end
    end

    // ================================================================ LED
    reg l_res, l_rx;
    reg [25:0] hb;
    always @(posedge sclk or negedge rstn) begin
        if (!rstn) begin l_res <= 1'b0; l_rx <= 1'b0; hb <= 0; end
        else begin
            if (svm_done) l_res <= ~l_res;
            if (f_done) l_rx <= ~l_rx;
            hb <= (hb == CLK_HZ / 2 - 1) ? 0 : hb + 1'b1;
        end
    end
    reg hbt;
    always @(posedge sclk or negedge rstn) if (!rstn) hbt <= 1'b0; else if (hb == 0) hbt <= ~hbt;
    assign led = ~{hbt, e_busy, (sd_err != 0), l_rx, r_anom, l_res};
endmodule
