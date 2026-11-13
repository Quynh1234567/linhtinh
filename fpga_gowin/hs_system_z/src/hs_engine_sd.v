// -----------------------------------------------------------------------------
// hs_engine_sd.v – khối trích đặc trưng dùng SDRAM thật (bản demo 1)
//   Mỗi vòng Jacobi: hs_img_reader đọc lại 2 khung từ SDRAM → hs_deriv tính lại gói (Ix4, Iy4,
//   It4, a, b) → hs_iter. Không cần lưu 1,2 Mbit gói DERIV; kết quả giống hệt (gói là hàm của ảnh).
//   Vòng cuối → hs_feat → 48 z → ocsvm_core (bên ngoài).
// Thời gian ≈ N_ITER × 9 600 lần đọc × 5 chu kỳ (bộ điều khiển SDRAM không burst):
//   54 MHz: 64 vòng ≈ 57 ms, 32 vòng ≈ 29 ms.
// -----------------------------------------------------------------------------
`timescale 1ns / 1ps
module hs_engine_sd #(
    parameter integer NIT       = 0,
    parameter         LUT_FILE  = "log_lut.mem",
    parameter         MEAN_FILE = "scaler_mean_q.mem",
    parameter         INV_FILE  = "scaler_inv_q.mem"
)(
    input  wire        clk,
    input  wire        rstn,
    input  wire        start,
    input  wire [22:0] base_a,
    input  wire [22:0] base_b,
    output wire        r_req,
    output wire [22:0] r_addr,
    input  wire        r_ack,
    input  wire        r_valid,
    input  wire [31:0] r_data,
    output wire        x_we,
    output wire [7:0]  x_addr,
    output wire [15:0] x_data,
    output reg         svm_start,
    output reg         busy,
    output wire [31:0] n_sat,
    output reg  [31:0] cycles
);
`undef HS_PARAMS_VH
`include "hs_params.vh"
    localparam integer NI = (NIT > 0) ? NIT : N_ITER;

    reg         p_start;
    reg  [7:0]  pass;
    reg  [22:0] la, lb;
    wire        pv, pr, dv, dr, d_done, d_busy, i_done, i_busy, f_done, o_valid;
    wire [15:0] pd;
    wire [63:0] dd;
    wire signed [12:0] o_u, o_v;
    wire [6:0]  iter;

    hs_img_reader u_rd (.clk(clk), .rstn(rstn), .start(p_start), .base_a(la), .base_b(lb),
        .r_req(r_req), .r_addr(r_addr), .r_ack(r_ack), .r_valid(r_valid), .r_data(r_data),
        .out_valid(pv), .out_ready(pr), .out_data(pd));
    hs_deriv u_d (.clk(clk), .rst_n(rstn), .start(p_start), .in_valid(pv), .in_ready(pr), .in_data(pd),
        .out_valid(dv), .out_ready(dr), .out_data(dd), .done(d_done), .busy(d_busy));
    hs_iter #(.NIT(NI)) u_i (.clk(clk), .rst_n(rstn), .start(start), .in_valid(dv), .in_ready(dr), .in_data(dd),
        .out_valid(o_valid), .out_u(o_u), .out_v(o_v), .done(i_done), .busy(i_busy), .n_sat(n_sat), .iter(iter));
    hs_feat #(.LUT_FILE(LUT_FILE), .MEAN_FILE(MEAN_FILE), .INV_FILE(INV_FILE)) u_f (
        .clk(clk), .rst_n(rstn), .start(start), .in_valid(o_valid), .in_u(o_u), .in_v(o_v),
        .mag_valid(), .mag(), .res_valid(), .res_d(), .res_sum(), .res_mean(), .res_log(), .res_z(),
        .x_we(x_we), .x_addr(x_addr), .x_data(x_data), .done(f_done));

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin p_start <= 1'b0; pass <= 0; la <= 0; lb <= 0; busy <= 1'b0; svm_start <= 1'b0; cycles <= 0; end
        else begin
            p_start <= 1'b0; svm_start <= 1'b0;
            if (busy) cycles <= cycles + 1'b1;
            if (start && !busy) begin
                busy <= 1'b1; la <= base_a; lb <= base_b; pass <= 1; p_start <= 1'b1; cycles <= 0;
            end else if (busy) begin
                if (d_done && pass < NI) begin pass <= pass + 1'b1; p_start <= 1'b1; end
                if (f_done) begin busy <= 1'b0; svm_start <= 1'b1; end
            end
        end
    end
endmodule
