// -----------------------------------------------------------------------------
// sdram_arb.v – phân xử 3 nguồn trước bộ điều khiển SDRAM (third_party/sdram.v):
//   refresh (ưu tiên 1, mỗi REFRESH_CYC chu kỳ) > ghi byte (ảnh từ SPI) > đọc từ 32 bit (ảnh cho HS)
//
// Bộ điều khiển: mỗi lệnh (đọc/ghi/refresh) mất đúng 5 chu kỳ, địa chỉ phải giữ nguyên tới chu kỳ
// thứ 2. Khối này giữ lệnh trong thanh ghi, nạp lệnh kế tiếp đúng ở chu kỳ thứ 4 để lệnh sau được
// nhận ngay khi busy = 0 → 1 lệnh / 5 chu kỳ liên tục.
// Dữ liệu đọc: EARLY_CAPTURE = 1 lấy DQ đã chốt ở chu kỳ có data_ready lên (giữa cửa sổ dữ liệu
// khi SDRAM chạy 54 MHz, CAS = 2); 0 = lấy DQ ở chu kỳ sau như ví dụ gốc.
// -----------------------------------------------------------------------------
`timescale 1ns / 1ps
module sdram_arb #(
    parameter integer REFRESH_CYC   = 700,      // < 15 µs ở mọi xung nhịp ≤ 54 MHz
    parameter integer EARLY_CAPTURE = 1
)(
    input  wire        clk,
    input  wire        rstn,
    // tới bộ điều khiển
    output reg         c_rd,
    output reg         c_wr,
    output reg         c_ref,
    output reg  [22:0] c_addr,
    output reg  [7:0]  c_din,
    input  wire        c_busy,
    input  wire        c_dready,
    input  wire [31:0] c_dout32,
    // nguồn ghi byte (đầu FIFO): w_pop = 1 chu kỳ khi lệnh ghi được nhận
    input  wire        w_valid,
    input  wire [22:0] w_addr,
    input  wire [7:0]  w_data,
    output reg         w_pop,
    // nguồn đọc từ: giữ r_req + r_addr tới khi r_ack; dữ liệu về theo thứ tự với r_valid
    input  wire        r_req,
    input  wire [22:0] r_addr,
    output reg         r_ack,
    output reg         r_valid,
    output reg  [31:0] r_data,
    output reg         ready                    // SDRAM khởi động xong
);
    localparam [1:0] K_RD = 2'd1, K_WR = 2'd2, K_REF = 2'd3;
    reg        pend, active;
    reg [2:0]  cnt;
    reg [1:0]  kind;
    reg [15:0] rcnt;
    reg        ref_due;
    reg [31:0] dq_d;
    always @(posedge clk) dq_d <= c_dout32;

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            c_rd <= 1'b0; c_wr <= 1'b0; c_ref <= 1'b0; c_addr <= 0; c_din <= 0;
            pend <= 1'b0; active <= 1'b0; cnt <= 0; kind <= 0; rcnt <= 0; ref_due <= 1'b0;
            w_pop <= 1'b0; r_ack <= 1'b0; r_valid <= 1'b0; r_data <= 0; ready <= 1'b0;
        end else begin
            w_pop <= 1'b0; r_ack <= 1'b0; r_valid <= 1'b0;
            if (!c_busy) ready <= 1'b1;
            if (rcnt == REFRESH_CYC - 1) begin rcnt <= 0; ref_due <= 1'b1; end
            else rcnt <= rcnt + 1'b1;
            if (c_dready) begin
                r_valid <= 1'b1;
                r_data  <= (EARLY_CAPTURE != 0) ? dq_d : c_dout32;
            end
            if (active) begin
                cnt <= cnt + 1'b1;
                if (cnt == 3'd3) active <= 1'b0;
            end
            if (pend && !c_busy && !active) begin                 // bộ điều khiển nhận lệnh ở cạnh này
                pend <= 1'b0; active <= 1'b1; cnt <= 0;
                if (kind == K_WR) w_pop <= 1'b1;
                if (kind == K_RD) r_ack <= 1'b1;
                if (kind == K_REF && !(rcnt == REFRESH_CYC - 1)) ref_due <= 1'b0;
            end else if (!pend && (!active || cnt == 3'd3)) begin  // nạp lệnh kế tiếp
                c_rd <= 1'b0; c_wr <= 1'b0; c_ref <= 1'b0;
                if (ready && ref_due) begin
                    c_ref <= 1'b1; kind <= K_REF; pend <= 1'b1;
                end else if (ready && w_valid) begin
                    c_wr <= 1'b1; c_addr <= w_addr; c_din <= w_data; kind <= K_WR; pend <= 1'b1;
                end else if (ready && r_req) begin
                    c_rd <= 1'b1; c_addr <= r_addr; kind <= K_RD; pend <= 1'b1;
                end
            end
        end
    end
endmodule
