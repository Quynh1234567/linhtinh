// -----------------------------------------------------------------------------
// hs_img_reader.v – đọc 2 khung từ SDRAM, phát luồng {i2, i1} theo hàng (19 200 nhịp) cho hs_deriv
// Mỗi nhóm 4 điểm: đọc 1 từ khung A (i1) + 1 từ khung B (i2) → 4 nhịp. Byte p của khung nằm ở
// địa chỉ base + p, làn byte p mod 4 của từ 32 bit (little-endian, giống cách ghi byte của SDRAM).
// FIFO 64 × 64 bit giữ các cặp từ để bù lúc hs_deriv đang quét hàng (không nhận).
// -----------------------------------------------------------------------------
`timescale 1ns / 1ps
module hs_img_reader (
    input  wire        clk,
    input  wire        rstn,
    input  wire        start,
    input  wire [22:0] base_a,
    input  wire [22:0] base_b,
    // nguồn đọc của sdram_arb
    output reg         r_req,
    output reg  [22:0] r_addr,
    input  wire        r_ack,
    input  wire        r_valid,
    input  wire [31:0] r_data,
    // luồng ra
    output wire        out_valid,
    input  wire        out_ready,
    output wire [15:0] out_data
);
    localparam integer NGRP = 4800;          // 19 200 / 4
    localparam integer DEPTH = 64;
    reg [22:0] ba, bb;
    reg [12:0] g_req;                         // nhóm đang yêu cầu
    reg        half;                          // 0: từ A, 1: từ B
    reg        running;
    reg [6:0]  inflight;                      // cặp đã yêu cầu nhưng chưa vào FIFO
    // FIFO
    reg [63:0] mem [0:DEPTH-1] /* synthesis syn_ramstyle = "distributed_ram" */;
    reg [5:0]  wp, rp;
    reg [6:0]  cnt;
    reg [31:0] wa;                            // từ A đang chờ ghép
    reg        rsel;                          // dữ liệu về: 0 = A, 1 = B
    wire       push = r_valid && rsel;
    reg        pop;

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            r_req <= 1'b0; r_addr <= 0; ba <= 0; bb <= 0; g_req <= 0; half <= 1'b0; running <= 1'b0;
            inflight <= 0; rsel <= 1'b0; wa <= 0;
        end else begin
            if (start) begin
                ba <= base_a; bb <= base_b; g_req <= 0; half <= 1'b0; running <= 1'b1; rsel <= 1'b0;
                r_req <= 1'b0; inflight <= 0;
            end else begin
                // yêu cầu: chỉ mở nhóm mới khi FIFO còn chỗ cho mọi cặp đang bay
                if (r_req && r_ack) begin
                    r_req <= 1'b0;
                    if (half) begin half <= 1'b0; g_req <= g_req + 1'b1; end
                    else half <= 1'b1;
                end else if (!r_req && running) begin
                    if (half) begin r_req <= 1'b1; r_addr <= bb + {g_req, 2'b00}; end
                    else if (g_req < NGRP && (cnt + inflight) < DEPTH - 2) begin
                        r_req <= 1'b1; r_addr <= ba + {g_req, 2'b00};
                    end else if (g_req >= NGRP) running <= 1'b0;
                end
                if (r_req && r_ack && !half) inflight <= inflight + 1'b1 - (push ? 1'b1 : 1'b0);
                else if (push) inflight <= inflight - 1'b1;
                // dữ liệu về theo thứ tự A, B, A, B …
                if (r_valid) begin
                    if (!rsel) wa <= r_data;
                    rsel <= ~rsel;
                end
            end
        end
    end

    always @(posedge clk) if (push) mem[wp] <= {r_data, wa};           // {B, A}
    reg [1:0] beat;
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin wp <= 0; rp <= 0; cnt <= 0; beat <= 0; end
        else if (start) begin wp <= 0; rp <= 0; cnt <= 0; beat <= 0; end
        else begin
            if (push) wp <= wp + 1'b1;
            if (pop) rp <= rp + 1'b1;
            cnt <= cnt + (push ? 1 : 0) - (pop ? 1 : 0);
            if (out_valid && out_ready) beat <= beat + 1'b1;
        end
    end
    wire [63:0] head = mem[rp];
    always @(*) pop = out_valid && out_ready && (beat == 2'd3);
    assign out_valid = (cnt != 0);
    wire [31:0] wA = head[31:0], wB = head[63:32];
    assign out_data = {wB[8*beat +: 8], wA[8*beat +: 8]};
endmodule
