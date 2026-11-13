// -----------------------------------------------------------------------------
// spi_frame_rx.v – nhận ảnh 160×120 (19 200 byte) từ ESP32 qua SPI (mode 0, MSB trước) và ghi SDRAM
//
// Mỗi giao dịch (CS thấp): 4 byte đầu  A5 | cờ | chunk | seq , sau đó tối đa 4 096 byte ảnh.
//   cờ bit0 = RESET_PAIR (khung tối / ngắt cặp) · chunk 0…4: byte i ghi vào slot·32K + chunk·4096 + i
//   chunk 0 mở khung mới (chốt slot ghi wslot). Đủ 19 200 byte → chờ ghi xong SDRAM → frame_done.
// SCK, MOSI, CS được lấy mẫu bằng clk hệ thống (2 tầng đồng bộ) → SCK ≤ clk / 5 (≤ 10 MHz ở 54 MHz).
// -----------------------------------------------------------------------------
`timescale 1ns / 1ps
module spi_frame_rx #(
    parameter integer FRAME_BYTES = 19200
)(
    input  wire        clk,
    input  wire        rstn,
    input  wire        spi_sck,
    input  wire        spi_mosi,
    input  wire        spi_cs_n,
    input  wire [1:0]  wslot,            // slot cho khung tiếp theo (từ bộ quản lý khung)
    // đầu FIFO ghi SDRAM
    output wire        w_valid,
    output wire [22:0] w_addr,
    output wire [7:0]  w_data,
    input  wire        w_pop,
    // khung xong (đã nằm hết trong SDRAM)
    output reg         frame_done,
    output reg  [1:0]  done_slot,
    output reg  [7:0]  done_seq,
    output reg         done_reset,
    output reg  [15:0] n_bad              // giao dịch sai header / tràn FIFO
);
    // ---------------------------------------------------------------- đồng bộ + bắt cạnh
    reg [2:0] sck_s;
    reg [1:0] cs_s, mosi_s;
    always @(posedge clk) begin
        sck_s  <= {sck_s[1:0], spi_sck};
        cs_s   <= {cs_s[0], spi_cs_n};
        mosi_s <= {mosi_s[0], spi_mosi};
    end
    wire cs_act = !cs_s[1];
    wire sck_up = (sck_s[2:1] == 2'b01);

    reg [2:0]  bitc;
    reg [7:0]  shr;
    reg [12:0] bidx;           // byte trong giao dịch
    reg        byte_v;
    reg [7:0]  byte_d;
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin bitc <= 0; shr <= 0; byte_v <= 1'b0; byte_d <= 0; end
        else begin
            byte_v <= 1'b0;
            if (!cs_act) bitc <= 0;
            else if (sck_up) begin
                shr  <= {shr[6:0], mosi_s[1]};
                bitc <= bitc + 1'b1;
                if (bitc == 3'd7) begin byte_v <= 1'b1; byte_d <= {shr[6:0], mosi_s[1]}; end
            end
        end
    end

    // ---------------------------------------------------------------- header + địa chỉ
    reg [7:0]  h_flags, h_chunk, h_seq;
    reg        t_ok;               // giao dịch hợp lệ
    reg        f_open;             // đang nhận một khung
    reg [1:0]  f_slot;
    reg [7:0]  f_seq;
    reg        f_reset;
    reg [14:0] f_cnt;              // byte ảnh đã nhận
    reg        flush;
    reg [3:0]  flush_cnt;
    reg        cs_prev;
    // FIFO 32 × 31 bit
    reg [30:0] fifo [0:31];
    reg [4:0]  wp, rp;
    reg [5:0]  fcount;
    wire       fifo_full = (fcount == 6'd32);
    reg        push;
    reg [30:0] push_d;

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            bidx <= 0; t_ok <= 1'b0; f_open <= 1'b0; f_slot <= 0; f_seq <= 0; f_reset <= 1'b0; f_cnt <= 0;
            flush <= 1'b0; flush_cnt <= 0; frame_done <= 1'b0; done_slot <= 0; done_seq <= 0; done_reset <= 1'b0;
            n_bad <= 0; h_flags <= 0; h_chunk <= 0; h_seq <= 0; push <= 1'b0; push_d <= 0; cs_prev <= 1'b0;
        end else begin
            frame_done <= 1'b0;
            push <= 1'b0;
            cs_prev <= cs_act;
            if (cs_act && !cs_prev) begin bidx <= 0; t_ok <= 1'b0; end
            if (byte_v) begin
                bidx <= bidx + 1'b1;
                case (bidx)
                13'd0: begin t_ok <= (byte_d == 8'hA5); if (byte_d != 8'hA5) n_bad <= n_bad + 1'b1; end
                13'd1: h_flags <= byte_d;
                13'd2: h_chunk <= byte_d;
                13'd3: begin
                    h_seq <= byte_d;
                    if (t_ok && h_chunk == 0) begin                  // khung mới
                        f_open <= 1'b1; f_slot <= wslot; f_seq <= byte_d; f_reset <= h_flags[0];
                        f_cnt <= 0; flush <= 1'b0;
                    end else if (t_ok && !f_open) t_ok <= 1'b0;       // mảnh lẻ không thuộc khung nào
                end
                default: if (t_ok && f_open && !flush) begin
                    if (fifo_full) n_bad <= n_bad + 1'b1;
                    else begin
                        push   <= 1'b1;
                        push_d <= {f_slot, ({h_chunk[2:0], 12'd0} + (bidx - 13'd4)) & 15'h7FFF, byte_d} ;
                    end
                    f_cnt <= f_cnt + 1'b1;
                    if (f_cnt == FRAME_BYTES - 1) flush <= 1'b1;
                end
                endcase
            end
            // đủ byte → đợi FIFO rỗng (đã ghi SDRAM) + vài chu kỳ cho lệnh ghi cuối
            if (flush && f_open) begin
                if (fcount != 0 || push) flush_cnt <= 0;
                else if (flush_cnt == 4'd12) begin
                    frame_done <= 1'b1; done_slot <= f_slot; done_seq <= f_seq; done_reset <= f_reset;
                    f_open <= 1'b0; flush <= 1'b0; flush_cnt <= 0;
                end else flush_cnt <= flush_cnt + 1'b1;
            end
        end
    end

    // ---------------------------------------------------------------- FIFO
    always @(posedge clk) if (push && !fifo_full) fifo[wp] <= push_d;
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin wp <= 0; rp <= 0; fcount <= 0; end
        else begin
            if (push && !fifo_full) wp <= wp + 1'b1;
            if (w_pop && fcount != 0) rp <= rp + 1'b1;
            fcount <= fcount + ((push && !fifo_full) ? 1 : 0) - ((w_pop && fcount != 0) ? 1 : 0);
        end
    end
    wire [30:0] head = fifo[rp];
    assign w_valid = (fcount != 0);
    assign w_data  = head[7:0];
    assign w_addr  = {6'd0, head[24:23], head[22:8]};
endmodule
