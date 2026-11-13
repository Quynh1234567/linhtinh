// -----------------------------------------------------------------------------
// uart_tx.v – bộ phát UART 8N1
// -----------------------------------------------------------------------------
module uart_tx #(
    parameter integer CLKS_PER_BIT = 234
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] data,
    input  wire       send,                       // xung yêu cầu gửi (bỏ qua khi busy)
    output reg        tx,
    output wire       busy
);
    localparam integer CW = $clog2(CLKS_PER_BIT + 1);
    reg [CW-1:0] cnt;
    reg [3:0]    bit_idx;                         // 0 = start, 1..8 = data, 9 = stop
    reg [8:0]    shreg;
    reg          active;
    assign busy = active;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx <= 1'b1; cnt <= 0; bit_idx <= 0; shreg <= 9'h1FF; active <= 1'b0;
        end else if (!active) begin
            tx <= 1'b1;
            if (send) begin
                active <= 1'b1; shreg <= {1'b1, data}; tx <= 1'b0;   // bit start
                cnt <= 0; bit_idx <= 0;
            end
        end else if (cnt == CLKS_PER_BIT - 1) begin
            cnt <= 0;
            if (bit_idx == 4'd9) begin
                active <= 1'b0; tx <= 1'b1;
            end else begin
                tx <= shreg[0]; shreg <= {1'b1, shreg[8:1]};
                bit_idx <= bit_idx + 1'b1;
            end
        end else cnt <= cnt + 1'b1;
    end
endmodule
