// hs_demo_top.v – sinh bởi scripts/26_gowin_system.py (top của project Gowin)
module hs_demo_top (
    input wire clk, input wire spi_sck, input wire spi_mosi, input wire spi_cs_n,
    output wire uart_tx, output wire [5:0] led,
    output wire O_sdram_clk, output wire O_sdram_cke, output wire O_sdram_cs_n, output wire O_sdram_cas_n,
    output wire O_sdram_ras_n, output wire O_sdram_wen_n, inout wire [31:0] IO_sdram_dq,
    output wire [10:0] O_sdram_addr, output wire [1:0] O_sdram_ba, output wire [3:0] O_sdram_dqm
);
    hs_system_top #(
        .USE_PLL(1), .CLK_HZ(54000000), .NIT(0), .DUMP_Z(1),
        .LUT_FILE("D:/datn/ocsvm_anomaly_fpga/fpga_gowin/hs_system_z/src/log_lut.mem"), .MEAN_FILE("D:/datn/ocsvm_anomaly_fpga/fpga_gowin/hs_system_z/src/scaler_mean_q.mem"),
        .INV_FILE("D:/datn/ocsvm_anomaly_fpga/fpga_gowin/hs_system_z/src/scaler_inv_q.mem"), .SV_FILE("D:/datn/ocsvm_anomaly_fpga/fpga_gowin/hs_system_z/src/sv_lanes.mem"),
        .ALPHA_FILE("D:/datn/ocsvm_anomaly_fpga/fpga_gowin/hs_system_z/src/alpha.mem"), .EXP_FILE("D:/datn/ocsvm_anomaly_fpga/fpga_gowin/hs_system_z/src/exp_lut.mem")
    ) u (
        .clk(clk), .spi_sck(spi_sck), .spi_mosi(spi_mosi), .spi_cs_n(spi_cs_n), .uart_tx(uart_tx), .led(led),
        .O_sdram_clk(O_sdram_clk), .O_sdram_cke(O_sdram_cke), .O_sdram_cs_n(O_sdram_cs_n),
        .O_sdram_cas_n(O_sdram_cas_n), .O_sdram_ras_n(O_sdram_ras_n), .O_sdram_wen_n(O_sdram_wen_n),
        .IO_sdram_dq(IO_sdram_dq), .O_sdram_addr(O_sdram_addr), .O_sdram_ba(O_sdram_ba), .O_sdram_dqm(O_sdram_dqm)
    );
endmodule
