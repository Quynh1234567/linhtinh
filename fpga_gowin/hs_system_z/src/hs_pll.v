// -----------------------------------------------------------------------------
// hs_pll.v – xung nhịp hệ thống từ thạch anh 27 MHz của Tang Nano 20K
//   clkout  = 27 · (FBDIV_SEL+1) / (IDIV_SEL+1) = 54 MHz   (VCO = 54 · ODIV_SEL = 864 MHz)
//   clkoutp = clkout lệch 180° (PSDA_SEL = "1000") – cấp cho SDRAM như ví dụ của Sipeed
// USE_PLL = 0: bỏ PLL, chạy thẳng 27 MHz (phương án dự phòng khi SDRAM không ổn ở 54 MHz).
// -----------------------------------------------------------------------------
`timescale 1ns / 1ps
module hs_pll #(
    parameter integer USE_PLL = 1
)(
    input  wire clkin,
    output wire clkout,
    output wire clkoutp,
    output wire lock
);
    generate if (USE_PLL != 0) begin : g_pll
        wire gnd = 1'b0;
        rPLL #(
            .FCLKIN("27"), .DYN_IDIV_SEL("false"), .IDIV_SEL(0), .DYN_FBDIV_SEL("false"), .FBDIV_SEL(1),
            .DYN_ODIV_SEL("false"), .ODIV_SEL(16), .PSDA_SEL("1000"), .DYN_DA_EN("false"), .DUTYDA_SEL("1000"),
            .CLKOUT_FT_DIR(1'b1), .CLKOUTP_FT_DIR(1'b1), .CLKOUT_DLY_STEP(0), .CLKOUTP_DLY_STEP(0),
            .CLKFB_SEL("internal"), .CLKOUT_BYPASS("false"), .CLKOUTP_BYPASS("false"), .CLKOUTD_BYPASS("false"),
            .DYN_SDIV_SEL(2), .CLKOUTD_SRC("CLKOUT"), .CLKOUTD3_SRC("CLKOUT"), .DEVICE("GW2AR-18C")
        ) rpll_inst (
            .CLKOUT(clkout), .LOCK(lock), .CLKOUTP(clkoutp), .CLKOUTD(), .CLKOUTD3(),
            .RESET(gnd), .RESET_P(gnd), .CLKIN(clkin), .CLKFB(gnd),
            .FBDSEL({6{gnd}}), .IDSEL({6{gnd}}), .ODSEL({6{gnd}}), .PSDA({4{gnd}}), .DUTYDA({4{gnd}}), .FDLY({4{gnd}})
        );
    end else begin : g_bypass
        assign clkout  = clkin;
        assign clkoutp = ~clkin;
        assign lock    = 1'b1;
    end endgenerate
endmodule
