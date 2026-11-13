#!/usr/bin/env python3
"""DỰNG PROJECT GOWIN CHO HỆ THỐNG DEMO 1 (top rtl/src/sys/hs_system_top.v) trên Tang Nano 20K.

    python scripts/15_export_rtl.py  --config configs/hw.yaml --lanes 8      # (một lần, sau mỗi lần train)
    python scripts/26_gowin_system.py --config configs/hw.yaml                # → fpga_gowin/hs_system
    python scripts/26_gowin_system.py --config configs/hw.yaml --clk 27       # dự phòng: không dùng PLL
    python scripts/26_gowin_system.py --config configs/hw.yaml --nit 32       # dự phòng: 32 vòng (cần mô hình N=32)

Nối dây ESP32-S3 → Tang Nano 20K (cùng GND):
    GPIO14 (SCLK) → chân 52 · GPIO21 (MOSI) → chân 53 · GPIO47 (CS) → chân 71
"""
from __future__ import annotations

import re
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from ocsvm_fpga.cli import base_parser, setup  # noqa: E402

CST = """// Tang Nano 20K (GW2AR-LV18QN88C8/I7) – hs_system_top (demo 1)
// SDRAM tích hợp KHÔNG cần khai báo chân (tên cổng O_sdram_*/IO_sdram_* do Gowin nối sẵn)
IO_LOC  "clk" 4;
IO_PORT "clk" IO_TYPE=LVCMOS33 PULL_MODE=UP;
IO_LOC  "uart_tx" 69;                  // → BL616 → cổng COM trên PC
IO_PORT "uart_tx" IO_TYPE=LVCMOS33 PULL_MODE=UP;
IO_LOC  "spi_sck" 52;                  // ← ESP32 GPIO14
IO_PORT "spi_sck" IO_TYPE=LVCMOS33 PULL_MODE=DOWN;
IO_LOC  "spi_mosi" 53;                 // ← ESP32 GPIO21
IO_PORT "spi_mosi" IO_TYPE=LVCMOS33 PULL_MODE=DOWN;
IO_LOC  "spi_cs_n" 71;                 // ← ESP32 GPIO47
IO_PORT "spi_cs_n" IO_TYPE=LVCMOS33 PULL_MODE=UP;
IO_LOC  "led[0]" 15;
IO_PORT "led[0]" IO_TYPE=LVCMOS33 PULL_MODE=UP DRIVE=8;
IO_LOC  "led[1]" 16;
IO_PORT "led[1]" IO_TYPE=LVCMOS33 PULL_MODE=UP DRIVE=8;
IO_LOC  "led[2]" 17;
IO_PORT "led[2]" IO_TYPE=LVCMOS33 PULL_MODE=UP DRIVE=8;
IO_LOC  "led[3]" 18;
IO_PORT "led[3]" IO_TYPE=LVCMOS33 PULL_MODE=UP DRIVE=8;
IO_LOC  "led[4]" 19;
IO_PORT "led[4]" IO_TYPE=LVCMOS33 PULL_MODE=UP DRIVE=8;
IO_LOC  "led[5]" 20;
IO_PORT "led[5]" IO_TYPE=LVCMOS33 PULL_MODE=UP DRIVE=8;
"""
SDC = """// Thạch anh 27 MHz; Gowin tự suy ra xung nhịp 54 MHz ở đầu ra rPLL.
create_clock -name clk27 -period 37.037 -waveform {0 18.518} [get_ports {clk}]
// SPI là tín hiệu không đồng bộ (đã đồng bộ 2 tầng trong spi_frame_rx)
set_false_path -from [get_ports {spi_sck spi_mosi spi_cs_n}]
"""


def main():
    ap = base_parser(__doc__)
    ap.add_argument("--out", default="fpga_gowin/hs_system")
    ap.add_argument("--cases", default=None, help="thư mục hs (mặc định <output_dir>/fpga/hs)")
    ap.add_argument("--build", default="rtl/build")
    ap.add_argument("--clk", type=int, choices=[54, 27], default=54, help="54: qua rPLL · 27: thạch anh trực tiếp")
    ap.add_argument("--nit", type=int, default=0, help="số vòng HS (0 = N_ITER trong hs_params.vh)")
    ap.add_argument("--dump-z", action="store_true",
                    help="bản kiểm chứng: in thêm 48 z mỗi kết quả (dùng với --out fpga_gowin/hs_system_z, "
                         "KHÔNG ghi đè project demo)")
    args = ap.parse_args()
    cfg, out, log, _ = setup(args, "26_gowin_system")
    hs_dir = (Path(args.cases) if args.cases else out / "fpga" / "hs").resolve()
    build = (ROOT / args.build).resolve()
    for need in (hs_dir / "hs_params.vh", build / "ocsvm_params.vh", build / "sv_lanes.mem"):
        if not need.exists():
            sys.exit(f"Thiếu {need} – chạy 04_export_fpga.py rồi 15_export_rtl.py --lanes 8 với --config configs/hw.yaml")
    n_iter = int(re.search(r"N_ITER\s*=\s*(\d+)", (hs_dir / "hs_params.vh").read_text()).group(1))
    dst = Path(args.out).resolve()
    if args.dump_z and dst.name == "hs_system":
        sys.exit("--dump-z phải đi với --out khác (vd fpga_gowin/hs_system_z) để giữ nguyên project demo")
    src = dst / "src"
    if src.exists():
        shutil.rmtree(src)
    src.mkdir(parents=True)
    rtl = ROOT / "rtl" / "src"
    vfiles = sorted((rtl / "hs").glob("*.v")) + sorted((rtl / "sys").glob("*.v")) + \
        [rtl / "sys" / "third_party" / "sdram.v", rtl / "ocsvm_core.v", rtl / "uart_tx.v"]
    vfiles = [f for f in vfiles if f.name not in ("hs_board_test.v", "hs_pixgen.v", "hs_engine.v")]
    for f in vfiles:
        shutil.copy(f, src / f.name)
    shutil.copy(hs_dir / "hs_params.vh", src)
    shutil.copy(build / "ocsvm_params.vh", src)
    mem = {}
    for f in ("log_lut.mem", "scaler_mean_q.mem", "scaler_inv_q.mem"):
        shutil.copy(hs_dir / "mem" / f, src / f)
        mem[f] = (src / f).as_posix()
    for f in ("sv_lanes.mem", "alpha.mem", "exp_lut.mem"):
        shutil.copy(build / f, src / f)
        mem[f] = (src / f).as_posix()
    use_pll, clk_hz = (1, 54_000_000) if args.clk == 54 else (0, 27_000_000)
    top = f"""// hs_demo_top.v – sinh bởi scripts/26_gowin_system.py (top của project Gowin)
module hs_demo_top (
    input wire clk, input wire spi_sck, input wire spi_mosi, input wire spi_cs_n,
    output wire uart_tx, output wire [5:0] led,
    output wire O_sdram_clk, output wire O_sdram_cke, output wire O_sdram_cs_n, output wire O_sdram_cas_n,
    output wire O_sdram_ras_n, output wire O_sdram_wen_n, inout wire [31:0] IO_sdram_dq,
    output wire [10:0] O_sdram_addr, output wire [1:0] O_sdram_ba, output wire [3:0] O_sdram_dqm
);
    hs_system_top #(
        .USE_PLL({use_pll}), .CLK_HZ({clk_hz}), .NIT({args.nit}), .DUMP_Z({1 if args.dump_z else 0}),
        .LUT_FILE("{mem['log_lut.mem']}"), .MEAN_FILE("{mem['scaler_mean_q.mem']}"),
        .INV_FILE("{mem['scaler_inv_q.mem']}"), .SV_FILE("{mem['sv_lanes.mem']}"),
        .ALPHA_FILE("{mem['alpha.mem']}"), .EXP_FILE("{mem['exp_lut.mem']}")
    ) u (
        .clk(clk), .spi_sck(spi_sck), .spi_mosi(spi_mosi), .spi_cs_n(spi_cs_n), .uart_tx(uart_tx), .led(led),
        .O_sdram_clk(O_sdram_clk), .O_sdram_cke(O_sdram_cke), .O_sdram_cs_n(O_sdram_cs_n),
        .O_sdram_cas_n(O_sdram_cas_n), .O_sdram_ras_n(O_sdram_ras_n), .O_sdram_wen_n(O_sdram_wen_n),
        .IO_sdram_dq(IO_sdram_dq), .O_sdram_addr(O_sdram_addr), .O_sdram_ba(O_sdram_ba), .O_sdram_dqm(O_sdram_dqm)
    );
endmodule
"""
    (src / "hs_demo_top.v").write_text(top, encoding="utf-8")
    (src / "hs_demo.cst").write_text(CST, encoding="utf-8")
    (src / "hs_demo.sdc").write_text(SDC, encoding="utf-8")
    names = ["hs_demo_top.v"] + [f.name for f in vfiles]
    lines = "\n".join(f'        <File path="src/{n}" type="file.verilog" enable="1"/>' for n in names)
    (dst / "hs_demo.gprj").write_text(f"""<?xml version="1" encoding="UTF-8"?>
<!DOCTYPE gowin-fpga-project>
<Project>
    <Template>FPGA</Template>
    <Version>5</Version>
    <Device name="GW2AR-18C" pn="GW2AR-LV18QN88C8/I7">gw2ar18c-000</Device>
    <FileList>
{lines}
        <File path="src/hs_demo.cst" type="file.cst" enable="1"/>
        <File path="src/hs_demo.sdc" type="file.sdc" enable="1"/>
    </FileList>
</Project>
""", encoding="utf-8")
    nit = args.nit or n_iter
    est = nit * 9600 * 5 / clk_hz * 1e3
    log.info(f"Project Gowin → {dst}\n  top = hs_demo_top · include path = {src}\n"
             f"  {args.clk} MHz, {nit} vòng → HS ước lượng ≈ {est:.0f} ms / cặp ảnh (ngân sách 80 ms)")
    if est > 75:
        log.warning("Vượt ngân sách 80 ms: dùng --clk 54 hoặc --nit 32 (với mô hình train 32 vòng)")


if __name__ == "__main__":
    main()
