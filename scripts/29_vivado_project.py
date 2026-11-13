#!/usr/bin/env python3
"""XEM KẾT QUẢ PHẦN CỨNG TRÊN VIVADO – cùng kiểu bài mẫu MNIST-CNN-FPGA (Zybo Z7-20).

Thiết kế nạp board của đồ án dùng Gowin (Tang Nano 20K). RTL khối HS + đặc trưng + OC-SVM là Verilog
thuần, không dùng primitive Gowin → Vivado tổng hợp được nguyên văn. Script này dựng một project Vivado:

  • Mô phỏng hành vi (Behavioral Simulation) có dạng sóng: tb_hs_engine trên ca IITB (ảnh thật)
    → so 48 giá trị z và f_q với Golden Model, in số chu kỳ khối HS.
  • Tổng hợp + đặt-đi dây (out-of-context, không cần chân board) top = hs_board_test
    (HS 64 vòng + FEAT + OC-SVM 8 làn + UART) trên chip Xilinx (mặc định xc7z020clg400-1 = Zybo Z7-20)
    → báo cáo Resource Utilization, Timing Summary (WNS → Fmax), Power Summary.

    python scripts/29_vivado_project.py --config configs/hw.yaml                     # sinh Tcl
    vivado -mode batch -source outputs_hw/vivado/run_reports.tcl                      # tổng hợp + báo cáo (~10–20 phút)
    vivado -mode gui   -source outputs_hw/vivado/sim_wave.tcl                         # mở GUI + dạng sóng
    python scripts/29_vivado_project.py --config configs/hw.yaml --parse              # tóm tắt báo cáo

Lưu ý khi viết đồ án: số tài nguyên/Fmax/công suất là của CHIP XILINX (để so cùng nền với bài mẫu),
KHÔNG phải của Tang Nano 20K – tài nguyên Gowin lấy từ fpga_gowin/hs_system/impl/pnr/hs_demo.rpt.txt.
"""
from __future__ import annotations

import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from ocsvm_fpga.cli import base_parser, setup  # noqa: E402


def p(x: Path) -> str:
    return x.resolve().as_posix()


def build(args, out: Path, log):
    hs_dir, build_dir = out / "fpga" / "hs", ROOT / args.build
    need = [hs_dir / "hs_params.vh", hs_dir / "mem" / "log_lut.mem", build_dir / "ocsvm_params.vh",
            build_dir / "sv_lanes.mem", hs_dir / "case_00" / "img1.mem"]
    miss = [str(f) for f in need if not f.exists()]
    if miss:
        sys.exit("Thiếu file – chạy 04_export_fpga.py rồi 15_export_rtl.py --lanes 8 trước:\n  " + "\n  ".join(miss))
    rtl = ROOT / "rtl" / "src"
    srcs = sorted(f for f in (rtl / "hs").glob("*.v") if f.name != "ocsvm_core.v") + [rtl / "ocsvm_core.v", rtl / "uart_tx.v"]
    mems = [hs_dir / "mem" / n for n in ("log_lut.mem", "scaler_mean_q.mem", "scaler_inv_q.mem")] + \
           [build_dir / n for n in ("sv_lanes.mem", "alpha.mem", "exp_lut.mem")]
    case = hs_dir / args.case
    vdir = out / "vivado"
    vdir.mkdir(parents=True, exist_ok=True)
    period = 1000.0 / args.clk_mhz
    (vdir / "clock.xdc").write_text(
        f"# Ràng buộc out-of-context: chỉ xung nhịp, không gán chân\n"
        f"create_clock -period {period:.3f} -name clk [get_ports clk]\n", encoding="utf-8")
    gen = {"CLK_HZ": int(args.clk_mhz * 1e6), "LUT_FILE": mems[0], "MEAN_FILE": mems[1], "INV_FILE": mems[2],
           "SV_FILE": mems[3], "ALPHA_FILE": mems[4], "EXP_FILE": mems[5]}
    gen_s = " ".join(f"{k}={v}" if isinstance(v, int) else f'{k}="{p(v)}"' for k, v in gen.items())
    proj = vdir / "hs_vivado"
    create = f"""# create_project.tcl – sinh bởi scripts/29_vivado_project.py
set proj_dir "{p(proj)}"
if {{[file exists $proj_dir/hs_vivado.xpr]}} {{
    open_project $proj_dir/hs_vivado.xpr
}} else {{
    create_project hs_vivado $proj_dir -part {args.part}
    add_files -norecurse {{{" ".join(p(s) for s in srcs)}}}
    add_files -norecurse {{{p(hs_dir / "hs_params.vh")} {p(build_dir / "ocsvm_params.vh")}}}
    set_property include_dirs [list {{{p(hs_dir)}}} {{{p(build_dir)}}}] [get_filesets sources_1]
    set_property top hs_board_test [get_filesets sources_1]
    set_property generic {{{gen_s}}} [get_filesets sources_1]
    add_files -fileset constrs_1 -norecurse {{{p(vdir / "clock.xdc")}}}
    # tổng hợp out-of-context: không cần gán chân board, chỉ đo tài nguyên / timing / công suất của lõi
    set_property -name {{STEPS.SYNTH_DESIGN.ARGS.MORE OPTIONS}} -value {{-mode out_of_context}} -objects [get_runs synth_1]
    # mô phỏng hành vi: tb_hs_engine trên ca IITB
    add_files -fileset sim_1 -norecurse {{{p(ROOT / "rtl" / "sim" / "hs" / "tb_hs_engine.v")}}}
    set_property file_type SystemVerilog [get_files {p(ROOT / "rtl" / "sim" / "hs" / "tb_hs_engine.v")}]
    add_files -fileset sim_1 -norecurse {{{" ".join(p(m) for m in mems)}}}
    set_property include_dirs [list {{{p(hs_dir)}}} {{{p(build_dir)}}}] [get_filesets sim_1]
    set_property top tb_hs_engine [get_filesets sim_1]
    set_property top_lib xil_defaultlib [get_filesets sim_1]
    set_property generic {{NIT=64 SV_FILE="{p(mems[3])}" ALPHA_FILE="{p(mems[4])}" EXP_FILE="{p(mems[5])}"}} [get_filesets sim_1]
    set_property -name {{xsim.simulate.xsim.more_options}} -value {{-testplusarg CASE={p(case)} -testplusarg STALL=0 -testplusarg RATE=2}} -objects [get_filesets sim_1]
    set_property -name {{xsim.simulate.runtime}} -value {{0ns}} -objects [get_filesets sim_1]
    set_property -name {{xsim.simulate.log_all_signals}} -value {{false}} -objects [get_filesets sim_1]
}}
"""
    (vdir / "create_project.tcl").write_text(create, encoding="utf-8")
    rep = vdir / "reports"
    run = f"""# run_reports.tcl – tổng hợp + đặt-đi dây + 3 báo cáo như bài mẫu (Utilization / Timing / Power)
source "{p(vdir / "create_project.tcl")}"
file mkdir "{p(rep)}"
reset_run synth_1
launch_runs synth_1 -jobs 4
wait_on_run synth_1
if {{[get_property PROGRESS [get_runs synth_1]] != "100%"}} {{ error "Tổng hợp lỗi – xem {p(proj)}/hs_vivado.runs/synth_1/runme.log" }}
launch_runs impl_1 -jobs 4
wait_on_run impl_1
if {{[get_property PROGRESS [get_runs impl_1]] != "100%"}} {{ error "Đặt-đi dây lỗi – xem {p(proj)}/hs_vivado.runs/impl_1/runme.log" }}
open_run impl_1
report_utilization                 -file "{p(rep / "utilization.rpt")}"
report_utilization -hierarchical   -file "{p(rep / "utilization_hier.rpt")}"
report_timing_summary -max_paths 10 -file "{p(rep / "timing_summary.rpt")}"
report_power                       -file "{p(rep / "power.rpt")}"
report_clock_utilization           -file "{p(rep / "clock_utilization.rpt")}"
puts "XONG – báo cáo trong {p(rep)}. Tóm tắt: python scripts/29_vivado_project.py --config {args.config} --parse"
"""
    (vdir / "run_reports.tcl").write_text(run, encoding="utf-8")
    waves = ["/tb_hs_engine/clk", "/tb_hs_engine/rst_n", "/tb_hs_engine/start",
             "/tb_hs_engine/dut/phase", "/tb_hs_engine/dut/busy", "/tb_hs_engine/dut/done", "/tb_hs_engine/dut/cycles",
             "/tb_hs_engine/pv", "/tb_hs_engine/pix_ready", "/tb_hs_engine/dw_valid", "/tb_hs_engine/dr_req",
             "/tb_hs_engine/rv", "/tb_hs_engine/dr_ready",
             "/tb_hs_engine/x_we", "/tb_hs_engine/x_addr", "/tb_hs_engine/x_data", "/tb_hs_engine/svm_start",
             "/tb_hs_engine/u_svm/busy", "/tb_hs_engine/u_svm/done", "/tb_hs_engine/f_out", "/tb_hs_engine/anomaly",
             "/tb_hs_engine/dut/n_sat"]
    radix = {"/tb_hs_engine/dut/cycles": "unsigned", "/tb_hs_engine/x_addr": "unsigned",
             "/tb_hs_engine/x_data": "signed", "/tb_hs_engine/f_out": "signed", "/tb_hs_engine/dut/n_sat": "unsigned"}
    adds = "\n".join(f'catch {{add_wave {{{{{w}}}}} {"-radix " + radix[w] if w in radix else ""}}}' for w in waves)
    sim = f"""# sim_wave.tcl – mô phỏng hành vi có dạng sóng (Vivado GUI)
source "{p(vdir / "create_project.tcl")}"
launch_simulation -simset sim_1 -mode behavioral
# chỉ ghi các tín hiệu cần xem (ghi toàn bộ 2,5 triệu chu kỳ rất nặng)
{adds}
run all
puts "Xem Tcl Console: dòng 'PASS tb_hs_engine ... HS <N> chu ky = <ms> ms o 100 MHz'. Phóng to quanh x_we để thấy 48 giá trị z, rồi svm_start → f_out."
"""
    (vdir / "sim_wave.tcl").write_text(sim, encoding="utf-8")
    log.info(f"Đã sinh Tcl trong {vdir}:")
    log.info(f"  vivado -mode batch -source {p(vdir / 'run_reports.tcl')}      (Resource / Timing / Power, chip {args.part}, {args.clk_mhz:g} MHz)")
    log.info(f"  vivado -mode gui   -source {p(vdir / 'sim_wave.tcl')}         (mô phỏng hành vi + dạng sóng, ca {args.case})")


def num(s: str) -> float:
    return float(s.replace(",", ""))


def parse(args, out: Path, log):
    rep = out / "vivado" / "reports"
    res = {"part": args.part, "clk_mhz": args.clk_mhz}
    u = rep / "utilization.rpt"
    if u.exists():
        t = u.read_text(encoding="utf-8", errors="ignore")
        for key, pat in (("lut", r"\|\s*Slice LUTs\*?\s*\|\s*([\d,]+)\s*\|.*?\|\s*([\d,]+)\s*\|\s*([\d.<]+)\s*\|\s*$"),
                         ("ff", r"\|\s*Slice Registers\s*\|\s*([\d,]+)\s*\|.*?\|\s*([\d,]+)\s*\|\s*([\d.<]+)\s*\|\s*$"),
                         ("bram", r"\|\s*Block RAM Tile\s*\|\s*([\d.,]+)\s*\|.*?\|\s*([\d,]+)\s*\|\s*([\d.<]+)\s*\|\s*$"),
                         ("dsp", r"\|\s*DSPs\s*\|\s*([\d,]+)\s*\|.*?\|\s*([\d,]+)\s*\|\s*([\d.<]+)\s*\|\s*$")):
            m = re.search(pat, t, re.M)
            if m:
                res[key] = {"used": num(m.group(1)), "avail": num(m.group(2))}
    tm = rep / "timing_summary.rpt"
    if tm.exists():
        t = tm.read_text(encoding="utf-8", errors="ignore")
        m = re.search(r"WNS\(ns\)\s+TNS\(ns\).*?\n\s*-+.*?\n\s*(-?[\d.]+)\s+(-?[\d.]+)\s+\d+\s+\d+\s+(-?[\d.]+)", t, re.S)
        if m:
            wns, whs = float(m.group(1)), float(m.group(3))
            period = 1000.0 / args.clk_mhz
            res["timing"] = {"wns_ns": wns, "whs_ns": whs, "met": wns >= 0 and whs >= 0,
                             "fmax_mhz": round(1000.0 / (period - wns), 1)}
    pw = rep / "power.rpt"
    if pw.exists():
        t = pw.read_text(encoding="utf-8", errors="ignore")
        for key, lab in (("total_w", "Total On-Chip Power (W)"), ("dynamic_w", "Dynamic (W)"),
                         ("static_w", "Device Static (W)")):
            m = re.search(r"\|\s*" + re.escape(lab) + r"\s*\|\s*([\d.]+)", t)
            if m:
                res[key] = float(m.group(1))
    if len(res) <= 2:
        sys.exit(f"Chưa có báo cáo trong {rep} – chạy run_reports.tcl trước")
    hs_cycles = 2_496_502                      # mô phỏng engine (RATE=2, không chèn chờ)
    L = [f"# Kết quả Vivado – lõi HS + FEAT + OC-SVM (top hs_board_test, out-of-context)", "",
         f"Chip {args.part} · ràng buộc {args.clk_mhz:g} MHz · nguồn: outputs_hw/vivado/reports. "
         "Số liệu của chip Xilinx (để so cùng nền với bài mẫu Zybo Z7-20), không phải của Tang Nano 20K.", "",
         "| Tài nguyên | Dùng | Có | % |", "|---|---|---|---|"]
    for k, lab in (("lut", "Slice LUT"), ("ff", "Slice Register"), ("bram", "Block RAM Tile (36 Kb)"), ("dsp", "DSP48E1")):
        if k in res:
            L.append(f"| {lab} | {res[k]['used']:g} | {res[k]['avail']:g} | {100 * res[k]['used'] / res[k]['avail']:.1f} % |")
    if "timing" in res:
        tt = res["timing"]
        ms = hs_cycles / (tt["fmax_mhz"] * 1e3)
        L += ["", f"Timing: WNS = {tt['wns_ns']:.3f} ns, WHS = {tt['whs_ns']:.3f} ns → "
                  f"{'ĐẠT' if tt['met'] else 'KHÔNG đạt'} {args.clk_mhz:g} MHz; Fmax ≈ **{tt['fmax_mhz']} MHz** "
                  f"→ khối HS {hs_cycles:,} chu kỳ ≈ **{ms:.1f} ms / cặp** ở Fmax.".replace(f"{hs_cycles:,}", f"{hs_cycles:,}".replace(",", " "))]
    if "total_w" in res:
        e = res["total_w"] * hs_cycles / (args.clk_mhz * 1e6) * 1e3
        L += ["", f"Công suất (ước lượng vectorless): tổng {res['total_w']} W (động {res.get('dynamic_w', '?')} W, "
                  f"tĩnh {res.get('static_w', '?')} W) → năng lượng ≈ {e:.2f} mJ / cặp ở {args.clk_mhz:g} MHz."]
    (out / "vivado" / "vivado_summary.md").write_text("\n".join(L) + "\n", encoding="utf-8")
    (out / "vivado" / "vivado_summary.json").write_text(json.dumps(res, indent=2), encoding="utf-8")
    print("\n".join(L))


def main():
    ap = base_parser(__doc__)
    ap.add_argument("--part", default="xc7z020clg400-1", help="chip Xilinx (mặc định Zybo Z7-20 như bài mẫu)")
    ap.add_argument("--clk-mhz", type=float, default=100.0, help="xung nhịp ràng buộc")
    ap.add_argument("--case", default="case_00", help="ca IITB cho mô phỏng (thư mục trong outputs_hw/fpga/hs)")
    ap.add_argument("--build", default="rtl/build", help="thư mục ocsvm_params.vh + sv_lanes.mem")
    ap.add_argument("--parse", action="store_true", help="đọc báo cáo Vivado → vivado_summary.md")
    args = ap.parse_args()
    cfg, out, log, _ = setup(args, "29_vivado")
    (parse if args.parse else build)(args, out, log)


if __name__ == "__main__":
    main()
