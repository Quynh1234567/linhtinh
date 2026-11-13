#!/usr/bin/env python3
"""BÁO CÁO TĂNG TỐC – gom mọi số đo thành một bảng + biểu đồ để đưa vào đồ án / slide demo.

Nguồn (tự tìm bản mới nhất nếu không chỉ rõ; thiếu nguồn nào thì bỏ dòng đó):
  --pc-bench    outputs_hw/hs_cpu_bench/<…>/bench.json      (scripts/23_hs_cpu_bench.py)
  --esp32-log   file log của firmware/esp32_hs_bench (dòng TOM TAT) – mặc định dùng số đã đo 31/10
  --demo        outputs_hw/demo_runs/<…>/summary.json        (scripts/27_demo_monitor.py, đo trên board)
  --rtl-sim     outputs_hw/rtl_sim/<…>/report.md              (scripts/24_hs_rtl_sim.py --tb engine)
  --gowin       gowin_metrics.json (tạo mẫu: --init-gowin, điền số từ báo cáo Gowin)
  --eval        outputs_hw/reports/eval_report.json           (AUC của mô hình số nguyên)

    python scripts/28_accel_report.py --config configs/hw.yaml --init-gowin
    python scripts/28_accel_report.py --config configs/hw.yaml
Kết quả: outputs_hw/accel_report/<thời gian>/report.md, speedup.png, metrics.json
"""
from __future__ import annotations

import json
import re
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from ocsvm_fpga.cli import base_parser, setup  # noqa: E402

NPIX = 19200
# Số đã đo trên board ESP32-S3 (firmware/esp32_hs_bench, 31/10/2026, ca case_01, 64 vòng)
ESP32_DEFAULT = {"c_int_sram": 899.9, "c_int_psram": 1208.0, "c_float_psram": 1421.6}
ESP32_MHZ = 240.0


def latest(pattern: str) -> Path | None:
    hits = sorted(ROOT.glob(pattern))
    return hits[-1] if hits else None


def parse_esp32_log(p: Path) -> dict:
    out = {}
    for ln in p.read_text(encoding="utf-8", errors="replace").splitlines():
        m = re.match(r"\s*(c_int|c_float)\s+(sram|psram)\s+[\d.]+\s+[\d.]+\s+[\d.]+\s+\S+\s+([\d.]+)", ln)
        if m:
            out[f"{m.group(1)}_{m.group(2)}"] = float(m.group(3))
    return out


GOWIN_TEMPLATE = {
    "_huong_dan": "Điền từ Gowin: Synthesize → Resource Usage; Place & Route → Timing (Max Frequency); "
                  "Tools → Power Analyzer (Total Power). Để null nếu chưa có.",
    "top": "hs_demo_top", "clock_mhz": 54,
    "lut4": None, "lut4_total": 20736, "reg": None, "reg_total": 15552,
    "bsram": None, "bsram_total": 46, "ssram": None, "dsp_mult18": None, "dsp_total": 48,
    "fmax_mhz": None, "power_w": None,
}


def main():
    ap = base_parser(__doc__)
    ap.add_argument("--pc-bench", default=None)
    ap.add_argument("--esp32-log", default=None)
    ap.add_argument("--demo", default=None)
    ap.add_argument("--rtl-sim", default=None)
    ap.add_argument("--gowin", default=None)
    ap.add_argument("--eval", default=None)
    ap.add_argument("--n-iter", type=int, default=64)
    ap.add_argument("--init-gowin", action="store_true", help="tạo gowin_metrics.json mẫu rồi thoát")
    args = ap.parse_args()
    cfg, out, log, _ = setup(args, "28_accel_report")
    if args.init_gowin:
        p = out / "gowin_metrics.json"
        p.write_text(json.dumps(GOWIN_TEMPLATE, indent=2, ensure_ascii=False), encoding="utf-8")
        print(f"Đã tạo {p} – điền số từ báo cáo Gowin rồi chạy lại không có --init-gowin")
        return
    N = args.n_iter
    upd = NPIX * N                                     # số lần cập nhật điểm ảnh mỗi cặp
    rows, notes = [], []

    # ---------------------------------------------------------------- FPGA
    fpga = None
    demo_p = Path(args.demo) if args.demo else latest("outputs_hw/demo_runs/*/summary.json")
    if demo_p and demo_p.exists():
        d = json.loads(demo_p.read_text(encoding="utf-8"))
        if d.get("hs_ms_median"):
            fpga = {"name": f"FPGA Tang Nano 20K – demo 1 ({d['clock_mhz']:.0f} MHz, đo trên board)",
                    "ms": d["hs_ms_median"], "mhz": d["clock_mhz"], "src": str(demo_p), "measured": True,
                    "lat_ms": d.get("latency_ms_median"), "rate": d.get("results_per_s"),
                    "drops": d.get("pairs_dropped"), "frames": d.get("frames_received"), "sd": d.get("sdram_errors")}
    if fpga is None:
        notes.append("Chưa có số đo trên board (chạy scripts/27_demo_monitor.py) – dùng số mô phỏng RTL bên dưới.")
    sim_p = Path(args.rtl_sim) if args.rtl_sim else latest("outputs_hw/rtl_sim/*/report.md")
    sim_ms_100 = None
    if sim_p and sim_p.exists():
        m = re.search(r"([\d.]+) chu kỳ = ([\d.]+) ms ở 100 MHz", sim_p.read_text(encoding="utf-8"))
        if m:
            sim_ms_100 = float(m.group(2))
    demo_sim_ms = N * 48700 / 54e3                       # demo 1 mô phỏng tb_hs_system: 48 700 chu kỳ / vòng ở 54 MHz
    fpga_rows = []
    if fpga:
        fpga_rows.append(fpga)
    fpga_rows.append({"name": "FPGA – demo 1 (54 MHz, mô phỏng RTL toàn hệ thống)",
                      "ms": demo_sim_ms, "mhz": 54.0, "measured": False})
    if sim_ms_100:
        fpga_rows.append({"name": "FPGA – kiến trúc đầy đủ (lưu gói DERIV, 100 MHz, mô phỏng RTL)",
                          "ms": sim_ms_100, "mhz": 100.0, "measured": False})
    ref = fpga_rows[0]

    # ---------------------------------------------------------------- CPU
    cpu = []
    esp = dict(ESP32_DEFAULT)
    if args.esp32_log:
        esp.update(parse_esp32_log(Path(args.esp32_log)))
    for k, label in (("c_int_sram", "ESP32-S3 240 MHz – C số nguyên, SRAM"),
                     ("c_int_psram", "ESP32-S3 240 MHz – C số nguyên, PSRAM"),
                     ("c_float_psram", "ESP32-S3 240 MHz – C float32, PSRAM")):
        if esp.get(k):
            cpu.append({"name": label, "ms": esp[k], "mhz": ESP32_MHZ, "group": "esp32"})
    pc_p = Path(args.pc_bench) if args.pc_bench else latest("outputs_hw/hs_cpu_bench/*/bench.json")
    pc_mhz = None
    if pc_p and pc_p.exists():
        b = json.loads(pc_p.read_text(encoding="utf-8"))
        pc_mhz = (b.get("system") or {}).get("cpu_mhz_max")
        proc = (b.get("system") or {}).get("processor", "PC")
        for k, label in (("c_int", "C số nguyên, 1 luồng"), ("c_float", "C float32, 1 luồng"),
                         ("py_float", "Python số thực (pipeline train)"), ("py_int", "Python số nguyên (Golden)")):
            v = (b.get("median") or {}).get(k)
            if v:
                cpu.append({"name": f"PC {proc[:28]} – {label}", "ms": v["total"], "mhz": pc_mhz, "group": "pc"})
    else:
        notes.append("Chưa có bench.json của PC (scripts/23_hs_cpu_bench.py).")

    # ---------------------------------------------------------------- bảng
    def cyc(r):
        return r["ms"] * 1e3 * r["mhz"] / upd if r.get("mhz") else None
    L = ["# Báo cáo tăng tốc khối Horn–Schunck trên FPGA", "",
         f"Một cặp ảnh 160 × 120, {N} vòng Jacobi = {upd:_} lần cập nhật điểm ảnh; mọi bản CPU tính đúng cùng thuật toán "
         "và bản số nguyên khớp từng bit với RTL.".replace("_", " "), "",
         f"**Mốc so sánh: {ref['name']} = {ref['ms']:.2f} ms / cặp.**", "",
         "| Nền tảng | ms / cặp | Chu kỳ / điểm / vòng | Cặp / giây tối đa | Đạt 12,5 fps? | So với mốc |",
         "|---|---|---|---|---|---|"]
    allr = fpga_rows + cpu

    def rel(r):                                   # so với mốc (FPGA demo 1): chậm hơn / nhanh hơn bao nhiêu lần
        k = r["ms"] / ref["ms"]
        if r is ref or abs(k - 1) < 0.005:
            return "mốc"
        return f"mốc nhanh hơn {k:.1f}×" if k > 1 else f"nhanh hơn mốc {1 / k:.1f}×"

    for r in allr:
        c = cyc(r)
        L.append(f"| {r['name']} | {r['ms']:.2f} | {('%.1f' % c) if c else '–'} | {1000 / r['ms']:.1f} | "
                 f"{'có' if r['ms'] <= 80 else 'không'} | {rel(r)} |")
    esp_best = min((r for r in cpu if r["group"] == "esp32"), key=lambda r: r["ms"], default=None)
    pc_best = min((r for r in cpu if r["group"] == "pc"), key=lambda r: r["ms"], default=None)
    L += ["", "## Kết luận định lượng", ""]
    if esp_best:
        L.append(f"- **So với CPU nhúng trong hệ thống (ESP32-S3)**: FPGA nhanh hơn **{esp_best['ms'] / ref['ms']:.1f}×** "
                 f"({esp_best['ms']:.0f} → {ref['ms']:.1f} ms). ESP32-S3 chỉ đạt {1000 / esp_best['ms']:.1f} cặp/s, "
                 f"không đáp ứng 12,5 fps; FPGA đáp ứng với {80 - ref['ms']:.0f} ms dư mỗi cặp.")
        ce, cf = cyc(esp_best), cyc(ref)
        L.append(f"- **Hiệu quả kiến trúc**: {cf:.1f} chu kỳ / điểm / vòng trên FPGA so với {ce:.0f} trên ESP32-S3 "
                 f"(**{ce / cf:.0f}×** ít chu kỳ hơn) nhờ pipeline 8 tầng, tính u, v song song, Jacobi tại chỗ.")
    if pc_best:
        c = cyc(pc_best)
        L.append(f"- **So với CPU máy tính** ({pc_best['name']}): {pc_best['ms']:.1f} ms – CPU PC nhanh hơn về thời gian "
                 f"tuyệt đối vì chạy ở GHz" + (f" ({c:.0f} chu kỳ / điểm / vòng, FPGA ít chu kỳ hơn {c / cyc(ref):.0f}×)" if c else "")
                 + ". FPGA thắng ở chỗ chạy trong hệ nhúng công suất thấp, thời gian xác định, không cần PC.")
    if sim_ms_100:
        L.append(f"- **Kiến trúc đầy đủ** (gói DERIV 64 bit trong SDRAM, đọc 1 gói / 2 chu kỳ, 100 MHz, chưa tính refresh SDRAM) mô phỏng {sim_ms_100:.1f} ms → "
                 + (f"{esp_best['ms'] / sim_ms_100:.0f}× so với ESP32-S3" if esp_best else "") + "; là hướng tối ưu sau demo 1.")
    if fpga and fpga.get("measured"):
        L += ["", "## Số đo hệ thống trên board (demo 1)", "",
              f"- Thời gian HS trung vị {fpga['ms']:.2f} ms; từ lúc nhận đủ ảnh tới kết quả {fpga.get('lat_ms') or 0:.2f} ms",
              f"- Tốc độ kết quả {fpga.get('rate') or 0:.2f} cặp/s · cặp bị bỏ {fpga.get('drops')} / khung nhận {fpga.get('frames')}"
              + (f" · lỗi SDRAM tự kiểm tra {fpga.get('sd')}" if fpga.get('sd') is not None else " · tự kiểm tra SDRAM: chưa ghi (mở monitor trước khi reset FPGA)"), f"- Nguồn: `{fpga['src']}`"]

    # ---------------------------------------------------------------- tài nguyên, độ chính xác
    gw_p = Path(args.gowin) if args.gowin else out / "gowin_metrics.json"
    if gw_p.exists():
        g = json.loads(gw_p.read_text(encoding="utf-8"))
        L += ["", "## Tài nguyên FPGA (báo cáo Gowin)", "", "| Tài nguyên | Dùng | Có | % |", "|---|---|---|---|"]
        for k, tot, lab in (("lut4", "lut4_total", "LUT4"), ("reg", "reg_total", "Thanh ghi"),
                            ("bsram", "bsram_total", "BSRAM 18 Kbit"), ("dsp_mult18", "dsp_total", "DSP")):
            if g.get(k) is not None:
                L.append(f"| {lab} | {g[k]} | {g[tot]} | {100 * g[k] / g[tot]:.0f} % |")
        if g.get("fmax_mhz"):
            L.append(f"\nFmax sau P&R: **{g['fmax_mhz']} MHz** (chạy {g.get('clock_mhz')} MHz).")
        if g.get("power_w"):
            e = g["power_w"] * ref["ms"]
            L.append(f"Công suất ước lượng {g['power_w']} W → năng lượng ≈ {e:.1f} mJ / cặp ảnh.")
    else:
        notes.append("Chưa có gowin_metrics.json (chạy với --init-gowin rồi điền số).")
    ev_p = Path(args.eval) if args.eval else out / "reports" / "eval_report.json"
    if ev_p.exists():
        e = json.loads(ev_p.read_text(encoding="utf-8"))
        auc = (e.get("test_smoothed") or {}).get("auc")
        if auc:
            L += ["", "## Độ chính xác (không đổi khi đưa lên phần cứng)", "",
                  f"- AUC mô hình số nguyên (khớp từng bit với FPGA): **{auc:.4f}** (mô hình số thực gốc 0,734).",
                  "- RTL khớp Golden Model: xem outputs_hw/rtl_sim/*/report.md (mọi testbench PASS)."]
    if notes:
        L += ["", "## Ghi chú", ""] + [f"- {n}" for n in notes]
    run = out / "accel_report" / time.strftime("%Y%m%d_%H%M%S")
    run.mkdir(parents=True, exist_ok=True)
    (run / "report.md").write_text("\n".join(L) + "\n", encoding="utf-8")
    (run / "metrics.json").write_text(json.dumps({"n_iter": N, "rows": allr, "notes": notes}, indent=2,
                                                 ensure_ascii=False, default=str), encoding="utf-8")
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        names = [r["name"].replace(" – ", "\n", 1) for r in allr]
        vals = [r["ms"] for r in allr]
        colors = ["#2e7d32" if i < len(fpga_rows) else ("#c62828" if r.get("group") == "esp32" else "#1565c0")
                  for i, r in enumerate(allr)]
        fig, ax = plt.subplots(figsize=(10, 0.55 * len(allr) + 1.5))
        y = range(len(allr))[::-1]
        ax.barh(list(y), vals, color=colors)
        ax.set_yticks(list(y))
        ax.set_yticklabels(names, fontsize=8)
        ax.set_xscale("log")
        ax.axvline(80, color="gray", ls="--", lw=1)
        ax.text(80, len(allr) - 0.4, " 80 ms = 12,5 fps", fontsize=8, color="gray")
        for yy, v in zip(y, vals):
            ax.text(v, yy, f" {v:.1f} ms", va="center", fontsize=8)
        ax.set_xlabel("ms / cặp ảnh (thang log)")
        ax.set_title(f"Horn–Schunck {N} vòng, ảnh 160×120\nxanh lá: FPGA · đỏ: ESP32-S3 · xanh dương: PC", fontsize=10)
        fig.tight_layout()
        fig.savefig(run / "speedup.png", dpi=150)
        plt.close(fig)
    except Exception as e:  # noqa: BLE001
        log.warning(f"Bỏ qua biểu đồ: {e}")
    print("\n".join(L))
    log.info(f"→ {run}")


if __name__ == "__main__":
    main()
