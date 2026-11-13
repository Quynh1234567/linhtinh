#!/usr/bin/env python3
"""THỬ ĐỘ CHÍNH XÁC THEO ĐỘ PHÂN GIẢI (chỉ phần mềm – KHÔNG xuất file cho phần cứng).

Với mỗi độ phân giải: trích đặc trưng → huấn luyện OC-SVM → đánh giá, rồi gộp thành một bảng so sánh.
Mọi thứ khác GIỮ NGUYÊN để so sánh công bằng:
  - cùng cách chia tập (outputs/splits.json), cùng frame_step, cùng nhãn
  - cùng Horn–Schunck (α = 2, 64 vòng), Gauss 5×5 σ = 1
  - cùng lưới 4 × 4 ô, cùng thống kê [u, v, |v|] mỗi ô → 48 chiều, cùng nén log, cùng ν, γ, cách chọn ngưỡng
Chỉ đổi preprocess.width × height.

Đặc trưng dùng bản SỐ THỰC (descriptor grid_flow) vì Golden Model số nguyên (grid_flow_hw) cố định 160×120.
Vì vậy bảng có cả 160×120 số thực làm mốc; mô hình phần cứng (outputs_hw, AUC ≈ 0,73) để đối chiếu thêm.

    python scripts/35_resolution_sweep.py --config configs/hw.yaml --jobs 8
    python scripts/35_resolution_sweep.py --config configs/hw.yaml --jobs 8 --res 160x120 320x240 480x360
    python scripts/35_resolution_sweep.py --config configs/hw.yaml --report-only     # chỉ gộp bảng

Ràng buộc: cache đặc trưng cộng theo khối 10×10 → biên ô phải là bội của 10, tức W và H chia hết cho 40
(lưới 4 × 4): 160×120, 320×240, 480×360 dùng được; 80×60 thì KHÔNG.
Độ phân giải mới phải giải mã lại video gốc (đường dẫn trong outputs/splits.json phải còn đúng) – chậm,
và Horn–Schunck trên 320×240 chậm khoảng 4 lần 160×120. Kết quả mỗi độ phân giải ở outputs_res/<WxH>/;
bảng gộp ở outputs_res/resolution_report.md. Không đụng tới outputs/ hay outputs_hw/.
"""
import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def run(cmd, log):
    print("\n>>> " + " ".join(cmd), flush=True)
    log.write("\n>>> " + " ".join(cmd) + "\n")
    t = time.time()
    r = subprocess.run(cmd, cwd=ROOT)
    if r.returncode != 0:
        sys.exit(f"Lệnh lỗi (mã {r.returncode}) – xem log ở trên")
    return time.time() - t


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default="configs/hw.yaml")
    ap.add_argument("--res", nargs="+", default=["160x120", "320x240"], help="danh sách WxH")
    ap.add_argument("--jobs", type=int, default=None)
    ap.add_argument("--out", default="outputs_res")
    ap.add_argument("--report-only", action="store_true", help="chỉ đọc kết quả đã có và gộp bảng")
    ap.add_argument("--set", nargs="*", default=[], help="ghi đè thêm cho mọi lần chạy (vd model.nu=0.02)")
    a = ap.parse_args()

    out_root = (ROOT / a.out).resolve()
    out_root.mkdir(parents=True, exist_ok=True)
    res = []
    for s in a.res:
        w, h = (int(x) for x in s.lower().split("x"))
        if w % 40 or h % 40:
            sys.exit(f"{s}: W và H phải chia hết cho 40 (lưới 4×4, khối 10×10)")
        res.append((w, h))

    log = open(out_root / "sweep_log.txt", "a", encoding="utf-8")
    times = {}
    if not a.report_only:
        for w, h in res:
            tag = f"{w}x{h}"
            sets = [f"project.output_dir={a.out}/{tag}", "features.descriptor=grid_flow",
                    f"preprocess.width={w}", f"preprocess.height={h}",
                    "features.grid_rows=4", "features.grid_cols=4", "features.grid_stats=basic",
                    "features.grid_log_scale=0.5", "features.grid_diff=false"] + list(a.set)
            base = [sys.executable, "-X", "utf8"]
            common = ["--config", a.config, "--set", *sets]
            t1 = run(base + ["scripts/01_extract_features.py", *common] + (["--jobs", str(a.jobs)] if a.jobs else []), log)
            t2 = run(base + ["scripts/02_train.py", *common], log)
            t3 = run(base + ["scripts/03_evaluate.py", *common], log)
            times[tag] = {"extract_s": t1, "train_s": t2, "eval_s": t3}
            (out_root / tag / "sweep_times.json").write_text(json.dumps(times[tag], indent=1), encoding="utf-8")

    # ---------------------------------------------------------------- gộp bảng
    rows = []
    for w, h in res:
        tag = f"{w}x{h}"
        rp = out_root / tag / "reports" / "eval_report.json"
        if not rp.exists():
            print(f"(chưa có {rp})")
            continue
        r = json.loads(rp.read_text(encoding="utf-8"))
        tm = out_root / tag / "sweep_times.json"
        tm = json.loads(tm.read_text(encoding="utf-8")) if tm.exists() else {}
        n_sv = None
        try:
            sys.path.insert(0, str(ROOT / "src"))
            from ocsvm_fpga.models.base import AnomalyDetector
            n_sv = int(AnomalyDetector.load(out_root / tag / "models" / "detector.joblib").model.n_sv)
        except Exception:                                # noqa: BLE001
            pass
        raw, sm = r.get("test_raw", {}), r.get("test_smoothed", {})
        rows.append({"res": tag, "pixels": w * h,
                     "auc_raw": raw.get("auc"), "auc_smooth": sm.get("auc"), "eer": sm.get("eer"),
                     "acc_raw": raw.get("accuracy"), "p_raw": raw.get("precision"), "r_raw": raw.get("recall"),
                     "f1_raw": raw.get("f1"), "video_auc": (r.get("video_level_auc") or {}).get("auc"),
                     "n_sv": n_sv, "extract_s": tm.get("extract_s")})

    def f(x, p=4):
        return "–" if x is None else (f"{x:.{p}f}" if isinstance(x, float) else str(x))

    hw = ROOT / "outputs_hw" / "reports" / "eval_report.json"
    hw_line = ""
    if hw.exists():
        r = json.loads(hw.read_text(encoding="utf-8"))
        hw_line = (f"| 160×120 **phần cứng** (số nguyên, outputs_hw) | 19 200 | {f(r.get('test_raw', {}).get('auc'))} | "
                   f"{f(r.get('test_smoothed', {}).get('auc'))} | {f(r.get('test_smoothed', {}).get('eer'))} | "
                   f"{f(r.get('test_raw', {}).get('accuracy'), 3)} | {f(r.get('test_raw', {}).get('recall'), 3)} | "
                   f"{f(r.get('test_raw', {}).get('f1'), 3)} | – | – | – |")
    L = ["# Độ chính xác theo độ phân giải (chỉ phần mềm)", "",
         f"Chạy {time.strftime('%Y-%m-%d %H:%M')} · cùng splits, HS α = 2 × 64 vòng, lưới 4×4 [u, v, |v|] = 48 chiều, "
         "cùng ν, γ, ngưỡng · đặc trưng số thực (grid_flow).", "",
         "| Độ phân giải | Điểm ảnh | AUC thô | AUC làm mượt | EER | Accuracy thô | Recall thô | F1 thô | AUC mức video | Số SV | Trích đặc trưng (s) |",
         "|---|---|---|---|---|---|---|---|---|---|---|"]
    for x in rows:
        L.append(f"| {x['res']} | {x['pixels']:,} | {f(x['auc_raw'])} | {f(x['auc_smooth'])} | {f(x['eer'])} | "
                 f"{f(x['acc_raw'], 3)} | {f(x['r_raw'], 3)} | {f(x['f1_raw'], 3)} | {f(x['video_auc'])} | "
                 f"{f(x['n_sv'])} | {f(x['extract_s'], 0)} |")
    if hw_line:
        L.append(hw_line)
    L += ["", "Ghi chú:",
          "- AUC thô = điểm từng cặp khung (giống cờ FPGA); AUC làm mượt = trung bình trượt theo thời gian.",
          "- Accuracy/Recall/F1 tính tại ngưỡng chọn tự động của từng mô hình (phân vị điểm bình thường).",
          "- Giữ nguyên α, Gauss 5×5 và số vòng HS ở mọi độ phân giải: ở ảnh lớn, cùng một chuyển động "
          "có độ dịch (pixel) lớn hơn nên HS (giả định dịch nhỏ) có thể kém hơn – đây là một phần của kết quả.",
          "- Chỉ là thí nghiệm phần mềm, chưa xuất file phần cứng; 320×240 trên Tang Nano 20K cần ~120 BSRAM cho u, v."]
    (out_root / "resolution_report.md").write_text("\n".join(L) + "\n", encoding="utf-8")
    print("\n" + "\n".join(L))
    print(f"\nBảng → {out_root / 'resolution_report.md'}")


if __name__ == "__main__":
    main()
