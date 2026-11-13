#!/usr/bin/env python3
"""HUẤN LUYỆN LẠI VỚI DỮ LIỆU CẢNH THẬT (ghi bằng 29_record_scene.py) + so sánh với mô hình hiện tại.

    python scripts/36_train_scene.py --config configs/hw.yaml                     # IITB + cảnh thật
    python scripts/36_train_scene.py --config configs/hw.yaml --mode scene        # chỉ cảnh thật
    python scripts/36_train_scene.py --config configs/hw.yaml --scene-share 0.5   # cảnh thật chiếm 50 % mẫu học

Dữ liệu cảnh (data/scene/):
    normal/*.npz            → học + chọn ngưỡng (mỗi file tách 70 % đầu học, 30 % cuối kiểm tra báo giả)
    abnormal/*.npz          → chỉ để đánh giá tỉ lệ phát hiện
    test/normal, test/abnormal (ghi bằng --test) → chỉ để đánh giá, không bao giờ dùng để học
Đặc trưng IITB lấy từ cache của outputs_hw (không trích lại).

Kết quả ở outputs_scene/ (không đụng outputs_hw):
    models/detector.joblib, reports/scene_report.md + .json
    splits.json + features/ được chép từ outputs_hw để bước xuất FPGA chạy được ngay:
    python scripts/04_export_fpga.py --config configs/hw.yaml --set project.output_dir=outputs_scene
Rồi chép các file .mem/.vh mới vào project Gowin, build lại bitstream (như lần xuất mô hình trước).
"""
import json
import shutil
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import numpy as np  # noqa: E402
from sklearn.metrics import roc_auc_score  # noqa: E402

from ocsvm_fpga.cli import base_parser, get_splits, setup  # noqa: E402
from ocsvm_fpga.features.pipeline import load_split  # noqa: E402
from ocsvm_fpga.models.base import AnomalyDetector  # noqa: E402
from ocsvm_fpga.models.training import calibrate_threshold, fit_detector  # noqa: E402
from ocsvm_fpga.utils import save_json  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
Q = 1024.0                                                     # Q5.10


def load_dir(d: Path, drop_sat: bool) -> list[dict]:
    out = []
    for p in sorted(d.glob("*.npz")):
        z = np.load(p)
        x = z["Xq"].astype(np.float64) / Q
        keep = ~z["sat"] if (drop_sat and "sat" in z.files) else np.ones(len(x), bool)
        if keep.sum():
            out.append({"name": p.stem, "X": x[keep], "n_sat": int((~keep).sum())})
    return out


def stack(parts, start_id=0):
    if not parts:
        return np.zeros((0, 48)), np.zeros(0, int)
    return (np.concatenate([p["X"] for p in parts]),
            np.concatenate([np.full(len(p["X"]), start_id + i) for i, p in enumerate(parts)]))


def rate(s, thr):
    return float(np.mean(s > thr)) if len(s) else float("nan")


def auc(y, s):
    return float(roc_auc_score(y, s)) if len(np.unique(y)) == 2 else float("nan")


def main():
    ap = base_parser(__doc__)
    ap.add_argument("--scene-dir", default="data/scene")
    ap.add_argument("--mode", choices=["both", "scene"], default="both", help="both = IITB + cảnh thật")
    ap.add_argument("--scene-share", type=float, default=0.3, help="tỉ lệ mẫu cảnh thật trong tập học (mode both)")
    ap.add_argument("--hold", type=float, default=0.3, help="phần cuối mỗi file normal để kiểm tra báo giả")
    ap.add_argument("--out", default="outputs_scene")
    ap.add_argument("--keep-sat", action="store_true", help="giữ cả cặp có z bão hòa")
    args = ap.parse_args()
    cfg, src_out, log, rng = setup(args, "36_train_scene")          # src_out = outputs_hw (đặc trưng IITB)
    sd = Path(args.scene_dir)
    sd = sd if sd.is_absolute() else ROOT / sd

    # ------------------------------------------------------------ dữ liệu cảnh
    normal = load_dir(sd / "normal", not args.keep_sat)
    abn = load_dir(sd / "abnormal", not args.keep_sat) + load_dir(sd / "test" / "abnormal", not args.keep_sat)
    t_norm = load_dir(sd / "test" / "normal", not args.keep_sat)
    if not normal:
        sys.exit(f"Chưa có dữ liệu bình thường trong {sd / 'normal'} – chạy 29_record_scene.py --label normal trước")
    tr_parts, ho_parts = [], []
    for p in normal:                                             # tách theo THỜI GIAN trong mỗi file
        k = int(round(len(p["X"]) * (1 - args.hold)))
        tr_parts.append({"name": p["name"], "X": p["X"][:k]})
        if len(p["X"]) - k > 0:
            ho_parts.append({"name": p["name"], "X": p["X"][k:]})
    xs_tr, vs_tr = stack(tr_parts)
    xs_ho, vs_ho = stack(ho_parts + t_norm, 1000)
    xs_ab, vs_ab = stack(abn, 2000)
    log.info(f"Cảnh thật: học {len(xs_tr)} cặp ({len(normal)} file) | kiểm tra bình thường {len(xs_ho)} | "
             f"bất thường {len(xs_ab)} ({len(abn)} file)")
    if len(xs_tr) < 300:
        log.warning("Ít hơn 300 cặp bình thường (khoảng 25 s) – nên ghi ít nhất 5–10 phút, nhiều lượt khác nhau.")

    # ------------------------------------------------------------ dữ liệu IITB (cache outputs_hw)
    splits = get_splits(cfg, src_out, rng)
    tr = load_split(splits["train"], cfg, src_out)
    va = load_split(splits["val"], cfg, src_out)
    te = load_split(splits["test"], cfg, src_out)
    log.info(f"IITB: học {tr['X'].shape[0]} | val {va['X'].shape[0]} | test {te['X'].shape[0]}")

    if args.mode == "both" and len(tr["X"]) == 0:
        sys.exit("Không đọc được đặc trưng IITB trong cache outputs_hw (đúng --config configs/hw.yaml chưa? "
                 "đường dẫn trong splits.json còn đúng không – xem fix_split_paths.py). Hoặc dùng --mode scene.")
    if args.mode == "scene":
        x_fit = xs_tr
    else:
        share = min(max(args.scene_share, 0.0), 0.9)
        n_cap = int(cfg.model.max_train_samples)
        n_iitb = min(len(tr["X"]), int(n_cap * (1 - share)))
        n_scene = int(round(n_iitb * share / max(1 - share, 1e-9)))
        xi = tr["X"][rng.choice(len(tr["X"]), n_iitb, replace=False)] if n_iitb < len(tr["X"]) else tr["X"]
        reps = int(np.ceil(n_scene / max(len(xs_tr), 1)))
        xsr = np.tile(xs_tr, (reps, 1))[:n_scene]
        x_fit = np.concatenate([xi, xsr])
        log.info(f"Tập học: {len(xi)} IITB + {len(xsr)} cảnh thật (lặp ×{reps}, chiếm {share:.0%})")

    det, info = fit_detector(x_fit, cfg, rng, None, 1.0, log)
    # ngưỡng: phân vị điểm BÌNH THƯỜNG – lấy cả val IITB lẫn phần học của cảnh (không dùng phần kiểm tra)
    if args.mode == "scene":
        val = {"X": xs_tr, "video_id": vs_tr}
    else:
        val = {"X": np.concatenate([va["X"], xs_tr]), "video_id": np.concatenate([va["video_id"], vs_tr + 10**6])}
    thr = calibrate_threshold(det, cfg, val, log)

    # ------------------------------------------------------------ đánh giá: mô hình cũ vs mới
    old = AnomalyDetector.load(src_out / "models" / "detector.joblib")
    rows = {}
    for name, d in (("hiện tại (outputs_hw)", old), ("mới (outputs_scene)", det)):
        r = {"n_sv": int(d.model.n_sv)}
        if len(te["X"]) and te.get("y") is not None:
            r["iitb_auc_raw"] = auc(te["y"], d.score(te["X"], te["video_id"], smooth=False))
            r["iitb_auc_smooth"] = auc(te["y"], d.score(te["X"], te["video_id"]))
        s_ho = d.score(xs_ho, vs_ho, smooth=False) if len(xs_ho) else np.zeros(0)
        s_ab = d.score(xs_ab, vs_ab, smooth=False) if len(xs_ab) else np.zeros(0)
        r["scene_false_alarm_raw"] = rate(s_ho, d.threshold)
        r["scene_detect_raw"] = rate(s_ab, d.threshold)
        if len(s_ho) and len(s_ab):
            r["scene_auc_raw"] = auc(np.r_[np.zeros(len(s_ho)), np.ones(len(s_ab))], np.r_[s_ho, s_ab])
        rows[name] = r

    # ------------------------------------------------------------ lưu
    out = Path(args.out)
    out = out if out.is_absolute() else ROOT / out
    (out / "models").mkdir(parents=True, exist_ok=True)
    (out / "reports").mkdir(parents=True, exist_ok=True)
    if not (out / "splits.json").exists() and (src_out / "splits.json").exists():
        shutil.copy(src_out / "splits.json", out / "splits.json")
    if not (out / "features").exists() and (src_out / "features").exists():
        shutil.copytree(src_out / "features", out / "features")
    det.meta["scene"] = {"mode": args.mode, "scene_share": args.scene_share, "train_pairs": int(len(xs_tr)),
                         "files": [p["name"] for p in normal]}
    det.save(out / "models" / "detector.joblib")

    def f(x, pct=False):
        if x is None or (isinstance(x, float) and np.isnan(x)):
            return "–"
        return f"{x:.1%}" if pct else f"{x:.4f}"

    L = ["# Huấn luyện lại với dữ liệu cảnh thật", "",
         f"Chế độ: **{args.mode}**" + (f", cảnh thật chiếm {args.scene_share:.0%} tập học" if args.mode == "both" else ""),
         f"Cảnh thật: học {len(xs_tr)} cặp · kiểm tra bình thường {len(xs_ho)} cặp · bất thường {len(xs_ab)} cặp", "",
         "| Mô hình | Số SV | AUC IITB thô | AUC IITB làm mượt | Báo giả cảnh thật (thô) | Phát hiện cảnh thật (thô) | AUC cảnh thật |",
         "|---|---|---|---|---|---|---|"]
    for name, r in rows.items():
        L.append(f"| {name} | {r['n_sv']} | {f(r.get('iitb_auc_raw'))} | {f(r.get('iitb_auc_smooth'))} | "
                 f"{f(r['scene_false_alarm_raw'], True)} | {f(r['scene_detect_raw'], True)} | {f(r.get('scene_auc_raw'))} |")
    L += ["", "- Báo giả / phát hiện tính trên cờ TỪNG CẶP (chưa qua lọc 5/7), ở ngưỡng của từng mô hình.",
          "- Phần kiểm tra bình thường = 30 % cuối mỗi file normal + data/scene/test/normal (không dùng để học).",
          "- Số SV > 128 hoặc AUC IITB giảm mạnh → xem lại trước khi xuất FPGA.", "",
          "Xuất FPGA: `python scripts/04_export_fpga.py --config configs/hw.yaml --set project.output_dir=outputs_scene`"]
    (out / "reports" / "scene_report.md").write_text("\n".join(L) + "\n", encoding="utf-8")
    save_json({"rows": rows, "threshold": thr, "mode": args.mode, "info": {k: v for k, v in info.items()
               if isinstance(v, (int, float, str))}}, out / "reports" / "scene_report.json")
    print("\n" + "\n".join(L))
    print(f"\nMô hình → {out / 'models' / 'detector.joblib'}")


if __name__ == "__main__":
    main()
