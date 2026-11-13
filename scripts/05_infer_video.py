#!/usr/bin/env python3
r"""Suy luận trên một video bất kỳ (vd video ghi từ ESP32-S3/OV5640) hoặc webcam.

Mô hình vẫn xử lý đúng như lúc huấn luyện (xám 160×120, frame_step, làm mượt,
Horn–Schunck, đặc trưng, OC-SVM float và fixed-point bit-accurate như FPGA), nhưng
VIDEO KẾT QUẢ giữ nguyên bản gốc: màu, độ phân giải và tốc độ khung gốc, chỉ vẽ thêm
khung viền xanh/đỏ, nhãn NORMAL/ABNORMAL, điểm số và đồ thị điểm theo thời gian.

    python scripts/05_infer_video.py --video path/to/clip.avi --save-video
    python scripts/05_infer_video.py --video data\IITB-Corridor\Test\000216.avi --config configs/hw.yaml --save-video --show       # xem trực tiếp
    python scripts/05_infer_video.py --video clip.avi --save-video --scale 0.5 # thu nhỏ video ra
    python scripts/05_infer_video.py --camera 0
"""
import csv
import sys
from collections import deque
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import cv2  # noqa: E402
import numpy as np  # noqa: E402

from ocsvm_fpga.cli import base_parser, get_splits, setup, load_detector_checked  # noqa: E402
from ocsvm_fpga.evaluation.events import alarm_params, alarm_params_need_normal  # noqa: E402
from ocsvm_fpga.features.pipeline import load_split  # noqa: E402
from ocsvm_fpga.data.video_reader import preprocess_frame  # noqa: E402

try:                                                    # bản video_reader cũ chưa có apply_masks
    from ocsvm_fpga.data.video_reader import apply_masks  # noqa: E402
except ImportError:                                     # → không che vùng nào (giống hành vi cũ)
    def apply_masks(frame, path, pp):
        return frame
from ocsvm_fpga.features.pipeline import frame_pair_descriptor  # noqa: E402
from ocsvm_fpga.models.base import AnomalyDetector  # noqa: E402
from ocsvm_fpga.quantization.golden_model import FixedPointOCSVM  # noqa: E402

GREEN, RED, GREY = (60, 200, 60), (40, 40, 230), (170, 170, 170)


def draw_overlay(frame, state, history, thr, idx, fps, off=None):
    """Vẽ kết quả lên khung GỐC (màu, đúng độ phân giải)."""
    h, w = frame.shape[:2]
    s = max(w / 1280.0, 0.45)            # video nhỏ (vd 320×240) vẫn đọc được chữ
    vis = frame.copy()
    if state is None:                                    # chưa đủ 2 khung để có kết quả
        label, color, score = "WAITING", GREY, None
    else:
        label = ("ABNORMAL" if state["raw"] else "ABNORMAL (hold)") if state["alarm"] else "NORMAL"
        color = RED if state["alarm"] else GREEN
        score = state["score"]
    t = max(3, int(8 * s))
    cv2.rectangle(vis, (t // 2, t // 2), (w - t // 2, h - t // 2), color, t)
    # nhãn + điểm
    pad = int(12 * s)
    txt = label if score is None else f"{label}   score {score:.3f}  (thr {thr:.3f})"
    fs, th = 1.0 * s, max(1, int(2 * s))
    (tw, tht), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, fs, th)
    cv2.rectangle(vis, (pad, pad), (pad * 3 + tw, pad * 3 + tht), (0, 0, 0), -1)
    cv2.putText(vis, txt, (pad * 2, pad * 2 + tht), cv2.FONT_HERSHEY_SIMPLEX, fs, color, th, cv2.LINE_AA)
    sec = idx / fps if fps else 0
    tm = f"frame {idx}  t = {sec:6.2f} s"
    (tw2, th2), _ = cv2.getTextSize(tm, cv2.FONT_HERSHEY_SIMPLEX, 0.7 * s, th)
    ty2 = pad * 4 + tht + th2                             # dòng thứ hai, dưới nhãn → không chồng chữ
    cv2.rectangle(vis, (pad, ty2 - th2 - pad // 2), (pad * 3 + tw2, ty2 + pad // 2), (0, 0, 0), -1)
    cv2.putText(vis, tm, (pad * 2, ty2), cv2.FONT_HERSHEY_SIMPLEX, 0.7 * s, (255, 255, 255), th, cv2.LINE_AA)
    # đồ thị điểm theo thời gian (dải dưới cùng)
    if len(history) > 1:
        gh, gx0, gx1 = int(110 * s), pad, w - pad
        gy0 = h - pad - gh
        roi = vis[gy0:gy0 + gh, gx0:gx1]
        roi[:] = (roi * 0.35).astype(np.uint8)
        vals = np.array(history)
        lo, hi = min(vals.min(), thr, off if off is not None else thr), max(vals.max(), thr)
        span = hi - lo if hi > lo else 1.0
        ys = lambda v: int(gy0 + gh - 6 - (v - lo) / span * (gh - 12))  # noqa: E731
        xs = np.linspace(gx0, gx1, len(vals)).astype(int)
        ty = ys(thr)
        cv2.line(vis, (gx0, ty), (gx1, ty), (0, 215, 255), max(1, int(1.5 * s)), cv2.LINE_AA)
        cv2.putText(vis, "threshold", (gx0 + 6, ty - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5 * s, (0, 215, 255), th,
                    cv2.LINE_AA)
        if off is not None and off < thr:
            oy = ys(off)
            for xd in range(gx0, gx1, max(8, int(14 * s))):        # ngưỡng tắt: nét đứt
                cv2.line(vis, (xd, oy), (min(xd + int(7 * s) + 2, gx1), oy), (0, 140, 255), max(1, int(1.5 * s)))
            cv2.putText(vis, "off", (gx1 - int(40 * s), oy - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5 * s, (0, 140, 255), th,
                        cv2.LINE_AA)
        pts = np.stack([xs, [ys(v) for v in vals]], 1).astype(np.int32)
        cv2.polylines(vis, [pts], False, (255, 255, 255), max(1, int(2 * s)), cv2.LINE_AA)
        cv2.circle(vis, tuple(pts[-1]), max(3, int(5 * s)), color, -1)
    return vis


def main():
    ap = base_parser(__doc__)
    ap.add_argument("--video", default=None)
    ap.add_argument("--camera", type=int, default=None)
    ap.add_argument("--save-video", action="store_true", help="ghi video gốc có chú thích kết quả")
    ap.add_argument("--show", action="store_true", help="hiện cửa sổ xem trực tiếp (Esc để dừng)")
    ap.add_argument("--scale", type=float, default=1.0, help="thu nhỏ video kết quả (vd 0.5); 1 = giữ nguyên")
    ap.add_argument("--history", type=int, default=150, help="số điểm gần nhất vẽ trên đồ thị")
    args = ap.parse_args()
    if not args.video and args.camera is None:
        ap.error("cần --video <đường dẫn video> hoặc --camera <số>")
    cfg, out, log, _ = setup(args, "05_infer")  # _ = rng
    pp = cfg.preprocess
    det = load_detector_checked(cfg, out)
    fx = FixedPointOCSVM(det, cfg.fpga)
    k = max(1, det.smoothing)
    ap = alarm_params(det, cfg)
    if alarm_params_need_normal(det, cfg):              # cấu hình cảnh báo đổi / mô hình cũ → tính lại từ val
        try:
            val = load_split(get_splits(cfg, out, _)["val"], cfg, out)
            sv = det.score(val["X"], val["video_id"]) if len(val["X"]) else None
        except Exception:                               # noqa: BLE001 – không có cache → không trễ
            sv = None
        ap = alarm_params(det, cfg, sv)
    on_pairs = int(ap.get("on_pairs", 1))
    log.info(f"Cảnh báo: bật > {ap['on']:.4f} liên tục {on_pairs} cặp, tắt < {ap['off']:.4f} liên tục {ap['hold']} cặp khung")
    active, below, run = False, 0, 0
    win_float, win_fix = deque(maxlen=k), deque(maxlen=k)
    history = deque(maxlen=args.history)
    step = max(1, int(pp.frame_step))
    dark = float(pp.get("skip_dark_below", 0) or 0)
    hw = cfg.features.get("descriptor", "covariance") == "grid_flow_hw"
    if hw:
        from ocsvm_fpga.data.video_reader import to_sensor_frame
        from ocsvm_fpga.hwmodel import hs_q as HQ
        from ocsvm_fpga.hwmodel import preproc_q as PQ
        hw_sc = HQ.quantize_scaler(det.scaler.mean, det.scaler.scale)

    cap = cv2.VideoCapture(args.video if args.video else args.camera)
    if not cap.isOpened():
        sys.exit(f"Không mở được {'video ' + args.video if args.video else 'camera ' + str(args.camera)}")
    fps = cap.get(cv2.CAP_PROP_FPS) or 25.0
    name = Path(args.video).stem if args.video else f"camera{args.camera}"
    res_dir = out / "inference"
    res_dir.mkdir(parents=True, exist_ok=True)
    show = args.show or args.camera is not None
    writer, rows, prev, state, idx = None, [], None, None, -1
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        idx += 1
        if idx % step == 0:                                   # khung đưa vào mô hình (giống lúc huấn luyện)
            src = apply_masks(frame, args.video or "", pp)
            if hw:                                            # đường số nguyên: ảnh uint8 thô, Gauss làm bên trong
                gray = to_sensor_frame(src, pp)
                is_dark = dark > 0 and PQ.frame_sum(PQ.gauss5_q(gray)) < dark * gray.size
            else:
                gray = preprocess_frame(src, pp)
                is_dark = dark > 0 and float(gray.mean()) < dark
            if is_dark:                                       # khung đen → ngắt cặp, như lúc huấn luyện
                prev = None
            else:
                if prev is not None:
                    vec, _, motion = frame_pair_descriptor(prev, gray, cfg)
                    s_float = float(det.model.score_from_decision(det.decision(vec[None]))[0])
                    if hw:                                    # z số nguyên đúng như FPGA
                        xq = HQ.standardize_q(np.round(vec[None] * 1024).astype(np.int64), hw_sc)
                    else:
                        xq, _ = fx.quantize_input(vec[None])
                    f_q = fx.decision_q(xq)
                    s_fix = float(det.model.score_from_decision(fx.decision_float_equiv(f_q))[0])
                    win_float.append(s_float)
                    win_fix.append(s_fix)
                    sm_float, sm_fix = float(np.mean(win_float)), float(np.mean(win_fix))
                    raw_alarm = int(sm_fix > ap["on"])
                    run = run + 1 if raw_alarm else 0
                    if not active:                       # (a) cảnh báo có trễ: bật khi vượt ngưỡng on_pairs cặp liên tiếp
                        active, below = run >= on_pairs, 0
                    else:
                        below = below + 1 if sm_fix < ap["off"] else 0
                        active = below <= ap["hold"]
                    alarm = int(active)
                    state = {"alarm": alarm, "raw": raw_alarm, "score": sm_fix}
                    history.append(sm_fix)
                    rows.append([idx, round(idx / fps, 3), s_float, s_fix, sm_float, sm_fix,
                                 int(fx.anomaly_q(f_q)[0]), raw_alarm, alarm, motion])
                prev = gray
        # mọi khung gốc đều được ghi ra (giữ nguyên tốc độ), dùng kết quả gần nhất
        if args.save_video or show:
            vis = draw_overlay(frame, state, history, ap["on"], idx, fps, ap["off"])
            if args.scale != 1.0:
                vis = cv2.resize(vis, None, fx=args.scale, fy=args.scale, interpolation=cv2.INTER_AREA)
            if args.save_video:
                if writer is None:
                    writer = cv2.VideoWriter(str(res_dir / f"{name}_result.mp4"), cv2.VideoWriter_fourcc(*"mp4v"),
                                             fps, (vis.shape[1], vis.shape[0]))
                writer.write(vis)
            if show:
                cv2.imshow("OC-SVM anomaly", vis)
                if cv2.waitKey(1) & 0xFF == 27:
                    break
    cap.release()
    if writer:
        writer.release()
    if show:
        cv2.destroyAllWindows()
    with open(res_dir / f"{name}_scores.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["frame", "time_s", "score_float", "score_fixed", "smooth_float", "smooth_fixed",
                    "fpga_flag_raw", "alarm_no_hold", "alarm", "mean_motion"])
        w.writerows(rows)
    if rows:
        arr = np.array(rows)
        log.info(f"{len(rows)} cặp khung | tỉ lệ cảnh báo = {arr[:, 8].mean():.2%} "
                 f"(không trễ {arr[:, 7].mean():.2%}) | "
                 f"sai số float/fixed TB = {np.abs(arr[:, 2] - arr[:, 3]).mean():.2e}")
    if writer:
        log.info(f"Video kết quả (bản gốc, {fps:.0f} fps): {res_dir / (name + '_result.mp4')}")
    log.info(f"Điểm từng khung → {res_dir / (name + '_scores.csv')}")


if __name__ == "__main__":
    main()
