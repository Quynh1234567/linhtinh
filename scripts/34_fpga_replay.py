#!/usr/bin/env python3
"""PHÁT LẠI KHUNG IITB QUA HỆ THỐNG THẬT, SO TỪNG CẶP VỚI GOLDEN MODEL TRÊN PC (end-to-end).

    PC ──UART──► ESP32-S3 (firmware/esp32_replay: Gauss y hệt demo) ──SPI──► Tang Nano 20K
    (bitstream hs_system của demo, 54 MHz, KHÔNG đổi: DERIV → HS 64 vòng → FEAT → OC-SVM)
    ──UART BL616──► PC.   Cùng khung đó, PC tính Golden: Gauss → HS → FEAT → z → OC-SVM.

Mỗi cặp so: Σ ảnh Gauss (ESP ↔ Golden), F = f_q, A = cờ, S = số lần bão hòa, Z = Σz (16 bit) (FPGA ↔ Golden).
Khung lấy từ cache khung 160×120 của pipeline train (cùng frame_step, cùng bỏ khung tối), các cặp
liên tiếp trong video giống luồng camera; đầu mỗi đoạn gửi cờ RESET_PAIR.

    python scripts/34_fpga_replay.py --config configs/hw.yaml --esp COM5 --fpga COM9 --pairs 1000 --per-video 100
    python scripts/34_fpga_replay.py --config configs/hw.yaml --esp COM5 --fpga COM9 --videos 000251 000255 --per-video 50

CHẠY TRỌN VIDEO + CỬA SỔ DEMO (đúng luồng hệ thống, ảnh lấy từ video thay cho camera):
    python scripts/34_fpga_replay.py --config configs/hw.yaml --esp COM5 --fpga COM9 --require-z ^
        --videos 228 238 261 286 323 210 224 227 --per-video 0 --pairs 0 --show
  --per-video 0 : cả video từ khung đầu   --pairs 0 : không giới hạn số cặp
  --show        : cửa sổ khung hình + kết quả FPGA + nhãn thật + dải thời gian (q thoát, space tạm dừng)
  --record      : kèm --show, quay cửa sổ thành video_replay.avi (MJPG chất lượng cao)
  Cửa sổ hiện khung MÀU GỐC đọc từ file video (chỉ để xem; FPGA vẫn nhận ảnh xám 160×120 như camera),
  --gray để hiện ảnh xám 160×120 thật sự được gửi.
  Ba dải thời gian: cờ FPGA từng cặp (thô) · cảnh báo sau lọc (cửa sổ --smooth 7 cặp: bật khi ≥ 5,
  tắt khi ≤ 1 – y hệt 27_demo_monitor) · nhãn thật.
  Tên video viết tắt được: 228 ↔ 000228 (khớp theo số).

Trước khi chạy: nạp firmware/esp32_replay cho ESP32 (idf.py build flash), FPGA giữ bitstream demo
(External Flash), đóng 27_demo_monitor / idf.py monitor để giải phóng hai cổng COM.
Kết quả: <output_dir>/replay/<thời gian>/report.md, pairs.csv, log.txt
"""
import queue
import re
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import numpy as np  # noqa: E402

from ocsvm_fpga.cli import base_parser, get_splits, load_detector_checked, setup  # noqa: E402
from ocsvm_fpga.features.pipeline import load_sensor_frames  # noqa: E402
from ocsvm_fpga.hwmodel import hs_export as HX  # noqa: E402
from ocsvm_fpga.hwmodel import hs_q as HQ  # noqa: E402
from ocsvm_fpga.hwmodel import preproc_q as PQ  # noqa: E402
from ocsvm_fpga.quantization.golden_model import FixedPointOCSVM  # noqa: E402

RE_HS = re.compile(r"HS N=([0-9A-F]{4}) Q=([0-9A-F]{2}) C=([0-9A-F]{8}) T=([0-9A-F]{8}) F=([0-9A-F]{8}) "
                   r"A=([0-9A-F]) S=([0-9A-F]{8}) Z=([0-9A-F]{4}) D=([0-9A-F]{2}) R=([0-9A-F]{4})")
RE_SD = re.compile(r"SD E=([0-9A-F]{4}) W=([0-9A-F]{4}) K=([0-9A-F]{2})")
RE_ZV = re.compile(r"ZV=([0-9A-F]{192})")
RE_K = re.compile(r"K seq=(\d+) fl=(\d+) gsum=(\d+) g=(\d+) s=(\d+) spi=(\S+)")
RE_E = re.compile(r"E seq=(\d+) err=(\S+)")


def vh_int(path: Path, name: str) -> int:
    m = re.search(rf"{name}\s*=\s*(-?\d+)", path.read_text(encoding="ascii", errors="ignore"))
    if not m:
        sys.exit(f"Không thấy {name} trong {path}")
    return int(m.group(1))


def open_port(name: str, baud: int):
    import serial
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = name, baud, 0.05
    s.dtr = False                                   # tránh tự reset board khi mở cổng
    s.rts = False
    s.open()
    return s


def reader(ser, q: queue.Queue, tag: str, stop: threading.Event, raw_log):
    buf = b""
    while not stop.is_set():
        try:
            chunk = ser.read(4096)
        except Exception as e:                       # noqa: BLE001
            q.put((tag, f"#READ_ERROR {e}"))
            return
        if not chunk:
            continue
        buf += chunk
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            txt = line.decode("ascii", errors="replace").strip()
            if txt:
                raw_log.write(f"{time.time():.3f} {tag} {txt}\n")
                q.put((tag, txt))


class Gui:
    """Cửa sổ demo khi phát lại video: khung đang gửi, cờ FPGA, nhãn thật, khớp Golden, dải thời gian."""
    W, H = 960, 640

    def __init__(self, run: Path, record: bool, mhz: float, color: bool = True, crop: bool = True, frame_step: int = 1):
        import cv2
        self.crop = crop
        self.cv2, self.mhz, self.paused, self.color = cv2, mhz, False, color
        self.cap, self.cap_idx, self.cap_path = None, -1, None
        self.rec, self.record, self.step = None, record, max(1, int(frame_step))
        self.rec_path = run / "video_replay.avi"
        self.src_fps = None
        self.stem, self.n_pairs, self.strip = "", 1, []
        self.t_last = time.time()

    def new_video(self, stem: str, n_pairs: int, path=None):
        self.stem, self.n_pairs, self.strip = stem, max(1, n_pairs), []
        if self.cap is not None:
            self.cap.release()
        self.cap, self.cap_idx, self.cap_path = None, -1, path
        if self.color and path is None:
            print("[MAU] Không biết đường dẫn file video → ảnh xám", flush=True)

    def color_frame(self, fi: int):
        """Khung màu gốc số fi của file video (đọc tuần tự), cắt giữa đúng tỉ lệ 4:3 như ảnh cảm biến."""
        cv2 = self.cv2
        if not self.color or self.cap_path is None:
            return None
        if self.cap is None or fi <= self.cap_idx:
            if self.cap is not None:
                self.cap.release()
            self.cap, self.cap_idx = cv2.VideoCapture(str(self.cap_path)), -1
            if not self.cap.isOpened():
                print(f"[MAU] KHÔNG mở được file video {self.cap_path} (có tồn tại: {Path(self.cap_path).exists()}) "
                      "→ hiện ảnh xám 160×120. Kiểm tra đường dẫn dataset trong config.", flush=True)
                self.color = False
                return None
            print(f"[MAU] Đọc khung màu từ {self.cap_path}", flush=True)
            f = self.cap.get(cv2.CAP_PROP_FPS)
            if f and 1 <= f <= 120 and self.src_fps is None:
                self.src_fps = float(f)
            self.checked_gray = False
        while self.cap_idx < fi - 1:                     # bỏ qua nhanh tới khung fi
            if not self.cap.grab():
                return None
            self.cap_idx += 1
        ok, fr = self.cap.read()
        if not ok:
            print(f"[MAU] Không đọc được khung {fi} của {self.cap_path} → ảnh xám", flush=True)
            return None
        self.cap_idx += 1
        if not getattr(self, "checked_gray", True):
            self.checked_gray = True
            d = int(np.abs(fr.astype(np.int16) - fr[..., :1].astype(np.int16)).max()) if fr.ndim == 3 else 0
            if d <= 8:
                print(f"[MAU] Bản thân file {Path(self.cap_path).name} là video XÁM (camera đen trắng, chênh kênh màu "
                      f"tối đa {d}) – không có màu để hiện.", flush=True)
        if not self.crop:                                # pipeline co giãn cả khung (không cắt)
            return cv2.resize(fr, (480, 360), interpolation=cv2.INTER_AREA)
        h, w = fr.shape[:2]
        tw = min(w, int(round(h * 4 / 3)))
        th = min(h, int(round(tw * 3 / 4)))
        x0, y0 = (w - tw) // 2, (h - th) // 2
        return cv2.resize(fr[y0:y0 + th, x0:x0 + tw], (480, 360), interpolation=cv2.INTER_AREA)

    def rec_wanted(self):
        return self.record and self.color and (self.rec is None or getattr(self, "rec_per_frame", False))

    def _text(self, img, s, x, y, scale=0.6, col=(30, 30, 30), th=1):
        cv2 = self.cv2
        while scale > 0.35 and x + cv2.getTextSize(s, cv2.FONT_HERSHEY_SIMPLEX, scale, th)[0][0] > self.W - 10:
            scale -= 0.02
        cv2.putText(img, s, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, col, th, cv2.LINE_AA)

    def update(self, raw, row, rows):
        cv2 = self.cv2
        W, H = self.W, self.H
        a, y, al = row["A_fpga"], row["true_label"], row.get("alarm", row["A_fpga"])
        self.strip.append((a, al, y))
        img = np.full((H, W, 3), 245, np.uint8)
        red, green, grey = (0, 0, 220), (40, 160, 40), (150, 150, 150)
        col = red if al else green
        # khung để hiện/quay: khi quay video và có file gốc thì lấy CẢ các khung giữa frame1 → frame2
        # (video quay chạy đúng tốc độ thật, mượt như video gốc); không thì chỉ khung frame2
        f1, f2 = int(row["frame1"]), int(row["frame2"])
        want = list(range(max(f1 + 1, f2 - 8), f2 + 1)) if (self.rec_wanted() and f2 - f1 <= 12) else [f2]
        bigs = [b for b in (self.color_frame(fi) for fi in want) if b is not None]
        if not bigs:
            bigs = [cv2.resize(cv2.cvtColor(raw, cv2.COLOR_GRAY2BGR), (480, 360), interpolation=cv2.INTER_CUBIC)]
        cv2.rectangle(img, (16, 16), (504, 384), col, 6)
        self._text(img, "BAT THUONG" if al else "BINH THUONG", 530, 52, 1.2, col, 3)
        self._text(img, "canh bao (sau loc)", 532, 76, 0.45, (110, 110, 110))
        lab = "-" if y < 0 else ("BAT THUONG" if y else "binh thuong")
        n = len(rows)
        n_ok = sum(r["match"] for r in rows)
        lv = [r for r in rows if r["true_label"] >= 0]
        acc = f"{100 * sum(1 for r in lv if r['A_fpga'] == r['true_label']) / len(lv):.1f} %" if lv else "-"
        acc_al = f"{100 * sum(1 for r in lv if r.get('alarm', r['A_fpga']) == r['true_label']) / len(lv):.1f} %" if lv else "-"
        lines = [(f"Nhan that (IITB): {lab}", (red if y == 1 else (60, 60, 60))),
                 (f"Co FPGA cap nay: {'BAT THUONG' if a else 'binh thuong'}", red if a else (60, 60, 60)),
                 (f"f_q FPGA = {row['F_fpga']}", None),
                 (f"Golden Python  = {row['F_golden']}", None),
                 (f"Khop Golden: {'KHOP' if row['match'] else 'SAI'}" +
                  ("  (48 z)" if row["z_mismatch"] == 0 else ""), green if row["match"] else red),
                 (f"Tong: khop {n_ok} / {n} cap", None),
                 (f"Accuracy: tho {acc}  |  sau loc {acc_al}", None),
                 (f"HS tren FPGA: {row['C'] / self.mhz / 1000:.2f} ms @ {self.mhz:g} MHz", None),
                 ("Luong: PC doc video -> ESP32 Gauss -> SPI", (90, 90, 90)),
                 ("       -> FPGA HS+FEAT+OC-SVM -> PC", (90, 90, 90))]
        for i, (s, c) in enumerate(lines):
            self._text(img, s, 530, 104 + 27 * i, 0.58, c or (30, 30, 30))
        # ba dải thời gian của video hiện tại (đỏ = bất thường)
        x0, x1 = 150, W - 20
        self._text(img, f"Video {self.stem}  (do = bat thuong, xanh = binh thuong)", 20, 412, 0.5, (90, 90, 90))
        rows_y = [(424, "Co FPGA (tho)"), (480, "Canh bao loc"), (536, "Nhan that")]
        for (yy, name) in rows_y:
            self._text(img, name, 20, yy + 27, 0.5, (60, 60, 60))
        for k, (ak, alk, yk) in enumerate(self.strip):
            xa = x0 + int(k * (x1 - x0) / self.n_pairs)
            xb = max(xa + 1, x0 + int((k + 1) * (x1 - x0) / self.n_pairs))
            img[424:464, xa:xb] = red if ak else green
            img[480:520, xa:xb] = red if alk else green
            img[536:576, xa:xb] = grey if yk < 0 else (red if yk else green)
        for (yy, _) in rows_y:
            cv2.rectangle(img, (x0, yy), (x1, yy + 40), (120, 120, 120), 1)
        self._text(img, time.strftime("%Y-%m-%d %H:%M:%S"), W - 190, H - 12, 0.45, (120, 120, 120))
        if self.record and self.rec is None:
            per_pair = len(bigs) > 1
            fps = (self.src_fps or 25.0) if per_pair else (self.src_fps or 25.0) / self.step
            self.rec = cv2.VideoWriter(str(self.rec_path), cv2.VideoWriter_fourcc(*"MJPG"), fps, (W, H))
            try:
                self.rec.set(cv2.VIDEOWRITER_PROP_QUALITY, 100)
            except Exception:                            # noqa: BLE001
                pass
            self.rec_per_frame = per_pair
            print(f"Quay {self.rec_path} ở {fps:g} khung/s (= tốc độ thật của video gốc)", flush=True)
        for j, big in enumerate(bigs):
            fr = img.copy()
            fr[20:380, 20:500] = big
            fr[356:380, 20:500] = (fr[356:380, 20:500] * 0.4).astype(np.uint8)    # nền tối cho dòng chữ
            self._text(fr, f"{self.stem}  khung {f2 - len(bigs) + 1 + j}  (cap {f1}->{f2})", 26, 372, 0.5, (255, 255, 255))
            if self.rec is not None:
                if self.rec_per_frame or j == len(bigs) - 1:
                    self.rec.write(fr)
        cv2.imshow("DATN - phat lai video IITB qua ESP32 + FPGA", fr)
        while True:
            k = cv2.waitKey(1 if not self.paused else 50) & 0xFF
            if k == ord("q"):
                return "quit"
            if k == ord(" "):
                self.paused = not self.paused
            if not self.paused:
                return None

    def close(self):
        if self.cap is not None:
            self.cap.release()
        if self.rec is not None:
            self.rec.release()
            print(f"Video: {self.rec_path}")
        try:
            self.cv2.destroyAllWindows()
        except Exception:                                # noqa: BLE001
            pass


def main():
    ap = base_parser(__doc__)
    ap.add_argument("--esp", required=True, help="cổng COM của ESP32-S3 (console UART0)")
    ap.add_argument("--fpga", required=True, help="cổng COM UART của Tang Nano 20K (BL616)")
    ap.add_argument("--esp-baud", type=int, default=2_000_000)
    ap.add_argument("--fpga-baud", type=int, default=115_200)
    ap.add_argument("--pairs", type=int, default=1000, help="tổng số cặp cần so (0 = không giới hạn)")
    ap.add_argument("--per-video", type=int, default=100, help="số cặp liên tiếp lấy từ mỗi video (0 = cả video từ đầu)")
    ap.add_argument("--videos", nargs="*", default=None, help="tên video Test (mặc định chọn ngẫu nhiên)")
    ap.add_argument("--hs-dir", default=None, help="mặc định <output_dir>/fpga/hs (tham số của bitstream)")
    ap.add_argument("--timeout", type=float, default=3.0, help="giây chờ mỗi phản hồi")
    ap.add_argument("--quiet", action="store_true", help="chỉ in cặp lệch")
    ap.add_argument("--require-z", action="store_true",
                    help="bản kiểm chứng DUMP_Z: cặp nào thiếu ZV= (48 z) thì tính là KHÔNG khớp")
    ap.add_argument("--show", action="store_true", help="cửa sổ demo: khung hình, cờ FPGA, nhãn thật, dải thời gian")
    ap.add_argument("--record", action="store_true", help="(kèm --show) quay cửa sổ thành video_replay.avi")
    ap.add_argument("--gray", action="store_true", help="cửa sổ hiện ảnh xám 160×120 đã gửi thay vì khung màu gốc")
    ap.add_argument("--smooth", type=int, default=7, help="cửa sổ lọc cảnh báo (như 27_demo_monitor)")
    ap.add_argument("--enter", type=int, default=None, help="bật cảnh báo khi ≥ số cờ này trong cửa sổ (mặc định 5/7)")
    ap.add_argument("--exit", type=int, default=None, help="tắt cảnh báo khi ≤ số cờ này (mặc định 1/7)")
    args = ap.parse_args()
    if args.pairs <= 0:
        args.pairs = 10 ** 9
    if args.record and not args.show:
        args.show = True
    cfg, out, log, rng = setup(args, "34_fpga_replay")
    det = load_detector_checked(cfg, out)
    fx = FixedPointOCSVM(det, cfg.fpga)
    hs_dir = Path(args.hs_dir) if args.hs_dir else out / "fpga" / "hs"
    vh = hs_dir / "hs_params.vh"
    n_iter, inv_frac = vh_int(vh, "N_ITER"), vh_int(vh, "INV_FRAC")
    p = HQ.DEFAULT if n_iter == HQ.DEFAULT.n_iter else HQ.HSParams(n_iter=n_iter)
    sc = HQ.ScalerQ(HX.read_mem(hs_dir / "mem" / "scaler_mean_q.mem", 16),
                    HX.read_mem(hs_dir / "mem" / "scaler_inv_q.mem", 16, signed=False), inv_frac)
    lut = HX.read_mem(hs_dir / "mem" / "log_lut.mem", 12, signed=False)
    dark = float(cfg.preprocess.get("skip_dark_below", 0) or 0)

    # ------------------------------------------------------------- chọn video và đoạn khung
    splits = get_splits(cfg, out, rng)
    items = list(splits["test"])
    by_stem = {it.path.stem: it for it in items}
    if args.videos:
        def find(v):                                     # 228 ↔ 000228 / video_228 … (khớp theo số)
            if v in by_stem:
                return by_stem[v]
            if v.isdigit():
                hit = [it for st, it in by_stem.items() if re.sub(r"\D", "", st) and int(re.sub(r"\D", "", st)) == int(v)]
                if len(hit) == 1:
                    return hit[0]
            return None
        found = {v: find(v) for v in args.videos}
        miss = [v for v, it in found.items() if it is None]
        if miss:
            sys.exit(f"Không có trong tập Test: {miss}\nVí dụ tên trong tập Test: {sorted(by_stem)[:10]}")
        chosen = [found[v] for v in args.videos]
    else:
        n_vid = max(1, int(np.ceil(args.pairs / args.per_video))) + 5     # dư vài video: video ngắn bị bỏ
        r = np.random.default_rng(int(cfg.project.seed) + 34)
        chosen = [items[k] for k in np.sort(r.choice(len(items), min(n_vid, len(items)), replace=False))]
    from ocsvm_fpga.data.dataset import find_annotation, load_frame_labels  # noqa: F401
    from ocsvm_fpga.config import resolve_path
    from ocsvm_fpga.features.pipeline import video_info

    def labels_of(item):
        lab = item.labels_path if item.labels_path and Path(item.labels_path).exists() else None
        if lab is None and cfg.data.get("annotation_dir"):
            lab = find_annotation(item.path.stem, resolve_path(cfg.data.annotation_dir),
                                  cfg.data.annotation_patterns, item.path)
        return load_frame_labels(lab, int(video_info(item.path)["frames"])) if lab else None

    plan = []                                       # (stem, [(frame_idx, raw)], labels)
    r2 = np.random.default_rng(int(cfg.project.seed) + 340)
    for it in chosen:
        frames, fidx = load_sensor_frames(it, cfg, out)
        n_need = args.per_video + 1 if args.per_video > 0 else len(frames)
        if len(frames) < max(2, n_need):
            log.warning(f"{it.path.stem}: chỉ có {len(frames)} khung – bỏ qua")
            continue
        s0 = int(r2.integers(0, len(frames) - n_need + 1)) if args.per_video > 0 else 0
        plan.append((it.path.stem, list(zip(fidx[s0:s0 + n_need], frames[s0:s0 + n_need])), labels_of(it)))
        if not args.videos and sum(len(f) - 1 for _, f, _ in plan) >= args.pairs:
            break
    total = sum(len(f) - 1 for _, f, _ in plan)
    log.info(f"{len(plan)} video, tối đa {total} cặp · N_ITER {n_iter} · ρ_q {fx.rho_q} · THR_q {fx.thr_q}")

    run = out / "replay" / time.strftime("%Y%m%d_%H%M%S")
    run.mkdir(parents=True, exist_ok=True)
    raw_log = open(run / "log.txt", "w", encoding="utf-8", buffering=1)

    # ------------------------------------------------------------- cổng
    if args.esp.upper() == args.fpga.upper():
        sys.exit("--esp và --fpga phải là HAI cổng khác nhau (python -m serial.tools.list_ports -v để xem)")
    esp, fpga = open_port(args.esp, args.esp_baud), open_port(args.fpga, args.fpga_baud)
    qe, qf, stop = queue.Queue(), queue.Queue(), threading.Event()
    for ser, q, tag in ((esp, qe, "ESP"), (fpga, qf, "FPGA")):
        threading.Thread(target=reader, args=(ser, q, tag, stop, raw_log), daemon=True).start()
    log.info("Chờ ESP32 sẵn sàng (#INFO san_sang=1) – nếu đứng ở đây, nhấn nút RST/EN trên ESP32 …")
    t_end, ready, k_mhz = time.time() + 15, False, 54.0
    while time.time() < t_end and not ready:
        try:
            tag, line = qe.get(timeout=0.2)
        except queue.Empty:
            continue
        if "spi_init=" in line and "ESP_OK" not in line:
            sys.exit(f"ESP32 lỗi SPI: {line}")
        ready = "san_sang=1" in line
    if not ready:
        log.warning("Không thấy dòng san_sang từ ESP32 – vẫn thử gửi")
    while not qf.empty():                            # dòng SD … của FPGA (nếu vừa khởi động)
        _, line = qf.get()
        m = RE_SD.search(line)
        if m:
            k_mhz = float(int(m.group(3), 16))

    def wait(q, pred, timeout):
        t = time.time() + timeout
        while time.time() < t:
            try:
                _, line = q.get(timeout=0.05)
            except queue.Empty:
                continue
            res = pred(line)
            if res is not None:
                return res
        return None

    rows, seq = [], 0
    n_gauss_bad = 0
    gui = Gui(run, args.record, k_mhz, color=not args.gray,
                               crop=str(cfg.preprocess.get("aspect_mode", "crop")) == "crop",
                               frame_step=int(cfg.preprocess.get("frame_step", 1))) if args.show else None
    from collections import deque
    n_enter = args.enter if args.enter is not None else max(1, round(args.smooth * 5 / 7))
    n_exit = args.exit if args.exit is not None else max(0, round(args.smooth * 1 / 7))
    try:
        for stem, fr, labels in plan:
            reset, g_prev, i_prev = True, None, None
            hist, alarm = deque(maxlen=max(1, args.smooth)), 0       # bộ lọc cảnh báo, làm mới mỗi video
            if gui:
                gui.new_video(stem, len(fr) - 1, by_stem[stem].path)
            for fi, raw in fr:
                raw = np.ascontiguousarray(raw, np.uint8)
                g = PQ.gauss5_q(raw)
                if dark > 0 and PQ.frame_sum(g) < dark * g.size:
                    reset, g_prev = True, None                 # khung tối: như pipeline train và demo
                    continue
                seq = seq % 255 + 1
                flags = 1 if reset else 0
                payload = raw.tobytes()
                ck = int(np.sum(raw, dtype=np.uint64)) & 0xFFFF
                esp.write(b"RPLY" + bytes([seq, flags]) + payload + ck.to_bytes(2, "little"))
                k = wait(qe, lambda ln: (RE_K.search(ln) or RE_E.search(ln)), args.timeout)
                if k is None or k.re is RE_E or int(k.group(1)) != seq:
                    log.warning(f"{stem} khung {fi}: ESP không xác nhận ({k.group(0) if k else 'hết giờ'}) → bắt đầu lại đoạn")
                    reset, g_prev = True, None
                    continue
                gsum_esp, gsum_ref = int(k.group(3)), PQ.frame_sum(g)
                if gsum_esp != gsum_ref:
                    n_gauss_bad += 1
                if g_prev is not None and not reset:
                    tr = HQ.features_q(g_prev, g, p, sc, lut)
                    fqa = fx.decision_q(np.atleast_2d(tr.z_q))
                    e = {"F": int(fqa[0]) & 0xFFFFFFFF, "A": int(fx.anomaly_q(fqa)[0]), "S": int(tr.hs.n_sat),
                         "Z": int(np.sum(tr.z_q)) & 0xFFFF}
                    h = wait(qf, lambda ln: (lambda m: m if m and int(m.group(2), 16) == seq else None)(RE_HS.search(ln)),
                             args.timeout)
                    if h is None:
                        log.warning(f"{stem} khung {i_prev}→{fi}: FPGA không trả kết quả (seq {seq})")
                        reset, g_prev = True, None
                        continue
                    got = {"F": int(h.group(5), 16), "A": int(h.group(6), 16), "S": int(h.group(7), 16),
                           "Z": int(h.group(8), 16)}
                    mz = RE_ZV.search(h.string)
                    z_bad = -1                                  # -1: bitstream không in 48 z
                    if mz:
                        hx = mz.group(1)
                        zf = [int(hx[4 * j:4 * j + 4], 16) for j in range(48)]
                        zf = [v - 0x10000 if v & 0x8000 else v for v in zf]
                        z_bad = int(sum(1 for a_, b_ in zip(zf, np.asarray(tr.z_q).reshape(-1)) if a_ != int(b_)))
                    z_ok = (z_bad == 0) if args.require_z else (z_bad <= 0)   # −1 = không có ZV=
                    ok = got == e and gsum_esp == gsum_ref and z_ok
                    f_signed = got["F"] - (1 << 32) if got["F"] & 0x80000000 else got["F"]
                    y = int(labels[min(int(fi), len(labels) - 1)]) if labels is not None else -1
                    row = {"i": len(rows), "video": stem, "frame1": int(i_prev), "frame2": int(fi), "seq": seq,
                           "gsum_esp": gsum_esp, "gsum_golden": gsum_ref,
                           "F_fpga": f_signed, "F_golden": int(fqa[0]), "A_fpga": got["A"], "A_golden": e["A"],
                           "S_fpga": got["S"], "S_golden": e["S"], "Z_fpga": got["Z"], "Z_golden": e["Z"],
                           "true_label": y, "match": int(ok), "C": int(h.group(3), 16), "T": int(h.group(4), 16),
                           "D": int(h.group(9), 16), "z_mismatch": z_bad}
                    hist.append(got["A"])
                    if alarm == 0 and sum(hist) >= n_enter:
                        alarm = 1
                    elif alarm == 1 and sum(hist) <= n_exit:
                        alarm = 0
                    row["alarm"] = alarm
                    rows.append(row)
                    if gui and gui.update(raw, row, rows) == "quit":
                        raise KeyboardInterrupt
                    if not args.quiet or not ok:
                        res = "-" if y < 0 else ("Success" if got["A"] == y else "Fail   ")
                        print(f"Pair {row['i']} [{stem} {i_prev}->{fi}]: {res}, Prediction = {got['A']}, "
                              f"True Label = {y}, f_q = {f_signed}, Golden = {'KHOP' if ok else 'SAI'}")
                    if len(rows) >= args.pairs:
                        break
                reset, g_prev, i_prev = False, g, fi
            if len(rows) >= args.pairs:
                break
    except KeyboardInterrupt:
        log.warning("Dừng giữa chừng – vẫn ghi báo cáo cho các cặp đã chạy")
    finally:
        if gui:
            gui.close()
        stop.set()
        time.sleep(0.1)
        esp.close(); fpga.close(); raw_log.close()

    # ------------------------------------------------------------- báo cáo
    if not rows:
        sys.exit("Không có cặp nào – xem log.txt")
    import csv
    with open(run / "pairs.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    n = len(rows)
    n_ok = sum(r["match"] for r in rows)
    lab = [r for r in rows if r["true_label"] >= 0]
    tp = sum(1 for r in lab if r["A_fpga"] == 1 and r["true_label"] == 1)
    fp = sum(1 for r in lab if r["A_fpga"] == 1 and r["true_label"] == 0)
    tn = sum(1 for r in lab if r["A_fpga"] == 0 and r["true_label"] == 0)
    fn = sum(1 for r in lab if r["A_fpga"] == 0 and r["true_label"] == 1)
    c_med = float(np.median([r["C"] for r in rows]))
    t_med = float(np.median([r["T"] for r in rows]))
    drops = max(r["D"] for r in rows) - min(r["D"] for r in rows)
    vids = sorted({r["video"] for r in rows})
    fields = {k: sum(1 for r in rows if r[f"{k}_fpga"] != r[f"{k}_golden"]) for k in ("F", "A", "S", "Z")}
    zrows = [r for r in rows if r["z_mismatch"] >= 0]
    n_noz = sum(1 for r in rows if r["z_mismatch"] < 0)
    z_line = (f"| **Từng phần tử z (48 / cặp, bitstream DUMP_Z)** | **{sum(1 for r in zrows if r['z_mismatch'] == 0)} / "
              f"{len(zrows)} cặp khớp cả 48 z ({48 * len(zrows)} giá trị)** |" if zrows
              else "| Từng phần tử z | không đối chiếu (bitstream demo chỉ in Σz) |")
    lines = [
        "# Phát lại IITB qua hệ thống thật – so với Golden Model", "",
        f"Chạy {time.strftime('%Y-%m-%d %H:%M')} · {n} cặp từ {len(vids)} video Test ({', '.join(vids)}) · "
        f"bitstream hs_system ở {k_mhz:g} MHz · N_ITER {n_iter}", "",
        "Đường dữ liệu: PC → UART → ESP32-S3 (Gauss, preproc.c) → SPI → Tang Nano 20K "
        "(DERIV → HS → FEAT → OC-SVM) → UART → PC. Golden: cùng khung, Gauss → HS → FEAT → z → OC-SVM bằng Python số nguyên.", "",
        "| Chỉ số | Kết quả |", "|---|---|",
        f"| **Khớp Golden (Σ Gauss, f_q, cờ, bão hòa, Σz)** | **{n_ok} / {n}** |",
        z_line,
        f"| Cặp thiếu dữ liệu ZV= | {n_noz}" + (" (tính là KHÔNG khớp, --require-z)" if args.require_z else "") + " |",
        f"| Lệch Σ ảnh Gauss (ESP ↔ Golden) | {n_gauss_bad} khung |",
        f"| Lệch F / A / S / Z (FPGA ↔ Golden) | {fields['F']} / {fields['A']} / {fields['S']} / {fields['Z']} |",
        f"| Cặp bị FPGA bỏ trong lúc chạy | {drops} |",
        f"| Thời gian HS trên board (trung vị) | {c_med:.0f} chu kỳ = {c_med / k_mhz / 1000:.2f} ms |",
        f"| Từ lúc đủ ảnh tới kết quả (trung vị) | {t_med:.0f} chu kỳ = {t_med / k_mhz / 1000:.2f} ms |"]
    if lab:
        acc = (tp + tn) / len(lab)
        acc_al = sum(1 for r in lab if r["alarm"] == r["true_label"]) / len(lab)
        lines += [f"| Accuracy theo khung so với nhãn thật (cờ FPGA thô) | {tp + tn}/{len(lab)} = {acc:.1%} |",
                  f"| TP / FP / TN / FN (cờ thô) | {tp} / {fp} / {tn} / {fn} |",
                  f"| Accuracy sau lọc cảnh báo ({args.smooth} cặp, bật ≥ {n_enter}, tắt ≤ {n_exit}) | {acc_al:.1%} |"]
    if len(vids) > 1 or args.per_video <= 0:
        lines += ["", "### Theo từng video", "",
                  "| Video | Cặp | Khớp Golden | Cặp bất thường thật | FPGA báo (thô) | Accuracy thô | Accuracy sau lọc |",
                  "|---|---|---|---|---|---|---|"]
        for v in vids:
            rv = [r for r in rows if r["video"] == v]
            lv = [r for r in rv if r["true_label"] >= 0]
            acc_v = f"{sum(1 for r in lv if r['A_fpga'] == r['true_label']) / len(lv):.1%}" if lv else "–"
            acc_a = f"{sum(1 for r in lv if r['alarm'] == r['true_label']) / len(lv):.1%}" if lv else "–"
            lines.append(f"| {v} | {len(rv)} | {sum(r['match'] for r in rv)} / {len(rv)} | "
                         f"{sum(1 for r in lv if r['true_label'] == 1)} | {sum(r['A_fpga'] for r in rv)} | {acc_v} | {acc_a} |")
    lines += ["", "Accuracy là theo từng cặp khung tại ngưỡng THR_q (chưa làm trơn, chưa qua logic cảnh báo); "
              "Golden cho đúng con số này khi khớp hoàn toàn.",
              "", "Chi tiết từng cặp: pairs.csv · nhật ký hai cổng: log.txt"]
    (run / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print()
    print(f"------ Final Result for {n} Pairs (IITB Test, PC -> ESP32 -> FPGA) ------")
    what = "Gauss + f_q + co + bao hoa + 48 z" if zrows else "Gauss + f_q + co + bao hoa + sum z"
    print(f"Khop Golden Model ({what}) : {n_ok} / {n}")
    if lab:
        print(f"Accuracy so voi nhan that : {tp + tn} / {len(lab)} = {100 * (tp + tn) / len(lab):.1f} %")
        print(f"TP = {tp}  FP = {fp}  TN = {tn}  FN = {fn}")
    print(f"HS tren board: {c_med:.0f} chu ky = {c_med / k_mhz / 1000:.2f} ms @ {k_mhz:g} MHz")
    print(("PASS" if n_ok == n else "FAIL") + f": bao cao -> {run / 'report.md'}")


if __name__ == "__main__":
    main()