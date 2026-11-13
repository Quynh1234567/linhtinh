#!/usr/bin/env python3
"""GHI DỮ LIỆU CẢNH THẬT (camera OV5640 tại chỗ demo) ĐỂ HUẤN LUYỆN LẠI – qua đúng đường phần cứng.

Hệ thống chạy như demo:  OV5640 → ESP32-S3 (firmware esp32_demo: Gauss) → SPI → Tang Nano 20K
(bitstream hs_system_z, DUMP_Z = 1: DERIV → HS 64 vòng → FEAT → OC-SVM) ──UART──► PC.
Mỗi cặp ảnh FPGA in một dòng "HS … ZV=<48 z>". Script này đọc 48 z, KHÔI PHỤC véc-tơ đặc trưng
trước chuẩn hóa (g = μ + z / inv, đúng nghịch đảo khối FEAT) và lưu lại → đặc trưng khớp bit với
phần cứng, đúng 80 ms giữa hai ảnh, không cần ghi video hay firmware mới.

    python scripts/29_record_scene.py --fpga COM9 --label normal --minutes 5
    python scripts/29_record_scene.py --fpga COM9 --label normal --minutes 2 --name buoi_chieu
    python scripts/29_record_scene.py --fpga COM9 --label abnormal --minutes 1      # diễn chạy / đuổi nhau
    python scripts/29_record_scene.py --fpga COM9 --label normal --minutes 2 --test # để riêng đánh giá

Cổng: --fpga là cổng COM của Tang Nano 20K (BL616, 115 200 baud) – KHÔNG phải cổng ESP32.
ESP32 chỉ cần cắm nguồn và chạy esp32_demo (không mở cổng của nó bằng chương trình khác cũng được).

Kết quả: data/scene/<label>/<tên>.npz  (+ .csv đọc được bằng Excel)
    Xq   (N, 48) int16  đặc trưng log Q5.10 khôi phục   ← dùng để huấn luyện (giống cache grid_flow_hw)
    zq   (N, 48) int16  z FPGA in ra
    sat  (N,)    bool   cặp có z bão hòa (±32 σ) → g không khôi phục được chính xác
    f_q, flag, seq, t   điểm OC-SVM, cờ của MÔ HÌNH ĐANG NẠP, số thứ tự, thời điểm (s)
Lưu ý: phải dùng đúng scaler của bitstream đang nạp (mặc định outputs_hw/fpga/hs/mem/scaler_*_q.mem).
"""
import argparse
import csv
import json
import re
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
RE_HS = re.compile(r"HS N=([0-9A-F]{4}) Q=([0-9A-F]{2}) C=([0-9A-F]{8}) T=([0-9A-F]{8}) F=([0-9A-F]{8}) "
                   r"A=([0-9A-F]) S=([0-9A-F]{8}) Z=([0-9A-F]{4}) D=([0-9A-F]{2}) R=([0-9A-F]{4})")
RE_ZV = re.compile(r"ZV=([0-9A-F]{192})")
RE_SD = re.compile(r"SD E=([0-9A-F]{4}) W=([0-9A-F]{4}) K=([0-9A-F]{2})")
RE_FRAC = re.compile(r"(\d+)\s*bit\s*l")


def read_mem(path: Path) -> tuple[np.ndarray, list[str]]:
    vals, comments = [], []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        s = line.strip()
        if not s:
            continue
        if s.startswith("//"):
            comments.append(s)
            continue
        vals.append(int(s.split()[0], 16))
    return np.array(vals, np.int64), comments


def load_scaler(mem_dir: Path):
    pm, pi = mem_dir / "scaler_mean_q.mem", mem_dir / "scaler_inv_q.mem"
    if not (pm.exists() and pi.exists()):
        return None
    mean, _ = read_mem(pm)
    inv, com = read_mem(pi)
    mean = np.where(mean & 0x8000, mean - 0x10000, mean)          # Q5.10 có dấu 16 bit
    frac = None
    for c in com:
        m = RE_FRAC.search(c)
        if m:
            frac = int(m.group(1))
    if frac is None:                                               # thử đọc hs_params.vh cạnh đó
        for vh in (mem_dir.parent / "hs_params.vh", mem_dir / "hs_params.vh"):
            if vh.exists():
                m = re.search(r"INV_FRAC\s*=\s*(\d+)", vh.read_text(encoding="utf-8", errors="replace"))
                if m:
                    frac = int(m.group(1))
    if frac is None:
        sys.exit(f"Không đọc được số bit lẻ của 1/σ trong {pi} – thêm --inv-frac")
    if len(mean) != 48 or len(inv) != 48:
        sys.exit(f"scaler phải có 48 giá trị (đọc được {len(mean)}, {len(inv)})")
    return mean, inv, frac


def find_mem_dir(arg: str | None) -> Path:
    cands = [Path(arg)] if arg else [ROOT / "outputs_hw" / "fpga" / "hs" / "mem",
                                     ROOT / "fpga_gowin" / "hs_system_z" / "src",
                                     ROOT / "fpga_gowin" / "hs_system" / "src"]
    for c in cands:
        c = c if c.is_absolute() else (ROOT / c)
        if (c / "scaler_mean_q.mem").exists():
            return c
    sys.exit("Không tìm thấy scaler_mean_q.mem / scaler_inv_q.mem. Chỉ đường dẫn bằng --mem-dir "
             "(thư mục mem của ĐÚNG bitstream đang nạp, vd outputs_hw/fpga/hs/mem).")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--fpga", default=None, help="cổng COM của Tang Nano 20K (BL616)")
    ap.add_argument("--fpga-baud", type=int, default=115200)
    ap.add_argument("--port", default=None, help="(tương thích launcher cũ) – coi như --fpga")
    ap.add_argument("--baud", type=int, default=None, help="(tương thích launcher cũ) – bỏ qua, FPGA luôn 115 200")
    ap.add_argument("--label", choices=["normal", "abnormal"], required=True)
    ap.add_argument("--minutes", type=float, default=5.0)
    ap.add_argument("--name", default=None, help="tên file (mặc định ngày-giờ)")
    ap.add_argument("--test", action="store_true", help="ghi vào data/scene/test/<label>/ – chỉ để đánh giá")
    ap.add_argument("--mem-dir", default=None, help="thư mục chứa scaler_mean_q.mem, scaler_inv_q.mem của bitstream")
    ap.add_argument("--out", default="data/scene")
    a = ap.parse_args()

    port = a.fpga or a.port
    if not port:
        sys.exit("Thiếu --fpga COMx (cổng của Tang Nano 20K). Xem cổng: python -m serial.tools.list_ports -v")
    if a.port and not a.fpga:
        print(f"! --port {a.port} được dùng làm cổng FPGA. Nếu đó là cổng ESP32 thì sẽ không nhận được dòng HS.")
    if a.baud and a.baud != a.fpga_baud:
        print(f"! Bỏ qua --baud {a.baud}: UART của FPGA chạy {a.fpga_baud} baud.")
    try:
        import serial
    except ImportError:
        sys.exit("Thiếu thư viện: pip install pyserial")

    mem_dir = find_mem_dir(a.mem_dir)
    mean_q, inv_q, inv_frac = load_scaler(mem_dir)
    print(f"Scaler: {mem_dir}  (1/σ có {inv_frac} bit lẻ)")

    sub = Path(a.out) / ("test" if a.test else "") / a.label
    out_dir = sub if sub.is_absolute() else ROOT / sub
    out_dir.mkdir(parents=True, exist_ok=True)
    name = a.name or datetime.now().strftime("%Y%m%d_%H%M%S")
    npz_path, csv_path = out_dir / f"{name}.npz", out_dir / f"{name}.csv"
    if npz_path.exists():
        sys.exit(f"Đã có {npz_path} – đổi --name")

    ser = serial.Serial(port, a.fpga_baud, timeout=0.2)
    print(f"Đang nghe {port} @ {a.fpga_baud}. Nhãn '{a.label}', {a.minutes:g} phút. Ctrl+C để dừng sớm.")
    if a.label == "normal":
        print("→ Chỉ đi lại BÌNH THƯỜNG trong khung hình (đi qua, đi lại, đứng nói chuyện…).")
    else:
        print("→ Diễn hành vi BẤT THƯỜNG: chạy đột ngột, đuổi nhau, vung tay, ngã…")

    zs, gs, sats, fq, flag, seq, ts, drops = [], [], [], [], [], [], [], []
    n_hs_noz, buf, t0 = 0, b"", time.time()
    t_end = t0 + 60.0 * a.minutes
    last_print = t0
    fcsv = open(csv_path, "w", newline="", encoding="utf-8")
    w = csv.writer(fcsv)
    w.writerow(["t_s", "seq", "f_q", "flag", "sat"] + [f"g{i}" for i in range(48)])
    try:
        while time.time() < t_end:
            buf += ser.read(4096)
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                s = line.decode("ascii", errors="replace").strip()
                m = RE_SD.search(s)
                if m:
                    e = int(m.group(1), 16)
                    print(f"FPGA khởi động: SD E={m.group(1)} ({'SDRAM tốt' if e == 0 else 'LỖI SDRAM'}), "
                          f"{int(m.group(3), 16)} MHz")
                    continue
                h = RE_HS.search(s)
                if not h:
                    continue
                mz = RE_ZV.search(s)
                if not mz:
                    n_hs_noz += 1
                    if n_hs_noz == 20 and not zs:
                        sys.exit("FPGA không in ZV= (48 z). Nạp bitstream hs_system_z (DUMP_Z = 1) rồi chạy lại.")
                    continue
                hx = mz.group(1)
                z = np.array([int(hx[4 * j:4 * j + 4], 16) for j in range(48)], np.int64)
                z = np.where(z & 0x8000, z - 0x10000, z)
                sat = bool(np.any((z >= 32767) | (z <= -32768)))
                g = np.rint(mean_q + z * float(1 << inv_frac) / inv_q).astype(np.int64)
                g = np.clip(g, -32768, 32767)
                f = int(h.group(5), 16)
                f = f - (1 << 32) if f & 0x80000000 else f
                t = time.time() - t0
                zs.append(z); gs.append(g); sats.append(sat); fq.append(f)
                flag.append(int(h.group(6), 16)); seq.append(int(h.group(2), 16)); ts.append(t)
                drops.append(int(h.group(9), 16))
                w.writerow([f"{t:.3f}", seq[-1], f, flag[-1], int(sat)] + g.tolist())
            now = time.time()
            if now - last_print >= 5:
                n = len(zs)
                rate = n / max(now - t0, 1e-6)
                fr = np.mean(flag) if flag else 0.0
                print(f"  {now - t0:5.0f} s | {n} cặp ({rate:.1f}/s) | mô hình hiện tại gắn cờ {fr:.1%}"
                      f" | bão hòa {np.mean(sats) if sats else 0:.1%} | bỏ cặp (FPGA) {drops[-1] if drops else 0}")
                last_print = now
    except KeyboardInterrupt:
        print("\nDừng sớm.")
    finally:
        fcsv.close()
        ser.close()

    if not zs:
        csv_path.unlink(missing_ok=True)
        sys.exit("Không nhận được cặp nào. Kiểm tra: đúng cổng FPGA? ESP32 chạy esp32_demo? camera có hình?")
    np.savez_compressed(npz_path, Xq=np.array(gs, np.int16), zq=np.array(zs, np.int16), sat=np.array(sats),
                        f_q=np.array(fq, np.int64), flag=np.array(flag, np.uint8), seq=np.array(seq, np.int32),
                        t=np.array(ts, np.float64), label=a.label, test=a.test)
    meta = {"label": a.label, "test": a.test, "pairs": len(zs), "seconds": ts[-1],
            "pairs_per_s": len(zs) / max(ts[-1], 1e-6), "flag_rate_current_model": float(np.mean(flag)),
            "saturated_pairs": int(np.sum(sats)), "mem_dir": str(mem_dir), "inv_frac": inv_frac,
            "port": port, "recorded": datetime.now().isoformat(timespec="seconds")}
    npz_path.with_suffix(".json").write_text(json.dumps(meta, ensure_ascii=False, indent=1), encoding="utf-8")
    print(f"\nĐã lưu {len(zs)} cặp → {npz_path}")
    print(f"  Mô hình đang nạp gắn cờ bất thường {meta['flag_rate_current_model']:.1%} số cặp "
          f"({'tỉ lệ báo giả' if a.label == 'normal' else 'tỉ lệ phát hiện'} trên cảnh này)")
    if meta["saturated_pairs"]:
        print(f"  {meta['saturated_pairs']} cặp có z bão hòa ({meta['saturated_pairs'] / len(zs):.1%}) – "
              "chuyển động quá lớn so với IITB; nếu > 5 %, đặt camera xa hơn.")
    print("Tiếp theo: ghi thêm vài lượt rồi chạy  python scripts/36_train_scene.py --config configs/hw.yaml")


if __name__ == "__main__":
    main()
