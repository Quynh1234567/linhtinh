#!/usr/bin/env python3
"""DEMO 1 LAUNCHER – chạy toàn bộ quy trình demo bằng nút bấm, không cần nhớ lệnh.

Mở bằng cách nhấp đúp DEMO1_LAUNCHER.bat ở thư mục gốc project (hoặc: python scripts/demo1_launcher.py).
Mỗi nút gọi đúng một script / lệnh có sẵn của project; toàn bộ log hiện trong khung bên dưới.
Cấu hình (đường dẫn Vivado, Gowin, ESP-IDF, cổng COM, ngưỡng…) lưu trong demo1_launcher.json.
"""
import json
import os
import queue
import re
import shutil
import subprocess
import sys
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from tkinter.scrolledtext import ScrolledText

ROOT = Path(__file__).resolve().parents[1]
CFG_FILE = ROOT / "demo1_launcher.json"
PY = sys.executable
IS_WIN = os.name == "nt"

DEFAULTS = {
    "config": "configs/hw.yaml",
    "vivado_bin": r"C:\Xilinx\Vivado\2026.1\bin",
    "gowin_ide": r"C:\Gowin\Gowin_V1.9.11\IDE\bin\gw_ide.exe",
    "idf_export": r"C:\Espressif\frameworks\esp-idf-v5.3\export.bat",
    "sim": "xsim",
    "fpga_port": "",
    "esp_port": "",
    "threshold": "",
    "calib_sec": "60",
    "clk": "54",
    "nit": "0",
    "minutes": "0",
    "esp_baud": "921600",
    "rec_minutes": "10",
    "smooth": "7",
}


def load_cfg():
    cfg = dict(DEFAULTS)
    if CFG_FILE.exists():
        try:
            cfg.update(json.loads(CFG_FILE.read_text(encoding="utf-8")))
        except Exception:  # noqa: BLE001
            pass
    return cfg


def list_ports():
    try:
        from serial.tools import list_ports as lp
    except ImportError:
        return [], "Chưa cài pyserial: pip install pyserial"
    ports = sorted(lp.comports(), key=lambda p: p.device)
    return [f"{p.device} – {p.description}" for p in ports], ""


class Launcher(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("ĐATN – Demo 1 Launcher (OV5640 → ESP32-S3 → Tang Nano 20K)")
        self.geometry("1180x800")
        self.minsize(980, 640)
        self.cfg = load_cfg()
        self.vars = {k: tk.StringVar(value=str(v)) for k, v in self.cfg.items()}
        self.proc = None
        self.q = queue.Queue()
        self._build_ui()
        self.refresh_ports()
        self.after(100, self._drain)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ------------------------------------------------------------------ giao diện
    def _build_ui(self):
        style = ttk.Style(self)
        try:
            style.theme_use("vista" if IS_WIN else "clam")
        except tk.TclError:
            pass
        style.configure("Big.TButton", font=("Segoe UI", 10, "bold"), padding=6)
        style.configure("Step.TLabel", font=("Segoe UI", 10))
        style.configure("Head.TLabel", font=("Segoe UI", 12, "bold"))

        top = ttk.Frame(self, padding=(10, 8))
        top.pack(fill="x")
        ttk.Label(top, text="Cổng FPGA:").pack(side="left")
        self.cb_fpga = ttk.Combobox(top, width=34, textvariable=self.vars["fpga_port"])
        self.cb_fpga.pack(side="left", padx=(4, 12))
        ttk.Label(top, text="Cổng ESP32:").pack(side="left")
        self.cb_esp = ttk.Combobox(top, width=34, textvariable=self.vars["esp_port"])
        self.cb_esp.pack(side="left", padx=(4, 12))
        ttk.Button(top, text="↻ Quét cổng COM", command=self.refresh_ports).pack(side="left")
        ttk.Button(top, text="■ Dừng tác vụ", command=self.stop).pack(side="right")
        ttk.Button(top, text="💾 Lưu cài đặt", command=self.save_cfg).pack(side="right", padx=6)

        pan = ttk.PanedWindow(self, orient="vertical")
        pan.pack(fill="both", expand=True, padx=10, pady=(0, 8))
        nb = ttk.Notebook(pan)
        pan.add(nb, weight=3)
        logf = ttk.Frame(pan)
        pan.add(logf, weight=2)
        self.log = ScrolledText(logf, height=14, font=("Consolas", 10), bg="#101418", fg="#D8E0E8",
                                insertbackground="white")
        self.log.pack(fill="both", expand=True)
        self.log.tag_config("ok", foreground="#7CFC9A")
        self.log.tag_config("err", foreground="#FF7B7B")
        self.log.tag_config("cmd", foreground="#7FB8FF")
        self.status = tk.StringVar(value="Sẵn sàng")
        ttk.Label(self, textvariable=self.status, anchor="w", padding=(10, 2)).pack(fill="x")

        tabs = [("0. Cài đặt", self._tab_settings), ("1. Kiểm tra máy", self._tab_check),
                ("2. Dữ liệu + Mô phỏng", self._tab_sim), ("3. FPGA (Gowin)", self._tab_fpga),
                ("4. ESP32-S3", self._tab_esp), ("5. CHẠY DEMO", self._tab_demo), ("6. Báo cáo", self._tab_report), ("7. Ghi dữ liệu", self._tab_record)]
        for name, fn in tabs:
            f = ttk.Frame(nb, padding=12)
            nb.add(f, text=name)
            fn(f)

    def _step(self, parent, row, text, button, cmd, note=""):
        ttk.Label(parent, text=text, style="Step.TLabel", wraplength=520, justify="left").grid(
            row=row, column=0, sticky="w", pady=4)
        ttk.Button(parent, text=button, style="Big.TButton", command=cmd).grid(row=row, column=1, sticky="ew",
                                                                              padx=10, pady=4)
        if note:
            ttk.Label(parent, text=note, foreground="#555", wraplength=330, justify="left").grid(
                row=row, column=2, sticky="w", pady=4)

    def _path_row(self, parent, row, label, key, kind="file"):
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", pady=4)
        ttk.Entry(parent, textvariable=self.vars[key], width=80).grid(row=row, column=1, sticky="ew", padx=6)

        def browse():
            p = filedialog.askdirectory() if kind == "dir" else filedialog.askopenfilename()
            if p:
                self.vars[key].set(p)
        ttk.Button(parent, text="Chọn…", command=browse).grid(row=row, column=2)

    def _tab_settings(self, f):
        ttk.Label(f, text="Khai báo đường dẫn MỘT LẦN, bấm “Lưu cài đặt”. Lần sau mở launcher là dùng được ngay.",
                  style="Head.TLabel").grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 10))
        self._path_row(f, 1, "Cấu hình mô hình (.yaml)", "config")
        self._path_row(f, 2, "Thư mục bin của Vivado", "vivado_bin", "dir")
        self._path_row(f, 3, "Gowin IDE (gw_ide.exe)", "gowin_ide")
        self._path_row(f, 4, "ESP-IDF export.bat", "idf_export")
        ttk.Label(f, text="Trình mô phỏng").grid(row=5, column=0, sticky="w", pady=4)
        ttk.Combobox(f, textvariable=self.vars["sim"], values=["xsim", "icarus"], width=12, state="readonly").grid(
            row=5, column=1, sticky="w", padx=6)
        ttk.Label(f, text="xsim = Vivado (khuyên dùng); icarus = Icarus Verilog", foreground="#555").grid(
            row=6, column=1, sticky="w", padx=6)
        f.columnconfigure(1, weight=1)

    def _tab_check(self, f):
        ttk.Label(f, text="Chạy mỗi khi đổi máy hoặc trước buổi demo.", style="Head.TLabel").grid(
            row=0, column=0, columnspan=3, sticky="w", pady=(0, 8))
        self._step(f, 1, "Kiểm tra Python, pyserial, Vivado, Gowin, ESP-IDF, các file cần thiết của project.",
                   "Kiểm tra môi trường", self.check_env)
        self._step(f, 2, "Liệt kê cổng COM đang cắm (Tang Nano 20K hiện 2 cổng: chọn cổng UART; ESP32 cắm cổng COM).",
                   "Liệt kê cổng COM", self.print_ports)
        self._step(f, 3, "Cài pyserial nếu chưa có.", "pip install pyserial",
                   lambda: self.run([PY, "-m", "pip", "install", "pyserial"], "Cài pyserial"))

    def _tab_sim(self, f):
        ttk.Label(f, text="Chỉ cần làm lại khi train lại mô hình hoặc sửa RTL.", style="Head.TLabel").grid(
            row=0, column=0, columnspan=3, sticky="w", pady=(0, 8))
        self._step(f, 1, "Xuất véc-tơ kiểm tra + hs_params.vh (04) và tham số OC-SVM vào rtl/build (15).",
                   "① Xuất dữ liệu", self.export_data, "≈ 1–3 phút")
        self._step(f, 2, "Mô phỏng nhanh từng khối HS (DERIV, HS_ITER 1 vòng, FEAT).", "② Mô phỏng nhanh",
                   lambda: self.sim([]), "phải in 24/24 PASS")
        self._step(f, 3, "Mô phỏng toàn khối HS + OC-SVM (1 ca).", "③ Mô phỏng engine",
                   lambda: self.sim(["--tb", "engine", "--max-cases", "1"]), "≈ 5 phút")
        self._step(f, 4, "Mô phỏng cả hệ thống SPI → SDRAM → HS → OC-SVM → UART (1 ca).", "④ Mô phỏng hệ thống",
                   lambda: self.sim(["--tb", "system", "--max-cases", "1"]), "≈ 30 phút, in SD E=0000 + dòng HS")
        self._step(f, 5, "Mở Vivado (giao diện) để xem dạng sóng nếu cần.", "Mở Vivado GUI", self.open_vivado)

    def _tab_fpga(self, f):
        ttk.Label(f, text="Tang Nano 20K – làm theo thứ tự ① → ⑥.", style="Head.TLabel").grid(
            row=0, column=0, columnspan=3, sticky="w", pady=(0, 8))
        self._step(f, 1, "Dựng project Gowin “hs_board_test” (không cần SDRAM, không cần ESP32).",
                   "① Tạo project board test",
                   lambda: self.py("25_hs_board_test.py", ["--gowin", "fpga_gowin/hs_board_test"]))
        self._step(f, 2, "Mở Gowin với project board test → Synthesize → Place & Route → Programmer (SRAM).",
                   "② Mở Gowin (board test)", lambda: self.open_gowin("fpga_gowin/hs_board_test"))
        self._step(f, 3, "Đọc UART board và so với Golden Model (chọn Cổng FPGA ở trên).", "③ Kiểm tra board test",
                   lambda: self.py("25_hs_board_test.py", ["--port", self.port("fpga_port")], need=("fpga_port",)),
                   "mong đợi: board KHỚP Golden Model")
        opt = ttk.Frame(f)
        opt.grid(row=4, column=2, sticky="w")
        ttk.Label(opt, text="Xung nhịp").pack(side="left")
        ttk.Combobox(opt, textvariable=self.vars["clk"], values=["54", "27"], width=4, state="readonly").pack(
            side="left", padx=4)
        ttk.Label(opt, text="Số vòng (0 = mặc định)").pack(side="left")
        ttk.Combobox(opt, textvariable=self.vars["nit"], values=["0", "32", "64"], width=4).pack(side="left", padx=4)
        self._step(f, 4, "Dựng project hệ thống demo “hs_system” (SPI + SDRAM + HS + OC-SVM).",
                   "④ Tạo project hệ thống", self.make_system)
        self._step(f, 5, "Mở Gowin với project hệ thống → Synthesize → P&R → Programmer.",
                   "⑤ Mở Gowin (hệ thống)", lambda: self.open_gowin("fpga_gowin/hs_system"))
        self._step(f, 6, "Xem UART của FPGA 10 giây: phải thấy “SD E=0000” (SDRAM tốt), sau khi nối ESP32 có dòng “HS …”.",
                   "⑥ Xem UART FPGA", lambda: self.serial_peek("fpga_port", 115200, 10))

    def _tab_esp(self, f):
        ttk.Label(f, text="ESP32-S3 + OV5640 (firmware/esp32_demo).", style="Head.TLabel").grid(
            row=0, column=0, columnspan=3, sticky="w", pady=(0, 8))
        self._step(f, 1, "Lần đầu: chọn chip ESP32-S3 (chỉ cần 1 lần).", "① set-target esp32s3",
                   lambda: self.idf("set-target esp32s3"))
        self._step(f, 2, "Mở menuconfig (cửa sổ riêng): Demo 1 → Board camera, GPIO SPI. Lưu bằng S, thoát Q.",
                   "② menuconfig", lambda: self.idf("menuconfig", new_window=True))
        self._step(f, 3, "Biên dịch và nạp firmware (chọn Cổng ESP32 ở trên).", "③ Build + Flash",
                   lambda: self.idf(f"-p {self.port('esp_port')} build flash", need=("esp_port",)), "≈ 2–5 phút lần đầu")
        self._step(f, 4, "Xem log ESP32 (cửa sổ riêng, thoát Ctrl+]): phải có fps_ok=1, F seq=… đều 80 ms.",
                   "④ Monitor", lambda: self.idf(f"-p {self.port('esp_port')} monitor", new_window=True,
                                                 need=("esp_port",)))
        self._step(f, 5, "Khi build lỗi lạ hoặc đổi baud: xóa bản build cũ.", "fullclean", lambda: self.idf("fullclean"))
        self._step(f, 6, "Đọc log ESP32 8 giây (không reset ESP32) theo “Baud ESP32” ở tab 5: phải thấy F seq=… và PV.",
                   "⑥ Xem log ESP32", lambda: self.serial_peek("esp_port", int(self.vars["esp_baud"].get() or 115200), 8))

    def _tab_demo(self, f):
        ttk.Label(f, text="Trước khi chạy: thoát mọi monitor (ESP-IDF, Gowin Programmer) để giải phóng cổng COM.",
                  style="Head.TLabel").grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 8))
        opt = ttk.Frame(f)
        opt.grid(row=1, column=0, columnspan=3, sticky="w", pady=4)
        ttk.Label(opt, text="Hiệu chỉnh (giây)").pack(side="left")
        ttk.Entry(opt, textvariable=self.vars["calib_sec"], width=6).pack(side="left", padx=(4, 14))
        ttk.Label(opt, text="Ngưỡng f_q đã lưu").pack(side="left")
        ttk.Entry(opt, textvariable=self.vars["threshold"], width=16).pack(side="left", padx=(4, 14))
        ttk.Label(opt, text="Baud ESP32").pack(side="left")
        ttk.Combobox(opt, textvariable=self.vars["esp_baud"], values=["115200", "921600", "1500000", "2000000"], width=9).pack(
            side="left", padx=(4, 14))
        ttk.Label(opt, text="Làm mượt cờ (k)").pack(side="left")
        ttk.Entry(opt, textvariable=self.vars["smooth"], width=4).pack(side="left", padx=(4, 14))
        ttk.Label(opt, text="Tự dừng sau (phút, 0 = bấm q)").pack(side="left")
        ttk.Entry(opt, textvariable=self.vars["minutes"], width=6).pack(side="left", padx=4)
        self._step(f, 2, "LẦN ĐẦU ở phòng demo: để cảnh BÌNH THƯỜNG trong thời gian hiệu chỉnh; ngưỡng tự được lưu.",
                   "▶ Hiệu chỉnh + chạy", lambda: self.demo(calib=True))
        self._step(f, 3, "Các lần sau (và ngày demo): chạy với ngưỡng đã lưu.", "▶ CHẠY DEMO",
                   lambda: self.demo(calib=False), "cửa sổ hiển thị: q thoát, s chụp màn hình")
        self._step(f, 4, "Chạy khi chưa có ESP32 nối PC (chỉ đọc kết quả FPGA, không có ảnh xem trước).",
                   "▶ Chạy chỉ FPGA", lambda: self.demo(calib=False, esp=False))
        self._step(f, 5, "Mở thư mục kết quả các lần chạy (CSV, summary.json, ảnh chụp).", "📂 Kết quả demo",
                   lambda: self.open_dir(ROOT / "outputs_hw" / "demo_runs"))

    def _tab_record(self, f):
        ttk.Label(f, text="Ghi cảnh thật để train lại: chạy như demo (esp32_demo + bitstream hs_system_z), đọc 48 z từ cổng FPGA.",
                  style="Head.TLabel").grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 8))
        opt = ttk.Frame(f)
        opt.grid(row=1, column=0, columnspan=3, sticky="w", pady=4)
        ttk.Label(opt, text="Thời lượng (phút)").pack(side="left")
        ttk.Entry(opt, textvariable=self.vars["rec_minutes"], width=6).pack(side="left", padx=4)
        self._step(f, 2, "Cảnh BÌNH THƯỜNG để huấn luyện (phòng trống, đi bộ chậm) – nên 10–15 phút, chia vài phiên.",
                   "● Ghi bình thường → Train", lambda: self.record("normal", False))
        self._step(f, 3, "Cảnh BÌNH THƯỜNG để đánh giá (khác lúc ghi Train) – 2 phút.",
                   "● Ghi bình thường → Test", lambda: self.record("normal", True))
        self._step(f, 4, "Cảnh BẤT THƯỜNG để đánh giá (chạy, vung tay, ngã giả) – 2 phút.",
                   "● Ghi bất thường → Test", lambda: self.record("abnormal", True))
        self._step(f, 5, "Huấn luyện lại (IITB + cảnh thật) và so với mô hình hiện tại.", "▶ Train lại",
                   lambda: self.py("36_train_scene.py", []))
        self._step(f, 6, "Mở thư mục dữ liệu đã ghi.", "📂 data/scene",
                   lambda: self.open_dir(ROOT / "data" / "scene"))

    def record(self, label, test):
        if not self._need(("fpga_port",)):
            return
        args = [str(ROOT / "scripts" / "29_record_scene.py"), "--fpga", self.port("fpga_port"), "--label", label,
                "--minutes", self.vars["rec_minutes"].get() or "10"] + (["--test"] if test else [])
        self.run([PY] + args, f"Ghi dữ liệu ({label}{', Test' if test else ''})")

    def _tab_report(self, f):
        ttk.Label(f, text="Số liệu chứng minh tăng tốc.", style="Head.TLabel").grid(
            row=0, column=0, columnspan=3, sticky="w", pady=(0, 8))
        self._step(f, 1, "Đo HS trên CPU (PC: C số nguyên, Python) → bench.json.", "Đo CPU (PC)",
                   lambda: self.py("23_hs_cpu_bench.py", []))
        self._step(f, 2, "Tạo file mẫu gowin_metrics.json để điền tài nguyên, Fmax, công suất từ Gowin.",
                   "Tạo gowin_metrics.json", lambda: self.py("28_accel_report.py", ["--init-gowin"]))
        self._step(f, 3, "Mở gowin_metrics.json để điền.", "Sửa gowin_metrics.json",
                   lambda: self.open_file(ROOT / "outputs_hw" / "gowin_metrics.json"))
        self._step(f, 4, "Tổng hợp báo cáo tăng tốc (report.md, speedup.png).", "Tạo báo cáo",
                   lambda: self.py("28_accel_report.py", []))
        self._step(f, 5, "Mở thư mục báo cáo.", "📂 Báo cáo", lambda: self.open_dir(ROOT / "outputs_hw" / "accel_report"))

    # ------------------------------------------------------------------ tiện ích
    def save_cfg(self):
        self.cfg = {k: v.get() for k, v in self.vars.items()}
        CFG_FILE.write_text(json.dumps(self.cfg, ensure_ascii=False, indent=2), encoding="utf-8")
        self.say(f"Đã lưu cài đặt → {CFG_FILE}\n", "ok")

    def refresh_ports(self):
        ports, err = list_ports()
        self.cb_fpga["values"] = ports
        self.cb_esp["values"] = ports
        if err:
            self.say(err + "\n", "err")

    def print_ports(self):
        ports, err = list_ports()
        self.say("Cổng COM: " + (", ".join(ports) if ports else (err or "không có cổng nào")) + "\n", "cmd")

    def port(self, key):
        return self.vars[key].get().split(" ")[0].strip()

    def say(self, text, tag=None):
        self.log.insert("end", text, tag)
        self.log.see("end")

    def _need(self, keys):
        for k in keys:
            if not self.port(k):
                messagebox.showwarning("Thiếu cổng COM", "Chọn cổng ở hàng trên cùng trước (bấm ↻ Quét cổng COM).")
                return False
        return True

    def run(self, cmd, title, cwd=ROOT, shell=False, on_line=None):
        if self.proc and self.proc.poll() is None:
            messagebox.showinfo("Đang chạy", "Một tác vụ đang chạy. Bấm “Dừng tác vụ” hoặc chờ xong.")
            return
        self.save_cfg()
        env = dict(os.environ, PYTHONUTF8="1", PYTHONIOENCODING="utf-8", PYTHONUNBUFFERED="1")
        shown = cmd if isinstance(cmd, str) else " ".join(str(c) for c in cmd)
        self.say(f"\n▶ {title}\n$ {shown}\n", "cmd")
        self.status.set(f"Đang chạy: {title}")
        t0 = time.time()
        try:
            self.proc = subprocess.Popen(cmd, cwd=str(cwd), stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                         shell=shell, env=env, bufsize=1,
                                         creationflags=subprocess.CREATE_NO_WINDOW if IS_WIN else 0)
        except Exception as e:  # noqa: BLE001
            self.say(f"Không chạy được: {e}\n", "err")
            self.status.set("Lỗi")
            return

        def pump(p=self.proc):
            for raw in iter(p.stdout.readline, b""):
                line = raw.decode("utf-8", errors="replace")
                self.q.put((line, None))
                if on_line:
                    on_line(line)
            p.wait()
            ok = p.returncode == 0
            self.q.put((f"■ {title}: {'XONG' if ok else 'LỖI (mã ' + str(p.returncode) + ')'} "
                        f"sau {time.time() - t0:.0f} s\n", "ok" if ok else "err"))
            self.q.put((None, "done"))
        threading.Thread(target=pump, daemon=True).start()

    def _drain(self):
        try:
            while True:
                line, tag = self.q.get_nowait()
                if line is None:
                    self.status.set("Sẵn sàng")
                    continue
                if tag is None:
                    low = line.lower()
                    tag = "err" if ("fail" in low or "error" in low or "lỗi" in low) else (
                        "ok" if ("pass" in low or "khớp" in low) else None)
                self.say(line, tag)
        except queue.Empty:
            pass
        self.after(100, self._drain)

    def stop(self):
        if self.proc and self.proc.poll() is None:
            if IS_WIN:
                subprocess.call(["taskkill", "/F", "/T", "/PID", str(self.proc.pid)],
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            else:
                self.proc.terminate()
            self.say("■ Đã dừng tác vụ\n", "err")

    def _on_close(self):
        self.stop()
        self.save_cfg()
        self.destroy()

    def open_dir(self, p):
        p = Path(p)
        p.mkdir(parents=True, exist_ok=True)
        os.startfile(str(p)) if IS_WIN else subprocess.Popen(["xdg-open", str(p)])

    def open_file(self, p):
        p = Path(p)
        if not p.exists():
            messagebox.showwarning("Chưa có file", f"Chưa có {p}. Chạy bước tạo file trước.")
            return
        os.startfile(str(p)) if IS_WIN else subprocess.Popen(["xdg-open", str(p)])

    # ------------------------------------------------------------------ hành động
    def py(self, script, args, need=(), title=None, on_line=None, with_config=True):
        if need and not self._need(need):
            return
        cmd = [PY, str(ROOT / "scripts" / script)]
        if with_config:
            cmd += ["--config", self.vars["config"].get()]
        self.run(cmd + list(args), title or script, on_line=on_line)

    def export_data(self):
        cfg = self.vars["config"].get()
        cmd = (f'"{PY}" scripts/04_export_fpga.py --config "{cfg}" && '
               f'"{PY}" scripts/15_export_rtl.py --config "{cfg}" --lanes 8')
        self.run(cmd, "Xuất dữ liệu (04 + 15)", shell=True)

    def sim(self, extra):
        args = list(extra)
        if self.vars["sim"].get() == "xsim":
            args += ["--sim", "xsim", "--vivado", self.vars["vivado_bin"].get()]
        self.py("24_hs_rtl_sim.py", args, title="Mô phỏng " + (" ".join(extra) or "nhanh"))

    def open_vivado(self):
        exe = Path(self.vars["vivado_bin"].get()) / ("vivado.bat" if IS_WIN else "vivado")
        if not exe.exists():
            messagebox.showerror("Không thấy Vivado", f"Không có {exe}. Sửa ở tab 0. Cài đặt.")
            return
        subprocess.Popen([str(exe)], cwd=str(ROOT))

    def make_system(self):
        args = ["--clk", self.vars["clk"].get()]
        if self.vars["nit"].get() not in ("", "0"):
            args += ["--nit", self.vars["nit"].get()]
        self.py("26_gowin_system.py", args, title="Tạo project hệ thống")

    def open_gowin(self, rel):
        d = ROOT / rel
        projs = sorted(d.glob("*.gprj"))
        exe = Path(self.vars["gowin_ide"].get())
        if not d.exists():
            messagebox.showwarning("Chưa có project", f"Chưa có {d}. Bấm nút “Tạo project” trước.")
            return
        if not exe.exists():
            messagebox.showwarning("Không thấy Gowin", f"Không có {exe}. Sửa ở tab 0. Cài đặt. Mở thư mục project thay thế.")
            self.open_dir(d)
            return
        subprocess.Popen([str(exe)] + ([str(projs[0])] if projs else []), cwd=str(d))
        self.say(f"Đã mở Gowin: {projs[0] if projs else d}\n", "cmd")

    def idf(self, args, new_window=False, need=()):
        if need and not self._need(need):
            return
        exp = self.vars["idf_export"].get()
        proj = ROOT / "firmware" / "esp32_demo"
        if not Path(exp).exists():
            messagebox.showerror("Không thấy ESP-IDF", f"Không có {exp}. Sửa ở tab 0. Cài đặt.")
            return
        inner = f'call "{exp}" && cd /d "{proj}" && idf.py {args}'
        if new_window:
            subprocess.Popen(f'start "ESP-IDF {args}" cmd /k "{inner}"', shell=True, cwd=str(proj))
            self.say(f"Đã mở cửa sổ ESP-IDF: idf.py {args}\n", "cmd")
        else:
            self.run(f'cmd /c "{inner}"', f"idf.py {args}", cwd=proj, shell=True)

    def serial_peek(self, key, baud, sec):
        if not self._need((key,)):
            return
        self.run([PY, str(ROOT / "scripts" / "demo1_uart_peek.py"), self.port(key), str(baud), str(sec)]
                 + (["esp"] if key == "esp_port" else []),
                 f"Đọc UART {self.port(key)} trong {sec} s")

    def demo(self, calib, esp=True):
        need = ("fpga_port",) + (("esp_port",) if esp else ())
        if not self._need(need):
            return
        args = ["--fpga", self.port("fpga_port"), "--smooth", self.vars["smooth"].get() or "7"]
        if esp:
            args += ["--esp", self.port("esp_port"), "--esp-baud", self.vars["esp_baud"].get() or "115200"]
        if self.vars["minutes"].get() not in ("", "0"):
            args += ["--minutes", self.vars["minutes"].get()]
        if calib:
            args += ["--calib-sec", self.vars["calib_sec"].get()]
        elif self.vars["threshold"].get().strip():
            args += ["--thr", self.vars["threshold"].get().strip()]
        else:
            if not messagebox.askyesno("Chưa có ngưỡng", "Chưa có ngưỡng đã hiệu chỉnh. Chạy với cờ gốc của FPGA?"):
                return

        def grab(line):
            m = re.search(r"--thr\s+(-?\d+(?:\.\d+)?)", line) or re.search(r"ng[uư][ỡo]ng[^-\d]*(-?\d{4,})", line)
            if m and calib:
                self.vars["threshold"].set(m.group(1))
                self.q.put((f"→ Đã lưu ngưỡng {m.group(1)} cho các lần chạy sau\n", "ok"))
        self.py("27_demo_monitor.py", args, title="DEMO", on_line=grab, with_config=False)

    def check_env(self):
        lines = [f"Python: {sys.version.split()[0]} ({PY})"]
        try:
            import serial  # noqa: F401
            lines.append("pyserial: có")
        except ImportError:
            lines.append("pyserial: CHƯA CÀI → tab 1, nút pip install pyserial")
        viv = Path(self.vars["vivado_bin"].get())
        lines.append(f"Vivado xvlog: {'có' if (viv / ('xvlog.bat' if IS_WIN else 'xvlog')).exists() else 'KHÔNG THẤY'} ({viv})")
        lines.append(f"Icarus iverilog: {shutil.which('iverilog') or 'không có (không bắt buộc nếu dùng Vivado)'}")
        lines.append(f"Gowin IDE: {'có' if Path(self.vars['gowin_ide'].get()).exists() else 'KHÔNG THẤY'}")
        lines.append(f"ESP-IDF export.bat: {'có' if Path(self.vars['idf_export'].get()).exists() else 'KHÔNG THẤY'}")
        for rel in (self.vars["config"].get(), "scripts/24_hs_rtl_sim.py", "scripts/27_demo_monitor.py",
                    "firmware/esp32_demo/CMakeLists.txt", "rtl/src/sys/hs_system_top.v"):
            lines.append(f"{rel}: {'có' if (ROOT / rel).exists() else 'THIẾU'}")
        for ln in lines:
            self.say(ln + "\n", "err" if ("KHÔNG" in ln or "THIẾU" in ln or "CHƯA" in ln) else "ok")


if __name__ == "__main__":
    Launcher().mainloop()
