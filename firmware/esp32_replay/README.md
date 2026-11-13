# esp32_replay – phát lại khung IITB qua đúng hệ thống demo, so từng cặp với Golden

```
PC (34_fpga_replay.py) ──UART0 2 Mbaud──► ESP32-S3: Gauss 5×5 (preproc.c CHUNG với esp32_demo)
   ──SPI (spi_link.c CHUNG với esp32_demo)──► Tang Nano 20K: bitstream hs_system của demo, KHÔNG đổi
   ──UART BL616──► PC: so Σ Gauss, f_q, cờ, số bão hòa, Σz với Golden Model Python
```

Khác demo đúng một chỗ: ảnh 160×120 xám lấy từ cache khung IITB của pipeline train thay vì camera.

## Chạy

```bash
# 1. nạp firmware phát lại cho ESP32 (FPGA giữ nguyên bitstream demo trong External Flash)
cd firmware/esp32_replay
idf.py set-target esp32s3
idf.py build flash            # chân SPI mặc định giống esp32_demo: SCLK 20, MOSI 21, CS 47, 8 MHz
cd ../..
# 2. đóng mọi monitor đang mở cổng COM, rồi:
python scripts/34_fpga_replay.py --config configs/hw.yaml --esp COM5 --fpga COM9 --pairs 1000 --per-video 100
# 3. trước buổi demo: nạp lại firmware demo
cd firmware/esp32_demo && idf.py flash
```

Kết quả: `outputs_hw/replay/<thời gian>/report.md`, `pairs.csv`, `log.txt`.
