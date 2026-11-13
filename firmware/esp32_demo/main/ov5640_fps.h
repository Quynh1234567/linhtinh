/* Khởi động camera OV5640 ở 160×120 xám và đặt FPS chính xác bằng thanh ghi VTS.
 * Tách từ firmware/esp32_cam_timing (đã kiểm chứng trên board: 12,50 fps, chu kỳ 79,98 ms). */
#pragma once
#include <stdbool.h>
#include <stdint.h>

#include "esp_camera.h"

typedef struct {
    bool ok;             /* lệch < 1% so với mục tiêu */
    int vts_before, vts_after, hts;
    double period_before_us, period_after_us, err_pct;
} fps_result_t;

/* Khởi tạo camera theo board_pins.h. Trả mã lỗi của esp_camera_init().
 * color = false: PIXFORMAT_GRAYSCALE (Y8, 1 byte/điểm)
 * color = true : PIXFORMAT_YUV422    (Y U Y V, 2 byte/điểm) – kênh Y là cùng ảnh xám Y8 của cảm biến */
esp_err_t ov_cam_start(int xclk_mhz, int fb_count, bool grab_latest, bool fb_in_psram, bool color);
/* Như trên nhưng chọn kích thước khung (FRAMESIZE_QQVGA 160×120, FRAMESIZE_QVGA 320×240 …) */
esp_err_t ov_cam_start_fs(int xclk_mhz, int fb_count, bool grab_latest, bool fb_in_psram, bool color,
                          framesize_t fs);

/* Bỏ n khung đầu */
void ov_cam_warmup(int n);

/* Lấy timestamp (µs) driver gắn cho khung, hoặc thời điểm hiện tại nếu không có */
int64_t ov_cam_fb_ts_us(const camera_fb_t *fb);

/* Chu kỳ khung trung bình (µs) trên n khoảng */
double ov_cam_measure_period_us(int n);

/* Đặt FPS = fps_x10 / 10 bằng VTS, tắt night mode; aec_follow: nới phơi sáng tối đa = VTS − 4.
 * In log #INFO fps_ctrl_*. */
fps_result_t ov5640_set_fps(int fps_x10, bool aec_follow);

/* Ghi lại VTS nếu camera đã tự đổi (giữ đúng 12,5 fps). Trả VTS đọc được trước khi sửa. */
int ov5640_hold_vts(int vts, bool aec_follow);
