#include "ov5640_fps.h"

#include <math.h>
#include <stdio.h>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "board_pins.h"

static bool s_on;

esp_err_t ov_cam_start(int xclk_mhz, int fb_count, bool grab_latest, bool fb_in_psram, bool color)
{
    return ov_cam_start_fs(xclk_mhz, fb_count, grab_latest, fb_in_psram, color, FRAMESIZE_QQVGA);
}

esp_err_t ov_cam_start_fs(int xclk_mhz, int fb_count, bool grab_latest, bool fb_in_psram, bool color,
                          framesize_t fs)
{
    if (s_on) {
        esp_camera_deinit();
        s_on = false;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    camera_config_t c = {
        .pin_pwdn = CAM_PIN_PWDN,
        .pin_reset = CAM_PIN_RESET,
        .pin_xclk = CAM_PIN_XCLK,
        .pin_sccb_sda = CAM_PIN_SIOD,
        .pin_sccb_scl = CAM_PIN_SIOC,
        .pin_d7 = CAM_PIN_D7,
        .pin_d6 = CAM_PIN_D6,
        .pin_d5 = CAM_PIN_D5,
        .pin_d4 = CAM_PIN_D4,
        .pin_d3 = CAM_PIN_D3,
        .pin_d2 = CAM_PIN_D2,
        .pin_d1 = CAM_PIN_D1,
        .pin_d0 = CAM_PIN_D0,
        .pin_vsync = CAM_PIN_VSYNC,
        .pin_href = CAM_PIN_HREF,
        .pin_pclk = CAM_PIN_PCLK,
        .xclk_freq_hz = xclk_mhz * 1000000,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = color ? PIXFORMAT_YUV422 : PIXFORMAT_GRAYSCALE,
        .frame_size = fs,
        .jpeg_quality = 12,
        .fb_count = fb_count,
        .fb_location = fb_in_psram ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM,
        .grab_mode = grab_latest ? CAMERA_GRAB_LATEST : CAMERA_GRAB_WHEN_EMPTY,
    };
    esp_err_t err = esp_camera_init(&c);
    s_on = (err == ESP_OK);
    return err;
}

void ov_cam_warmup(int n)
{
    for (int i = 0; i < n; i++) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) {
            esp_camera_fb_return(fb);
        }
    }
}

int64_t ov_cam_fb_ts_us(const camera_fb_t *fb)
{
    int64_t ts = (int64_t)fb->timestamp.tv_sec * 1000000LL + fb->timestamp.tv_usec;
    return ts > 0 ? ts : esp_timer_get_time();
}

double ov_cam_measure_period_us(int n)
{
    int64_t t0 = -1, t1 = -1;
    int got = 0;
    for (int i = 0; i <= n; i++) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            continue;
        }
        int64_t t = ov_cam_fb_ts_us(fb);
        esp_camera_fb_return(fb);
        if (t0 < 0) {
            t0 = t;
        } else {
            t1 = t;
            got++;
        }
    }
    return got > 0 ? (double)(t1 - t0) / got : 0;
}

static int reg16(sensor_t *s, int hi, int lo)
{
    int a = s->get_reg(s, hi, 0xFF), b = s->get_reg(s, lo, 0xFF);
    return (a < 0 || b < 0) ? -1 : ((a << 8) | b);
}

static void wreg16(sensor_t *s, int hi, int lo, int v)
{
    s->set_reg(s, hi, 0xFF, (v >> 8) & 0xFF);
    s->set_reg(s, lo, 0xFF, v & 0xFF);
}

/* FPS = PCLK / (HTS × VTS) → chu kỳ tỉ lệ thuận với VTS: VTS_mới = VTS · T_mục_tiêu / T_đo */
fps_result_t ov5640_set_fps(int fps_x10, bool aec_follow)
{
    fps_result_t r = {0};
    sensor_t *s = esp_camera_sensor_get();
    const double target_us = 1e7 / fps_x10;
    r.hts = reg16(s, 0x380C, 0x380D);
    int vts = reg16(s, 0x380E, 0x380F);
    double p = ov_cam_measure_period_us(30);
    r.vts_before = vts;
    r.period_before_us = p;
    printf("#INFO fps_ctrl_before hts=%d vts=%d period_ms=%.3f fps=%.3f\n", r.hts, vts, p / 1000,
           p > 0 ? 1e6 / p : 0);
    if (vts <= 0 || p <= 0) {
        printf("#INFO fps_ctrl=FAIL reason=khong_doc_duoc_VTS_hoac_chu_ky\n");
        return r;
    }
    for (int it = 0; it < 3; it++) {
        int nv = (int)lround(vts * target_us / p);
        nv = nv < 130 ? 130 : (nv > 0xFFFF ? 0xFFFF : nv);
        wreg16(s, 0x380E, 0x380F, nv);
        if (aec_follow) {
            wreg16(s, 0x3A02, 0x3A03, nv - 4);
            wreg16(s, 0x3A14, 0x3A15, nv - 4);
        }
        s->set_reg(s, 0x3A00, 0x04, 0x00);    /* tắt night mode */
        ov_cam_warmup(5);
        double np = ov_cam_measure_period_us(30);
        printf("#INFO fps_ctrl_iter=%d vts=%d->%d period_ms=%.3f fps=%.3f\n", it, vts, nv, np / 1000,
               np > 0 ? 1e6 / np : 0);
        if (np <= 0) {
            break;
        }
        vts = nv;
        p = np;
        if (fabs(p - target_us) / target_us < 0.002) {
            break;
        }
    }
    r.vts_after = vts;
    r.period_after_us = p;
    r.err_pct = fabs(p - target_us) / target_us * 100;
    r.ok = r.err_pct < 1;
    printf("#INFO fps_ctrl=%s target_fps=%.2f period_ms=%.3f err_pct=%.3f vts=%d\n", r.ok ? "OK" : "APPROX",
           1e6 / target_us, p / 1000, r.err_pct, vts);
    return r;
}

/* Giữ VTS: một số chế độ (YUV, tự phơi sáng) có thể ghi lại VTS khiến FPS trôi về ~19 fps.
 * Gọi định kỳ: nếu VTS hiện tại khác giá trị đã chốt thì ghi lại. Trả VTS đọc được trước khi sửa. */
int ov5640_hold_vts(int vts, bool aec_follow)
{
    sensor_t *s = esp_camera_sensor_get();
    int cur = reg16(s, 0x380E, 0x380F);
    if (vts > 0 && cur != vts) {
        wreg16(s, 0x380E, 0x380F, vts);
        if (aec_follow) {
            wreg16(s, 0x3A02, 0x3A03, vts - 4);
            wreg16(s, 0x3A14, 0x3A15, vts - 4);
        }
        s->set_reg(s, 0x3A00, 0x04, 0x00);
    }
    return cur;
}
