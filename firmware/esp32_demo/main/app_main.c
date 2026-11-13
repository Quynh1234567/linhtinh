/* =============================================================================
 *  DEMO 1 – ESP32-S3: OV5640 160×120 12,5 fps → kênh Y (xám) → Gauss 5×5 số nguyên → SPI → Tang Nano 20K
 *
 *  Camera chạy YUV422 (DEMO_COLOR_PREVIEW = y): kênh Y chính là ảnh xám Y8 cảm biến vẫn xuất ở chế độ
 *  GRAYSCALE → đường HS/OC-SVM nhận đúng dữ liệu như trước; U, V chỉ dùng cho ảnh xem trước MÀU trên PC.
 *
 *  Mỗi khung được bộ chọn cặp (frame_pacer, 80 ms) giữ lại:
 *    - Gauss /1024 (khớp từng bit Golden Model, đã kiểm trên board ở mục 2.2–2.3)
 *    - khung tối → không gửi, ngắt cặp
 *    - gửi 19 200 byte qua SPI (5 giao dịch A5|cờ|chunk|seq|dữ liệu); cờ RESET_PAIR khi khung này
 *      bắt đầu chuỗi mới (khung đầu, sau khung tối, sau khi rớt khung)
 *  FPGA tính Horn–Schunck + OC-SVM và tự báo kết quả qua UART của nó (BL616 → PC).
 *
 *  Log (console, 921 600 baud – đọc bởi scripts/27_demo_monitor.py --esp):
 *    #INFO …
 *    F seq=<n> ts=<µs> dt=<ms×10> g=<µs Gauss> s=<µs SPI> fl=<cờ> d=<quyết định pacer>
 *    D ts=<µs>                       (khung tối, không gửi)
 *    PJ <seq> W H age / base64 JPEG / END   ảnh MÀU gốc nén JPEG: 160×120, hoặc 320×240 khi bật
 *                                            DEMO_PREVIEW_QVGA (camera 320×240, ảnh phân tích = TB 2×2)
 *    PV <seq> 80 60 / base64 / END   ảnh xám 80×60 (khi tắt DEMO_COLOR_PREVIEW)
 *    (mỗi DEMO_PREVIEW_EVERY khung gửi một ảnh)
 * ============================================================================= */
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "img_converters.h"
#if defined(CONFIG_CAMERA_AF_SUPPORT) && CONFIG_CAMERA_AF_SUPPORT
#include "esp_camera_af.h"
#endif
#include "sdkconfig.h"

#if defined(CONFIG_IDF_TARGET) && !defined(CONFIG_IDF_TARGET_ESP32S3)
#error "Project dang build cho chip KHAC ESP32-S3. Chay: idf.py set-target esp32s3  (roi build lai)"
#endif

#include "board_pins.h"
#include "frame_pacer.h"
#include "ov5640_fps.h"
#include "preproc.h"
#include "spi_link.h"

#ifndef CONFIG_DEMO_PREVIEW_EVERY
#define CONFIG_DEMO_PREVIEW_EVERY 6
#endif
#ifdef CONFIG_PP_AEC_MAX_FOLLOW_VTS
#define AEC_FOLLOW true
#else
#define AEC_FOLLOW false
#endif

#ifdef CONFIG_DEMO_PREVIEW_QVGA
#define CAM_W 320                           /* camera 320×240; ảnh phân tích 160×120 = trung bình 2×2 */
#define CAM_H 240
#define CAM_FS FRAMESIZE_QVGA
#else
#define CAM_W PP_W
#define CAM_H PP_H
#define CAM_FS FRAMESIZE_QQVGA
#endif
#define CAM_PIX (CAM_W * CAM_H)
#ifndef CONFIG_DEMO_PREVIEW_JPEG_Q
#define CONFIG_DEMO_PREVIEW_JPEG_Q 80
#endif

#ifdef CONFIG_DEMO_COLOR_PREVIEW
#define COLOR 1
#define PV_W CAM_W                          /* chép nguyên khung YUV422 của camera, nén JPEG rồi mới gửi */
#define PV_H CAM_H
#define PV_BYTES (2 * CAM_PIX)
#define PV_TAG "PJ"
#else
#define COLOR 0
#define PV_W 80
#define PV_H 60
#define PV_BYTES (PV_W * PV_H)
#define PV_TAG "PV"
#endif

#if COLOR && (CONFIG_DEMO_PREVIEW_EVERY > 0)
#define PV_EVERY 1                          /* JPEG ~3–4 KB/ảnh: gửi MỌI khung (12,5 ảnh/s); ảnh nào chưa kịp gửi
                                               thì bị ảnh mới đè (không xếp hàng) → không bao giờ trễ dồn */
#else
#define PV_EVERY CONFIG_DEMO_PREVIEW_EVERY
#endif
#define PV_DIV (PV_EVERY > 0 ? PV_EVERY : 1)

static pp_work_t s_work;
static QueueHandle_t s_pv_q;               /* hàng đợi 1 phần tử: chỉ số bộ đệm xem trước mới nhất */
typedef struct {
    uint32_t seq;
    int64_t t_cap;                          /* thời điểm camera chụp xong khung (µs) – đo độ trễ */
    uint8_t px[PV_BYTES];
} preview_t;
static preview_t *s_pv_buf[3];             /* 3 bộ đệm luân phiên: không chép 28 KB qua hàng đợi */
static volatile int s_pv_busy = -1;       /* bộ đệm tác vụ in đang đọc */
static int s_pv_queued = -1;               /* bộ đệm đang nằm trong hàng đợi */
static int s_y_off = 0;                    /* vị trí byte Y trong cặp (YUYV: 0, UYVY: 1) */

/* In base64 (có đệm '=') thành các dòng 76 ký tự, kết thúc bằng END */
static void print_b64(const uint8_t *b, size_t n)
{
    static const char tbl[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    static char line[96];
    int li = 0;
    for (size_t i = 0; i < n; i += 3) {
        uint32_t v = (uint32_t)b[i] << 16;
        if (i + 1 < n) v |= (uint32_t)b[i + 1] << 8;
        if (i + 2 < n) v |= b[i + 2];
        line[li++] = tbl[(v >> 18) & 63];
        line[li++] = tbl[(v >> 12) & 63];
        line[li++] = (i + 1 < n) ? tbl[(v >> 6) & 63] : '=';
        line[li++] = (i + 2 < n) ? tbl[v & 63] : '=';
        if (li >= 76) { line[li] = 0; puts(line); li = 0; }
    }
    if (li) { line[li] = 0; puts(line); }
    puts("END");
}

/* In ảnh xem trước (chạy ở lõi 1, ưu tiên thấp – không chặn vòng camera) */
static void preview_task(void *arg)
{
    int idx;
    for (;;) {
        if (xQueueReceive(s_pv_q, &idx, portMAX_DELAY) != pdTRUE) continue;
        s_pv_busy = idx;
        const preview_t *pv = s_pv_buf[idx];
#if COLOR
        uint8_t *jpg = NULL;
        size_t jl = 0;
        if (fmt2jpg((uint8_t *)pv->px, PV_BYTES, PV_W, PV_H, PIXFORMAT_YUV422, CONFIG_DEMO_PREVIEW_JPEG_Q,
                    &jpg, &jl) && jpg) {
            int age_ms = (int)((esp_timer_get_time() - pv->t_cap) / 1000);   /* chụp → bắt đầu gửi */
            printf(PV_TAG " %" PRIu32 " %d %d %d\n", pv->seq, PV_W, PV_H, age_ms);
            print_b64(jpg, jl);
        }
        free(jpg);
#else
        printf(PV_TAG " %" PRIu32 " %d %d\n", pv->seq, PV_W, PV_H);
        print_b64(pv->px, PV_BYTES);
#endif
        s_pv_busy = -1;
    }
}

/* src: COLOR ? khung YUV422 gốc (2 byte/điểm) : ảnh xám đã làm mượt 160×120 */
static void make_preview(const uint8_t *src, uint32_t seq, int64_t t_cap)
{
    int wr = 0;                             /* chọn bộ đệm không bị tác vụ in đọc và không nằm trong hàng đợi */
    while (wr == s_pv_busy || wr == s_pv_queued) wr++;
    preview_t *pv = s_pv_buf[wr];
    pv->seq = seq;
    pv->t_cap = t_cap;
#if COLOR
    memcpy(pv->px, src, PV_BYTES);                 /* khung YUV422 gốc – nén JPEG ở tác vụ in */
#else
    for (int y = 0; y < PV_H; y++)
        for (int x = 0; x < PV_W; x++) {
            const uint8_t *p = src + (2 * y) * PP_W + 2 * x;
            pv->px[y * PV_W + x] = (uint8_t)((p[0] + p[1] + p[PP_W] + p[PP_W + 1] + 2) >> 2);
        }
#endif
    s_pv_queued = wr;
    xQueueOverwrite(s_pv_q, &wr);
}

#if COLOR
/* Tìm byte Y trong YUV422: Y có phương sai lớn (sáng/tối của cảnh), U/V gần 128 và ít thay đổi */
static int detect_y_offset(void)
{
#if defined(CONFIG_DEMO_YUV_Y_FIRST)
    return 0;
#elif defined(CONFIG_DEMO_YUV_Y_SECOND)
    return 1;
#else
    double var[2] = {0, 0};
    int got = 0;
    for (int k = 0; k < 5; k++) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) continue;
        if (fb->len >= 2 * CAM_PIX) {
            for (int ph = 0; ph < 2; ph++) {
                double s = 0, s2 = 0;
                for (int i = 0; i < CAM_PIX; i += 7) { double b = fb->buf[2 * i + ph]; s += b; s2 += b * b; }
                double n = (CAM_PIX + 6) / 7;
                var[ph] += s2 / n - (s / n) * (s / n);
            }
            got++;
        }
        esp_camera_fb_return(fb);
    }
    int off = (var[1] > var[0]) ? 1 : 0;
    printf("#INFO yuv_detect khung=%d var0=%.0f var1=%.0f y_off=%d (%s)\n", got, var[0], var[1], off,
           off ? "UYVY" : "YUYV");
    return off;
#endif
}
#endif

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(300));
    printf("\n#INFO demo=1 board=%s spi_sclk=%d spi_mosi=%d spi_cs=%d spi_hz=%d preview_every=%d color=%d\n",
           CAM_BOARD_NAME, CONFIG_DEMO_SPI_SCLK, CONFIG_DEMO_SPI_MOSI, CONFIG_DEMO_SPI_CS, CONFIG_DEMO_SPI_HZ,
           PV_EVERY, COLOR);

    esp_err_t err = link_init(CONFIG_DEMO_SPI_SCLK, CONFIG_DEMO_SPI_MOSI, CONFIG_DEMO_SPI_CS, CONFIG_DEMO_SPI_HZ);
    printf("#INFO spi_init=%s\n", esp_err_to_name(err));
    if (err != ESP_OK) return;

    /* 320×240: 1 bộ đệm ở RAM trong; 160×120: 2 bộ đệm như bản cũ. (320×240 YUV hiện báo FB-OVF trên board
     * này – giữ DEMO_PREVIEW_QVGA TẮT) */
    err = ov_cam_start_fs(CONFIG_PP_XCLK_MHZ, CAM_W > PP_W ? 1 : 2, CAM_W == PP_W, false, COLOR, CAM_FS);
    printf("#INFO cam_init=%s cam=%dx%d jpeg_q=%d\n", esp_err_to_name(err), CAM_W, CAM_H,
           CONFIG_DEMO_PREVIEW_JPEG_Q);
    if (err != ESP_OK) return;
    int vts_lock = 0;
    if (CONFIG_PP_TARGET_FPS_X10 > 0) {
        fps_result_t fr = ov5640_set_fps(CONFIG_PP_TARGET_FPS_X10, AEC_FOLLOW);
        printf("#INFO fps_ok=%d period_us=%.0f\n", fr.ok, fr.period_after_us);
        if (fr.ok) vts_lock = fr.vts_after;
    }
    ov_cam_warmup(CONFIG_PP_WARMUP_FRAMES);
#if defined(CONFIG_CAMERA_AF_SUPPORT) && CONFIG_CAMERA_AF_SUPPORT
    {   /* LẤY NÉT TỰ ĐỘNG MỘT LẦN lúc khởi động rồi GIỮ NGUYÊN ống kính (không lấy nét liên tục: ống kính
         * di chuyển giữa hai khung sẽ tạo chuyển động giả cho Horn–Schunck). Chỉ có tác dụng với module
         * OV5640 có motor lấy nét (AF); module ống kính cố định thì không đổi gì. */
        sensor_t *cs = esp_camera_sensor_get();
        esp_camera_af_status_t st = {0};
        esp_err_t ae = ESP_ERR_NOT_SUPPORTED;
        if (cs && esp_camera_af_is_supported(cs)) {
            esp_camera_af_config_t afc = {.mode = ESP_CAMERA_AF_MODE_MANUAL, .timeout_ms = 3000};
            ae = esp_camera_af_init(cs, &afc);                       /* nạp firmware AF vào cảm biến */
            if (ae == ESP_OK) ae = esp_camera_af_trigger(cs);       /* lấy nét 1 lần (vùng giữa ảnh) */
            if (ae == ESP_OK) ae = esp_camera_af_wait(cs, 3000, &st);
        }
        printf("#INFO af=%s focused=%d raw=0x%02X\n", esp_err_to_name(ae), st.focused, st.raw);
        ov_cam_warmup(5);
    }
#endif
#if COLOR
    s_y_off = detect_y_offset();
    uint8_t *gray = heap_caps_malloc(PP_PIX, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);   /* kênh Y tách ra */
    if (!gray) { printf("#INFO loi=khong_du_ram\n"); return; }
#endif

    uint8_t *smooth = heap_caps_malloc(PP_PIX, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!smooth) { printf("#INFO loi=khong_du_ram\n"); return; }
    if (PV_EVERY > 0) {
        for (int i = 0; i < 3; i++) {
            s_pv_buf[i] = heap_caps_malloc(sizeof(preview_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            if (!s_pv_buf[i]) { printf("#INFO loi=khong_du_ram_preview\n"); return; }
        }
        s_pv_q = xQueueCreate(1, sizeof(int));
        xTaskCreatePinnedToCore(preview_task, "preview", 8192, NULL, 1, NULL, 1);   /* nén JPEG cần ngăn xếp lớn hơn */
    }

    frame_pacer_t pacer;
    pacer_init(&pacer, CONFIG_PP_TARGET_INTERVAL_MS, CONFIG_PP_TOL_PCT);
    uint32_t seq = 0, n_sent = 0, n_dark = 0, n_err = 0;
    bool need_reset = true;
    int64_t t_stat = esp_timer_get_time(), t_vts = t_stat;
    uint32_t n_vts_fix = 0;
    printf("#INFO bat_dau=1\n");

    for (;;) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) { pacer_reset(&pacer); need_reset = true; continue; }
        if (fb->len < (COLOR ? 2 : 1) * CAM_PIX) { esp_camera_fb_return(fb); continue; }
        int64_t ts = ov_cam_fb_ts_us(fb);
        pacer_decision_t d = pacer_feed(&pacer, ts);
        if (d == PACER_SKIP) { esp_camera_fb_return(fb); continue; }

        int64_t t0 = esp_timer_get_time();
#if COLOR
#if CAM_W == PP_W
        for (int i = 0; i < PP_PIX; i++) gray[i] = fb->buf[2 * i + s_y_off];
#else
        /* 320×240 → 160×120: trung bình 2×2 kênh Y, làm tròn (giống thu nhỏ INTER_AREA của pipeline train) */
        for (int y = 0; y < PP_H; y++) {
            const uint8_t *r0 = fb->buf + (size_t)(2 * y) * CAM_W * 2 + s_y_off;
            const uint8_t *r1 = r0 + CAM_W * 2;
            uint8_t *g = gray + y * PP_W;
            for (int x = 0; x < PP_W; x++) {
                int k = 4 * x;                          /* 2 điểm ngang × 2 byte/điểm */
                g[x] = (uint8_t)((r0[k] + r0[k + 2] + r1[k] + r1[k + 2] + 2) >> 2);
            }
        }
#endif
        const uint8_t *src = gray;
#else
        const uint8_t *src = fb->buf;
#endif
        uint32_t sum = pp_gauss5(src, smooth, PP_BORDER_REFLECT101, &s_work);
        uint32_t t_g = (uint32_t)(esp_timer_get_time() - t0);
        bool want_pv = PV_EVERY > 0 && ((seq + 1) % PV_DIV) == 0;
#if COLOR
        if (want_pv && !pp_is_dark(sum, CONFIG_PP_DARK_MEAN)) make_preview(fb->buf, seq + 1, ts);   /* ảnh gốc */
#endif
        esp_camera_fb_return(fb);

        if (pp_is_dark(sum, CONFIG_PP_DARK_MEAN)) {        /* khung tối: không gửi, ngắt cặp */
            pacer_reset(&pacer);
            need_reset = true;
            n_dark++;
            printf("D ts=%" PRId64 "\n", ts);
            continue;
        }
        uint8_t flags = 0;
        if (need_reset || d != PACER_PAIR_OK) flags |= LINK_FLAG_RESET_PAIR;
        seq++;
        uint32_t t_s = 0;
        err = link_send_frame(smooth, (uint8_t)seq, flags, &t_s);
        if (err != ESP_OK) { n_err++; need_reset = true; continue; }
        need_reset = false;
        n_sent++;
        printf("F seq=%" PRIu32 " ts=%" PRId64 " dt=%" PRId64 " g=%" PRIu32 " s=%" PRIu32 " fl=%u d=%s\n",
               seq, ts, pacer.last_pair_dt_us / 100, t_g, t_s, flags, pacer_name(d));
#if !COLOR
        if (want_pv) make_preview(smooth, seq, ts);
#endif

        int64_t now = esp_timer_get_time();
        if (vts_lock > 0 && now - t_vts > 1000000) {          /* mỗi 1 s: giữ camera đúng 12,5 fps */
            int was = ov5640_hold_vts(vts_lock, AEC_FOLLOW);
            if (was != vts_lock) {
                n_vts_fix++;
                printf("#INFO vts_bi_doi=%d -> ghi_lai=%d\n", was, vts_lock);
            }
            t_vts = now;
        }
        if (now - t_stat > 10000000) {                       /* thống kê mỗi 10 s */
            printf("#INFO gui=%" PRIu32 " toi=%" PRIu32 " loi_spi=%" PRIu32 " sua_vts=%" PRIu32 "\n",
                   n_sent, n_dark, n_err, n_vts_fix);
            t_stat = now;
        }
    }
}
