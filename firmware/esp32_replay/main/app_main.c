/* esp32_replay – PHÁT LẠI KHUNG IITB QUA ĐÚNG ĐƯỜNG CỦA HỆ THỐNG DEMO
 *
 *   PC (scripts/34_fpga_replay.py) ──UART0──► ESP32-S3: Gauss 5×5 (preproc.c, y hệt demo)
 *        ──SPI (spi_link.c, y hệt demo)──► Tang Nano 20K (bitstream hs_system, KHÔNG đổi)
 *        ──UART BL616──► PC so với Golden Model.
 *   Chỉ khác demo ở chỗ ảnh lấy từ PC thay vì camera.
 *
 * Gói PC → ESP:  "RPLY" | seq (1 B) | cờ (1 B, bit0 = RESET_PAIR) | 19 200 byte ảnh 160×120 xám
 *                (CHƯA Gauss, đúng khung cảm biến của pipeline train) | tổng 16 bit của ảnh (LE)
 * Trả lời:       "K seq=.. fl=.. gsum=<Σ ảnh Gauss> g=<µs Gauss> s=<µs SPI> spi=<mã lỗi>"
 *                "E seq=.. err=checksum|timeout" */
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "preproc.h"
#include "spi_link.h"

#define PORT UART_NUM_0
#define RX_BUF (32 * 1024)

static pp_work_t s_work;
static uint8_t s_img[PP_PIX];
static uint8_t s_smooth[PP_PIX];

static int read_exact(uint8_t *p, int n, int ms)
{
    int got = 0;
    while (got < n) {
        int r = uart_read_bytes(PORT, p + got, n - got, pdMS_TO_TICKS(ms));
        if (r <= 0) return got;
        got += r;
    }
    return got;
}

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(300));
    ESP_ERROR_CHECK(uart_driver_install(PORT, RX_BUF, 0, 0, NULL, 0));
    printf("\n#INFO replay=1 spi_sclk=%d spi_mosi=%d spi_cs=%d spi_hz=%d\n", CONFIG_REPLAY_SPI_SCLK,
           CONFIG_REPLAY_SPI_MOSI, CONFIG_REPLAY_SPI_CS, CONFIG_REPLAY_SPI_HZ);
    esp_err_t err = link_init(CONFIG_REPLAY_SPI_SCLK, CONFIG_REPLAY_SPI_MOSI, CONFIG_REPLAY_SPI_CS, CONFIG_REPLAY_SPI_HZ);
    printf("#INFO spi_init=%s\n", esp_err_to_name(err));
    if (err != ESP_OK) return;
    printf("#INFO san_sang=1\n");

    static const uint8_t magic[4] = {'R', 'P', 'L', 'Y'};
    for (;;) {
        int m = 0;
        while (m < 4) {                                       /* đồng bộ theo "RPLY" */
            uint8_t c;
            if (uart_read_bytes(PORT, &c, 1, portMAX_DELAY) != 1) continue;
            m = (c == magic[m]) ? m + 1 : (c == 'R' ? 1 : 0);
        }
        uint8_t hdr[2], ck[2];
        if (read_exact(hdr, 2, 500) != 2) { printf("E seq=0 err=timeout\n"); continue; }
        if (read_exact(s_img, PP_PIX, 3000) != PP_PIX || read_exact(ck, 2, 500) != 2) {
            printf("E seq=%u err=timeout\n", hdr[0]);
            uart_flush_input(PORT);
            continue;
        }
        uint16_t s = 0;
        for (int i = 0; i < PP_PIX; i++) s = (uint16_t)(s + s_img[i]);
        if (s != (uint16_t)(ck[0] | (ck[1] << 8))) { printf("E seq=%u err=checksum\n", hdr[0]); continue; }

        int64_t t0 = esp_timer_get_time();
        uint32_t gsum = pp_gauss5(s_img, s_smooth, PP_BORDER_REFLECT101, &s_work);   /* như esp32_demo */
        uint32_t t_g = (uint32_t)(esp_timer_get_time() - t0);
        uint32_t t_s = 0;
        err = link_send_frame(s_smooth, hdr[0], hdr[1], &t_s);
        printf("K seq=%u fl=%u gsum=%" PRIu32 " g=%" PRIu32 " s=%" PRIu32 " spi=%s\n", hdr[0], hdr[1], gsum, t_g,
               t_s, esp_err_to_name(err));
    }
}
