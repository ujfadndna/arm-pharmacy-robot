#include "audio.h"
#include "drv_iic0.h"
#include "drv_ssi0.h"
#include "wm8960.h"
#include "debug_uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <string.h>

/*
 * 上层音频 API + FreeRTOS 集成
 * 双缓冲 ping-pong, xTaskNotify 唤醒
 */

/* ---------- 配置 ---------- */
#define AUDIO_SAMPLE_RATE   46875   /* 24MHz / 512 = 46.875kHz (无 PLL) */
#define AUDIO_BUF_SAMPLES   960     /* 每缓冲区 960 立体声样本 = 20ms */
#define AUDIO_BUF_SIZE      (AUDIO_BUF_SAMPLES * 2)  /* × 2 ch = 1920 int16_t = 3840 bytes */

/* ---------- 状态 ---------- */
static volatile int g_audio_ready = 0;

/* 双缓冲 */
static int16_t g_tx_buf_a[AUDIO_BUF_SIZE];
static int16_t g_tx_buf_b[AUDIO_BUF_SIZE];
static int16_t g_rx_buf_a[AUDIO_BUF_SIZE];
static int16_t g_rx_buf_b[AUDIO_BUF_SIZE];

/* 播放状态 */
static const int16_t *g_play_src = NULL;
static uint32_t g_play_total = 0;    /* 总立体声样本数 */
static uint32_t g_play_pos = 0;      /* 已拷贝位置 */
static volatile int g_play_done = 0;

/* 录音状态 */
static int16_t *g_rec_dst = NULL;
static uint32_t g_rec_max = 0;
static uint32_t g_rec_pos = 0;
static volatile int g_rec_done = 0;

/* ---------- TX 回调: SSI0 需要下一批数据 ---------- */
static void audio_tx_callback(int16_t *next_buf, uint32_t *samples)
{
    (void)next_buf;

    if (!g_play_src || g_play_pos >= g_play_total) {
        /* 播放完毕, 填静音 */
        *samples = 0;
        g_play_done = 1;
        return;
    }

    /* 选择下一个 ping-pong 缓冲区 */
    int16_t *buf = (g_play_pos / AUDIO_BUF_SAMPLES) & 1 ? g_tx_buf_b : g_tx_buf_a;
    uint32_t remain = g_play_total - g_play_pos;
    uint32_t count = (remain < AUDIO_BUF_SAMPLES) ? remain : AUDIO_BUF_SAMPLES;
    /* 拷贝 PCM 数据到缓冲区 */
    memcpy(buf, &g_play_src[g_play_pos * 2], count * 2 * sizeof(int16_t));
    g_play_pos += count;

    /* 如果不足一个缓冲区, 尾部填零 */
    if (count < AUDIO_BUF_SAMPLES) {
        memset(&buf[count * 2], 0, (AUDIO_BUF_SAMPLES - count) * 2 * sizeof(int16_t));
    }

    *samples = AUDIO_BUF_SAMPLES;
    /* 启动下一轮 DMA/中断传输 */
    drv_ssi0_start_playback(buf, AUDIO_BUF_SAMPLES);
}

/* ---------- RX 回调: SSI0 录满一个缓冲区 ---------- */
static void audio_rx_callback(int16_t *filled_buf, uint32_t samples)
{
    if (!g_rec_dst || g_rec_pos >= g_rec_max) {
        g_rec_done = 1;
        return;
    }

    uint32_t remain = g_rec_max - g_rec_pos;
    uint32_t count = (remain < samples) ? remain : samples;

    memcpy(&g_rec_dst[g_rec_pos * 2], filled_buf, count * 2 * sizeof(int16_t));
    g_rec_pos += count;

    if (g_rec_pos >= g_rec_max) {
        g_rec_done = 1;
        drv_ssi0_stop();
    } else {
        /* 继续录音到下一个缓冲区 */
        int16_t *next = (filled_buf == g_rx_buf_a) ? g_rx_buf_b : g_rx_buf_a;
        drv_ssi0_start_record(next, AUDIO_BUF_SAMPLES);
    }
}

/* ---------- 公共 API ---------- */

int audio_init(void)
{
    debug_println("[AUDIO] Initializing IIC0...");
    drv_iic0_init();
    vTaskDelay(pdMS_TO_TICKS(10));  /* WM8960 上电稳定, 等 VMID 充电 */

    debug_println("[AUDIO] Initializing WM8960...");
    int r = wm8960_init();
    if (r != 0) {
        debug_println("[AUDIO] WM8960 init FAILED (I2C NACK?)");
        return -1;
    }
    debug_println("[AUDIO] WM8960 reset OK");

    debug_println("[AUDIO] Configuring playback path...");
    wm8960_config_playback();

    debug_println("[AUDIO] Configuring record path...");
    wm8960_config_record();

    debug_println("[AUDIO] Initializing SSI0...");
    drv_ssi0_init();

    /* 注册回调 */
    drv_ssi0_set_tx_callback(audio_tx_callback);
    drv_ssi0_set_rx_callback(audio_rx_callback);

    g_audio_ready = 1;
    debug_println("[AUDIO] Init complete");
    return 0;
}

int audio_play(const int16_t *data, uint32_t samples)
{
    if (!g_audio_ready) return -1;

    g_play_src = data;
    g_play_total = samples;
    g_play_pos = 0;
    g_play_done = 0;

    /* 填充第一个缓冲区并启动 */
    uint32_t count = (samples < AUDIO_BUF_SAMPLES) ? samples : AUDIO_BUF_SAMPLES;
    memcpy(g_tx_buf_a, data, count * 2 * sizeof(int16_t));
    if (count < AUDIO_BUF_SAMPLES) {
        memset(&g_tx_buf_a[count * 2], 0, (AUDIO_BUF_SAMPLES - count) * 2 * sizeof(int16_t));
    }
    g_play_pos = count;

    drv_ssi0_start_playback(g_tx_buf_a, AUDIO_BUF_SAMPLES);

    /* 阻塞等待播放完成 */
    while (!g_play_done) {
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    drv_ssi0_stop();
    return 0;
}

int audio_record(int16_t *buf, uint32_t max_samples)
{
    if (!g_audio_ready) return -1;

    g_rec_dst = buf;
    g_rec_max = max_samples;
    g_rec_pos = 0;
    g_rec_done = 0;

    drv_ssi0_start_record(g_rx_buf_a, AUDIO_BUF_SAMPLES);

    /* 阻塞等待录音完成 */
    while (!g_rec_done) {
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    return (int)g_rec_pos;
}

int audio_play_tone(uint32_t duration_ms)
{
    if (!g_audio_ready) return -1;

    /* 预计算一个周期的 1kHz 正弦波查表 (47 样本 @ 46.875kHz ≈ 1 周期)
     * 循环输出这个查表实现任意时长播放 */
    #define TONE_PERIOD  47   /* 46875Hz / 1kHz ≈ 47 samples per period */
    static int16_t sine_lut[TONE_PERIOD];
    static int lut_ready = 0;

    if (!lut_ready) {
        for (int i = 0; i < TONE_PERIOD; i++) {
            float t = (float)i / (float)TONE_PERIOD;
            sine_lut[i] = (int16_t)(20000.0f * sinf(2.0f * 3.14159265f * t));
        }
        lut_ready = 1;
    }

    /* 总样本数 */
    uint32_t total = (AUDIO_SAMPLE_RATE * duration_ms) / 1000;
    uint32_t idx = 0;

    /* Polling 模式播放: 循环填充 FIFO, 不需要大缓冲区 */
    drv_ssi0_tx_enable();

    /* SSI0 诊断: 确认 TX 已使能, BCK/LRCK 正在输出 */
    {
        volatile uint32_t *ssicr = (volatile uint32_t *)0x4009D000UL;
        volatile uint32_t *ssisr = (volatile uint32_t *)0x4009D004UL;
        volatile uint32_t *ssifcr = (volatile uint32_t *)0x4009D010UL;
        volatile uint32_t *ssifsr = (volatile uint32_t *)0x4009D014UL;
        debug_print("[AUDIO] SSI0 diag: SSICR=0x");
        debug_print_hex(*ssicr);
        debug_print(" SSISR=0x");
        debug_print_hex(*ssisr);
        debug_print(" SSIFCR=0x");
        debug_print_hex(*ssifcr);
        debug_print(" SSIFSR=0x");
        debug_print_hex(*ssifsr);
        debug_println("");
        if (!(*ssicr & (1UL << 1))) {
            debug_println("[AUDIO] WARNING: SSI0 TEN not set!");
        }
    }

    uint32_t write_cnt = 0;
    int write_fail = 0;
    TickType_t t_start = xTaskGetTickCount();
    for (uint32_t i = 0; i < total; i++) {
        int16_t val = sine_lut[idx];
        idx++;
        if (idx >= TONE_PERIOD) idx = 0;

        /* 立体声: L+R 打包成 32-bit 写入 FIFO (SWL=32, DWL=16) */
        uint32_t sample = ((uint32_t)(uint16_t)val << 16) | (uint16_t)val;
        if (0 != drv_ssi0_poll_write(sample)) {
            volatile uint32_t *ssifsr_to = (volatile uint32_t *)0x4009D014UL;
            debug_print("[AUDIO] ERROR: TX FIFO wait timeout at i=");
            debug_print_int((int)i);
            debug_print(" ssifsr=0x");
            debug_print_hex(*ssifsr_to);
            debug_println("");
            write_fail = 1;
            break;
        }
        write_cnt++;
    }
    TickType_t t_end = xTaskGetTickCount();
    volatile uint32_t *ssifsr_end = (volatile uint32_t *)0x4009D014UL;
    drv_ssi0_stop();

    debug_print("[AUDIO] Tone done, writes=");
    debug_print_int((int)write_cnt);
    debug_print(" expected=");
    debug_print_int((int)total);
    debug_print(" elapsed_ms=");
    debug_print_int((int)((t_end - t_start) * portTICK_PERIOD_MS));
    debug_print(" ssifsr_end=0x");
    debug_print_hex(*ssifsr_end);
    debug_print(" fail=");
    debug_print_int(write_fail);
    debug_println("");

    return 0;
}

void audio_set_volume(uint8_t vol)
{
    wm8960_set_volume(vol);
}

void audio_diag(void)
{
    ssi0_diag_t d = {0};
    drv_ssi0_get_diag(&d);

    debug_println("[AUDIO] ===== DIAG =====");
    debug_print("[AUDIO] ready=");
    debug_print_int(g_audio_ready);
    debug_println("");

    debug_print("[AUDIO][SSI] SSICR=0x");
    debug_print_hex(d.ssicr);
    debug_print(" SSISR=0x");
    debug_print_hex(d.ssisr);
    debug_print(" SSIFCR=0x");
    debug_print_hex(d.ssifcr);
    debug_println("");

    debug_print("[AUDIO][SSI] SSIFSR=0x");
    debug_print_hex(d.ssifsr);
    debug_print(" SSIOFR=0x");
    debug_print_hex(d.ssiofr);
    debug_print(" SSISCR=0x");
    debug_print_hex(d.ssiscr);
    debug_println("");

    debug_print("[AUDIO][SSI] TEN=");
    debug_print_int((int) ((d.ssicr >> 1) & 1U));
    debug_print(" REN=");
    debug_print_int((int) (d.ssicr & 1U));
    debug_print(" TXI_cnt=");
    debug_print_int((int) d.txi_count);
    debug_print(" RXI_cnt=");
    debug_print_int((int) d.rxi_count);
    debug_println("");
    debug_println("[AUDIO][SSI] Note: audio tone uses polling, TXI/RXI counters may stay 0.");

    debug_print("[AUDIO][WM8960] R19(POWER1)=0x");
    debug_print_hex(wm8960_read_reg(WM8960_REG_POWER1));
    debug_print(" R1A(POWER2)=0x");
    debug_print_hex(wm8960_read_reg(WM8960_REG_POWER2));
    debug_print(" R2F(POWER3)=0x");
    debug_print_hex(wm8960_read_reg(WM8960_REG_POWER3));
    debug_println("");

    debug_print("[AUDIO][WM8960] R04(CLOCK1)=0x");
    debug_print_hex(wm8960_read_reg(WM8960_REG_CLOCK1));
    debug_print(" R07(IFACE1)=0x");
    debug_print_hex(wm8960_read_reg(WM8960_REG_IFACE1));
    debug_print(" R05(DACCTL1)=0x");
    debug_print_hex(wm8960_read_reg(WM8960_REG_DACCTL1));
    debug_println("");

    debug_print("[AUDIO][WM8960] R22(LOUTMIX)=0x");
    debug_print_hex(wm8960_read_reg(WM8960_REG_LOUTMIX));
    debug_print(" R25(ROUTMIX)=0x");
    debug_print_hex(wm8960_read_reg(WM8960_REG_ROUTMIX));
    debug_print(" R31(CLASSD1)=0x");
    debug_print_hex(wm8960_read_reg(WM8960_REG_CLASSD1));
    debug_println("");

    debug_print("[AUDIO][WM8960] R02(LOUT1)=0x");
    debug_print_hex(wm8960_read_reg(WM8960_REG_LOUT1));
    debug_print(" R03(ROUT1)=0x");
    debug_print_hex(wm8960_read_reg(WM8960_REG_ROUT1));
    debug_print(" R28(LOUT2)=0x");
    debug_print_hex(wm8960_read_reg(WM8960_REG_LOUT2));
    debug_print(" R29(ROUT2)=0x");
    debug_print_hex(wm8960_read_reg(WM8960_REG_ROUT2));
    debug_println("");
}

int audio_is_ready(void)
{
    return g_audio_ready;
}
