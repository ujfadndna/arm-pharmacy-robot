#ifndef AUDIO_H
#define AUDIO_H

#include <stdint.h>

/**
 * @brief 上层音频 API
 *
 * 封装 IIC0 + WM8960 + SSI0, 提供简单的播放/录音接口
 * FreeRTOS 任务管理双缓冲, xTaskNotifyFromISR 唤醒
 */

/** 初始化音频子系统 (IIC0 + WM8960 + SSI0) */
int audio_init(void);

/** 播放 PCM 数据 (阻塞, 16-bit 立体声 48kHz) */
int audio_play(const int16_t *data, uint32_t samples);

/** 录音到缓冲区 (阻塞, 16-bit 立体声 48kHz) */
int audio_record(int16_t *buf, uint32_t max_samples);

/** 播放测试正弦波 (1kHz, 指定时长 ms) */
int audio_play_tone(uint32_t duration_ms);

/** 设置音量 (0~100) */
void audio_set_volume(uint8_t vol);

/** 音频子系统是否已初始化 */
int audio_is_ready(void);

/** 打印音频关键诊断信息（WM8960 寄存器影子 + SSI 状态） */
void audio_diag(void);

#endif /* AUDIO_H */
