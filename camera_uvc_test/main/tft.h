#ifndef TFT_H
#define TFT_H

/* 初始化 ILI9163（LQ_TFT18SPIV33）并创建显示任务 */
void tft_init(void);

/* 请求一次整屏刷新（由控制任务每 0.2s 调用，非阻塞） */
void tft_request_refresh(void);

/* 切换屏幕色块显示模式：近带判定 <-> 实际控制位图（串口 'c'） */
void tft_toggle_bits_mode(void);

#endif /* TFT_H */
