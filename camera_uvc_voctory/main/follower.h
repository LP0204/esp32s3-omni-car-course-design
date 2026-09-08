#ifndef FOLLOWER_H
#define FOLLOWER_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    AVOID_IDLE = 0,
    AVOID_STOP,
    AVOID_STRAFE_OUT,
    AVOID_FORWARD_PASS,
    AVOID_STRAFE_BACK,
    AVOID_REACQUIRE_PAUSE,
    AVOID_ALIGN_LINE,      /* 避障后：找线并原地转对正 */
    AVOID_ALIGN_STRAIGHT,  /* 已对正：直行一段 */
    AVOID_DONE             /* 停在此处，需按 BOOT 重新开始 */
} avoid_state_t;

/* 初始化 BOOT 键与状态（烧录后默认不运行） */
void follower_init(void);

/* 控制任务周期调用（10ms）：BOOT 启停 + 超声 + 避障 + 循迹 + 屏显触发。
 * bits 为有效位图（近带优先，近带空用远带兜底）。 */
void follower_tick(uint32_t now_ms, uint8_t bits);

/* 串口 g/s 命令等效操作 */
void follower_enable(bool en);

bool follower_is_enabled(void);

/* 当前线位位图（供显示/串口 b 命令） */
uint8_t follower_cur_bits(void);

int follower_avoid_state(void);

/* 击球阶段（T 终点停车 3s 后自动进入）：
 *   是否正处于击球阶段 / 当前击球子状态 */
bool follower_ball_phase_active(void);
int  follower_ball_state(void);
/* 当前是否处于“击绿球”子阶段（红球完成之后） */
bool follower_green_phase_active(void);

/* 串口调试：不经过 T 终点，直接强制进入击球阶段（找红球推球） */
void follower_force_ball_phase(void);

/* 独立击球控制任务入口（red-ball-push 架构）：
 * 击球阶段由它每 20ms 接管电机，不再挤在 10ms 循线控制 tick 里 */
void follower_ball_task(void *arg);

#endif /* FOLLOWER_H */
