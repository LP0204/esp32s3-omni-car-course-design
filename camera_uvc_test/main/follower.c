/*
 * 循迹 + 超声避障控制 - ESP-IDF 版
 *
 * 由 Arduino 版 line_follower_muy_bien_ultrasonic.ino 移植，逻辑不变：
 *   - 16 状态目标转向查表 turnTable（负=左转，正=右转）
 *   - 三路压线弯道降速 ×0.7
 *   - 丢线（0000）：停车后向最近压线侧慢速原地转向寻线
 *   - 超声避障状态机：停车 -> 三轮联动左移（1:1:2）直到障碍不再识别
 *     -> 独立速度直行越障 -> 右移直到重新压线 -> 短暂停顿交还原循迹
 *   - BOOT 键（GPIO0）边沿触发 + 消抖：开始/停止；四路全白时拒绝启动
 *   - TFT 每 0.2s 触发一次整屏刷新（由独立显示任务执行，不阻塞控制）
 */

#include "follower.h"

#include <stdio.h>

#include "freertos/FreeRTOS.h"

#include "driver/gpio.h"
#include "esp_timer.h"

#include "motor.h"
#include "ultrasonic.h"
#include "tft.h"
#include "ball_vision.h"

/* main.c 暴露的视觉原始数据（近/远带分别、远带中心、处理帧号） */
extern uint8_t camera_near_bits(void);
extern uint8_t camera_far_bits(void);
extern int camera_far_cx(void);
extern uint32_t camera_seq(void);

/* ================= BOOT 键 ================= */
#define BOOT_BTN_PIN       GPIO_NUM_0
#define BOOT_DEBOUNCE_MS   40

/* ================= 避障参数（与 Arduino 版一致） ================= */
#define OBSTACLE_TRIGGER_CM      9.0f
#define OBSTACLE_CONFIRM_COUNT   1
#define OBSTACLE_CLEAR_CM        20.0f
#define AVOID_CLEAR_CONFIRM_MS   150   /* 障碍清除后再多左移 150ms */

#define DIAG_LOG_MS   1000
#define NEAR_LOG_MS   200
#define NEAR_LOG_CM   12.0f

#define AVOID_STOP_MS      120
#define STRAFE_LF_SPEED    70
#define STRAFE_RF_SPEED    77
#define STRAFE_REAR_SPEED  140
#define PASS_FORWARD_MS    1500   /* 避障直行越障时长（-0.2s） */
#define PASS_LF_SPEED      114
#define PASS_RF_SPEED      126
#define STRAFE_BACK_MS     1600   /* 直行越障后右移时长（途中看到黑线则提前结束） */
#define REACQUIRE_PAUSE_MS 120

/* 避障结束后的收尾：再看到黑线 -> 原地转对正 -> 直行后停止 */
#define POST_ALIGN_STRAIGHT_MS  1000
#define ALIGN_CENTER_HOLD_MS    80    /* 线居中后先保持静止确认，避免抖动 */
#define AVOID_ALIGN_TURN_SPEED  50    /* 对正时的转向速度 */
#define AVOID_SEARCH_TURN_SPEED 45    /* 还没看到线时的慢转找线速度 */
#define AVOID_SEARCH_FLIP_MS    1200  /* 找线超时后反转搜索方向 */

/* ================= 丢线寻线 ================= */
#define LEFT_SIDE  1
#define RIGHT_SIDE 2
#define LOST_STOP_MS     150
#define LOST_TURN_SPEED  45    /* 丢线寻线转向速度（再调低） */
#define LOST_TURN_MAX_MS 4400

/* ================= 速度/转向参数 ================= */
#define LF_SPEED            110  /* 循线基础速度（保持 110:121 走直比例） */
#define RF_SPEED            121
#define REAR_GAIN           0.5f
#define CURVE_SPEED_SCALE   0.7f
#define TURN_GAIN           0.67f /* 转向量整体缩放 */
#define PIVOT_THRESHOLD     30   /* |steer| ≥ 该值进入原地转；小偏差仍前进差速 */
#define PIVOT_MIN_SPEED     20   /* 原地转最小速度（低于起转阈值会转不动） */
#define PIVOT_MAX_SPEED     50   /* 原地转最大速度 */
#define PIVOT_DRIVEON_MS    30   /* 大转向前先直行，抵消摄像头底线与车心的 6-7cm 视差 */
#define SMALL_TURN_BOOST    1.2f /* 小弯（差速修正段）转向分量增益 */
#define DISPLAY_REFRESH_MS  200
#define T_LINE_STOP_MS      3000 /* T 型终点（四路全黑）停车时长 */

/* ================= 大转弯 PID（P+D，Ki 暂不用） =================
 * 误差口径：16 状态表原始转向量（0..90）或远带中心偏差（0..90）。
 * 速度 = clamp(Kp*err + Kd*d(err)/dt, min, max)。
 * D 项只在视觉新帧到达时更新，避免 10ms 控制周期把同一帧重复当作差分。
 */
#define PIVOT_KP                0.85f
#define PIVOT_KD                1.8f
#define PIVOT_PID_MIN_SPEED     40
#define PIVOT_PID_MAX_SPEED     80
#define PIVOT_ERR_FULL          90     /* 完全没看到线时的满幅误差 */
#define PIVOT_ERR_LP            0.5f   /* 误差低通（按视觉帧） */
#define FAR_CENTER_TOL_PX       12     /* 远带中心接近车头中心即认为已对准远处 */
#define FAR_CENTER_STABLE_FRAMES 3
#define CAPTURE_DRIVE_LF        55     /* 远带已居中、近带未到：低速直行去咬线 */
#define CAPTURE_DRIVE_RF        60
#define CAPTURE_DRIVE_MAX_MS    800

/* ================= 击球阶段参数（整体移植 red-ball-push 最新版） =================
 * 找球：短脉冲左转 -> 短刹 -> 稳定后再读一帧；
 * 对准：红球偏左/偏右就各转一个脉冲，中心过冲反向并把脉冲减半；
 * 推球：两前轮前进 PUSH_FORWARD_MS -> 停 PUSH_PAUSE_MS -> 后退 PUSH_RETURN_MS。
 * 后轮只在原地转时参与，直行/后退时后轮始终 0。
 */
#define BALL_MAP_CENTER_X         40     /* 80x60 红度图横向中心 */
#define BALL_DETECT_PIXELS        10     /* 达到该红点数才算“看到球” */
#define BALL_CONFIRM_FRAMES       3
#define BALL_CENTER_DEADBAND_PX   5      /* 红球中心与中线允许误差（80 网格 px） */
#define BALL_CENTER_CONFIRM_FRAMES 3
#define BALL_ALIGN_LOST_FRAMES    3      /* 对准中连续丢球帧数 -> 回找球 */

#define BALL_SEARCH_STEP_MS       200    /* 找球每次左转脉冲时长 */
#define BALL_ALIGN_STEP_INIT_MS   200    /* 首次对准脉冲时长（实测 <200ms 轮子可能不转） */
#define BALL_ALIGN_STEP_MIN_MS    50     /* 微调档最短脉冲 */
#define BALL_TURN_BRAKE_MS        35     /* 每个脉冲后的短刹车 */
#define BALL_TURN_SETTLE_MS       120    /* 刹停后等车身稳定再读帧 */

/* 电机速度按 camera_uvc_test 的 0..255（8-bit）速度表示；
 * 与 red-ball-push 的百分比对应：speed = percent * 255 / 100 */
#define BALL_TURN_SPEED           255    /* 转向脉冲功率（对应 TURN_STEP_POWER=120，其内部钳到 100%） */
#define BALL_PUSH_LF_SPEED        138    /* 左前轮 54%（red-ball-push LEFT_STRAIGHT_POWER_PERCENT） */
#define BALL_PUSH_RF_SPEED        166    /* 右前轮 65%（red-ball-push RIGHT_STRAIGHT_POWER_PERCENT） */
#define BALL_PUSH_FORWARD_MS      900
#define BALL_PUSH_PAUSE_MS        500
#define BALL_PUSH_RETURN_MS       930

/* 16 状态目标转向查表：
 *   bit3=最左区 ... bit0=最右区，1=压黑线
 *   负值 = 左转，正值 = 右转；0 = 直行 */
static const int8_t turnTable[16] = {
    /*0000*/ 0,   /*0001*/ 90,  /*0010*/ 0,   /*0011*/ 70,
    /*0100*/ 0,   /*0101*/ 50,  /*0110*/ 0,   /*0111*/ 70,
    /*1000*/ -90, /*1001*/ 0,   /*1010*/ -50, /*1011*/ 0,
    /*1100*/ -70, /*1101*/ 0,   /*1110*/ -70, /*1111*/ 0
};

static bool follow_enabled = false;      /* 烧录后默认不运行 */
static uint8_t cur_bits = 0;
static int last_side = 0;                /* 最近压线侧：0=未知 1=左 2=右 */
static bool lost_active = false;
static int lost_phase = 0;
static uint32_t lost_phase_start_ms = 0;
static int prev_steer = 0;
static uint8_t last_bits = 0xFF;

static int btn_prev_level = 1;
static uint32_t btn_last_change_ms = 0;

static uint32_t last_diag_ms = 0;
static uint32_t last_near_ms = 0;
static uint32_t last_disp_ms = 0;
static int obstacle_hit_count = 0;
static uint32_t clear_since_ms = 0;

static avoid_state_t avoid_state = AVOID_IDLE;
static uint32_t avoid_state_start_ms = 0;
static uint32_t avoid_timer_ms = 0;          /* 找线翻转/计时用 */
static uint32_t avoid_center_since_ms = 0;   /* 对正：线居中保持起点 */
static int avoid_search_dir = -1;            /* 找线方向：-1 左 / +1 右 */

/* 任务阶段：是否触发过避障、是否进入击球阶段 */
static bool avoidance_happened = false;
static bool t_end_stopping = false;
static uint32_t t_end_stop_start_ms = 0;
static volatile bool ball_phase = false;

/* 击球阶段子状态机（与 red-ball-push 一致） */
typedef enum {
    BALL_OFF = 0,
    BALL_SEARCH,
    BALL_ALIGN,
    BALL_PUSH,
    BALL_PAUSE,
    BALL_RETURN,
    BALL_DONE
} ball_phase_state_t;

static ball_phase_state_t ball_state = BALL_OFF;
static uint32_t ball_state_start_ms = 0;   /* PUSH/PAUSE/RETURN 阶段起点 */

/* 找球/对准状态 */
static int ball_detect_conf = 0;           /* 连续看到球帧数 */
static int ball_center_conf = 0;           /* 连续居中帧数 */
static int ball_lost_frames = 0;           /* 对准中连续丢球帧数 */
static int ball_align_step_ms = BALL_ALIGN_STEP_INIT_MS;
static int ball_prev_err_side = 0;         /* 上一次红球偏差方向：-1 左 / +1 右 / 0 未知 */
static int ball_prev_err = 0;              /* 上一次偏差值（判断脉冲是否有改善） */
static bool ball_search_need_step = false;

/* 脉冲-刹车-稳定 时序 */
static bool ball_turn_active = false;
static bool ball_brake_active = false;
static uint32_t ball_turn_until_ms = 0;
static uint32_t ball_brake_until_ms = 0;
static uint32_t ball_settle_until_ms = 0;
static uint32_t ball_last_frame_id = 0;

static bool big_turn_pending = false;   /* 大转向前置直行状态 */
static uint32_t big_turn_start_ms = 0;

/* PID / 远带捕获状态 */
static float pid_err_f = 0.0f;
static float pid_d_term = 0.0f;
static uint32_t pid_last_seq = 0;
static int64_t pid_last_us = 0;
static int far_center_frames = 0;
static uint32_t far_center_last_seq = 0;
static uint32_t center_drive_start_ms = 0;
static uint32_t turn_last_log_ms = 0;

/* ================= BOOT 键 ================= */

static bool boot_button_pressed(uint32_t now)
{
    int level = gpio_get_level(BOOT_BTN_PIN);
    if (level != btn_prev_level && now - btn_last_change_ms >= BOOT_DEBOUNCE_MS) {
        btn_prev_level = level;
        btn_last_change_ms = now;
        return level == 0;
    }
    return false;
}

/* ================= 电机动作 ================= */

static void stop_all_motors(void)
{
    motor_set_lf(0);
    motor_set_rf(0);
    motor_set_rear(0);
}

/* 原地转向：side<0 左转，side>0 右转（1:1:1 纯旋转） */
static void apply_turn_dir(int side, int speed)
{
    if (side < 0) {   /* 左转 */
        motor_set_lf(-speed);
        motor_set_rf(speed);
        motor_set_rear(speed);
    } else {
        motor_set_lf(speed);
        motor_set_rf(-speed);
        motor_set_rear(-speed);
    }
}

/* ================= 大转弯 PID 辅助 ================= */

static void turn_pid_reset(void)
{
    pid_last_seq = 0;
    pid_last_us = 0;
    pid_err_f = 0.0f;
    pid_d_term = 0.0f;
}

/* 视觉新帧到达时推进误差低通与微分项（D 为负 = 误差在缩小 = 刹车） */
static void turn_pid_new_frame(int err_mag)
{
    uint32_t seq = camera_seq();
    if (seq == pid_last_seq) {
        return;
    }
    if (err_mag > PIVOT_ERR_FULL) err_mag = PIVOT_ERR_FULL;
    if (err_mag < 0) err_mag = 0;

    int64_t now_us = esp_timer_get_time();
    if (pid_last_seq != 0 && pid_last_us > 0) {
        float dt = (float)(now_us - pid_last_us) / 1000000.0f;
        if (dt <= 0.002f || dt > 0.5f) {
            dt = 0.08f;   /* 防除零 / 长时间停帧 */
        }
        float prev = pid_err_f;
        pid_err_f += PIVOT_ERR_LP * ((float)err_mag - pid_err_f);
        pid_d_term = PIVOT_KD * (pid_err_f - prev) / dt;
    } else {
        pid_err_f = (float)err_mag;
        pid_d_term = 0.0f;
    }
    pid_last_seq = seq;
    pid_last_us = now_us;
}

/* PID 转向速度（≥0）；err_mag 为当前误差幅值（0..90） */
static int turn_pid_speed(int err_mag)
{
    turn_pid_new_frame(err_mag);
    float v = PIVOT_KP * (float)err_mag + pid_d_term;
    int vi = (int)v;
    if (vi < PIVOT_PID_MIN_SPEED) vi = PIVOT_PID_MIN_SPEED;
    if (vi > PIVOT_PID_MAX_SPEED) vi = PIVOT_PID_MAX_SPEED;
    return vi;
}

/*
 * 三轮联动横移（1:1:2，三参数独立）：
 *   direction: -1=左移，+1=右移
 */
static void apply_strafe(int direction)
{
    if (direction < 0) {
        motor_set_lf(-STRAFE_LF_SPEED);
        motor_set_rf(STRAFE_RF_SPEED);
        motor_set_rear(-STRAFE_REAR_SPEED);
    } else {
        motor_set_lf(STRAFE_LF_SPEED);
        motor_set_rf(-STRAFE_RF_SPEED);
        motor_set_rear(STRAFE_REAR_SPEED);
    }
}

/* 线位于车头正前方（中间两路）的位图模式 */
static bool line_centered_bits(uint8_t bits)
{
    return (bits == 0x02 || bits == 0x04 || bits == 0x06);
}

static void start_avoidance(uint32_t now)
{
    avoid_state = AVOID_STOP;
    avoid_state_start_ms = now;
    avoid_timer_ms = 0;
    avoid_center_since_ms = 0;
    avoid_search_dir = -1;
    avoidance_happened = true;   /* 触发过避障后，才允许 T 型终点判定 */
    obstacle_hit_count = 0;
    clear_since_ms = 0;
    big_turn_pending = false;

    lost_active = false;
    lost_phase = 0;
    prev_steer = 0;
    turn_pid_reset();
    far_center_frames = 0;
    center_drive_start_ms = 0;

    stop_all_motors();
    printf("AVOID: obstacle detected, stop\n");
}

/* 返回 true 表示当前由避障状态机接管电机 */
static bool handle_avoidance(uint32_t now, uint8_t bits)
{
    if (avoid_state == AVOID_IDLE) {
        return false;
    }

    uint32_t elapsed = now - avoid_state_start_ms;
    float raw = ultrasonic_raw_cm();

    switch (avoid_state) {
    case AVOID_STOP:
        stop_all_motors();
        if (elapsed >= AVOID_STOP_MS) {
            avoid_state = AVOID_STRAFE_OUT;
            avoid_state_start_ms = now;
            printf("AVOID: strafe LEFT\n");
        }
        break;

    case AVOID_STRAFE_OUT:
        apply_strafe(-1);
        {
            bool still_seen = (raw > 1.5f && raw <= OBSTACLE_CLEAR_CM);
            if (!still_seen) {
                if (clear_since_ms == 0) {
                    clear_since_ms = now;
                } else if (now - clear_since_ms >= AVOID_CLEAR_CONFIRM_MS) {
                    clear_since_ms = 0;
                    avoid_state = AVOID_FORWARD_PASS;
                    avoid_state_start_ms = now;
                    printf("AVOID: obstacle clear, pass obstacle\n");
                }
            } else {
                clear_since_ms = 0;
            }
        }
        break;

    case AVOID_FORWARD_PASS:
        motor_set_lf(PASS_LF_SPEED);
        motor_set_rf(PASS_RF_SPEED);
        motor_set_rear(0);
        if (elapsed >= PASS_FORWARD_MS) {
            avoid_state = AVOID_STRAFE_BACK;
            avoid_state_start_ms = now;
            printf("AVOID: strafe RIGHT, searching line\n");
        }
        break;

    case AVOID_STRAFE_BACK:
        apply_strafe(1);
        /* 右移途中重新看到黑线：立即进入 1000ms 直行段 */
        if (bits != 0) {
            stop_all_motors();
            clear_since_ms = 0;
            lost_active = false;
            prev_steer = 0;
            avoid_timer_ms = 0;
            avoid_center_since_ms = 0;
            avoid_state = AVOID_ALIGN_STRAIGHT;
            avoid_state_start_ms = now;
            printf("AVOID: line seen during right strafe (%lums) -> straight %dms\n",
                   (unsigned long)elapsed, POST_ALIGN_STRAIGHT_MS);
        } else if (elapsed >= STRAFE_BACK_MS) {
            /* 满时长仍没看到线：按原流程走“停顿 -> 找线对正 -> 直行” */
            stop_all_motors();
            avoid_state = AVOID_REACQUIRE_PAUSE;
            avoid_state_start_ms = now;
            printf("AVOID: fixed right strafe done (%dms)\n", STRAFE_BACK_MS);
        }
        break;

    case AVOID_REACQUIRE_PAUSE:
        stop_all_motors();
        if (elapsed >= REACQUIRE_PAUSE_MS) {
            clear_since_ms = 0;
            lost_active = false;
            prev_steer = 0;
            avoid_timer_ms = 0;
            avoid_center_since_ms = 0;
            avoid_search_dir = -1;   /* 右移过线后默认左转找线 */
            avoid_state = AVOID_ALIGN_LINE;
            avoid_state_start_ms = now;
            printf("AVOID: pause done, search & align to line\n");
        }
        break;

    case AVOID_ALIGN_LINE:
        /* 还没看到线：慢转找线，超时自动反转方向 */
        if (bits == 0) {
            avoid_center_since_ms = 0;
            if (avoid_timer_ms == 0) {
                avoid_timer_ms = now;
            } else if (now - avoid_timer_ms >= AVOID_SEARCH_FLIP_MS) {
                avoid_search_dir = -avoid_search_dir;
                avoid_timer_ms = now;
                printf("AVOID: no line, flip search dir %s\n",
                       avoid_search_dir < 0 ? "LEFT" : "RIGHT");
            }
            apply_turn_dir(avoid_search_dir, AVOID_SEARCH_TURN_SPEED);
            break;
        }

        avoid_timer_ms = 0;

        /* 严格对正：只看近带（车正下方）中间位型，先停住确认 80ms 再直行 */
        if (line_centered_bits(camera_near_bits())) {
            stop_all_motors();
            if (avoid_center_since_ms == 0) {
                avoid_center_since_ms = now;
            } else if (now - avoid_center_since_ms >= ALIGN_CENTER_HOLD_MS) {
                avoid_center_since_ms = 0;
                avoid_state = AVOID_ALIGN_STRAIGHT;
                avoid_state_start_ms = now;
                printf("AVOID: line centered, straight %dms\n",
                       POST_ALIGN_STRAIGHT_MS);
            }
            break;
        }

        /* 线在侧边：原地向线方向转，直到居中 */
        avoid_center_since_ms = 0;
        {
            int raw = (int)turnTable[bits & 0x0F];
            int dir;
            int v;
            if (raw == 0) {
                /* 全黑/异常位图：先按找线方向慢慢转，等位图变正常 */
                dir = avoid_search_dir;
                v = AVOID_SEARCH_TURN_SPEED;
            } else {
                if (raw < 0) last_side = LEFT_SIDE;
                else last_side = RIGHT_SIDE;
                int av = (raw < 0) ? -raw : raw;
                dir = (raw < 0) ? -1 : 1;
                v = (av >= 70) ? AVOID_ALIGN_TURN_SPEED
                               : AVOID_SEARCH_TURN_SPEED;
            }
            apply_turn_dir(dir, v);
        }
        break;

    case AVOID_ALIGN_STRAIGHT:
        /* 直行途中遇到 T 型终点（四路全黑）：停车 3s -> 击球阶段 */
        if (bits == 0x0F) {
            stop_all_motors();
            avoid_state = AVOID_IDLE;
            clear_since_ms = 0;
            lost_active = false;
            prev_steer = 0;
            t_end_stopping = true;
            t_end_stop_start_ms = now;
            printf("AVOID: T line during straight, stop %dms -> ball phase\n",
                   T_LINE_STOP_MS);
            break;
        }
        motor_set_lf(LF_SPEED);
        motor_set_rf(RF_SPEED);
        motor_set_rear(0);
        if (elapsed >= POST_ALIGN_STRAIGHT_MS) {
            stop_all_motors();
            avoid_state = AVOID_IDLE;
            clear_since_ms = 0;
            lost_active = false;
            prev_steer = 0;
            t_end_stopping = true;
            t_end_stop_start_ms = now;
            printf("AVOID: straight %dms done without T, stop %dms -> ball phase\n",
                   POST_ALIGN_STRAIGHT_MS, T_LINE_STOP_MS);
        }
        break;

    case AVOID_DONE:
        stop_all_motors();
        break;

    default:
        avoid_state = AVOID_IDLE;
        break;
    }

    return true;
}

/* ================= 丢线 / 大转弯恢复（远带 PID 捕获） =================
 * lost_phase:
 *   0 = 停车刹车
 *   1 = 转向：远带空时按满幅误差 PID；远带出现后按远带中心误差 PID，
 *       误差越小转得越慢（D 项提前刹车），避免高速冲过头
 *   2 = 长时间找不到线，停车放弃
 *   3 = 远带已居中但近带未到：低速直行一段去咬近带
 */
static bool handle_lost_state(uint32_t now, uint8_t near_b, uint8_t far_b)
{
    int side = (last_side == LEFT_SIDE) ? -1 : 1;

    if (!lost_active) {
        if (near_b != 0 || far_b != 0) {
            return false;
        }
        lost_active = true;
        lost_phase = 0;
        lost_phase_start_ms = now;
        prev_steer = 0;
        big_turn_pending = false;
        far_center_frames = 0;
        far_center_last_seq = 0;
        center_drive_start_ms = 0;
        turn_pid_reset();
        stop_all_motors();
        printf("LOST: stop\n");
        return true;
    }

    /* 近带重新咬线：恢复完成，交给 16 状态表继续 */
    if (near_b != 0) {
        lost_active = false;
        lost_phase = 0;
        far_center_frames = 0;
        far_center_last_seq = 0;
        center_drive_start_ms = 0;
        prev_steer = 0;
        turn_pid_reset();
        printf("LOST: line reacquired\n");
        return false;
    }

    if (lost_phase == 0) {
        if (now - lost_phase_start_ms >= LOST_STOP_MS) {
            lost_phase = 1;
            lost_phase_start_ms = now;
            printf("LOST: turning %s\n", side < 0 ? "LEFT" : "RIGHT");
        }
        return true;
    }

    if (lost_phase == 2) {
        stop_all_motors();
        return true;
    }

    if (lost_phase == 1 && far_b == 0 &&
        now - lost_phase_start_ms >= LOST_TURN_MAX_MS) {
        lost_phase = 2;
        stop_all_motors();
        printf("LOST: give up turning (timeout)\n");
        return true;
    }

    /* 误差幅值：远带空 = 满幅；远带出现 = 用远带中心到车头中心的距离 */
    int err_mag = PIVOT_ERR_FULL;
    if (far_b != 0) {
        int fcx = camera_far_cx();
        if (fcx >= 0) {
            err_mag = fcx - 80;
            if (err_mag < 0) err_mag = -err_mag;
        }
    }

    /* 远带已居中、近带还没到：低速直行去咬近带，避免继续转造成过冲 */
    if (far_b != 0 && err_mag <= FAR_CENTER_TOL_PX) {
        uint32_t seq = camera_seq();
        if (seq != far_center_last_seq) {
            far_center_last_seq = seq;
            far_center_frames++;
        }
        if (far_center_frames >= FAR_CENTER_STABLE_FRAMES) {
            if (lost_phase != 3) {
                lost_phase = 3;
                center_drive_start_ms = now;
                printf("TURN: far centered err=%d -> creep forward\n", err_mag);
            }
        } else if (lost_phase == 3) {
            lost_phase = 1;
        }
    } else {
        far_center_last_seq = camera_seq();
        far_center_frames = 0;
        if (lost_phase == 3) {
            lost_phase = 1;
        }
    }

    if (lost_phase == 3) {
        if (now - center_drive_start_ms >= CAPTURE_DRIVE_MAX_MS) {
            lost_phase = 1;
            printf("TURN: creep timeout, resume turning\n");
        } else {
            motor_set_lf(CAPTURE_DRIVE_LF);
            motor_set_rf(CAPTURE_DRIVE_RF);
            motor_set_rear(0);
            return true;
        }
    }

    int v = turn_pid_speed(err_mag);
    apply_turn_dir(side, v);

    if (now - turn_last_log_ms >= 250) {
        turn_last_log_ms = now;
        printf("TURN: lost far=0b%d%d%d%d err=%d v=%d %s\n",
               (far_b >> 3) & 1, (far_b >> 2) & 1, (far_b >> 1) & 1, far_b & 1,
               err_mag, v, side < 0 ? "LEFT" : "RIGHT");
    }
    return true;
}

/* ================= 击球阶段（整体移植 red-ball-push 最新版） ================= */

static void ball_reset_all(void)
{
    ball_state = BALL_OFF;
    ball_state_start_ms = 0;
    ball_detect_conf = 0;
    ball_center_conf = 0;
    ball_lost_frames = 0;
    ball_align_step_ms = BALL_ALIGN_STEP_INIT_MS;
    ball_prev_err_side = 0;
    ball_prev_err = 0;
    ball_search_need_step = false;
    ball_turn_active = false;
    ball_brake_active = false;
    ball_turn_until_ms = 0;
    ball_brake_until_ms = 0;
    ball_settle_until_ms = 0;
    ball_last_frame_id = 0;
}

static void enter_ball_phase(uint32_t now)
{
    ball_phase = true;
    ball_reset_all();
    ball_state = BALL_SEARCH;
    ball_search_need_step = true;
    ball_settle_until_ms = now;   /* 进入即允许读帧/发第一个脉冲 */

    /* ===== 彻底遗忘循线/避障的一切状态，完全交给击球逻辑 ===== */
    last_side = 0;
    cur_bits = 0;
    last_bits = 0xFF;
    lost_active = false;
    lost_phase = 0;
    lost_phase_start_ms = 0;
    prev_steer = 0;
    avoid_state = AVOID_IDLE;
    avoid_state_start_ms = 0;
    avoid_timer_ms = 0;
    avoid_center_since_ms = 0;
    avoid_search_dir = -1;
    obstacle_hit_count = 0;
    clear_since_ms = 0;
    t_end_stopping = false;
    t_end_stop_start_ms = 0;
    big_turn_pending = false;
    big_turn_start_ms = 0;
    far_center_frames = 0;
    far_center_last_seq = 0;
    center_drive_start_ms = 0;
    turn_pid_reset();

    ball_vision_set_enabled(true);
    stop_all_motors();
    /* 进入击球立刻让屏幕切红，不等 200ms 的定时刷新 */
    tft_request_refresh();
    printf("BALL: phase entered - stepped LEFT search (step=%dms)\n",
           BALL_SEARCH_STEP_MS);
}

static void ball_start_turn(uint32_t now, int side, int step_ms)
{
    apply_turn_dir(side, BALL_TURN_SPEED);
    ball_turn_active = true;
    ball_turn_until_ms = now + (uint32_t)step_ms;
}

static void handle_ball_phase(uint32_t now)
{
    ball_vision_t v;
    bool got = ball_vision_get(&v);

    /* 每个转向脉冲：到时 -> 短刹（抑制惯量） -> 刹停 -> 稳定等待再读帧 */
    if (ball_turn_active && now >= ball_turn_until_ms) {
        motor_brake_all();
        ball_turn_active = false;
        ball_brake_active = true;
        ball_brake_until_ms = now + BALL_TURN_BRAKE_MS;
    }
    if (ball_brake_active && now >= ball_brake_until_ms) {
        stop_all_motors();
        ball_brake_active = false;
        ball_settle_until_ms = now + BALL_TURN_SETTLE_MS;
    }

    bool frame_new = got && (v.frame_id != ball_last_frame_id);
    if (frame_new) {
        ball_last_frame_id = v.frame_id;
    }

    /* 找球 / 对准只在：无脉冲、无刹车、已稳定、且视觉出了新帧时判断 */
    if ((ball_state == BALL_SEARCH || ball_state == BALL_ALIGN) &&
        !ball_turn_active && !ball_brake_active &&
        now >= ball_settle_until_ms && frame_new) {

        bool red_detected = got && v.ball_pixels >= BALL_DETECT_PIXELS;

        if (ball_state == BALL_SEARCH) {
            ball_detect_conf = red_detected ? ball_detect_conf + 1 : 0;
            ball_search_need_step = !red_detected;
            if (ball_detect_conf >= BALL_CONFIRM_FRAMES) {
                ball_state = BALL_ALIGN;
                ball_center_conf = 0;
                ball_lost_frames = 0;
                ball_align_step_ms = BALL_ALIGN_STEP_INIT_MS;
                ball_prev_err_side = 0;
                ball_prev_err = 0;
                ball_search_need_step = false;
                printf("BALL: red detected pix=%d cx=%d -> ALIGN\n",
                       v.ball_pixels, v.ball_cx);
            }
        }

        if (ball_state == BALL_ALIGN) {
            if (!red_detected) {
                ++ball_lost_frames;
                ball_center_conf = 0;
                if (ball_lost_frames >= BALL_ALIGN_LOST_FRAMES) {
                    ball_state = BALL_SEARCH;
                    ball_detect_conf = 0;
                    ball_align_step_ms = BALL_ALIGN_STEP_INIT_MS;
                    ball_prev_err_side = 0;
                    ball_prev_err = 0;
                    ball_search_need_step = true;
                    printf("BALL: red lost while aligning -> SEARCH\n");
                }
            } else {
                ball_lost_frames = 0;
                int err = v.ball_cx - BALL_MAP_CENTER_X;
                int dist = (err < 0) ? -err : err;
                if (dist <= BALL_CENTER_DEADBAND_PX) {
                    stop_all_motors();
                    ++ball_center_conf;
                    if (ball_center_conf >= BALL_CENTER_CONFIRM_FRAMES) {
                        ball_state = BALL_PUSH;
                        ball_state_start_ms = now;
                        ball_prev_err_side = 0;
                        ball_prev_err = 0;
                        printf("*** BALL: centered cx=%d pix=%d -> PUSH ***\n",
                               v.ball_cx, v.ball_pixels);
                    }
                } else {
                    ball_center_conf = 0;
                    int err_side = (err < 0) ? -1 : 1;
                    if (ball_prev_err_side != 0 &&
                        err_side != ball_prev_err_side) {
                        /* 中心过冲：反向并把脉冲时长减半 */
                        ball_align_step_ms /= 2;
                        if (ball_align_step_ms < BALL_ALIGN_STEP_MIN_MS) {
                            ball_align_step_ms = BALL_ALIGN_STEP_MIN_MS;
                        }
                        printf("BALL: center crossed; reverse half step=%dms\n",
                               ball_align_step_ms);
                    } else if (ball_prev_err_side != 0 &&
                               err_side == ball_prev_err_side &&
                               ball_align_step_ms < BALL_ALIGN_STEP_INIT_MS) {
                        /* 同方向但误差没改善（多半是脉冲太短轮子没走）：
                         * 自动把脉冲时长加回去，避免卡在减半后的“死脉冲” */
                        int dist_prev = (ball_prev_err < 0) ? -ball_prev_err
                                                            : ball_prev_err;
                        if (dist_prev > BALL_CENTER_DEADBAND_PX &&
                            dist >= dist_prev) {
                            ball_align_step_ms *= 2;
                            if (ball_align_step_ms > BALL_ALIGN_STEP_INIT_MS) {
                                ball_align_step_ms = BALL_ALIGN_STEP_INIT_MS;
                            }
                            printf("BALL: no progress, restore step=%dms\n",
                                   ball_align_step_ms);
                        }
                    }
                    ball_prev_err_side = err_side;
                    ball_prev_err = err;
                    ball_start_turn(now, err_side, ball_align_step_ms);
                    printf("BALL: align step err=%d %s step=%dms\n",
                           err, err_side < 0 ? "LEFT" : "RIGHT",
                           ball_align_step_ms);
                }
            }
        }
    }

    if (ball_state == BALL_SEARCH && ball_search_need_step &&
        !ball_turn_active && !ball_brake_active &&
        now >= ball_settle_until_ms) {
        ball_start_turn(now, -1, BALL_SEARCH_STEP_MS);
        ball_search_need_step = false;
        printf("BALL: search step LEFT %dms\n", BALL_SEARCH_STEP_MS);
    } else if (ball_state == BALL_PUSH) {
        motor_set_lf(BALL_PUSH_LF_SPEED);
        motor_set_rf(BALL_PUSH_RF_SPEED);
        motor_set_rear(0);
        if (now - ball_state_start_ms >= BALL_PUSH_FORWARD_MS) {
            stop_all_motors();
            ball_state = BALL_PAUSE;
            ball_state_start_ms = now;
            printf("BALL: push forward done, pause\n");
        }
    } else if (ball_state == BALL_PAUSE) {
        stop_all_motors();
        if (now - ball_state_start_ms >= BALL_PUSH_PAUSE_MS) {
            ball_state = BALL_RETURN;
            ball_state_start_ms = now;
            printf("BALL: pause done, return backward\n");
        }
    } else if (ball_state == BALL_RETURN) {
        motor_set_lf(-BALL_PUSH_LF_SPEED);
        motor_set_rf(-BALL_PUSH_RF_SPEED);
        motor_set_rear(0);
        if (now - ball_state_start_ms >= BALL_PUSH_RETURN_MS) {
            stop_all_motors();
            ball_state = BALL_DONE;
            printf("BALL: return done, phase complete\n");
        }
    } else {
        /* BALL_DONE / BALL_OFF */
        stop_all_motors();
    }
}

/* ================= 控制周期 ================= */

static void print_debug(uint8_t bits, int target, int lf, int rf, int rear)
{
    printf("CAM=0b%d%d%d%d tg=%d lf=%d rf=%d rear=%d\n",
           (bits >> 3) & 1, (bits >> 2) & 1, (bits >> 1) & 1, bits & 1,
           target, lf, rf, rear);
}

void follower_tick(uint32_t now, uint8_t bits)
{
    cur_bits = bits;

    /* ---------- BOOT 键：启动/停止（边沿触发 + 消抖） ---------- */
    if (boot_button_pressed(now)) {
        if (follow_enabled) {
            follower_enable(false);
            printf("BTN: STOP\n");
        } else if (bits == 0) {
            /* 无线可循时拒绝启动 */
            printf("BTN: start rejected (no line under sensors)\n");
        } else {
            follower_enable(true);
            printf("BTN: GO\n");
        }
    }

    /* ---------- 击球阶段：循线/避障程序完全退出 ----------
     * 只保留 BOOT 启停、小屏幕刷新和击球状态机；
     * 电机由独立 follower_ball_task 驱动，这里只刷屏并让出。 */
    if (ball_phase) {
        if (now - last_disp_ms >= DISPLAY_REFRESH_MS) {
            last_disp_ms = now;
            tft_request_refresh();
        }
        return;
    }

    /* 超声测距由独立 ultrasonic_task 负责（阻塞式读取不占控制周期） */

    /* ---------- 运行诊断日志 ---------- */
    if (follow_enabled && now - last_diag_ms >= DIAG_LOG_MS) {
        last_diag_ms = now;
        bool in_range = (ultrasonic_raw_cm() > 1.5f &&
                         ultrasonic_raw_cm() <= OBSTACLE_TRIGGER_CM);

        printf("DBG dist=");
        if (ultrasonic_raw_cm() > 0.0f) {
            printf("%.1f", ultrasonic_raw_cm());
        } else {
            printf("?");
        }
        printf("cm range=%c follow=%d avd=%d hit=%d CAM=0b%d%d%d%d trigOK=%c\n",
               in_range ? 'Y' : 'N',
               follow_enabled ? 1 : 0,
               (int)avoid_state,
               obstacle_hit_count,
               (bits >> 3) & 1, (bits >> 2) & 1, (bits >> 1) & 1, bits & 1,
               (follow_enabled && avoid_state == AVOID_IDLE && in_range) ? 'Y' : 'N');
    }

    if (follow_enabled && !ball_phase &&
        ultrasonic_raw_cm() > 1.5f &&
        ultrasonic_raw_cm() <= NEAR_LOG_CM &&
        now - last_near_ms >= NEAR_LOG_MS) {
        last_near_ms = now;
        printf("NEAR dist=%.1fcm hit=%d\n",
               ultrasonic_raw_cm(), obstacle_hit_count);
    }

    /* ---------- 障碍触发：刹车优先，先于屏显 ---------- */
    if (follow_enabled && !ball_phase &&
        avoid_state == AVOID_IDLE &&
        ultrasonic_raw_cm() > 1.5f &&
        ultrasonic_raw_cm() <= OBSTACLE_TRIGGER_CM) {
        obstacle_hit_count++;

        if (obstacle_hit_count >= OBSTACLE_CONFIRM_COUNT) {
            printf("AVOID TRIGGER: %.1f cm\n", ultrasonic_raw_cm());
            start_avoidance(now);
        }
    } else {
        obstacle_hit_count = 0;
    }

    /* ---------- TFT 定时整屏刷新（0.2s，由显示任务执行） ---------- */
    if (now - last_disp_ms >= DISPLAY_REFRESH_MS) {
        last_disp_ms = now;
        tft_request_refresh();
    }

    if (!follow_enabled) {
        return;
    }

    /* ---------- 避障状态机独占电机 ---------- */
    if (handle_avoidance(now, bits)) {
        return;
    }

    /* ---------- 避障完成后的 T 型终点检测 ----------
     * 必须先触发过一次避障；识别到 T 型终点（四路全黑 0b1111）
     * 才停车 3 秒并进入击球阶段。 */
    if (avoidance_happened) {
        if (t_end_stopping) {
            stop_all_motors();
            if (now - t_end_stop_start_ms >= T_LINE_STOP_MS) {
                t_end_stopping = false;
                printf("T END: 3s stop done, entering ball phase\n");
                enter_ball_phase(now);
            }
            return;
        }
        if (bits == 0x0F) {
            t_end_stopping = true;
            t_end_stop_start_ms = now;
            stop_all_motors();
            printf("T END: all black detected, stop %dms\n", T_LINE_STOP_MS);
            return;
        }
    }

    /* ---------- 丢线（近带+远带全空） / 大转弯远带 PID 捕获 ---------- */
    {
        uint8_t near_b = camera_near_bits();
        uint8_t far_b = camera_far_bits();
        if (handle_lost_state(now, near_b, far_b)) {
            return;
        }
    }

    /* ---------- 16 状态查表 ---------- */
    int target = (int)(turnTable[bits & 0x0F] * TURN_GAIN);

    if (target < 0) {
        last_side = LEFT_SIDE;
    } else if (target > 0) {
        last_side = RIGHT_SIDE;
    }

    /* 一阶平滑（新目标占 1/3，响应比原来 1/4 快一些） */
    int steer = (target + 2 * prev_steer) / 3;
    prev_steer = steer;

    /* 弯道降速：三路压线时 ×0.7 */
    int on_count = ((bits & 0x08) ? 1 : 0) + ((bits & 0x04) ? 1 : 0) +
                   ((bits & 0x02) ? 1 : 0) + ((bits & 0x01) ? 1 : 0);
    float speed_scale = (on_count == 3) ? CURVE_SPEED_SCALE : 1.0f;

    /* 大转向：原地转（1:1:1 纯旋转）；小偏差：前进差速修正 */
    int lf, rf, rear;
    int av = (steer < 0) ? -steer : steer;
    if (av >= PIVOT_THRESHOLD) {
        /* 进入大转向前先直行 30ms（只触发一次），
         * 让车前 6-7cm 视差的那段误差先被走掉，避免提前原地转 */
        if (!big_turn_pending) {
            big_turn_pending = true;
            big_turn_start_ms = now;
            turn_pid_reset();   /* 新一轮大转弯，清掉上一轮 PID 差分 */
        }
        if (now - big_turn_start_ms < PIVOT_DRIVEON_MS) {
            lf = (int)(LF_SPEED * speed_scale);
            rf = (int)(RF_SPEED * speed_scale);
            rear = 0;
        } else {
            big_turn_pending = false;
            /* PID 大转弯：误差取 16 状态表原始量（未乘 TURN_GAIN），
             * 误差大转得快；误差随线进入而缩小时 D 项提前刹车防过冲 */
            int raw_mag = (int)turnTable[bits & 0x0F];
            if (raw_mag < 0) raw_mag = -raw_mag;
            int v = turn_pid_speed(raw_mag);
            if (speed_scale < 1.0f) {
                v = (int)((float)v * speed_scale);
                if (v < PIVOT_MIN_SPEED) v = PIVOT_MIN_SPEED;
            }
            if (steer < 0) {   /* 左转 */
                lf = -v;
                rf = v;
                rear = v;
            } else {           /* 右转 */
                lf = v;
                rf = -v;
                rear = -v;
            }
        }
    } else {
        big_turn_pending = false;
        /* 小弯：差速分量略放大，让小幅修正更跟手 */
        int ts = (int)(steer * SMALL_TURN_BOOST);
        lf = (int)((LF_SPEED + ts) * speed_scale);
        rf = (int)((RF_SPEED - ts) * speed_scale);
        rear = (int)(-ts * REAR_GAIN);
    }

    motor_set_lf(lf);
    motor_set_rf(rf);
    motor_set_rear(rear);

    if (bits != last_bits) {
        print_debug(bits, target, lf, rf, rear);
        last_bits = bits;
    }
}

void follower_enable(bool en)
{
    follow_enabled = en;
    prev_steer = 0;
    last_side = 0;
    avoidance_happened = false;
    avoid_state = AVOID_IDLE;
    avoid_timer_ms = 0;
    avoid_center_since_ms = 0;
    avoid_search_dir = -1;
    obstacle_hit_count = 0;
    clear_since_ms = 0;
    t_end_stopping = false;
    ball_phase = false;
    ball_reset_all();
    big_turn_pending = false;
    lost_active = false;
    lost_phase = 0;
    far_center_frames = 0;
    center_drive_start_ms = 0;
    turn_pid_reset();
    ball_vision_set_enabled(false);
    if (!en) {
        stop_all_motors();
    }
}

bool follower_is_enabled(void)
{
    return follow_enabled;
}

uint8_t follower_cur_bits(void)
{
    return cur_bits;
}

int follower_avoid_state(void)
{
    return (int)avoid_state;
}

bool follower_ball_phase_active(void)
{
    return ball_phase;
}

int follower_ball_state(void)
{
    return (int)ball_state;
}

/* 串口调试：跳过“先跑完全程再 T 终点”，直接进入击球阶段 */
void follower_force_ball_phase(void)
{
    if (!follow_enabled) {
        follow_enabled = true;
    }
    lost_active = false;
    lost_phase = 0;
    prev_steer = 0;
    avoid_state = AVOID_IDLE;
    avoid_timer_ms = 0;
    avoid_center_since_ms = 0;
    avoid_search_dir = -1;
    obstacle_hit_count = 0;
    clear_since_ms = 0;
    avoidance_happened = true;
    t_end_stopping = false;
    big_turn_pending = false;
    enter_ball_phase((uint32_t)(esp_timer_get_time() / 1000));
    printf("BALL: forced start via serial (skip line section)\n");
}

/* ================= 独立击球控制任务（red-ball-push 架构） ================= */
void follower_ball_task(void *arg)
{
    (void)arg;
    while (1) {
        if (ball_phase) {
            uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
            handle_ball_phase(now);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void follower_init(void)
{
    gpio_set_direction(BOOT_BTN_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BOOT_BTN_PIN, GPIO_PULLUP_ONLY);
    btn_prev_level = gpio_get_level(BOOT_BTN_PIN);
    btn_last_change_ms = 0;

    follow_enabled = false;
    prev_steer = 0;
    last_side = 0;
    avoidance_happened = false;
    lost_active = false;
    lost_phase = 0;
    avoid_state = AVOID_IDLE;
    avoid_timer_ms = 0;
    avoid_center_since_ms = 0;
    avoid_search_dir = -1;
    obstacle_hit_count = 0;
    clear_since_ms = 0;
    t_end_stopping = false;
    ball_phase = false;
    ball_reset_all();
    far_center_frames = 0;
    center_drive_start_ms = 0;
    turn_pid_reset();
    ball_vision_set_enabled(false);
    last_diag_ms = 0;
    last_near_ms = 0;
    last_disp_ms = 0;
}
