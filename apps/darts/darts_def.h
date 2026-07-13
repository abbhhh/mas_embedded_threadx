/*
 * @Description: 机器人定义模板
 *
 * 使用说明:
 *   1. 复制本文件到你的机器人目录, 改名为 <机器人名>_def.h (如 hero_def.h)
 *   2. 修改机械参数 (角度、速度、减速比等)
 *   3. 如需要单板/双板模式, 参考 sentry 添加板间通讯结构体
 *   4. 如底盘模式与模板不同, 修改 chassis_mode_e 枚举
 */

#ifndef _DARTS_DEF_H_
#define _DARTS_DEF_H_

#include <stdint.h>

/* ================================================================
 * 机械参数 (按实际机械图纸修改)
 * ================================================================ */
// clang-format off

// ── 云台参数 ──
#define PITCH_MAX_ANGLE           40.0f     /* 云台竖直最大角度 (度) */
#define PITCH_MIN_ANGLE           -20.0f    /* 云台竖直最小角度 (度) */
#define YAW_CHASSIS_ALIGN_ECD     2048      /* yaw与底盘对齐时的电机编码器值, 机械改动后需修改 */

// ── 底盘参数 ──
#define CHASSIS_MAX_SPEED_MPS     3.0f      /* 底盘最大线速度 (m/s), 遥控满杆映射到此值       */

// ── 发射参数 ──
#define REDUCTION_RATIO_LOADER    36.0f     /* 拨盘电机减速比 (例: 2006 = 36:1)，注意：达妙电机的位置是输出轴位置，减速比给1即可 */
#define ONE_BULLET_DELTA_ANGLE    60.0f     /* 单发弹丸拨盘转动角度 (度) */
#define NUM_PER_CIRCLE            6         /* 拨盘一圈装载弹丸数 */

// clang-format on

/* ================================================================
 * 控制模式枚举
 * ================================================================ */
#pragma pack(1)

// ── 云台模式 ──
typedef enum
{
    gimbal_zero_force = 0, /* 云台无力 */
    gimbal_gyro_mode,      /* 手动跟随 */
    gimbal_auto_mode,      /* 自动瞄准 */
} gimbal_mode_e;

typedef struct
{
    float         yaw;
    float         pitch;
    uint8_t       auto_search;
    gimbal_mode_e gimbal_mode;
} Gimbal_Ctrl_Cmd_t;

// ── 发射模式 ──
typedef enum
{
    shoot_off = 0,
    shoot_on,
} shoot_mode_e;

typedef enum
{
    friction_off = 0,
    friction_on,
} friction_mode_e;

typedef enum
{
    load_stop   = 0,
    load_start,
    load_restart,
    load_get,
    load_reload,
    load_ready,
} loader_mode_e;

typedef struct
{
    shoot_mode_e    shoot_mode;
    loader_mode_e   load_mode;
} Shoot_Ctrl_Cmd_t;

#endif /* _DARTS_DEF_H_ */
