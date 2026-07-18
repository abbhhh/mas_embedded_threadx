/*
 * @Author: abbhhh 804433588@qq.com
 * @Date: 2026-07-14 14:35:27
 * @LastEditors: abbhhh 804433588@qq.com
 * @LastEditTime: 2026-07-17 18:30:47
 * @FilePath: \mas_embedded_threadx\apps\darts\darts_def.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef DARTS_DEF_H
#define DARTS_DEF_H

#include <stdint.h>

#define left_rise_sation   180 // 左抬升（高）
#define right_rise_sation  0 // 右抬升（高）
#define left_low_sation    70  // 左抬升（低）
#define right_low_sation   110 // 右抬升（低）
#define transfer_medium    120 // 平移（中）
#define transfer_left      0  // 平移（左）
#define transfer_right     240 // 平移（右）
#define gripper_in         42  // 夹爪（夹紧）
#define gripper_out        70  // 夹爪（放下）
#define trigger_ready      70  // 扳机（上膛）
#define trigger_fire       5   // 扳机（开火）
#define friction_high_speed     100  // (左轮蓄力为正)
#define friction_low_speed      30  // (左轮蓄力为正)
#define friction_time      6500 //(同步带运动时间)
#define friction_low_time  1500 //(同步带慢速运动时间)
#define servo_time         700 //(舵机动作等待时间)
#define servo_load_time    400


typedef enum
{
    shoot_off ,
    shoot_start_1,
    shoot_start_2,
    shoot_restart,
} shoot_mode_e;

typedef struct
{
    shoot_mode_e    shoot_mode;
} Shoot_Ctrl_Cmd_t;
#endif /* DARTS_DEF_H */
