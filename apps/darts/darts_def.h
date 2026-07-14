/*
 * @Author: abbhhh 804433588@qq.com
 * @Date: 2026-07-14 14:35:27
 * @LastEditors: abbhhh 804433588@qq.com
 * @LastEditTime: 2026-07-14 15:01:33
 * @FilePath: \mas_embedded_threadx\apps\darts\darts_def.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef DARTS_DEF_H
#define DARTS_DEF_H

#include <stdint.h>

typedef enum
{
    shoot_off = 0,
    shoot_lock,
    shoot_fire,
    shoot_finished,
    shoot_error,
} shoot_mode_e;

typedef enum
{
    load_origin = 0,
    load_stop,
    load_cock,
    load_return,
    load_reload,
} loader_mode_e;

typedef enum
{
    shoot_no_error = 0,
    shoot_config_error,
    shoot_motor_offline,
    shoot_motion_timeout,
    shoot_sync_error,
    shoot_reload_error,
    shoot_reload_timeout,
    shoot_salvo_timeout,
    shoot_internal_error,
} shoot_fault_e;

typedef struct
{
    shoot_mode_e shoot_mode;
} Shoot_Ctrl_Cmd_t;

#endif /* DARTS_DEF_H */
