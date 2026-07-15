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

#define trigger_station 20  //扳机位置

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
