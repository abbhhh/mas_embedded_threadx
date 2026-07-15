/*
 * @Author: abbhhh 804433588@qq.com
 * @Date: 2026-07-14 13:43:04
 * @LastEditors: abbhhh 804433588@qq.com
 * @LastEditTime: 2026-07-15 14:32:26
 * @FilePath: \mas_embedded_threadx\apps\darts\single_board\robot_func\robot_func.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "robot_func.h"

#include "darts_def.h"
#include "module_remote.h"
#include "shoot_mode.h"

#include <stddef.h>


void RemoteControlSet(Shoot_Ctrl_Cmd_t *Shoot_Ctrl)
{
    if ( !Shoot_Ctrl ) return;

    uint8_t state = Module_Remote_get_offline_status();

    /* RC 在线 */
    if (state & 0x01)
    {
        /* 摇杆 → 速度比例 (-1.0 ~ +1.0)
         * SBUS 通道值: 中位 1024, 上 240, 下 1807 → 零偏后 -784 ~ +783 */
        int16_t ch5 = Module_Remote_get_channel(5);
        int16_t ch6 = Module_Remote_get_channel(6);
        int16_t ch8 = Module_Remote_get_channel(8);
        if (ch5 >500)
        {
            Shoot_Ctrl->shoot_mode    = shoot_off;
            if (ch8 >500)
            {
                Shoot_Ctrl ->shoot_mode = shoot_restart;
            }
        }
        else if (ch5 <500)
        {
            if (ch6 <500)
                Shoot_Ctrl->shoot_mode = shoot_start_1;
            else if (ch6 >500)
                Shoot_Ctrl->shoot_mode = shoot_start_2;
        }
    }
    else
    {
        Shoot_Ctrl->shoot_mode     = shoot_off;
    }
}
