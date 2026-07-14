/*
 * @Author: abbhhh 804433588@qq.com
 * @Date: 2026-07-14 13:43:04
 * @LastEditors: abbhhh 804433588@qq.com
 * @LastEditTime: 2026-07-14 13:52:03
 * @FilePath: \mas_embedded_threadx\apps\darts\single_board\robot_func\robot_func.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "robot_func.h"

#include "module_remote.h"
#include "shoot_mode.h"

#include <stddef.h>

#define REMOTE_FIRE_CHANNEL 8U

void RemoteControlSet(Shoot_Ctrl_Cmd_t *shoot_ctrl)
{
    if (shoot_ctrl == NULL) return;

    shoot_ctrl->shoot_mode = shoot_off;

    if ((Module_Remote_get_offline_status() & 0x01U) == 0U)
    {
        shoot_remote_init();
        return;
    }

    int16_t channel_8 = Module_Remote_get_channel(8);
    if (channel_8 == SBUS_CHX_UP)
    {
        (void)shoot_remote_update(1, 0);
    }
    else if (channel_8 == SBUS_CHX_BIAS || channel_8 == SBUS_CHX_DOWN)
    {
        shoot_ctrl->shoot_mode = shoot_remote_update(0 ,1);
    }
    else
    {
        shoot_remote_init();
    }
}
