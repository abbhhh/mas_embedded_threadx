#include "shoot_mode.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static const int8_t reload_plan[4] = {-1, 0, 1, 2};

static shoot_fault_e       shoot_fault;
static uint8_t             salvo_index;
static uint8_t             shot_index;
static int8_t              reload_index;
static uint8_t             remote_armed;

static float left_home_angle;
static float right_home_angle;

static uint64_t mode_start_us;
static uint64_t origin_stable_start_us;
static uint64_t position_stable_start_us;
static uint64_t sync_error_start_us;
static uint64_t salvo_start_us;
static uint8_t  origin_stable_active;
static uint8_t  position_stable_active;
static uint8_t  sync_error_active;
static uint8_t  salvo_timer_active;

static uint8_t elapsed_ms(uint64_t now_us, uint64_t start_us, uint32_t duration_ms) { return (now_us - start_us) >= (uint64_t)duration_ms * 1000ULL; }

static int8_t current_reload_index(void)
{
    uint8_t index = (uint8_t)(salvo_index * 2 + shot_index);
    if (index >= (uint8_t)(sizeof(reload_plan) / sizeof(reload_plan[0]))) return -1;
    return reload_plan[index];
}

static void reset_motion_monitor(uint64_t now_us)
{
    mode_start_us          = now_us;
    position_stable_active = 0;
    sync_error_active      = 0;
}

static uint8_t origin_is_ready(const Shoot_Mode_Feedback_t *feedback)
{
    if (!feedback->motors_online || fabsf(feedback->left_speed) > 0.5f || fabsf(feedback->right_speed) > 0.5f)
    {
        origin_stable_active = 0;
        return 0;
    }

    if (!origin_stable_active)
    {
        origin_stable_active   = 1U;
        origin_stable_start_us = feedback->now_us;
    }

    return elapsed_ms(feedback->now_us, origin_stable_start_us, 500);
}

static uint8_t position_is_ready(const Shoot_Mode_Feedback_t *feedback, float left_target, float right_target, shoot_mode_e *shoot_mode,
                                 loader_mode_e *load_mode)
{
    float left_span      = -6.283185307f;
    float right_span     = 6.283185307f;
    float left_progress  = (feedback->left_angle - left_home_angle) / left_span;
    float right_progress = (feedback->right_angle - right_home_angle) / right_span;
    float progress_error = fabsf(left_progress - right_progress);

    if (progress_error > 0.15f)
    {
        if (!sync_error_active)
        {
            sync_error_active   = 1;
            sync_error_start_us = feedback->now_us;
        }
        else if (elapsed_ms(feedback->now_us, sync_error_start_us, 100))
        {
            shoot_mode_set_error(shoot_sync_error, shoot_mode, load_mode);
            return 0;
        }
    }
    else
    {
        sync_error_active = 0;
    }

    uint8_t in_position = fabsf(feedback->left_angle - left_target) <= 0.10f && fabsf(feedback->right_angle - right_target) <= 0.10f &&
                          fabsf(feedback->left_speed) <= 0.5f && fabsf(feedback->right_speed) <= 0.5f && progress_error <= 0.15f;

    if (!in_position)
    {
        position_stable_active = 0;
        return 0;
    }

    if (!position_stable_active)
    {
        position_stable_active   = 1;
        position_stable_start_us = feedback->now_us;
    }

    return elapsed_ms(feedback->now_us, position_stable_start_us, 100);
}

void shoot_remote_init(void) { remote_armed = 0U; }

shoot_mode_e shoot_remote_update(uint8_t is_upper, uint8_t is_active)
{
    if (is_upper)
    {
        remote_armed = 1;
        return shoot_off;
    }

    if (is_active && remote_armed)
    {
        remote_armed = 0;
        return shoot_fire;
    }

    return shoot_off;
}

void shoot_mode_init(shoot_mode_e *shoot_mode, loader_mode_e *load_mode)
{
    shoot_fault            = shoot_no_error;
    salvo_index            = 0;
    shot_index             = 0;
    reload_index           = -1;
    left_home_angle        = 0.0f;
    right_home_angle       = 0.0f;
    mode_start_us          = 0;
    origin_stable_active   = 0;
    position_stable_active = 0;
    sync_error_active      = 0;
    salvo_timer_active     = 0;

    if (shoot_mode != NULL) *shoot_mode = shoot_off;
    if (load_mode != NULL) *load_mode = load_origin;
}

void shoot_mode_set_error(shoot_fault_e fault, shoot_mode_e *shoot_mode, loader_mode_e *load_mode)
{
    if (shoot_fault == shoot_no_error) shoot_fault = fault;
    salvo_timer_active = 0U;
    if (shoot_mode != NULL) *shoot_mode = shoot_error;
    if (load_mode != NULL) *load_mode = load_stop;
}

void shoot_mode_update(const Shoot_Ctrl_Cmd_t *shoot_cmd, const Shoot_Mode_Feedback_t *feedback, shoot_mode_e *shoot_mode, loader_mode_e *load_mode)
{
    if (feedback == NULL || shoot_mode == NULL || load_mode == NULL) return;
    if (*shoot_mode == shoot_error || *shoot_mode == shoot_finished) return;

    if (*load_mode != load_origin && !feedback->motors_online)
    {
        shoot_mode_set_error(shoot_motor_offline, shoot_mode, load_mode);
        return;
    }

    if (salvo_timer_active && elapsed_ms(feedback->now_us, salvo_start_us, 30000))
    {
        shoot_mode_set_error(shoot_salvo_timeout, shoot_mode, load_mode);
        return;
    }

    if (*shoot_mode == shoot_fire)
    {
        if (!elapsed_ms(feedback->now_us, mode_start_us, 500)) return;

        if (shot_index == 0)
        {
            shot_index   = 1;
            *shoot_mode  = shoot_off;
            *load_mode   = load_cock;
            reload_index = current_reload_index();
            reset_motion_monitor(feedback->now_us);
        }
        else
        {
            shot_index = 0;
            ++salvo_index;
            salvo_timer_active = 0;
            *load_mode         = load_stop;
            if (salvo_index >= 2)
                *shoot_mode = shoot_finished;
            else
                *shoot_mode = shoot_off;
        }
        return;
    }

    if (*load_mode == load_origin)
    {
        if (!origin_is_ready(feedback)) return;

        left_home_angle  = feedback->left_angle;
        right_home_angle = feedback->right_angle;
        *shoot_mode      = shoot_off;
        *load_mode       = load_stop;
        return;
    }

    if (*shoot_mode == shoot_off && *load_mode == load_stop)
    {
        if (shoot_cmd == NULL || shoot_cmd->shoot_mode != shoot_fire) return;

        *load_mode         = load_cock;
        reload_index       = current_reload_index();
        salvo_timer_active = 1U;
        salvo_start_us     = feedback->now_us;
        reset_motion_monitor(feedback->now_us);
        return;
    }

    if (*shoot_mode == shoot_off && *load_mode == load_cock)
    {
        if (elapsed_ms(feedback->now_us, mode_start_us, 3000))
        {
            shoot_mode_set_error(shoot_motion_timeout, shoot_mode, load_mode);
            return;
        }

        if (position_is_ready(feedback, left_home_angle - 6.283185307f, right_home_angle + 6.283185307f, shoot_mode, load_mode))
        {
            *shoot_mode   = shoot_lock;
            mode_start_us = feedback->now_us;
        }
        return;
    }

    if (*shoot_mode == shoot_lock && *load_mode == load_cock)
    {
        if (elapsed_ms(feedback->now_us, mode_start_us, 500))
        {
            *load_mode = load_return;
            reset_motion_monitor(feedback->now_us);
        }
        return;
    }

    if (*shoot_mode == shoot_lock && *load_mode == load_return)
    {
        if (elapsed_ms(feedback->now_us, mode_start_us, 3000))
        {
            shoot_mode_set_error(shoot_motion_timeout, shoot_mode, load_mode);
            return;
        }

        if (!position_is_ready(feedback, left_home_angle, right_home_angle, shoot_mode, load_mode)) return;

        reload_index  = current_reload_index();
        mode_start_us = feedback->now_us;
        if (reload_index < 0)
        {
            *shoot_mode = shoot_fire;
            *load_mode  = load_stop;
        }
        else
        {
            *load_mode = load_reload;
        }
        return;
    }

    if (*shoot_mode == shoot_lock && *load_mode == load_reload)
    {
        if (feedback->reload_error)
        {
            shoot_mode_set_error(shoot_reload_error, shoot_mode, load_mode);
        }
        else if (feedback->reload_finished)
        {
            *shoot_mode   = shoot_fire;
            *load_mode    = load_stop;
            mode_start_us = feedback->now_us;
        }
        else if (elapsed_ms(feedback->now_us, mode_start_us, 6000))
        {
            shoot_mode_set_error(shoot_reload_timeout, shoot_mode, load_mode);
        }
        return;
    }

    shoot_mode_set_error(shoot_internal_error, shoot_mode, load_mode);
}

float         shoot_mode_get_left_home_angle(void) { return left_home_angle; }
float         shoot_mode_get_right_home_angle(void) { return right_home_angle; }
int8_t        shoot_mode_get_reload_index(void) { return reload_index; }
uint8_t       shoot_mode_get_salvo_index(void) { return salvo_index; }
shoot_fault_e shoot_mode_get_fault(void) { return shoot_fault; }
