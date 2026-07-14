#include "shoot_mode.h"

#include <assert.h>
#include <stdio.h>

static shoot_mode_e          shoot_mode;
static loader_mode_e         load_mode;
static Shoot_Mode_Feedback_t feedback;

static void advance_ms(uint32_t duration_ms) { feedback.now_us += (uint64_t)duration_ms * 1000ULL; }

static void reset_modes(void)
{
    feedback = (Shoot_Mode_Feedback_t){
        .motors_online = 1U,
    };
    shoot_mode_init(&shoot_mode, &load_mode);
    assert(shoot_mode == shoot_off);
    assert(load_mode == load_origin);

    shoot_mode_update(NULL, &feedback, &shoot_mode, &load_mode);
    assert(shoot_mode == shoot_off);
    assert(load_mode == load_origin);

    advance_ms(500U);
    shoot_mode_update(NULL, &feedback, &shoot_mode, &load_mode);
    assert(shoot_mode == shoot_off);
    assert(load_mode == load_stop);
}

static void start_salvo(void)
{
    Shoot_Ctrl_Cmd_t command = {.shoot_mode = shoot_fire};
    shoot_mode_update(&command, &feedback, &shoot_mode, &load_mode);
    assert(shoot_mode == shoot_off);
    assert(load_mode == load_cock);
}

static void complete_shot(int8_t expected_reload)
{
    feedback.left_angle  = shoot_mode_get_left_home_angle() - 6.283185307f;
    feedback.right_angle = shoot_mode_get_right_home_angle() + 6.283185307f;
    shoot_mode_update(NULL, &feedback, &shoot_mode, &load_mode);
    advance_ms(100U);
    shoot_mode_update(NULL, &feedback, &shoot_mode, &load_mode);
    assert(shoot_mode == shoot_lock);
    assert(load_mode == load_cock);

    advance_ms(500U);
    shoot_mode_update(NULL, &feedback, &shoot_mode, &load_mode);
    assert(shoot_mode == shoot_lock);
    assert(load_mode == load_return);

    feedback.left_angle  = shoot_mode_get_left_home_angle();
    feedback.right_angle = shoot_mode_get_right_home_angle();
    shoot_mode_update(NULL, &feedback, &shoot_mode, &load_mode);
    advance_ms(100U);
    shoot_mode_update(NULL, &feedback, &shoot_mode, &load_mode);

    if (expected_reload >= 0)
    {
        assert(shoot_mode == shoot_lock);
        assert(load_mode == load_reload);
        assert(shoot_mode_get_reload_index() == expected_reload);

        feedback.reload_finished = 1U;
        shoot_mode_update(NULL, &feedback, &shoot_mode, &load_mode);
        feedback.reload_finished = 0U;
    }

    assert(shoot_mode == shoot_fire);
    assert(load_mode == load_stop);
    advance_ms(500U);
    shoot_mode_update(NULL, &feedback, &shoot_mode, &load_mode);
}

static void test_remote_trigger_requires_rearm(void)
{
    shoot_remote_init();
    assert(shoot_remote_update(1U, 0U) == shoot_off);
    assert(shoot_remote_update(0U, 1U) == shoot_fire);
    assert(shoot_remote_update(0U, 1U) == shoot_off);
    assert(shoot_remote_update(1U, 0U) == shoot_off);
    assert(shoot_remote_update(0U, 1U) == shoot_fire);
}

static void test_two_requests_fire_four_darts(void)
{
    reset_modes();

    start_salvo();
    complete_shot(-1);
    assert(shoot_mode == shoot_off);
    assert(load_mode == load_cock);

    complete_shot(0);
    assert(shoot_mode == shoot_off);
    assert(load_mode == load_stop);
    assert(shoot_mode_get_salvo_index() == 1U);

    start_salvo();
    complete_shot(1);
    assert(shoot_mode == shoot_off);
    assert(load_mode == load_cock);

    complete_shot(2);
    assert(shoot_mode == shoot_finished);
    assert(load_mode == load_stop);
    assert(shoot_mode_get_salvo_index() == 2U);

    Shoot_Ctrl_Cmd_t command = {.shoot_mode = shoot_fire};
    shoot_mode_update(&command, &feedback, &shoot_mode, &load_mode);
    assert(shoot_mode == shoot_finished);
    assert(load_mode == load_stop);
}

static void test_fault_is_latched(void)
{
    reset_modes();
    start_salvo();

    feedback.motors_online = 0U;
    shoot_mode_update(NULL, &feedback, &shoot_mode, &load_mode);
    assert(shoot_mode == shoot_error);
    assert(load_mode == load_stop);
    assert(shoot_mode_get_fault() == shoot_motor_offline);

    feedback.motors_online   = 1U;
    Shoot_Ctrl_Cmd_t command = {.shoot_mode = shoot_fire};
    shoot_mode_update(&command, &feedback, &shoot_mode, &load_mode);
    assert(shoot_mode == shoot_error);
    assert(load_mode == load_stop);
}

int main(void)
{
    test_remote_trigger_requires_rearm();
    test_two_requests_fire_four_darts();
    test_fault_is_latched();
    puts("shoot mode tests passed");
    return 0;
}
