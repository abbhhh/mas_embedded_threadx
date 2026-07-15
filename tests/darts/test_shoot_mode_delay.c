#include "shoot_mode.h"

#include <assert.h>
#include <stdint.h>

static uint64_t now_us;

uint64_t BSP_DWT_GetTimeline_us(void) { return now_us; }

int main(void)
{
    now_us = 1000000ULL;
    assert(!shoot_mode_delay_ms(500U));

    now_us += 499999ULL;
    assert(!shoot_mode_delay_ms(500U));

    ++now_us;
    assert(shoot_mode_delay_ms(500U));

    assert(!shoot_mode_delay_ms(500U));

    assert(shoot_mode_delay_ms(0U));
    return 0;
}
