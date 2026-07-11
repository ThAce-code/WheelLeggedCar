/*********************************************************************************************************************
* File: servo_motion.c
* Description: Pure first-order low-pass servo motion unit.
*              This unit is deterministic and contains no hardware dependencies.
*********************************************************************************************************************/

#include "servo_motion.h"
#include <math.h>

static float servo_motion_absf(float value)
{
    return (0.0f > value) ? -value : value;
}

static uint8 servo_motion_isfinite(float value)
{
    if(value != value)
    {
        return APP_FALSE;
    }
    /* ±1e38 guards against inf without pulling in <math.h> isinf on every toolchain. */
    if(1.0e38f < value)
    {
        return APP_FALSE;
    }
    if((-1.0e38f) > value)
    {
        return APP_FALSE;
    }
    return APP_TRUE;
}

static float servo_motion_clamp(float value, float min_val, float max_val)
{
    if(value < min_val)
    {
        return min_val;
    }
    if(value > max_val)
    {
        return max_val;
    }
    return value;
}

void servo_motion_init(servo_motion_state_struct *state, float angle_deg)
{
    if(NULL == state)
    {
        return;
    }
    if(APP_FALSE == servo_motion_isfinite(angle_deg))
    {
        return;
    }
    state->target_deg = angle_deg;
    state->filtered_deg = angle_deg;
    state->output_deg = angle_deg;
    state->settled_ticks = 0U;
    state->settled = APP_FALSE;
}

void servo_motion_step(servo_motion_state_struct *state,
                       float target_deg,
                       float alpha,
                       float max_step_deg,
                       float settle_error_deg,
                       uint16 settle_ticks_required)
{
    float delta_deg;
    float error_deg;

    if((NULL == state) ||
       (APP_FALSE == servo_motion_isfinite(target_deg)) ||
       (0.0f >= alpha) ||
       (1.0f < alpha) ||
       (0.0f >= max_step_deg) ||
       (0.0f > settle_error_deg) ||
       (0U == settle_ticks_required))
    {
        return;
    }

    state->target_deg = target_deg;
    state->filtered_deg += alpha * (target_deg - state->filtered_deg);
    delta_deg = state->filtered_deg - state->output_deg;
    delta_deg = servo_motion_clamp(delta_deg, -max_step_deg, max_step_deg);
    state->output_deg += delta_deg;
    error_deg = servo_motion_absf(target_deg - state->output_deg);
    if(error_deg <= settle_error_deg)
    {
        if(state->settled_ticks < settle_ticks_required)
        {
            state->settled_ticks++;
        }
    }
    else
    {
        state->settled_ticks = 0U;
    }
    state->settled = (state->settled_ticks >= settle_ticks_required) ? APP_TRUE : APP_FALSE;
}

void servo_motion_apply_immediate(servo_motion_state_struct *state, float angle_deg)
{
    if(NULL == state)
    {
        return;
    }
    if(APP_FALSE == servo_motion_isfinite(angle_deg))
    {
        return;
    }
    state->target_deg = angle_deg;
    state->filtered_deg = angle_deg;
    state->output_deg = angle_deg;
    state->settled_ticks = 0U;
    state->settled = APP_FALSE;
}
