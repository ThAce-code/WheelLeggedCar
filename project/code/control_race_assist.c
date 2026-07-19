/*********************************************************************************************************************
* File: control_race_assist.c
* Description: Pure fail-closed race-assist supervisor.  Runtime integration is intentionally separate.
*********************************************************************************************************************/

#include "control_race_assist.h"
#include "app_config.h"

static race_assist_output_struct control_race_assist_output;
static float control_race_assist_accel_gain;
static float control_race_assist_error_gain;
static float control_race_assist_hold_bias;
static float control_race_assist_previous_ramped_rpm;
static race_assist_fault_reason_enum control_race_assist_latched_fault_reason;
static uint8 control_race_assist_previous_ramped_valid;
static uint8 control_race_assist_level;
static uint8 control_race_assist_disable_pending;
static uint8 control_race_assist_leg_path_fault_latched;
static uint8 control_race_assist_fault_latched;

static float control_race_assist_absf(float value)
{
    return (0.0f > value) ? -value : value;
}

static float control_race_assist_clamp(float value, float low, float high)
{
    if(low > value)
    {
        return low;
    }
    if(high < value)
    {
        return high;
    }
    return value;
}

static float control_race_assist_clamp01(float value)
{
    return control_race_assist_clamp(value, 0.0f, 1.0f);
}

static uint8 control_race_assist_is_finite(float value)
{
    if(value != value)
    {
        return APP_FALSE;
    }
    if(APP_BALANCE_FINITE_ABS_LIMIT < value)
    {
        return APP_FALSE;
    }
    if((-APP_BALANCE_FINITE_ABS_LIMIT) > value)
    {
        return APP_FALSE;
    }
    return APP_TRUE;
}

static uint8 control_race_assist_input_is_valid(const race_assist_input_struct *input)
{
    if(NULL == input)
    {
        return APP_FALSE;
    }

    if((APP_FALSE == control_race_assist_is_finite(input->target_rpm)) ||
       (APP_FALSE == control_race_assist_is_finite(input->ramped_rpm)) ||
       (APP_FALSE == control_race_assist_is_finite(input->measured_rpm)) ||
       (APP_FALSE == control_race_assist_is_finite(input->pitch_deg)) ||
       (APP_FALSE == control_race_assist_is_finite(input->pitch_rate_dps)) ||
       (APP_FALSE == control_race_assist_is_finite(input->leg_u_actual)) ||
       (1.0f < control_race_assist_absf(input->leg_u_actual)) ||
       (APP_FALSE == control_race_assist_is_finite(input->dt_s)) ||
       (0.0f >= input->dt_s) ||
       (1.0f < input->dt_s))
    {
        return APP_FALSE;
    }
    return APP_TRUE;
}

static float control_race_assist_safe_hold_u(float leg_u_actual)
{
    if((APP_FALSE == control_race_assist_is_finite(leg_u_actual)) ||
       (1.0f < control_race_assist_absf(leg_u_actual)))
    {
        return 0.0f;
    }
    return control_race_assist_clamp(leg_u_actual, -1.0f, 1.0f);
}

static uint8 control_race_assist_entry_pose_required(void)
{
    return ((RACE_ASSIST_DISABLED == control_race_assist_output.state) ||
            (RACE_ASSIST_LOW_RACE == control_race_assist_output.state) ||
            (RACE_ASSIST_ARMED == control_race_assist_output.state)) ?
           APP_TRUE : APP_FALSE;
}

static void control_race_assist_apply_level_profile(uint8 level)
{
    race_assist_level_profile_struct profile;

    if(APP_FALSE == control_race_assist_get_level_profile(level, &profile))
    {
        (void)control_race_assist_get_level_profile(1U, &profile);
    }

    control_race_assist_output.forward_limit_rpm = profile.forward_limit_rpm;
    control_race_assist_output.balance_limit_rpm = profile.balance_limit_rpm;
    control_race_assist_output.dx_mm = profile.dx_mm;
    control_race_assist_output.dy_mm = profile.dy_mm;
    control_race_assist_output.pitch_offset_limit_deg = APP_RACE_ASSIST_PITCH_OFFSET_LIMIT_DEG;
    control_race_assist_output.level = level;
}

static void control_race_assist_set_disabled_output(void)
{
    control_race_assist_output.state = RACE_ASSIST_DISABLED;
    control_race_assist_output.fault_reason = RACE_ASSIST_FAULT_NONE;
    control_race_assist_output.u_request = 0.0f;
    control_race_assist_output.enable = APP_FALSE;
    control_race_assist_output.leg_command_enable = APP_FALSE;
}

static void control_race_assist_enter_fault(race_assist_fault_reason_enum reason,
                                            float leg_u_actual)
{
    if(APP_FALSE == control_race_assist_fault_latched)
    {
        control_race_assist_latched_fault_reason = reason;
        control_race_assist_fault_latched = APP_TRUE;
    }

    control_race_assist_output.state = RACE_ASSIST_FAULT_HOLD;
    control_race_assist_output.fault_reason = control_race_assist_latched_fault_reason;
    control_race_assist_output.requested_accel_rpm_s = 0.0f;
    control_race_assist_output.speed_error_rpm = 0.0f;
    control_race_assist_output.speed_blend = 0.0f;
    control_race_assist_output.u_request = control_race_assist_safe_hold_u(leg_u_actual);
    control_race_assist_output.enable = APP_FALSE;
    control_race_assist_output.leg_command_enable = APP_TRUE;
}

static void control_race_assist_update_disable(const race_assist_input_struct *input)
{
    control_race_assist_output.requested_accel_rpm_s = 0.0f;
    control_race_assist_output.speed_error_rpm = 0.0f;
    control_race_assist_output.speed_blend = 0.0f;
    control_race_assist_output.forward_limit_rpm =
        control_race_assist_clamp(control_race_assist_output.forward_limit_rpm,
                                  0.0f,
                                  APP_RACE_ASSIST_RECENTER_RPM);
    control_race_assist_output.fault_reason = RACE_ASSIST_FAULT_NONE;
    control_race_assist_output.enable = APP_FALSE;

    if(APP_FALSE == control_race_assist_disable_pending)
    {
        control_race_assist_set_disabled_output();
        return;
    }

    if(APP_RACE_ASSIST_RECENTER_RPM <= control_race_assist_absf(input->measured_rpm))
    {
        control_race_assist_output.state = RACE_ASSIST_RECENTER;
        control_race_assist_output.u_request = control_race_assist_safe_hold_u(input->leg_u_actual);
        control_race_assist_output.leg_command_enable = APP_TRUE;
        return;
    }

    control_race_assist_output.u_request = 0.0f;
    control_race_assist_output.leg_command_enable = APP_TRUE;
    if(APP_RACE_ASSIST_REQUEST_DEADBAND >= control_race_assist_absf(input->leg_u_actual))
    {
        control_race_assist_disable_pending = APP_FALSE;
        control_race_assist_set_disabled_output();
    }
    else
    {
        control_race_assist_output.state = RACE_ASSIST_RECENTER;
    }
}

void control_race_assist_init(void)
{
    control_race_assist_accel_gain = APP_RACE_ASSIST_GAIN_A_DEFAULT;
    control_race_assist_error_gain = APP_RACE_ASSIST_GAIN_E_DEFAULT;
    control_race_assist_hold_bias = APP_RACE_ASSIST_HOLD_DEFAULT;
    control_race_assist_previous_ramped_rpm = 0.0f;
    control_race_assist_latched_fault_reason = RACE_ASSIST_FAULT_NONE;
    control_race_assist_previous_ramped_valid = APP_FALSE;
    control_race_assist_level = 0U;
    control_race_assist_disable_pending = APP_FALSE;
    control_race_assist_leg_path_fault_latched = APP_FALSE;
    control_race_assist_fault_latched = APP_FALSE;

    control_race_assist_output.requested_accel_rpm_s = 0.0f;
    control_race_assist_output.speed_error_rpm = 0.0f;
    control_race_assist_output.speed_blend = 0.0f;
    control_race_assist_output.turn_scale = 1.0f;
    control_race_assist_apply_level_profile(0U);
    control_race_assist_set_disabled_output();
}

uint8 control_race_assist_set_level(uint8 level)
{
    if(APP_RACE_ASSIST_MAX_VALIDATED_LEVEL < level)
    {
        return APP_FALSE;
    }

    if((0U == level) && (0U != control_race_assist_level))
    {
        control_race_assist_disable_pending = APP_TRUE;
    }
    control_race_assist_level = level;
    return APP_TRUE;
}

uint8 control_race_assist_set_gains(float accel_gain, float error_gain, float hold_bias)
{
    if((APP_FALSE == control_race_assist_is_finite(accel_gain)) ||
       (APP_FALSE == control_race_assist_is_finite(error_gain)) ||
       (APP_FALSE == control_race_assist_is_finite(hold_bias)) ||
       (0.0f > accel_gain) ||
       (APP_RACE_ASSIST_GAIN_A_MAX < accel_gain) ||
       (0.0f > error_gain) ||
       (APP_RACE_ASSIST_GAIN_E_MAX < error_gain) ||
       (0.0f > hold_bias) ||
       (APP_RACE_ASSIST_HOLD_MAX < hold_bias))
    {
        return APP_FALSE;
    }

    control_race_assist_accel_gain = accel_gain;
    control_race_assist_error_gain = error_gain;
    control_race_assist_hold_bias = hold_bias;
    return APP_TRUE;
}

void control_race_assist_report_leg_path_fault(void)
{
    control_race_assist_leg_path_fault_latched = APP_TRUE;
}

void control_race_assist_update(const race_assist_input_struct *input)
{
    float requested_accel_rpm_s;
    float speed_error_rpm;
    float speed_abs_rpm;
    float t;
    float u_raw;
    uint8 pitch_arm_ok;

    control_race_assist_apply_level_profile(control_race_assist_level);

    if(APP_FALSE == control_race_assist_input_is_valid(input))
    {
        control_race_assist_output.turn_scale = 0.0f;
        control_race_assist_enter_fault(RACE_ASSIST_FAULT_INPUT_INVALID,
                                        (NULL != input) ? input->leg_u_actual : 0.0f);
        return;
    }

    if((APP_TRUE == control_race_assist_leg_path_fault_latched) ||
       (APP_TRUE == input->leg_path_fault))
    {
        control_race_assist_enter_fault(RACE_ASSIST_FAULT_LEG_PATH, input->leg_u_actual);
        return;
    }

    if((APP_RACE_ASSIST_PITCH_ABORT_LIMIT_DEG < control_race_assist_absf(input->pitch_deg)) ||
       (APP_RACE_ASSIST_RATE_ABORT_LIMIT_DPS < control_race_assist_absf(input->pitch_rate_dps)))
    {
        control_race_assist_enter_fault(RACE_ASSIST_FAULT_PITCH_LIMIT, input->leg_u_actual);
        return;
    }

    if(APP_TRUE == control_race_assist_fault_latched)
    {
        control_race_assist_enter_fault(control_race_assist_latched_fault_reason,
                                        input->leg_u_actual);
        return;
    }

    speed_abs_rpm = control_race_assist_absf(input->measured_rpm);
    control_race_assist_output.turn_scale =
        control_race_assist_clamp01((400.0f - speed_abs_rpm) / 100.0f);

    if((0U == control_race_assist_level) || (APP_TRUE != input->fast_enable))
    {
        if((RACE_ASSIST_DISABLED != control_race_assist_output.state) &&
           (RACE_ASSIST_RECENTER != control_race_assist_output.state))
        {
            control_race_assist_disable_pending = APP_TRUE;
        }
        control_race_assist_update_disable(input);
        control_race_assist_previous_ramped_rpm = input->ramped_rpm;
        control_race_assist_previous_ramped_valid = APP_TRUE;
        return;
    }

    if(APP_TRUE != input->feedback_healthy)
    {
        control_race_assist_enter_fault(RACE_ASSIST_FAULT_LEG_NOT_READY, input->leg_u_actual);
        return;
    }

    if((APP_TRUE == control_race_assist_entry_pose_required()) &&
       (APP_TRUE != input->low_pose_ready))
    {
        control_race_assist_enter_fault(RACE_ASSIST_FAULT_LEG_NOT_READY, input->leg_u_actual);
        return;
    }

    if(APP_FALSE == control_race_assist_previous_ramped_valid)
    {
        requested_accel_rpm_s = 0.0f;
        control_race_assist_previous_ramped_valid = APP_TRUE;
    }
    else
    {
        requested_accel_rpm_s =
            (input->ramped_rpm - control_race_assist_previous_ramped_rpm) / input->dt_s;
    }
    control_race_assist_previous_ramped_rpm = input->ramped_rpm;
    speed_error_rpm = input->ramped_rpm - input->measured_rpm;

    if(APP_RACE_ASSIST_ACCEL_DEADBAND_RPM_S > control_race_assist_absf(requested_accel_rpm_s))
    {
        requested_accel_rpm_s = 0.0f;
    }
    if(APP_RACE_ASSIST_SPEED_ERROR_DEADBAND_RPM > control_race_assist_absf(speed_error_rpm))
    {
        speed_error_rpm = 0.0f;
    }

    t = control_race_assist_clamp01(
        (speed_abs_rpm - APP_RACE_ASSIST_ARM_START_RPM) /
        (APP_RACE_ASSIST_FULL_RPM - APP_RACE_ASSIST_ARM_START_RPM));
    control_race_assist_output.speed_blend = t * t * (3.0f - (2.0f * t));
    u_raw = (control_race_assist_accel_gain * requested_accel_rpm_s) +
            (control_race_assist_error_gain * speed_error_rpm) +
            (control_race_assist_hold_bias * control_race_assist_output.speed_blend);
    control_race_assist_output.requested_accel_rpm_s = requested_accel_rpm_s;
    control_race_assist_output.speed_error_rpm = speed_error_rpm;
    control_race_assist_output.u_request = control_race_assist_clamp(u_raw, -1.0f, 1.0f);
    control_race_assist_output.fault_reason = RACE_ASSIST_FAULT_NONE;
    control_race_assist_output.enable = APP_TRUE;
    control_race_assist_output.leg_command_enable = APP_TRUE;

    pitch_arm_ok =
        ((APP_RACE_ASSIST_PITCH_ARM_LIMIT_DEG >= control_race_assist_absf(input->pitch_deg)) &&
         (APP_RACE_ASSIST_RATE_ARM_LIMIT_DPS >= control_race_assist_absf(input->pitch_rate_dps))) ?
        APP_TRUE : APP_FALSE;

    if(APP_RACE_ASSIST_ARM_START_RPM > speed_abs_rpm)
    {
        control_race_assist_output.state = RACE_ASSIST_LOW_RACE;
        control_race_assist_output.u_request = 0.0f;
    }
    else if((RACE_ASSIST_DISABLED == control_race_assist_output.state) ||
            (RACE_ASSIST_LOW_RACE == control_race_assist_output.state) ||
            (APP_FALSE == pitch_arm_ok))
    {
        control_race_assist_output.state = RACE_ASSIST_ARMED;
        control_race_assist_output.u_request = 0.0f;
    }
    else if((0.0f > requested_accel_rpm_s) || (0.0f > speed_error_rpm))
    {
        control_race_assist_output.state = RACE_ASSIST_BRAKE;
    }
    else if((0.0f < requested_accel_rpm_s) || (0.0f < speed_error_rpm))
    {
        control_race_assist_output.state = RACE_ASSIST_BOOST;
    }
    else
    {
        control_race_assist_output.state = RACE_ASSIST_CRUISE_HOLD;
    }
}

const race_assist_output_struct *control_race_assist_get_output(void)
{
    return &control_race_assist_output;
}

uint8 control_race_assist_get_level_profile(uint8 level,
                                            race_assist_level_profile_struct *profile)
{
    if(NULL == profile)
    {
        return APP_FALSE;
    }

    switch(level)
    {
        case 1U:
            profile->forward_limit_rpm = 250.0f;
            profile->balance_limit_rpm = 300.0f;
            profile->dx_mm = 2.0f;
            profile->dy_mm = 2.0f;
            break;

        case 2U:
            profile->forward_limit_rpm = 300.0f;
            profile->balance_limit_rpm = 350.0f;
            profile->dx_mm = 2.0f;
            profile->dy_mm = 2.0f;
            break;

        case 3U:
            profile->forward_limit_rpm = 350.0f;
            profile->balance_limit_rpm = 410.0f;
            profile->dx_mm = 2.0f;
            profile->dy_mm = 2.0f;
            break;

        case 4U:
            profile->forward_limit_rpm = 400.0f;
            profile->balance_limit_rpm = 460.0f;
            profile->dx_mm = 2.0f;
            profile->dy_mm = 2.0f;
            break;

        default:
            return APP_FALSE;
    }
    return APP_TRUE;
}
