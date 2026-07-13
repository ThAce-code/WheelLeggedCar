$ErrorActionPreference = "Stop"

function Require-Pattern {
    param(
        [string]$Text,
        [string]$Pattern,
        [string]$Message
    )
    if($Text -notmatch $Pattern) {
        throw $Message
    }
}

function Reject-Pattern {
    param(
        [string]$Text,
        [string]$Pattern,
        [string]$Message
    )
    if($Text -match $Pattern) {
        throw $Message
    }
}

$servo = Get-Content "project/code/actuator_servo.c" -Raw
$motor = Get-Content "project/code/actuator_motor.c" -Raw
$leg = Get-Content "project/code/control_leg.c" -Raw
$scheduler = Get-Content "project/code/app_scheduler.c" -Raw
$schedulerH = Get-Content "project/code/app_scheduler.h" -Raw
$servoH = Get-Content "project/code/actuator_servo.h" -Raw
$telemetry = Get-Content "project/code/telemetry.c" -Raw
$collector = Get-Content "tools/collect_balance_data.ps1" -Raw
$imu = Get-Content "project/code/sensor_imu.c" -Raw
$imuH = Get-Content "project/code/sensor_imu.h" -Raw
$imuDriver = Get-Content "project/code/lsm6dsv16x_driver.c" -Raw
$imuDriverH = Get-Content "project/code/lsm6dsv16x_driver.h" -Raw
$gyroCalibration = Get-Content "project/code/imu_gyro_calibration.c" -Raw
$gyroCalibrationH = Get-Content "project/code/imu_gyro_calibration.h" -Raw
$safety = Get-Content "project/code/app_safety.c" -Raw
$config = Get-Content "project/code/app_config.h" -Raw
$hostSource = Get-Content "project/code/host_command.c" -Raw
$chassis = Get-Content "project/code/control_chassis.c" -Raw
$bldc = Get-Content "project/code/bldc_foc_uart.c" -Raw
$pwmDriver = Get-Content "libraries/zf_driver/zf_driver_pwm.c" -Raw

# Runtime servo updates must write the buffered compare register and then issue
# the TCPWM PWM-mode CAPTURE0 command that swaps the buffered value into CC0.
# This matches the proven vendor update sequence without modifying the driver.
Require-Pattern $servo 'Cy_Tcpwm_Pwm_SetCompare0_Buff\(' 'Servo runtime PWM writes must use the buffered compare API.'
Require-Pattern $servo 'Cy_Tcpwm_Pwm_SetCompare0_Buff\([^;]+;\s*Cy_Tcpwm_TriggerCapture0\(' 'Servo runtime PWM writes must trigger the buffered compare swap.'
Reject-Pattern $servo 'static void actuator_servo_write[^}]*pwm_set_duty\(' 'Servo runtime PWM writes must keep the vendor driver out of the 300 Hz ISR.'
Require-Pattern $servo 'actuator_servo_write_duty\(i, 0U\)' 'Runtime disable paths must also use the buffered PWM writer.'

# Keep the shared vendor driver byte-for-byte behavior out of the project fix.
Require-Pattern $pwmDriver 'unCC0_BUFF\.u32Register\s*=\s*compare;[\s\S]*unTR_CMD\.u32Register\s*\|=\s*1' 'Vendor PWM baseline unexpectedly changed.'

# A disabled RPM loop may maintain a throttled zero command, but it must not
# call the unconditional blocking stop sender on every 1 ms scheduler pass.
Reject-Pattern $motor 'actuator_motor_send_duty_periodic\(now_ms, 0, 0\);\s*actuator_motor_stop\(\);' 'Disabled RPM loop must not send an unconditional stop frame every millisecond.'
Require-Pattern $motor 'actuator_motor_last_send_ms\s*=\s*app_scheduler_get_ms\(\);' 'Stopping must preserve a real send timestamp for periodic throttling.'

# Target, filtered value, output, error and settled must all come from the same
# actuator ISR snapshot. Mixing the current planner command with the previous
# actuator snapshot makes settling measurements internally inconsistent.
Require-Pattern $leg 'servo_target_deg\[i\]\s*=\s*control_leg_actuator_diag\.target_deg\[i\]' 'Servo target telemetry must come from the actuator snapshot.'
Reject-Pattern $leg 'servo_target_deg\[i\]\s*=\s*control_leg_servo_cmd\.angle_deg\[i\]' 'Planner and actuator snapshots must not be mixed in one telemetry frame.'

# The hardware log must make scheduler skips, servo ISR progress and telemetry
# backpressure observable while keeping the 10 ms UART frame below 50% usage.
Require-Pattern $schedulerH 'app_scheduler_get_missed_tick_count' 'Scheduler must expose merged tick count.'
Require-Pattern $schedulerH 'app_scheduler_get_max_gap_ms' 'Scheduler must expose its maximum service gap.'
Require-Pattern $scheduler 'app_scheduler_missed_tick_count\s*\+=' 'Scheduler must count merged 1 ms ticks.'
Require-Pattern $servoH 'actuator_servo_get_tick_count' 'Servo actuator must expose its 300 Hz ISR tick count.'
Require-Pattern $telemetry 'float vofa_data\[55\]' 'Timing telemetry must emit 55 floats.'
Require-Pattern $telemetry 'telemetry_drop_count\+\+' 'Telemetry must count frames skipped under backpressure.'
Require-Pattern $collector '\$FloatCount\s*=\s*55' 'Collector must parse the timing diagnostics frame.'

$frameBytes = (55 * 4) + 4
$txMs = $frameBytes * 10.0 * 1000.0 / 460800.0
if(($txMs / 10.0) -ge 0.5) {
    throw 'Timing telemetry line utilization must remain below 50 percent.'
}

# Invalid SFLP payloads must not refresh a healthy sample, and failed SFLP
# initialization must not silently enter an INT1 configuration that cannot run.
Require-Pattern $imuDriver 'lsm6dsv16x_float_is_finite' 'SFLP quaternion inputs must be checked for finite values.'
Require-Pattern $imuDriver 'LSM6DSV_FIFO_OVR_IA' 'SFLP update must detect FIFO overrun.'
Require-Pattern $imuDriverH 'lsm6dsv16x_get_invalid_sample_count' 'IMU driver must expose rejected sample count.'
Require-Pattern $imu 'return SENSOR_IMU_ERR_SFLP;' 'SFLP initialization failure must be reported to the application.'
Require-Pattern $imuH 'sensor_imu_take_data_ready\(uint32 \*source_ms\)' 'IMU data-ready handoff must include the interrupt timestamp.'
Require-Pattern $safety 'app_safety_is_finite\(imu->roll\)' 'Top-level safety must reject non-finite attitude data.'

# Gyro bias calibration must use distinct 120 Hz samples and reject a moving
# base instead of silently storing motion as a permanent balance-rate bias.
Require-Pattern $imuDriverH 'uint8\s+lsm6dsv16x_gyro_offset_init' 'Gyro calibration must report stationarity failure.'
Require-Pattern $gyroCalibrationH 'imu_gyro_calibration_state_struct' 'Gyro calibration must use an online statistics state.'
Require-Pattern $gyroCalibration 'm2_y' 'Gyro calibration must compute variance with a stable online algorithm.'
Require-Pattern $gyroCalibration 'max_abs_mean_dps' 'Gyro calibration must reject constant angular motion.'
Require-Pattern $config 'APP_IMU_GYRO_CAL_MAX_VARIANCE_DPS2\s+\(4\.0f\)' 'Gyro variance threshold must include measured stationary-noise margin.'
Require-Pattern $imu 'APP_IMU_GYRO_CAL_RETRY_COUNT' 'IMU initialization must retry a transient moving-base calibration.'
Require-Pattern $imu 'return SENSOR_IMU_ERR_GYRO_CAL;' 'Persistent gyro calibration failure needs a distinct error code.'

# Drain the 64-byte vendor RX ring at 1 kHz under a short critical section,
# expire incomplete lines, and stop drive commands after host silence.
Require-Pattern $config 'APP_HOST_COMMAND_PERIOD_MS\s+\(1U\)' 'Host RX ring must be drained every millisecond.'
Require-Pattern $config 'APP_HOST_COMMAND_TIMEOUT_MS\s+\(500U\)' 'Direct motor commands need a host-loss timeout.'
Require-Pattern $config 'APP_CHASSIS_CMD_TIMEOUT_MS\s+\(500U\)' 'Chassis drive commands need a host-loss timeout.'
Require-Pattern $hostSource 'HOST_COMMAND_RX_BUFFER_LEN\s+\(64U\)' 'One host pass must be able to drain the full vendor RX ring.'
Require-Pattern $hostSource 'interrupt_global_disable\(\)[\s\S]*debug_read_ring_buffer' 'Host FIFO read must be atomic against the UART ISR writer.'
Require-Pattern $hostSource 'HOST_COMMAND_LINE_TIMEOUT_MS' 'Incomplete command lines must expire.'
Require-Pattern $chassis 'APP_CHASSIS_CMD_TIMEOUT_MS\s*<\s*\(now_ms - control_chassis_cmd\.last_cmd_ms\)' 'Chassis must stop stale drive commands.'
Require-Pattern $hostSource 'if\(0U == lsm6dsv16x_gyro_offset_init\(\)\)[\s\S]*actuator_motor_record_command_error\(APP_FALSE\)[\s\S]*actuator_motor_record_command_error\(APP_TRUE\)' 'IMU_ZERO must report a moving-base calibration failure.'

# All long-running millisecond logic must remain valid across uint32 wrap.
Reject-Pattern $leg 'control_leg_pose_start_ms\s*>\s*now_ms' 'S7 elapsed time must use unsigned subtraction across tick wrap.'
Reject-Pattern $motor 'now_ms\s*>=\s*raw->last_rx_ms' 'Wheel feedback age must use unsigned subtraction across tick wrap.'
Require-Pattern $safety 'app_safety_armed' 'Safety startup delay must latch armed state instead of repeating after tick wrap.'
Require-Pattern $safety 'actuator_servo_disable\(\);\s*actuator_motor_stop\(\);' 'Fault handling must clear servo PWM before the blocking motor stop frame.'
Require-Pattern $bldc 'APP_BLDC_FEEDBACK_RPM_ABS_MAX' 'BLDC speed feedback must enforce a physical range.'
Require-Pattern $bldc 'feedback_range_error_count\+\+' 'Rejected BLDC feedback must be counted.'

Write-Host "timing/noise regression checks passed"
