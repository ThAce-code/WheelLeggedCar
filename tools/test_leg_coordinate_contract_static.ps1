$ErrorActionPreference = "Stop"

function Require-Pattern {
    param([string]$Text, [string]$Pattern, [string]$Message)
    if($Text -notmatch $Pattern) { throw $Message }
}

function Reject-Pattern {
    param([string]$Text, [string]$Pattern, [string]$Message)
    if($Text -match $Pattern) { throw $Message }
}

$types = Get-Content "project/code/app_types.h" -Raw
$leg = Get-Content "project/code/control_leg.c" -Raw
$telemetry = Get-Content "project/code/telemetry.c" -Raw
$collector = Get-Content "tools/collect_balance_data.ps1" -Raw
$calibration = Get-Content "tools/calib_ik_servo.ps1" -Raw
$ikDoc = Get-Content "docs/leg-ik-zero-calibration-hardware-test.md" -Raw
$heightDoc = Get-Content "docs/leg-height-phase1-hardware-test.md" -Raw

Require-Pattern $types 'LEG_POSE_SOURCE_NONE\s*=\s*0[\s\S]*LEG_POSE_SOURCE_MEASURED_CALIBRATION\s*=\s*1[\s\S]*LEG_POSE_SOURCE_MIRROR_ASSUMPTION\s*=\s*2' 'Leg pose provenance enum is missing.'
Require-Pattern $types 'LEG_POSE_STATUS_IK_VALID\s*=\s*\(1U\s*<<\s*0\)[\s\S]*LEG_POSE_STATUS_LEFT_VALID\s*=\s*\(1U\s*<<\s*1\)[\s\S]*LEG_POSE_STATUS_RIGHT_VALID\s*=\s*\(1U\s*<<\s*2\)[\s\S]*LEG_POSE_STATUS_LEFT_MEASURED\s*=\s*\(1U\s*<<\s*3\)[\s\S]*LEG_POSE_STATUS_RIGHT_MIRROR\s*=\s*\(1U\s*<<\s*4\)' 'Leg pose status-bit enum is missing.'
Require-Pattern $types 'typedef struct\s*\{\s*float x_mm;\s*float y_mm;\s*leg_pose_source_enum source;\s*uint8 valid;\s*\}leg_pose_command_estimate_struct;' 'Leg command-pose estimate type is missing.'
Require-Pattern $types 'leg_pose_command_estimate_struct left_command_pose_body_mm;' 'Leg diagnostics must contain the left physical command pose.'
Require-Pattern $types 'leg_pose_command_estimate_struct right_command_pose_body_mm;' 'Leg diagnostics must contain the right physical command pose.'
Reject-Pattern $types '\bactual_height_mm\b' 'PWM-only diagnostics must not expose actual_height_mm.'
Reject-Pattern $types '\b(left|right)_[xy]_mm\b' 'Flat physical pose fields must be removed.'

Require-Pattern $leg 'leg_kinematics_forward_command\(right_side,[\s\S]*control_leg_actuator_diag\.output_deg\[servo_a_index\][\s\S]*control_leg_actuator_diag\.output_deg\[servo_b_index\]' 'Physical poses must be derived from actuator output commands.'
Require-Pattern $leg 'control_leg_publish_command_pose\(APP_FALSE,[\s\S]*LEG_POSE_SOURCE_MEASURED_CALIBRATION,[\s\S]*LEG_SERVO_FL,[\s\S]*LEG_SERVO_RL' 'Left physical pose must use the left actuator pair.'
Require-Pattern $leg 'control_leg_publish_command_pose\(APP_TRUE,[\s\S]*LEG_POSE_SOURCE_MIRROR_ASSUMPTION,[\s\S]*LEG_SERVO_FR,[\s\S]*LEG_SERVO_RR' 'Right physical pose must use the right actuator pair.'
Require-Pattern $leg 'LEG_POSE_SOURCE_MEASURED_CALIBRATION' 'Left pose must report measured-calibration provenance.'
Require-Pattern $leg 'LEG_POSE_SOURCE_MIRROR_ASSUMPTION' 'Right pose must report mirror-assumption provenance.'
Require-Pattern $leg 'APP_TRUE\s*==\s*leg_kinematics_forward_command[\s\S]*command_pose->x_mm\s*=[\s\S]*command_pose->y_mm\s*=[\s\S]*command_pose->valid\s*=\s*APP_TRUE;[\s\S]*else[\s\S]*command_pose->valid\s*=\s*APP_FALSE;' 'Forward-command failure must preserve coordinates and clear validity.'
Reject-Pattern $leg '(left|right)_x_mm\s*=\s*0\.0f' 'Physical X must never be fabricated as zero.'
Reject-Pattern $leg '(left|right)_y_mm\s*=\s*control_leg_legacy_stance_ref_units' 'Legacy stance must never be copied into physical Y.'
Reject-Pattern $leg 'actual_height_mm\s*=' 'Legacy stance must not be published as actual height.'

Require-Pattern $telemetry 'float vofa_data\[55\]' 'Telemetry must remain exactly 55 floats.'
Require-Pattern $telemetry 'vofa_data\[13\]\s*=\s*leg->legacy_stance_target_units' 'Telemetry index 13 must retain legacy_stance_target_units.'
Require-Pattern $telemetry 'vofa_data\[14\]\s*=\s*leg->legacy_stance_ref_units' 'Telemetry index 14 must retain legacy_stance_ref_units.'
Require-Pattern $telemetry 'vofa_data\[15\]\s*=\s*leg->legacy_stance_norm' 'Telemetry index 15 must retain legacy_stance_norm.'
Require-Pattern $telemetry 'vofa_data\[16\]\s*=\s*\(float\)pose_status_flags' 'Telemetry index 16 must encode pose status flags.'
Require-Pattern $telemetry 'vofa_data\[17\]\s*=\s*\(float\)leg->output_enable' 'Telemetry index 17 must retain output enable.'
Require-Pattern $telemetry 'vofa_data\[33\]\s*=\s*leg->left_command_pose_body_mm\.x_mm' 'Telemetry index 33 must be left command X.'
Require-Pattern $telemetry 'vofa_data\[34\]\s*=\s*leg->left_command_pose_body_mm\.y_mm' 'Telemetry index 34 must be left command Y.'
Require-Pattern $telemetry 'vofa_data\[35\]\s*=\s*leg->right_command_pose_body_mm\.x_mm' 'Telemetry index 35 must be right command X.'
Require-Pattern $telemetry 'vofa_data\[36\]\s*=\s*leg->right_command_pose_body_mm\.y_mm' 'Telemetry index 36 must be right command Y.'
Require-Pattern $telemetry 'vofa_data\[37\]\s*=\s*leg->ik_margin' 'Telemetry index 37 must retain IK margin.'
Require-Pattern $telemetry 'LEG_POSE_STATUS_IK_VALID[\s\S]*LEG_POSE_STATUS_LEFT_VALID[\s\S]*LEG_POSE_STATUS_RIGHT_VALID[\s\S]*LEG_POSE_STATUS_LEFT_MEASURED[\s\S]*LEG_POSE_STATUS_RIGHT_MIRROR' 'Telemetry must encode all five pose status bits.'

Require-Pattern $collector '\$FloatCount\s*=\s*55' 'Collector must keep the 55-float wire contract.'
Require-Pattern $collector 'leg_left_command_x_mm' 'Collector must decode left physical X.'
Require-Pattern $collector 'leg_left_command_y_mm' 'Collector must decode left physical Y.'
Require-Pattern $collector 'leg_right_command_x_mm' 'Collector must decode right physical X.'
Require-Pattern $collector 'leg_right_command_y_mm' 'Collector must decode right physical Y.'
Require-Pattern $collector 'leg_left_pose_source' 'Collector must decode left provenance.'
Require-Pattern $collector 'leg_right_pose_source' 'Collector must decode right provenance.'
Require-Pattern $collector 'leg_legacy_stance_target_units' 'Collector must name the legacy stance target honestly.'
Require-Pattern $collector 'leg_legacy_stance_ref_units' 'Collector must name the legacy stance reference honestly.'
Require-Pattern $collector 'leg_legacy_stance_norm' 'Collector must name the legacy stance norm honestly.'
Require-Pattern $calibration 'left_command_x_mm' 'Calibration CSV must record left physical X.'
Require-Pattern $calibration 'right_command_x_mm' 'Calibration CSV must record right physical X.'
Require-Pattern $calibration 'left_pose_valid' 'Calibration CSV must record left pose validity.'
Require-Pattern $calibration 'right_pose_valid' 'Calibration CSV must record right pose validity.'
Require-Pattern $calibration 'left_pose_source' 'Calibration CSV must record left pose provenance.'
Require-Pattern $calibration 'right_pose_source' 'Calibration CSV must record right pose provenance.'
Require-Pattern $calibration '\$csvFields\s*=[\s\S]*"leg_pose_status_flags"' 'Calibration CSV must record the packed pose status.'
Require-Pattern $calibration 'legacy_stance_ref_units' 'Calibration CSV must name the legacy stance reference honestly.'

$runtimeFiles = @(
    "project/code/app_types.h",
    "project/code/control_leg.h",
    "project/code/control_leg.c",
    "project/code/control_balance.c",
    "project/code/control_chassis.c",
    "project/code/telemetry.c"
)
foreach($path in $runtimeFiles) {
    $runtimeText = Get-Content $path -Raw
    Reject-Pattern $runtimeText '\b(actual_height_mm|target_height_mm|height_ref_mm|height_norm|reference_x_mm|reference_y_mm|control_leg_ik_validation_point_valid)\b' `
        "stale coordinate-domain symbol remains in $path"
}
Reject-Pattern $leg 'left_command_pose_body_mm\.x_mm\s*=\s*0\.0f' `
    'Invalid pose must not fabricate left command X=0.'
Reject-Pattern $leg 'left_command_pose_body_mm\.y_mm\s*=\s*control_leg_legacy' `
    'Legacy stance must not fabricate left command Y.'
Require-Pattern $ikDoc 'BODY_WHEEL[\s\S]*cross-circle[\s\S]*\+X[\s\S]*forward[\s\S]*\+Y[\s\S]*downward' `
    'BODY_WHEEL frame definition is missing from the IK hardware procedure.'
Require-Pattern $ikDoc 'LIKREF\s*=\s*\(-20\.766667,\s*47\.356667\)' `
    'Exact physical LIKREF point is missing from the IK hardware procedure.'
Require-Pattern $ikDoc 'LXY,0,55[\s\S]*accepted only\s+when[\s\S]*model-reachable' `
    'The model-reachable LXY,0,55 command must not be pre-rejected.'
Require-Pattern $ikDoc 'LXY,1000,1000[\s\S]*must be rejected' `
    'The truly unreachable LXY example must remain fail-closed.'
Require-Pattern $ikDoc 'model-reachable[\s\S]*0\.02[\s\S]*servo limits' `
    'Hardware procedure must define model-reachable LXY acceptance.'
Require-Pattern $ikDoc 'command/model estimate' `
    'Hardware procedure must distinguish command/model estimates from feedback.'
Reject-Pattern $ikDoc 'convex workspace inset by 2 mm|command hull' `
    'Hardware procedure must not retain the legacy hull as an active boundary.'
Reject-Pattern $ikDoc 'hardware-tested mirrored command' `
    'Hardware procedure must not claim hardware acceptance for the model example.'
Require-Pattern $ikDoc 'right leg[\s\S]*mirror\s+assumption[\s\S]*independent' `
    'Right-leg independent validation gate is missing from the IK hardware procedure.'
Require-Pattern $ikDoc 'motor-disabled[\s\S]*Target X[\s\S]*Measured X[\s\S]*Error X[\s\S]*Pose-status[\s\S]*Servo outputs' `
    'Motor-disabled physical-coordinate evidence table is missing from the IK hardware procedure.'
Require-Pattern $heightDoc 'legacy stance units' `
    'Height hardware procedure must label LH/LHF inputs as legacy stance units.'

$frameBytes = (55 * 4) + 4
$txMs = $frameBytes * 10.0 * 1000.0 / 460800.0
if(($txMs / 10.0) -ge 0.5) { throw 'Telemetry line utilization must remain below 50 percent.' }

Write-Host "leg coordinate contract static checks passed"
