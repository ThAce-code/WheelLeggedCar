$ErrorActionPreference = "Stop"

function Require-Pattern([string]$Path, [string]$Pattern, [string]$Message) {
    $text = Get-Content $Path -Raw
    if($text -notmatch $Pattern) {
        throw $Message
    }
}

function Reject-Pattern([string]$Path, [string]$Pattern, [string]$Message) {
    $text = Get-Content $Path -Raw
    if($text -match $Pattern) {
        throw $Message
    }
}

$configHeader = "project/code/leg_config.h"
$configSource = "project/code/leg_config.c"
$kinematicsHeader = "project/code/leg_kinematics.h"
$kinematicsSource = "project/code/leg_kinematics.c"
$controlSource = "project/code/control_leg.c"
$ikDoc = "docs/leg-ik-zero-calibration-hardware-test.md"

Require-Pattern $kinematicsHeader 'leg_kinematics_forward_command\(uint8 right_side,' `
    "logical-servo-command FK API missing"
Require-Pattern $kinematicsSource 'cfg->alpha_reference_deg' `
    "command FK must reconstruct alpha from the calibrated reference"
Require-Pattern $kinematicsSource 'cfg->beta_reference_deg' `
    "command FK must reconstruct beta from the calibrated reference"
Require-Pattern $kinematicsSource 'leg_config_get_servo\(servo_a_index\)' `
    "command FK must use the per-servo logical direction"
Require-Pattern $kinematicsSource 'servo_a_command_deg\s*<\s*servo_a->min_deg' `
    "command FK must reject servo A below its configured minimum"
Require-Pattern $kinematicsSource 'servo_a_command_deg\s*>\s*servo_a->max_deg' `
    "command FK must reject servo A above its configured maximum"
Require-Pattern $kinematicsSource 'servo_b_command_deg\s*<\s*servo_b->min_deg' `
    "command FK must reject servo B below its configured minimum"
Require-Pattern $kinematicsSource 'servo_b_command_deg\s*>\s*servo_b->max_deg' `
    "command FK must reject servo B above its configured maximum"

$kinematicsText = Get-Content $kinematicsSource -Raw
$forwardStart = $kinematicsText.IndexOf("uint8 leg_kinematics_forward_command")
$nextFunction = $kinematicsText.IndexOf("`n}", $forwardStart)
if($forwardStart -lt 0 -or $nextFunction -lt 0) {
    throw "could not isolate command FK implementation"
}
$forwardCommand = $kinematicsText.Substring($forwardStart, $nextFunction - $forwardStart)
$rangeCheck = $forwardCommand.IndexOf("servo_a_command_deg < servo_a->min_deg")
$firstOutputWrite = $forwardCommand.IndexOf("*x_mm =")
if($rangeCheck -lt 0 -or $firstOutputWrite -lt 0 -or $rangeCheck -gt $firstOutputWrite) {
    throw "command range rejection must happen before either output is written"
}
Require-Pattern $kinematicsHeader '(?s)BODY_WHEEL.*cross-circle.*\+X forward.*\+Y down.*millimetres.*command/model estimates.*not measured feedback' `
    "public physical-coordinate contract comment missing"
Require-Pattern $configHeader 'physical_reference_x_mm' `
    "kinematics config must expose the physical reference"
Require-Pattern $configHeader 'model_to_physical_m00' `
    "kinematics config must expose the constrained similarity matrix"
Reject-Pattern $configHeader 'physical_workspace|experimental_race_|x_min_mm|x_max_mm|y_min_mm|y_max_mm' `
    "legacy workspace fields must be removed"
Reject-Pattern $kinematicsSource 'leg_kinematics_experimental_race_target_valid|leg_kinematics_model_workspace_valid' `
    "coordinate and rectangle gates must be removed"

Require-Pattern $configSource '\.physical_reference_x_mm\s*=\s*-20\.766667f' `
    "physical reference X must be the three-reference mean"
Require-Pattern $configSource '\.physical_reference_y_mm\s*=\s*47\.356667f' `
    "physical reference Y must be the three-reference mean"
Require-Pattern $configSource '\.alpha_reference_deg\s*=\s*170\.536799f' `
    "fitted alpha reference missing"
Require-Pattern $configSource '\.beta_reference_deg\s*=\s*-4\.081158f' `
    "fitted beta reference missing"
Require-Pattern $configSource '\.model_reference_x_mm\s*=\s*22\.830129f' `
    "fitted model reference X missing"
Require-Pattern $configSource '\.model_reference_y_mm\s*=\s*46\.929213f' `
    "fitted model reference Y missing"
Require-Pattern $configSource '\.model_to_physical_scale\s*=\s*0\.955219899f' `
    "fitted uniform scale missing"
Require-Pattern $configSource '\.model_to_physical_m00\s*=\s*-0\.996313812f' `
    "fitted matrix m00 missing"
Require-Pattern $configSource '\.model_to_physical_m01\s*=\s*0\.085783378f' `
    "fitted matrix m01 missing"
Require-Pattern $configSource '\.model_to_physical_m10\s*=\s*0\.085783378f' `
    "fitted matrix m10 missing"
Require-Pattern $configSource '\.model_to_physical_m11\s*=\s*0\.996313812f' `
    "fitted matrix m11 missing"
Require-Pattern $configSource '\.ik_min_margin\s*=\s*0\.02f' `
    "model workspace must use margin 0.02"

Require-Pattern $configSource '\{0,\s*90\.0f,\s*90\.0f,\s*10\.0f,\s*175\.0f,\s*-1\.0f' `
    "servo0 IK direction must match the fitted visible-leg direction"
Require-Pattern $configSource '\{2,\s*90\.0f,\s*90\.0f,\s*10\.0f,\s*175\.0f,\s*-1\.0f' `
    "servo2 IK direction must match the fitted visible-leg direction"

Require-Pattern $kinematicsHeader 'leg_kinematics_target_valid\(float x_mm,\s*float y_mm\)' `
    "public physical target validation API missing"
Require-Pattern $kinematicsSource 'leg_kinematics_physical_to_model' `
    "physical-to-model inverse similarity missing"
Require-Pattern $kinematicsSource 'leg_kinematics_model_to_physical' `
    "model-to-physical similarity missing"
Require-Pattern $kinematicsSource 'leg_kinematics_solve_model' `
    "five-bar model solver must be isolated from physical conversion"
Require-Pattern $kinematicsSource 'profile->ik_min_margin' `
    "normal targets must retain the calibrated IK margin"
Require-Pattern $kinematicsSource 'leg_kinematics_map_candidate' `
    "candidate reachability must use servo mapping"
Reject-Pattern $kinematicsSource 'return\s+APP_TRUE;\s*/\*.*bypass' `
    "physical target validation must not be globally bypassed"
Reject-Pattern $kinematicsSource 'x\s*=\s*x_mm\s*\+\s*cfg->x_offset_mm' `
    "physical coordinates must not use the obsolete additive X offset"
Reject-Pattern $kinematicsSource 'y\s*=\s*y_mm\s*\+\s*cfg->y_offset_mm' `
    "physical coordinates must not use the obsolete additive Y offset"

Require-Pattern $controlSource 'leg_kinematics_target_valid\(x_mm, y_mm\)' `
    "control_leg_set_xy must delegate to the model reachability validator"
Reject-Pattern $controlSource 'control_leg_ik_validation_point_valid' `
    "legacy rectangular cross-band validation must not define physical reachability"
Require-Pattern $controlSource 'kinematics->physical_reference_x_mm' `
    "LIKREF must use the measured physical reference X"
Require-Pattern $controlSource 'kinematics->physical_reference_y_mm' `
    "LIKREF must use the measured physical reference Y"
Reject-Pattern $controlSource 'kinematics->reference_[xy]_mm' `
    "controller must not use the removed model-coordinate reference fields"

@'
import math

pref = (-20.766667, 47.356667)
lref = (22.830129, 46.929213)
scale = 0.955219899
m = (-0.996313812, 0.085783378, 0.085783378, 0.996313812)

def model_to_physical(point):
    dx, dy = point[0] - lref[0], point[1] - lref[1]
    return (pref[0] + scale * (m[0] * dx + m[1] * dy),
            pref[1] + scale * (m[2] * dx + m[3] * dy))

def physical_to_model(point):
    dx, dy = (point[0] - pref[0]) / scale, (point[1] - pref[1]) / scale
    return (lref[0] + m[0] * dx + m[2] * dy,
            lref[1] + m[1] * dx + m[3] * dy)

def command_to_geometric(command, neutral, offset, direction, reference):
    return reference + ((command - neutral - offset) / direction)

assert max(abs(a-b) for a,b in zip(model_to_physical(lref), pref)) < 1e-6
assert abs(command_to_geometric(90.0, 90.0, 0.0, -1.0, 170.536799) - 170.536799) < 1e-9
assert abs(command_to_geometric(90.0, 90.0, 0.0, -1.0, -4.081158) - (-4.081158)) < 1e-9
probe = (20.0, 70.0)
assert max(abs(a-b) for a,b in zip(physical_to_model(model_to_physical(probe)), probe)) < 1e-5
'@ | python -
if(0 -ne $LASTEXITCODE) {
    throw "physical IK transform numeric checks failed"
}

Require-Pattern $ikDoc 'model-reachable' `
    "hardware procedure must define model-reachable LXY acceptance"
Require-Pattern $ikDoc '0\.02' `
    "hardware procedure must state the minimum IK margin"
Require-Pattern $ikDoc 'servo limits' `
    "hardware procedure must require mapped servo-limit validation"
Require-Pattern $ikDoc 'command/model estimate' `
    "hardware procedure must distinguish command/model estimates from feedback"
Reject-Pattern $ikDoc 'convex workspace inset by 2 mm|command hull' `
    "hardware procedure must not claim the legacy hull remains the active boundary"

Write-Host "physical-coordinate leg IK static test passed"
