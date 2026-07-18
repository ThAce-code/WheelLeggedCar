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

Require-Pattern $kinematicsHeader 'leg_kinematics_forward_command\(uint8 right_side,' `
    "logical-servo-command FK API missing"
Require-Pattern $kinematicsSource 'cfg->alpha_reference_deg' `
    "command FK must reconstruct alpha from the calibrated reference"
Require-Pattern $kinematicsSource 'cfg->beta_reference_deg' `
    "command FK must reconstruct beta from the calibrated reference"
Require-Pattern $kinematicsSource 'leg_config_get_servo\(servo_a_index\)' `
    "command FK must use the per-servo logical direction"
Require-Pattern $configHeader 'LEG_PHYSICAL_WORKSPACE_VERTEX_COUNT\s*=\s*8' `
    "physical workspace must contain the fitted eight-vertex hull"
Require-Pattern $configHeader 'physical_reference_x_mm' `
    "kinematics config must expose the physical reference"
Require-Pattern $configHeader 'model_to_physical_m00' `
    "kinematics config must expose the constrained similarity matrix"
Require-Pattern $configHeader 'physical_workspace\[LEG_PHYSICAL_WORKSPACE_VERTEX_COUNT\]\[2\]' `
    "kinematics config must store the physical hull"

Require-Pattern $configSource '\.physical_reference_x_mm\s*=\s*-20\.766667f' `
    "physical reference X must be the three-reference mean"
Require-Pattern $configSource '\.physical_reference_y_mm\s*=\s*47\.356667f' `
    "physical reference Y must be the three-reference mean"
Require-Pattern $configSource '\.alpha_reference_deg\s*=\s*170\.536799f' `
    "fitted alpha reference missing"
Require-Pattern $configSource '\.beta_reference_deg\s*=\s*-4\.081158f' `
    "fitted beta reference missing"
Require-Pattern $configSource '\.model_to_physical_scale\s*=\s*0\.955219899f' `
    "fitted uniform scale missing"
Require-Pattern $configSource '\{-40\.620f,\s*47\.370f\}' `
    "fitted physical hull is missing"
Require-Pattern $configSource '\.physical_workspace_inset_mm\s*=\s*2\.0f' `
    "physical hull must be inset by 2 mm"

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
Require-Pattern $kinematicsSource 'leg_kinematics_target_valid\(x_mm, y_mm\)' `
    "public IK solve must reject targets outside the physical hull"
Reject-Pattern $kinematicsSource 'x\s*=\s*x_mm\s*\+\s*cfg->x_offset_mm' `
    "physical coordinates must not use the obsolete additive X offset"
Reject-Pattern $kinematicsSource 'y\s*=\s*y_mm\s*\+\s*cfg->y_offset_mm' `
    "physical coordinates must not use the obsolete additive Y offset"

Require-Pattern $controlSource 'leg_kinematics_target_valid\(x_mm, y_mm\)' `
    "control_leg_set_xy must delegate to the physical hull validator"
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
hull = [
    (-40.620, 47.370), (-30.910, 39.630),
    (-20.380, 32.170), (-15.040, 47.600),
    (-22.030, 88.490), (-31.420, 74.120),
    (-37.940, 59.340), (-39.580, 53.010),
]

def model_to_physical(point):
    dx, dy = point[0] - lref[0], point[1] - lref[1]
    return (pref[0] + scale * (m[0] * dx + m[1] * dy),
            pref[1] + scale * (m[2] * dx + m[3] * dy))

def physical_to_model(point):
    dx, dy = (point[0] - pref[0]) / scale, (point[1] - pref[1]) / scale
    return (lref[0] + m[0] * dx + m[2] * dy,
            lref[1] + m[1] * dx + m[3] * dy)

def valid(point, inset=2.0):
    for index, first in enumerate(hull):
        second = hull[(index + 1) % len(hull)]
        ex, ey = second[0] - first[0], second[1] - first[1]
        distance = (ex * (point[1] - first[1]) - ey * (point[0] - first[0])) / math.hypot(ex, ey)
        if distance < inset:
            return False
    return True

def command_to_geometric(command, neutral, offset, direction, reference):
    return reference + ((command - neutral - offset) / direction)

assert max(abs(a-b) for a,b in zip(model_to_physical(lref), pref)) < 1e-6
assert abs(command_to_geometric(90.0, 90.0, 0.0, -1.0, 170.536799) - 170.536799) < 1e-9
assert abs(command_to_geometric(90.0, 90.0, 0.0, -1.0, -4.081158) - (-4.081158)) < 1e-9
probe = (20.0, 70.0)
assert max(abs(a-b) for a,b in zip(physical_to_model(model_to_physical(probe)), probe)) < 1e-5
assert valid(pref)
assert not valid((0.0, 55.0))
'@ | python -
if(0 -ne $LASTEXITCODE) {
    throw "physical IK transform numeric checks failed"
}

Write-Host "physical-coordinate leg IK static test passed"
