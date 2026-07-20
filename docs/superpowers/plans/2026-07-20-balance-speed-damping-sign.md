# Balance Speed-Damping Sign Correction Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make a positive wheel-speed damping gain oppose controller-visible vehicle-forward wheel motion.

**Architecture:** Preserve the installed-car contract that positive wheel RPM means vehicle-forward motion and preserve positive `Ks` as a damping magnitude. Enforce the cross-layer sign relationship in the existing motor-installation semantics test, then apply an explicit negative sign at the balance speed-term calculation.

**Tech Stack:** Embedded C for CYT4BB7/Traveo II, PowerShell static regression tests, Git.

## Global Constraints

- Address only the P0 balance speed-damping regression.
- Do not change host-command timeouts, balance P/D gains, pitch setpoint, motor direction, fast-mode limits, race assist, or leg control.
- Keep `APP_BALANCE_WHEEL_SPEED_KS` and `APP_BALANCE_FAST_WHEEL_SPEED_KS` positive and numerically unchanged.
- Do not claim unsupported standing is fixed without restrained hardware validation.

---

### Task 1: Enforce and Correct the Wheel-Speed Damping Sign

**Files:**
- Modify: `tools/test_motor_installation_semantics.ps1:16-37`
- Modify: `project/code/control_balance.c:393-396`

**Interfaces:**
- Consumes: `effective_wheel_speed_ks` as the interpolated positive damping magnitude and `wheel_speed_rpm` as controller-visible RPM that is positive vehicle-forward.
- Produces: `speed_term_rpm`, which must oppose nonzero `wheel_speed_rpm` when `effective_wheel_speed_ks` is positive.

- [ ] **Step 1: Write the failing cross-layer sign test**

Add the balance source read next to the existing source reads in `tools/test_motor_installation_semantics.ps1`:

```powershell
$balance = Get-Content "project/code/control_balance.c" -Raw
```

Add this assertion after the installed direction assertions:

```powershell
# Positive controller-visible wheel RPM is vehicle-forward, while positive Ks
# is a damping magnitude. The balance speed term must therefore oppose RPM.
Require-Pattern $balance `
    'speed_term_rpm\s*=\s*-\s*effective_wheel_speed_ks\s*\*\s*wheel_speed_rpm\s*;' `
    'Positive balance Ks must oppose positive vehicle-forward wheel RPM.'
```

- [ ] **Step 2: Run the test and verify the RED state**

Run from `D:\smartcar\WheelLeggedCar_cyt4bb7_v1\.worktree\balance-fast-mode-spec`:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools\test_motor_installation_semantics.ps1
```

Expected: FAIL with `Positive balance Ks must oppose positive vehicle-forward wheel RPM.` because production code still uses positive multiplication.

- [ ] **Step 3: Implement the minimal production correction**

Replace the speed-term line in `project/code/control_balance.c` with:

```c
        /* Wheel RPM is positive vehicle-forward; positive Ks is damping magnitude. */
        speed_term_rpm = -effective_wheel_speed_ks * wheel_speed_rpm;
```

Do not change the adjacent pitch, rate, position, or feedforward terms.

- [ ] **Step 4: Run the focused test and verify the GREEN state**

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools\test_motor_installation_semantics.ps1
```

Expected: PASS with `motor installation semantics checks passed`.

- [ ] **Step 5: Run balance and data-path regressions**

```powershell
$tests = @(
    'tools\test_balance_drive_v1_static.ps1',
    'tools\test_balance_drive_v2_static.ps1',
    'tools\test_timing_noise_regressions.ps1',
    'tools\test_collect_balance_data.ps1',
    'tools\test_tune_drive_loops_static.ps1'
)
foreach($test in $tests) {
    & powershell -NoProfile -ExecutionPolicy Bypass -File $test
    if($LASTEXITCODE -ne 0) {
        throw "Regression failed: $test"
    }
}
```

Expected: every script exits `0` and prints its pass message.

- [ ] **Step 6: Verify scope and patch hygiene**

```powershell
git diff --check
git status --short --branch --untracked-files=no
git diff -- project/code/control_balance.c tools/test_motor_installation_semantics.ps1
```

Expected: `git diff --check` exits `0`; the implementation diff contains only the balance speed-term sign/comment and its regression assertion. The already committed design document is not part of the implementation diff.

- [ ] **Step 7: Commit the software correction**

```powershell
git add -- project/code/control_balance.c tools/test_motor_installation_semantics.ps1 docs/superpowers/plans/2026-07-20-balance-speed-damping-sign.md
git diff --cached --check
git commit -m "Fix balance wheel speed damping sign"
```

Expected: commit succeeds with exactly the production correction, regression test, and implementation plan.

- [ ] **Step 8: Record the hardware handoff without claiming board acceptance**

Report that software tests passed and that restrained hardware validation remains required. The first board sequence is `STOP`, telemetry health check, `BL,18,8,0,0`, the currently appropriate restrained `BS` value, `B,1`, `B,2`, and explicit `C,0,0`; positive damping must be reintroduced from a small value only after PD polarity is confirmed.
