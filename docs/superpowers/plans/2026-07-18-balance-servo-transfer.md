# Balance Fast Mode Servo Control Transfer Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fast-forward `codex/balance-fast-mode-spec` from `da1ebe5` to the clean servo/leg milestone `4f057a1a`, validate the transferred control stack, and attach the approved design and implementation-plan documents without importing the later cone-perception runtime tail.

**Architecture:** Preserve the existing linear Git history with `git merge --ff-only 4f057a1a`; do not reconstruct the servo stack through cherry-picks or file copies. Validate servo, leg/IK, balance, and timing contracts at the transferred milestone, then cherry-pick only the two documentation commits onto the target branch.

**Tech Stack:** Git worktrees, PowerShell, embedded C, GCC C99 host-side numeric harnesses, IAR Embedded Workbench 9.40.1 for the later CM7_0 hardware build gate.

## Global Constraints

- Work only in `D:\smartcar\WheelLeggedCar_cyt4bb7_v1\.worktree\balance-fast-mode-spec`.
- The code milestone must be exactly `4f057a1a`; do not import `5c216e4` or any later cone-perception runtime commit.
- Preserve the existing untracked `.superpowers/`, `data/`, and `fast_tune_kit.zip`; never stage or delete them.
- Do not change `APP_CHASSIS_FAST_FORWARD_RPM_LIMIT`, `APP_BALANCE_RPM_LIMIT`, motor limits, fast-mode gates, or fault gates.
- Do not push, create a PR, flash hardware, or run a loaded high-speed test in this plan.
- The local environment has no discoverable `IarBuild.exe`; report the CM7_0 IAR build as `NOT RUN` rather than treating host tests as a substitute.
- Do not use `git reset --hard`, `git checkout --`, or bulk formatting to clean unrelated files.

## File and History Map

- Move branch ref: `codex/balance-fast-mode-spec`, from `da1ebe5` through the existing linear history to `4f057a1a`.
- Retain source history for `project/code/actuator_servo.c`, `project/code/actuator_servo.h`, `project/code/servo_motion.c`, `project/code/servo_motion.h`, `project/code/control_leg.c`, `project/code/leg_config.c`, `project/code/leg_kinematics.c`, scheduler/telemetry integration, and the CM7_0 IAR project file.
- Attach after validation: `docs/superpowers/specs/2026-07-18-balance-servo-transfer-design.md`.
- Attach after validation: `docs/superpowers/plans/2026-07-18-balance-servo-transfer.md`.
- Execute existing tests only; no production source or test source is edited by this plan.

---

### Task 1: Revalidate and fast-forward the target branch

**Files:**

- Modify through existing commits: `project/code`, `project/user`, `project/iar/project_config/cyt4bb7_cm_7_0.ewp`, `tools`, and associated documentation already present in `da1ebe5..4f057a1a`.
- Do not modify manually: any source file.

**Interfaces:**

- Consumes: target ref `codex/balance-fast-mode-spec` at `da1ebe5`; source milestone `4f057a1a`.
- Produces: target ref fast-forwarded to `4f057a1a` with no merge commit.

- [ ] **Step 1: Verify the planning worktree has only the known untracked artifacts**

Run:

```powershell
git status --short --branch
```

Expected: branch `codex/balance-servo-transfer-design`; no tracked modifications; only `.superpowers/`, `data/`, and `fast_tune_kit.zip` may appear as untracked.

- [ ] **Step 2: Revalidate the exact ancestry immediately before the write**

Run:

```powershell
git rev-parse codex/balance-fast-mode-spec
git merge-base codex/balance-fast-mode-spec 4f057a1a
git rev-list --left-right --count codex/balance-fast-mode-spec...4f057a1a
```

Expected output:

```text
da1ebe5ce7a8437970d0f4d884e7502efbb526a1
da1ebe5ce7a8437970d0f4d884e7502efbb526a1
0    84
```

Stop if either SHA differs or the count is not `0 84`.

- [ ] **Step 3: Switch to the target branch**

Run:

```powershell
git switch codex/balance-fast-mode-spec
git branch --show-current
```

Expected output ends with:

```text
codex/balance-fast-mode-spec
```

- [ ] **Step 4: Fast-forward to the clean servo/leg milestone**

Run:

```powershell
git merge --ff-only 4f057a1a
git rev-parse HEAD
git log -1 --oneline
```

Expected final output:

```text
4f057a1adf5764e5cce38e713e1b992fd1c11a0e
4f057a1 Fix measured five-bar IK geometry
```

- [ ] **Step 5: Prove the excluded runtime tail is absent**

Run:

```powershell
git merge-base --is-ancestor 5c216e4 HEAD
if(0 -eq $LASTEXITCODE) {
    throw 'Excluded cone-perception runtime tail is present.'
}
git merge-base --is-ancestor 4f057a1a HEAD
if(0 -ne $LASTEXITCODE) {
    throw 'Required clean leg milestone is missing.'
}
Write-Host 'transfer boundary checks passed'
```

Expected output:

```text
transfer boundary checks passed
```

---

### Task 2: Verify the 300 Hz servo execution stack

**Files:**

- Verify: `project/code/actuator_servo.c`.
- Verify: `project/code/actuator_servo.h`.
- Verify: `project/code/servo_motion.c`.
- Verify: `project/code/servo_motion.h`.
- Test: `tools/test_servo_motion_numeric.ps1`.
- Test: `tools/test_servo_300hz_integration_static.ps1`.
- Test: `tools/test_servo_pwm_frequency_static.ps1`.
- Test: `tools/test_servo_pwm_resolution_static.ps1`.

**Interfaces:**

- Consumes: the transferred 300 Hz PIT/TCPWM execution path and servo motion filter.
- Produces: evidence that duty mapping, LPF/rate limiting, ISR integration, settle reporting, compare-buffer swap, and nonblocking telemetry contracts remain intact.

- [ ] **Step 1: Run the servo motion numeric harness**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_servo_motion_numeric.ps1
```

Expected output:

```text
servo motion numeric checks passed
```

- [ ] **Step 2: Run the 300 Hz integration contract**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_servo_300hz_integration_static.ps1
```

Expected output:

```text
servo 300 Hz integration static checks passed
```

- [ ] **Step 3: Verify production PWM frequency and PIT scheduling**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_servo_pwm_frequency_static.ps1
```

Expected output:

```text
servo PWM 300 Hz production static check passed
```

- [ ] **Step 4: Verify 500/1500/2500 us duty mapping**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_servo_pwm_resolution_static.ps1
```

Expected output:

```text
servo PWM 300 Hz resolution static check passed
```

---

### Task 3: Verify synchronized leg motion and IK safety

**Files:**

- Verify: `project/code/control_leg.c`.
- Verify: `project/code/leg_config.c`.
- Verify: `project/code/leg_kinematics.c`.
- Test: `tools/test_leg_transition_numeric.ps1`.
- Test: `tools/test_ik_height_control_static.ps1`.

**Interfaces:**

- Consumes: S7 trajectory state, servo settle status, measured five-bar geometry, and chassis drive gate.
- Produces: evidence that the transferred leg controller remains synchronized, numerically valid, fail-closed, and compatible with the balance drive gate.

- [ ] **Step 1: Run the leg transition and IK numeric sweep**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_leg_transition_numeric.ps1
```

Expected output ends with:

```text
leg transition numeric checks passed
```

- [ ] **Step 2: Run the leg-height static safety contract**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_ik_height_control_static.ps1
```

Expected output:

```text
ik height control static checks passed
```

---

### Task 4: Verify balance and shared timing safety

**Files:**

- Verify: `project/code/control_balance.c`.
- Verify: `project/code/control_chassis.c`.
- Verify: `project/code/app_scheduler.c`.
- Verify: `project/code/app_safety.c`.
- Verify: `project/code/sensor_imu.c`.
- Test: `tools/test_timing_noise_regressions.ps1`.
- Test: `tools/test_balance_drive_v2_static.ps1`.

**Interfaces:**

- Consumes: the original fast-mode balance implementation plus transferred scheduler, IMU calibration, host timeout, and servo fault-path changes.
- Produces: evidence that the servo transfer did not remove fast mode or weaken shared freshness, timeout, wraparound, and fault gates.

- [ ] **Step 1: Run timing and noise regressions**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_timing_noise_regressions.ps1
```

Expected output:

```text
timing/noise regression checks passed
```

- [ ] **Step 2: Run the fast-mode balance static contract**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools/test_balance_drive_v2_static.ps1
```

Expected output:

```text
balance drive v2 static checks passed
```

- [ ] **Step 3: Verify the fast balance limits were not changed by the transferred history**

Run:

```powershell
$limitDiff = git diff da1ebe5..4f057a1a -- project/code/app_config.h |
    Select-String 'APP_CHASSIS_FAST_FORWARD_RPM_LIMIT|APP_BALANCE_RPM_LIMIT'
if($null -ne $limitDiff) {
    $limitDiff | Write-Host
    throw 'Fast balance limits changed in the transfer range.'
}
Write-Host 'fast balance limits unchanged'
```

Expected output:

```text
fast balance limits unchanged
```

---

### Task 5: Attach documentation and finalize local verification

**Files:**

- Add through cherry-pick: `docs/superpowers/specs/2026-07-18-balance-servo-transfer-design.md`.
- Add through cherry-pick: `docs/superpowers/plans/2026-07-18-balance-servo-transfer.md`.

**Interfaces:**

- Consumes: validated target at `4f057a1a`; documentation commits on `codex/balance-servo-transfer-design`.
- Produces: a documented local target branch with the full clean servo/leg ancestry and no source edits after validation.

- [ ] **Step 1: Attach the approved design and implementation plan**

Run:

```powershell
git cherry-pick a68c304 codex/balance-servo-transfer-design
```

Expected: two clean documentation cherry-picks; the first adds the design and the second adds this plan. No source file is changed.

- [ ] **Step 2: Verify documentation-only commits and clean tracked state**

Run:

```powershell
git show --stat --oneline HEAD~1
git show --stat --oneline HEAD
git diff --check
git status --short --branch
```

Expected: the last two commits touch only the two paths under `docs/superpowers`; `git diff --check` emits nothing; tracked state is clean. The known `.superpowers/`, `data/`, and `fast_tune_kit.zip` remain untracked and untouched.

- [ ] **Step 3: Run final ancestry assertions**

Run:

```powershell
git merge-base --is-ancestor 4f057a1a HEAD
if(0 -ne $LASTEXITCODE) {
    throw 'Final branch lost the clean servo/leg milestone.'
}
git merge-base --is-ancestor 5c216e4 HEAD
if(0 -eq $LASTEXITCODE) {
    throw 'Final branch contains the excluded cone-perception runtime tail.'
}
Write-Host 'final ancestry checks passed'
```

Expected output:

```text
final ancestry checks passed
```

- [ ] **Step 4: Record the remaining hardware gate**

Report exactly:

```text
IAR cyt4bb7_cm_7_0 build: NOT RUN - IarBuild.exe not available in this environment.
Hardware PWM/servo/fast-speed validation: NOT RUN - requires supported or wheel-off bench setup.
```

Do not push the branch. Hand the local result back to the user with the final target SHA, test results, excluded-tail assertion, untouched untracked artifacts, and the two explicit `NOT RUN` gates.
