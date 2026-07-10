# Fast Height Profile Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a bench-only `LHF,<height_mm>` command that moves the global empirical leg height between 45 mm and 65 mm in exactly 500 ms without trajectory overshoot.

**Architecture:** Keep the existing `LH` jerk-limited controller unchanged as the smooth mode. Add a separate fast-height leg mode that evaluates a time-parameterized quintic blend `s(u)=10u^3-15u^4+6u^5` from the current reference height to the requested target. The profile starts and finishes with zero reference velocity and acceleration, maps through the existing empirical height-to-servo conversion, and keeps all safety/output gates unchanged.

**Tech Stack:** Embedded C on CYT4BB, existing host UART parser, PowerShell static/numeric checks, IAR hardware build.

## Global Constraints

- `LHF` accepts only the existing empirical interval 45.0–65.0 mm.
- The duration is exactly `500U` ms and is stored in `leg_height_profile_struct`.
- `LH`, `LIK`, `LJ`, balance, roll, and left/right independent control behavior remain unchanged.
- The profile is global-height only and must not drive the chassis or bypass PWM slew.
- Raw telemetry CSV files are not committed.

---

### Task 1: Specify and verify the 500 ms profile numerically

**Files:**
- Modify: `tools/test_leg_transition_numeric.ps1`
- Modify: `project/code/leg_config.h`
- Modify: `project/code/leg_config.c`

**Interfaces:**
- Consumes: `leg_height_profile_struct`
- Produces: `fast_height_transition_ms` set to `500U` and an executable numeric assertion for a 45→65 mm move.

- [ ] **Step 1: Write the failing profile assertions**

Add a PowerShell function that samples the quintic blend every 6 ms and asserts all values stay in `[45.0,65.0]`, reach exactly 65.0 mm at 500 ms, and have nonnegative velocity for a rising move. Assert the configuration value is `500.0`.

- [ ] **Step 2: Run the numeric test to verify it fails**

Run: `powershell -ExecutionPolicy Bypass -File .\tools\test_leg_transition_numeric.ps1`

Expected: FAIL because `fast_height_transition_ms` and the fast-profile assertion are absent.

- [ ] **Step 3: Add only the duration configuration**

Append `uint32 fast_height_transition_ms;` after `height_settle_ms` in `leg_height_profile_struct` and initialize it as `.fast_height_transition_ms = 500U` in `leg_config_default`.

- [ ] **Step 4: Re-run the numeric test**

Run: `powershell -ExecutionPolicy Bypass -File .\tools\test_leg_transition_numeric.ps1`

Expected: PASS for the configuration and standalone quintic math assertion.

### Task 2: Implement the fast global-height mode and UART command

**Files:**
- Modify: `project/code/control_leg.h`
- Modify: `project/code/control_leg.c`
- Modify: `project/code/host_command.c`
- Modify: `tools/test_ik_height_control_static.ps1`

**Interfaces:**
- Consumes: `control_leg_set_fast_height(float height_mm, uint32 now_ms)` and `fast_height_transition_ms`.
- Produces: `LEG_MODE_FAST_HEIGHT` and host command `LHF,<height_mm>`.

- [ ] **Step 1: Write failing static assertions**

Assert that the public API, mode, `LHF` parser, quintic blend, duration use, empirical mapping, and normal PWM command path exist. Assert no call to `actuator_servo_apply_immediate` is associated with the fast mode.

- [ ] **Step 2: Run static checks to verify failure**

Run: `powershell -ExecutionPolicy Bypass -File .\tools\test_ik_height_control_static.ps1`

Expected: FAIL because no fast-height API/mode/parser exists.

- [ ] **Step 3: Add the minimal fast-mode state and update branch**

Store start height and start timestamp. In `LEG_MODE_FAST_HEIGHT`, compute `u=clamp((now_ms-start_ms)/duration,0,1)`, evaluate the quintic blend, set `control_leg_height_ref_mm`, derive the diagnostic rate, map the height with `control_leg_apply_empirical_height`, and transition to `LEG_MOTION_STABLE` when `u==1`. Reuse existing servo limit checks, output-enable gates, and `actuator_servo_set_cmd` path.

- [ ] **Step 4: Add `LHF,<height_mm>` parsing**

Parse the command before the existing `LH` parser, call `control_leg_set_fast_height`, and retain the existing command-error behavior for invalid ranges or a latched fault.

- [ ] **Step 5: Re-run checks**

Run: `powershell -ExecutionPolicy Bypass -File .\tools\test_ik_height_control_static.ps1`

Expected: PASS.

### Task 3: Document and verify the bench test

**Files:**
- Modify: `docs/leg-height-phase1-hardware-test.md`

**Interfaces:**
- Consumes: `LHF,45` and `LHF,65`.
- Produces: a safe, repeatable capture command and pass/fail criteria.

- [ ] **Step 1: Document the command sequence**

Add an elevated-wheel test: `0:STOP;2:LHF,65;4:LHF,45;6:STOP`. State that this is global-height only, wheels must be stopped, and the test is a failure if `leg_fault_reason` is nonzero, `leg_height_ref_mm` leaves 45–65 mm, or either direction overshoots its target.

- [ ] **Step 2: Run the full automated suite**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\test_ik_height_control_static.ps1
powershell -ExecutionPolicy Bypass -File .\tools\test_leg_transition_numeric.ps1
powershell -ExecutionPolicy Bypass -File .\tools\test_fit_leg_ik_calibration.ps1
powershell -ExecutionPolicy Bypass -File .\tools\test_leg_first_height_frame_static.ps1
powershell -ExecutionPolicy Bypass -File .\tools\test_servo_pwm_resolution_static.ps1
git diff --check
```

- [ ] **Step 3: Build and bench verify**

Build the affected IAR core, flash the target, run the documented capture command, and inspect the returned CSV before allowing any moving-chassis test.
