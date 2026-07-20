# Low-race Assist Cross-fix Final Report

## Scope and baseline

- Worktree: `D:\smartcar\WheelLeggedCar_cyt4bb7_v1\.worktree\balance-fast-mode-spec`
- Branch: `codex/balance-fast-mode-spec`
- Reviewed baseline: `76f95d97ab1d59d3040b5e61fa9a13832e5fade4`
- Scope: close the final review's 1 Critical and 4 Important findings without
  changing validated level, default gains, LQR/tuning constants, motor hard
  limit, telemetry contract, or hardware acceptance status.
- Existing untracked `data/` captures and `fast_tune_kit.zip` were not modified
  or staged.

## Findings closed

### Final review follow-up: operational-session entry latch

The supervisor now records whether the current enabled session has actually
entered `BOOST`, `CRUISE_HOLD`, or `BRAKE`. Until that happens, the neutral
`low_pose_ready` entry gate remains mandatory; this also prevents an
uncompleted pre-operational `RECENTER` from being cancelled into an entry-gate
bypass. Once operational motion has begun, a normal BRA1 speed drop below
230 RPM may enter `LOW_RACE` and command `u_request=0` for as many scheduler
cycles as needed without treating the expected off-neutral/unsettled return
trajectory as a new entry.

The latch is cleared only by `control_race_assist_init()` (the explicit fault
reset path) or by reaching the real `DISABLED` output after recenter/disable
completion. `FAULT_HOLD` does not clear it, and the existing latched-fault
checks continue to run before any entry decision.

The pure-supervisor numeric harness and the real scheduler combination harness
both cover the final-review sequence:

1. a non-neutral fresh `DISABLED` session fails closed;
2. neutral entry reaches `ARMED -> BOOST`, with nonzero `u` and an observed
   unsettled/off-neutral servo trajectory;
3. BRA1 remains selected while measured speed falls to 220 RPM;
4. real 1/5/10 ms scheduler cycles remain fault-free with `u_request=0` and
   monotonic `u_actual -> 0`;
5. neutral plus settled remains in `LOW_RACE`;
6. BRA0 completes `RECENTER -> DISABLED`, after which a new non-neutral session
   is rejected again.

### A. Entry-only low-pose readiness

`control_race_assist_update()` now requires `low_pose_ready` until the current
session has actually entered `BOOST`, `CRUISE_HOLD`, or `BRAKE`. That
operational-session latch remains set while an armed session returns through
`LOW_RACE`, so normal off-neutral recentering does not re-run the entry gate.
It is cleared by initialization/fault reset or after a completed
`RECENTER -> DISABLED`; a pre-operational RECENTER restart therefore still
requires the neutral, settled entry pose. During an operational session,
leaving the neutral pose and an unsettled open-loop servo planner are expected
motion, not a `LEG_NOT_READY` fault.

The always-on gates remain unchanged: finite input, fresh wheel/IMU feedback,
pitch and pitch-rate abort limits, and leg/path fault still fail closed.

The production-combination scheduler harness observes the actual sequence
neutral -> armed -> boost, then proves a nonzero `u`, off-neutral Cartesian
target, and `servo_settled=0` coexist without a race fault. It also reaches the
positive endpoint, an explicit zero endpoint, and the negative endpoint.

### B. Cached path preflight and default-off WCET hook

The full dual-leg, positive/negative 21-sample path preflight is cached after a
successful run. The cache key contains:

- `dx_mm` and `dy_mm` (the active path/profile geometry); and
- the persisted left/right alpha/beta branch identity.

The cache is invalidated on controller init, first race entry, generic leg
fault, or race fault hold. A failed preflight is never cached. Repeated 5 ms
requests with an unchanged profile and branch identity reuse the validated
result.

The real IK call-count harness showed the defect before the fix:

```text
steady request repeated path preflight: solve count 12090 -> 13770
```

After the fix, twenty identical requests add zero IK solve calls. Runtime
trajectory solves remain active at the 10 ms leg period.

`APP_RACE_ASSIST_PREFLIGHT_WCET_ENABLE` defaults to `0U`. At zero it removes
all DWT reads and WCET counters at preprocessing time. When set to `1`, CM7_0
enables DWT `CYCCNT` and publishes debugger-visible count, last-cycle, and
maximum-cycle symbols around uncached preflights only. The enabled code path
was compiled by the host static harness and the normal default-off firmware
was compiled by IAR.

**on-target WCET: NOT RUN.** No physical CM7_0/DWT capture was performed, so
there is no claimed 5 ms WCET acceptance.

### C. Runtime IK fault hold and controlled deceleration

The leg diagnostic now separates:

- `ik_valid`: the new target solved this cycle; and
- `held_command_valid`: the frozen last command was previously validated,
  remains finite, and remains inside all configured servo limits.

On a runtime race IK solve failure, the controller keeps the last four valid
servo commands, marks the failed new target invalid, invalidates the path/cache,
and enters `LEG_MOTION_RACE_FAULT_HOLD`. Balance remains active only when
`held_command_valid`, servo output, and drive permission are all true.

The new harness compiles and executes the real cooperative scheduler order:

```text
leg (10 ms) -> chassis (5 ms) -> balance (5 ms) -> motor (1 ms)
```

It injects one production IK solve failure after the negative endpoint and
asserts:

- the four servo commands freeze exactly;
- `held_command_valid=1` and `ik_valid=0`;
- forward intent decreases by exactly `0.3 RPM` each 5 ms update, matching the
  unchanged `60 RPM/s` ramp;
- balance continues issuing RPM commands and does not call a motor-stop API;
- an invalid held-command snapshot stops immediately;
- an IMU fault stops immediately; and
- `APP_STATE_FAULT` stops immediately.

Generic leg fault, disabled servo output, stale/unhealthy IMU or wheel
feedback, disabled chassis output, non-finite terms, and pitch safety limits
retain their immediate-stop behavior.

### D. BRA0 recenter and fault turn authority

`control_chassis_update()` now consumes the supervisor result after the same
cycle update:

- `RECENTER` publishes and enforces the supervisor's `200 RPM` forward and fast
  caps instead of falling back to the legacy `220 RPM` fast cap;
- `FAULT_HOLD` publishes and enforces `0 RPM` forward caps;
- `RECENTER` retains speed-derived turn derating until safe; and
- `FAULT_HOLD` commands turn intent toward zero, so a high-speed fault cannot
  increase steering authority.

The chassis harness sustains `C,250`, sends BRA0, checks target and both caps
are at most 200 RPM, checks turn permission does not increase at 350 RPM, then
sets measured speed below 200 and `u_actual=0` and observes `DISABLED`. A
separate high-speed request-rejection fault check confirms turn permission
does not increase.

### E. Gate 0 wording

The hardware procedure now says to reject **any large or unexpected command
jump** between manual endpoints. The previous inverse phrase, `no large command
jump`, was removed. The section-scoped static assertion requires the corrected
meaning.

## TDD evidence

Tests were written and observed failing before the corresponding production or
documentation change:

1. Entry gate RED:
   `expected state 4, got 7` and
   `operational assist incorrectly rechecked the neutral-pose entry gate`.
2. Chassis combination RED:
   `race setup faulted after the leg left neutral`.
3. BRA0 RED:
   `state 6 target 250.000 caps 250.000 250.000`.
4. Preflight cache RED:
   `steady request repeated path preflight: solve count 12090 -> 13770`.
5. Held-command RED: the scheduler harness failed compilation because
   `leg_diag_struct` had no `held_command_valid` member.
6. Gate 0 wording RED:
   `Gate 0 must reject any large or unexpected manual command jump.`
7. WCET handoff RED:
   `The DWT preflight WCET status must remain explicit until measured on CM7_0.`
8. Operational-session numeric RED:
   `operational recenter cycle: expected state 1, got 7 fault 2 u 0.475`.
9. Real scheduler sequence RED:
   `operational recenter failed: fault 2 request 1.000 |u| 1.0000->1.0000`.
10. Pre-operational re-entry RED:
    `pre-operational RECENTER bypassed the entry gate`.

Each focused test was rerun to GREEN before the full suite.

## Fresh software verification

The previous 19-script suite plus the new operational-session numeric harness
all exited zero: **20/20 passed**.

1. `tools/test_race_assist_numeric.ps1`
2. `tools/test_race_assist_session_latch.ps1`
3. `tools/test_low_race_leg_assist_static.ps1`
4. `tools/test_leg_transition_numeric.ps1`
5. `tools/test_leg_ik_zero_calibration_static.ps1`
6. `tools/test_leg_physical_ik_static.ps1`
7. `tools/test_servo_motion_numeric.ps1`
8. `tools/test_servo_300hz_integration_static.ps1`
9. `tools/test_balance_drive_v1_static.ps1`
10. `tools/test_balance_drive_v2_static.ps1`
11. `tools/test_tune_drive_loops_static.ps1`
12. `tools/test_collect_balance_data.ps1`
13. `tools/test_timing_noise_regressions.ps1`
14. `tools/test_ik_height_control_static.ps1`
15. `tools/test_leg_coordinate_contract_static.ps1`
16. `tools/test_control_chassis_race_assist.ps1`
17. `tools/test_host_command_race_assist.ps1`
18. `tools/test_collect_bldc_diagnostics.ps1`
19. `tools/test_calib_ik_servo.ps1`
20. `tools/test_race_assist_scheduler_integration.ps1`

The runner printed:

```text
ALL_RACE_SESSION_LATCH_TESTS_PASSED=20/20
```

`git diff --check` exited zero. Its only output was Git's LF/CRLF working-copy
conversion notices.

## Fresh three-core IAR build

The installed executable identified itself as IAR Command Line Build Utility
`V9.2.2.10770`; this report does not claim IAR EW 9.40.1. Full Debug builds ran
in release order with `-build Debug -log warnings`:

| Project | Errors | Warnings | Result |
| --- | ---: | ---: | --- |
| `cyt4bb7_cm_7_1` | 0 | 0 | Build succeeded |
| `cyt4bb7_cm_7_0` | 0 | 3 | Build succeeded |
| `cyt4bb7_cm_0_plus` | 0 | 0 | Build succeeded |

The runner printed `ALL_THREE_IAR_BUILDS_PASSED`. The three CM7_0
`Warning[Pe550]` records are the existing `control_leg_legacy_stance_cmd`,
`control_leg_pitch_cmd`, and `control_leg_roll_cmd` set-but-unused warnings;
none was introduced by this cross-fix.

## Preserved safety/configuration boundaries

- `APP_RACE_ASSIST_MAX_VALIDATED_LEVEL = 1U`
- race acceleration/error/hold defaults remain `0.0f`
- chassis speed `Ki = 0.0f`
- fast speed feed-forward remains `0.0f`
- race pitch-offset limit remains `7.0f`
- LQR/control gains were not changed
- motor hard target remains `1000.0f`
- telemetry remains exactly 72 floats at 20 ms

## Hardware status and next gate

- Gate 0 motor-disabled supported-path test: **NOT RUN**
- on-target CM7_0 preflight WCET: **NOT RUN**
- Level 1 250 RPM A/B: **NOT RUN — level 1 pending**
- Levels 2--4: **NOT RUN — compile-gated**
- 400 RPM measured acceptance: **NOT RUN**

No firmware was flashed, no wheel or servo motion was commanded, and no
hardware acceptance is inferred from host tests or IAR builds. The next
authorized physical action remains the documented Gate 0 with the wheel-motor
power stage isolated.
