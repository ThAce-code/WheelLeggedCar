# Low-race leg-assist staged hardware gate

## Status and authority

This is an execution procedure, not a hardware-acceptance claim. Software
checks, an IAR build, and a downloaded image do not prove mechanical clearance,
leg direction, balance stability, or speed. At publication the board status is
**level 1 pending**; levels 2--4 are also pending and must not be enabled by
default.

The initial compiled gate is `APP_RACE_ASSIST_MAX_VALIDATED_LEVEL = 1`.
`BRA,2`, `BRA,3`, and `BRA,4` are valid UART spellings but must reject while
that gate remains 1. A successful `B,3` selects fast balance; it does not arm
race assist. Only an explicit `BRA,1` can arm the initially permitted level.

This procedure is straight-line only. Keep turn request zero in every gate.
Do not tune wheel PID, balance gains, speed-loop gains, or servo rate while
running a speed-level gate.

## Fixed software facts and limits

- The four-servo PWM path is 300 Hz. `servo_settled=1` requires the command
  planner to remain within its configured `0.2 deg` error for `100 ms`; it is
  not measured joint or wheel-centre feedback.
- `LXY` and all telemetry X/Y values are open-loop **command estimate** values
  derived from PWM commands. They cannot prove physical pose, clearance, or
  right-leg symmetry.
- The low-race zero is `(-18.83,25.08) mm` in `BODY_WHEEL`; +X is forward and
  +Y is downward. The assist path is `x=-18.83-2*u`,
  `y=25.08+2*abs(u)` mm for the current level-1 profile.
- The speed loop starts with Ki = 0 and the fast speed feed-forward = 0.
  Race gains also reset to `BRG,0,0,0`; they are runtime-only and must not be
  treated as calibrated values.
- The 7 deg virtual pitch-offset cap is a race-assist safety limit, not
  permission to add pitch. The separate motor-layer 1000 RPM target limit is a
  hard ceiling only; it is not an approved race speed. The staged chassis and
  balance limits below remain authoritative.
- The left leg has measured-calibration provenance. The right leg is still a
  mirror assumption until independently measured. Servo PWM is open loop, so
  neither a valid IK solution nor a settled command authorizes a dynamic test.

## Firmware package, programming order, and recovery

1. Record the Git SHA and build all three workspace projects before touching
   the board: `cyt4bb7_cm_7_1`, `cyt4bb7_cm_7_0`, then
   `cyt4bb7_cm_0_plus`. Task-specific code is CM7_0, but the board must carry
   a matched three-core image set.
2. With the chassis supported, motor power physically disconnected, an operator
   on the physical power cutoff, and no wheel touching the ground, use IAR
   download-only in the same order: CM7_1, CM7_0, then CM0+. Do not run the
   target between partial downloads.
3. Power-cycle only after all three downloads report verification success.
   Start the first session with `STOP`; then verify a 72-float VOFA frame and
   that the leg is stationary before sending any pose command.
4. If a download, boot, or first telemetry check fails, keep wheel power
   disconnected. Restore the last known-good matched CM7_1 -> CM7_0 -> CM0+
   image set in that same order, power-cycle, send `STOP`, and verify the
   baseline telemetry before investigating further. Do not mix a new CM7_0
   image with an unknown CM0+/CM7_1 image.

The programming order is an operational release rule for this board image; it
is not evidence that a core has started correctly. Record the actual IAR
download result and any debugger-specific exception in the test log.

### CM7_0 path-preflight WCET diagnostic

The full signed IK path is preflighted only when the race profile first enters,
`dx_mm`/`dy_mm` changes, the persisted branch identity changes, or the cache is
invalidated by reset/fault. The production default is
`APP_RACE_ASSIST_PREFLIGHT_WCET_ENABLE=0`, for which the DWT reads and counters
are compiled out. For a supported, motor-disabled timing build only, set
`APP_RACE_ASSIST_PREFLIGHT_WCET_ENABLE=1`, rebuild CM7_0, and watch these symbols
in IAR while deliberately causing uncached preflights:

- `control_leg_race_preflight_count`
- `control_leg_race_preflight_last_cycles`
- `control_leg_race_preflight_max_cycles`

The hook enables CM7_0 DWT `CYCCNT` and records cycle deltas around only the
uncached dual-leg signed-path sweep. Convert the maximum cycle count with the
actual CM7_0 clock used by that image and compare it with the 5 ms chassis
budget; do not infer WCET from host GCC time or from a cached request.

Current measurement status: **on-target WCET: NOT RUN**. No board/DWT trace was
captured for this software review, so this section is an executable measurement
procedure, not timing acceptance evidence.

## VOFA+ setup and required 72-channel trace

Use the debug UART's current board mapping (normally COM6 at 460800, 8N1) and
JustFloat framing. The frame is exactly 72 floats plus the four-byte tail:
292 bytes at a 20 ms period. Reject a legacy 55-float decoder or a shifted
frame before interpreting the data.

| Channels | Meaning |
| --- | --- |
| 0--11 | Time, balance mode, roll/pitch/yaw, pitch rate, balance command, wheel-online state, wheel RPM, and duties. |
| 12--17 | Leg mode, legacy stance fields, pose-status flags, and leg output enable. |
| 18--32 | Four actual PWM command angles, four planner targets, four filtered commands, max error, `servo_settled`, and S7 progress. |
| 33--39 | Left/right BODY_WHEEL command X/Y, common IK margin, leg motion state, and leg fault reason. |
| 40--54 | Drive permission, servo trajectory state, telemetry/scheduler/servo/IMU integrity counters, IMU age, and gyro Y rate. |
| 55--58 | Race enable, level, state, and race fault reason. |
| 59--65 | `u_request`, `u_actual`, requested acceleration, forward target, ramped target, measured wheel speed, and speed error. |
| 66--71 | Pitch setpoint, active balance output cap, turn scale, left/right IK margins, and `ik_branch_flags`. |

For every gate save the raw 72-channel CSV and record its filename, firmware
SHA, frame/drop counters, and command timestamps. The minimum live safety
view is: 3/5/6/7/8/9/10/11, 31, 33--39, 41, 53, and 55--71.

## Universal prerequisites and stop rules

Before every gate, require all of the following:

1. A straight, clear lane; no people in the travel path; a tether or support
   operator; charged battery; and a physical power cutoff held by a separate
   operator.
2. The chassis is supported for the bench gate. Motor power is physically
   disconnected until that gate passes. The ground gates begin only after the
   chassis has been lowered safely, the lane is clear, and the cutoff operator
   confirms wheel power connection.
3. A fresh trace has IMU age channel 53 no more than 15 ms and leg fault
   channel 39 equal to 0. Gate 1 and later automatic race-path gates also
   require channel 7 = 1, race fault channel 58 = 0, both margins (69/70) at
   least `0.02`, and a saved baseline branch value at 71. Gate 0 instead uses
   its separate manual LXY channel criteria below.
4. Stop for mechanical contact, linkage/end-stop approach, chatter,
   overheating, supply sag, unexpected motion, or a non-finite value. No
   branch-bit change is required only for Gate 1 and later automatic race-path
   gates; Gate 0 has no branch-continuity hardware acceptance.

Stop immediately for any exception. First use the physical cutoff if motion is
unsafe, then send `STOP`, `BRA,0`, and `B,0` when communication is safe. A
race `FAULT_HOLD` removes race authority and ramps the forward target down, but
it is not a substitute for the physical emergency stop. `BRA,0` disarms; it
does not clear a latched race fault. After any race fault, preserve the CSV,
disconnect wheel power, power-cycle, send `STOP`, and repeat only the prior
passed gate after root-cause review.

The automatic arm window is `|pitch| <= 10 deg` and
`|pitch_rate| <= 100 deg/s`. A run is not promotable outside that window.
`|pitch| > 15 deg` or `|pitch_rate| > 180 deg/s` is a reject/stop condition and
causes the supervisor pitch fault. A margin below `0.02`, changed branch bits,
leg fault, race fault, stale/offline feedback, or persistent saturation also
rejects the run.

## Gate 0: motor-disabled stopped-only manual endpoint gate

Keep the chassis supported and motor power physically disconnected. This means
isolate only the wheel-motor power stage / motor-driver output; BLDC logic,
UART, and speed feedback remain powered, so channel 7 remains 1. No wheel may
touch the ground. `LIKREF` and `LXY` deliberately remain stopped-only manual
commands; do not replace them with `LJ`, manual servo angles, a direct-u UART
command, or a test image.

If the board cannot isolate the wheel-motor power stage while retaining
feedback, use a verified firmware/driver output-disabled bench mode. It must
prove motor torque output is disabled before any LXY command and record the
exact isolation limitation, firmware SHA, and output-disable evidence. Only
Gate 0 may waive channel 7 when that fallback itself makes feedback offline;
the Gate 1 automatic race path never has that waiver.

Send exactly one command at a time. First send `STOP`, then `LIKREF`, and wait
until `servo_settled=1`. Next run the following manual LXY endpoint/mechanical
gate in order, waiting for settle and inspecting the linkage after **every**
point:

```text
LXY,-18.83,25.08
LXY,-20.83,27.08
LXY,-18.83,25.08
LXY,-16.83,27.08
LXY,-18.83,25.08
```

At each LXY point require channel 16 to show IK valid and channel 37 common IK
margin `>=0.02`. Record channels 22--25 planner target values and channels
18--21 PWM command values before waiting for channel 31 `servo_settled=1`.
Reject any no-settle result, invalid IK, common-margin failure, leg fault, any
large or unexpected command jump between adjacent manual endpoints, or mechanical
interference. Record the command X/Y and visual mechanical result. The three
positions are neutral `(-18.83,25.08)`, rearward accel endpoint
`(-20.83,27.08)`, and forward brake endpoint `(-16.83,27.08)`; they remain
open-loop command estimates, not measured pose.

End the bench gate with `STOP`; do not arm assist during Gate 0. Branch
continuity is proved by software regression, not by this stopped-only manual
endpoint check. Gate 0 proves only manual endpoint reachability, direction,
and clearance. It does not test the automatic race supervisor, dynamic
`u_request/u_actual`, or the automatic race-path S7 execution. Gate 1 is the
first permitted low-speed automatic-supervisor validation.

## Gate 1: 250 RPM ground A/B comparison

Only after Gate 0 passes, set the vehicle on a clear straight lane with the
tether/support operator and immediate physical cutoff. Start each A or B run
from stopped and perform the complete low-race re-entry. The A baseline is
armed with zero gains; it is not an unarmed `BRA,0` comparison:

```text
STOP -> LIKREF -> settled -> LXY,-18.83,25.08 -> settled -> BRA,0 -> BRG,0,0,0 -> BRA,1 -> B,3
```

For B, repeat that complete sequence but set
`BRG,0.005,0.005,0.15` in place of `BRG,0,0,0`, still before `BRA,1`. Verify
the low-race point has settled after both `LIKREF` and `LXY`, then verify the
72-channel trace before requesting wheel motion.

`APP_CHASSIS_CMD_TIMEOUT_MS=500`: a single `C,250,0` expires after 500 ms and
the normal 60 RPM/s ramp can theoretically reach only about 30 RPM. A single
command is prohibited for acceptance. Send a `C,250,0` 100--200 ms heartbeat
throughout acceleration and the 230--250 RPM dwell, which must be sampled for
at least 1 second. For the stop, send a `C,0,0` 100--200 ms heartbeat until
measured speed is below 10 RPM; only then send `BRA,0` and `STOP`.

The commands are therefore:

```text
A: complete re-entry with BRG,0,0,0; C,250,0 heartbeat; 230--250 RPM dwell; C,0,0 heartbeat; <10 RPM; BRA,0; STOP
B: complete re-entry with BRG,0.005,0.005,0.15; C,250,0 heartbeat; 230--250 RPM dwell; C,0,0 heartbeat; <10 RPM; BRA,0; STOP
```

Do not add a gain, speed, leg-travel, balance, PID, or turning change to either
A or B.

For both runs record: time spent in 230--250 RPM, peak absolute pitch, peak
absolute pitch rate, all periods at or above 95% of the active output cap,
`u_request/u_actual`, command X/Y, branch flags, both margins, feedback/IMU
freshness, race/leg fault fields, and stop time from `C,0,0` to below 10 RPM.
For this automatic race path, margins 69/70 and channel-71 branch bits are
mandatory per-side path evidence; Gate 0's manual endpoint criteria do not
substitute for them.

Level-1 acceptance is all of the following:

- Measured speed remains in 230--250 RPM for at least 1 second; no sample may
  exceed 260 RPM.
- During that dwell, `|pitch| <= 10 deg`, `|pitch_rate| <= 100 deg/s`, channel
  7 remains 1, channel 53 remains `<=15 ms`, channels 39 and 58 remain 0, and
  margins 69/70 remain `>=0.02` with unchanged channel-71 branch bits.
- The 300 RPM balance cap (channel 67) is not persistently saturated: no
  contiguous `>=285 RPM` interval of 250 ms or more, and less than 5% of the
  active-run samples meet that occupancy threshold.
- When B requests a nonzero assist, `u_actual` follows its sign and settles
  within `|u_request-u_actual| <= 0.10`; its command X/Y follows the path in
  Gate 0. Positive request must use the rearward path and negative request the
  forward-brake path.
- B improves acceleration time to 230 RPM or controlled-braking stop time
  versus A without a larger unsafe pitch excursion. The stop must reach below
  10 RPM within 5 seconds.

Reject and remain at level 1 pending if any acceptance item fails. A hardware
fault, physical cutoff, pitch/rate abort, lost feedback, changed branch, margin
failure, or persistent saturation blocks all later levels; do not retry at a
higher speed.

## Promotion gates and numerical acceptance

Keep `dx_mm=2` and `dy_mm=2` for the first acceptance of a new speed level.
Do not expand `dx_mm/dy_mm` in the same run as a speed-level increase. Any
travel change needs its own wheel-power-disconnected Gate 0 replay, A/B result,
fresh review, and a separate build before it is combined with higher speed.

Changing `APP_RACE_ASSIST_MAX_VALIDATED_LEVEL` from 1 to 2, 2 to 3, or 3 to 4
requires the previous level's recorded acceptance, an explicit source review,
a fresh three-core build, and a repeat of the supported path gate for the
candidate image. It is never a runtime tuning action. Until that build is
installed, levels 2--4 are intentionally unavailable.

Every candidate level is straight-only and uses a 100--200 ms
`C,<target>,0` heartbeat rather than a single command. It must retain all
universal accept/reject gates and compare a zero-gain A run with the same
conservative-gain B run before promotion. The following numeric bands are
mandatory minimum evidence:

| Level | Forward / balance cap | Measured-speed acceptance | Stop-time acceptance | Output saturation reject |
| --- | --- | --- | --- | --- |
| 1 | 250 / 300 RPM | 230--250 RPM for >=1 s; no sample >260 RPM | <10 RPM within <=5 s | `>=285 RPM` for >=250 ms or >=5% occupancy |
| 2 | 300 / 350 RPM | 290--310 RPM for >=2 s; no sample >320 RPM | <10 RPM within <=6 s | `>=332.5 RPM` for >=250 ms or >=5% occupancy |
| 3 | 350 / 410 RPM | 340--360 RPM for >=2 s; no sample >370 RPM | <10 RPM within <=7 s | `>=389.5 RPM` for >=250 ms or >=5% occupancy |
| 4 | 400 / 460 RPM | 390--410 RPM for at least three seconds; no sample >420 RPM | <10 RPM within <=8 s | `>=437 RPM` for >=250 ms or >=5% occupancy |

At every level, a measured `|pitch|` above 10 deg or `|pitch_rate|` above
100 deg/s prevents acceptance; above 15 deg or 180 deg/s is an immediate
reject/fault threshold. Fresh feedback, no persistent balance saturation, no
IK/leg/race fault, unchanged branch identity, margins `>=0.02`, correct signed
assist direction, and controllable straight-line behavior are also mandatory.
At about 350 RPM channel 68 should show the programmed turn derating; at 400
RPM it reaches zero. This is intentional: no turning test belongs in this
procedure.

The final level-4 statement is deliberately narrow: a 400 RPM claim is valid
only after the measured 390--410 RPM dwell, all listed acceptance fields, and
the recorded three-second trace pass on the vehicle. It does not establish
thermal endurance, high-speed turning, or a larger leg workspace.

## Required record and current hardware result

For each attempted gate, record date, firmware SHA, three-core build/download
results, battery state, support/tether/cutoff operators, commands, raw CSV,
minimum left/right margin, baseline/final branch flags, all faults, measured
speed dwell, peak pitch/rate, cap occupancy, `u_request/u_actual`, command
X/Y, and stop time. Mark each row `PASS`, `REJECT`, or `NOT RUN`; a missing
measurement is `NOT RUN`, never pass by inference.

| Gate | Required evidence | Current board result |
| --- | --- | --- |
| 0 | Supported motor-disabled five-manual-LXY endpoint trace | NOT RUN |
| 1 | 250 RPM A/B trace and acceptance fields | NOT RUN — level 1 pending |
| 2 | Recorded level-1 acceptance plus 300 RPM trace | NOT RUN — compile-gated |
| 3 | Recorded level-2 acceptance plus 350 RPM trace | NOT RUN — compile-gated |
| 4 | Recorded level-3 acceptance plus 400 RPM three-second trace | NOT RUN — compile-gated |

No part of this document authorizes changing defaults, enabling levels 2--4,
or describing the vehicle as 400 RPM accepted before the corresponding board
record exists.
