# MT9V03X zero-frame diagnosis report

## Scope and safety

- Branch baseline: `e5ab22fe` (`Classify camera attach magic failures`).
- Objective: prove or falsify whether the missing reference CM0+ capture-service image caused Task 2 Hardware Gate 1 to initialize successfully but produce zero completed frames.
- Wheel motor power: operator-confirmed OFF.
- Camera wiring: operator-confirmed.
- Servo pose: operator-confirmed all 90 degrees.
- Prohibited throughout this diagnosis: motion, wheel, servo, and `LXY` commands.
- Task 3 is out of scope.

## Prior failing observation

Task 2 Gate 1 ran for 118.600 seconds. `producer_diag.init_state` remained `0x03` (`CAMERA_CAPTURE_INIT_OK`), while `producer_diag.frame_count`, `producer_diag.last_frame_ms`, `producer_diag.frame_valid`, `mt9v03x_finish_flag`, `mt9v03x_image[0][0]`, and `mt9v03x_image[60][94]` all remained zero. Gate 2 was correctly not run.

## Phase 1: read-only evidence

### Source-matched reference artifacts

The following authoritative reference files were inspected without modification:

- `D:\smartcar\CYT4BB7_Library\Example\Motherboard_Demo\E8_camera\E8_09_mt9v03x_uart_seekfree_assistant_cross_ram_m7_1_demo\iar\project_config\cyt4bb7_cm_7_0.ewx`
- `D:\smartcar\CYT4BB7_Library\Example\Motherboard_Demo\E9_seekfree_assistant\E9_01_seekfree_assistant_mt9v03x_demo\iar\project_config\cyt4bb7_cm_7_0.ewx`

Both files are exactly 56,964 bytes and have SHA-256:

```text
508CD93B33730384804E55794C3A11819E904740B3EC60519318B989DCF6A299
```

The corresponding E8_09 and E9_01 CM7_0 `.ewd` files are also byte-identical: both are 58,605 bytes with SHA-256 `BE5CA10C17603DD9728EE3AC4A7102867CE3A0E7C33BB98E2E8FE2DC920E94D2`.

Their loader configuration is identical and explicit:

```text
OCImagesPath1 = $PROJ_DIR$\cyt4bb7_cm_7_0.ewx
OCImagesUse1  = 1
OCImagesPath2 = $PROJ_DIR$\Debug_m7_1\Exe\cyt4bb7_cm_7_1.hex
OCImagesUse2  = 1
OCDownloadExtraImage = 1
```

The current robot CM7_0 `.ewd` differs materially:

```text
OCImagesPath1 = $PROJ_DIR$\Debug_m7_1\Exe\cyt4bb7_cm_7_1.hex
OCImagesUse1  = 0
OCImagesPath2 = empty
OCImagesUse2  = 0
OCDownloadExtraImage = 1
```

Thus the current loader records a CM7_1 path but disables it and has no reference `.ewx` configured.

### Intel HEX address audit

The Intel HEX extended-address records were parsed into absolute flash ranges:

| Artifact | Payload bytes | Inclusive flash range | Start linear address |
| --- | ---: | --- | --- |
| E8_09 reference `.ewx` | 20,234 | `0x10000000-0x10004F09` | `0x10004C09` |
| E9_01 reference `.ewx` | 20,234 | `0x10000000-0x10004F09` | `0x10004C09` |
| Current generic CM0+ HEX | 8,137 | `0x10000000-0x10001FC8` | `0x10001E51` |
| Current CM7_0 HEX | 64,688 | `0x10080000-0x1008FCAF` | `0x1008F9FD` |
| Current CM7_1 HEX | 12,046 | `0x10280000-0x10282F0D` | `0x10282CE5` |

The reference `.ewx` payload is wholly inside the CM0+ code-flash reservation. It does not overlap current CM7_0 or CM7_1 code flash. It intentionally replaces the generic CM0+ payload because both begin at `0x10000000`; therefore the controlled test must not separately download the generic CM0+ project afterward.

### Camera-contract evidence

The reference `.ewx` contains all four required little-endian constants at these payload locations:

| Constant | Reference flash location |
| --- | --- |
| `0x28006BF0` (`mt9v03x_h_num`) | `0x10003758` |
| `0x28006BF2` (`mt9v03x_w_num`) | `0x10003754` |
| `0x28026024` (`mt9v03x_image_temp`) | `0x10003798` |
| `0x40581D80` (TCPWM59 register block) | `0x1000378C` |

The freshly built current generic CM0+ HEX contains none of these four constants. Its `main_cm0plus.c` performs only `SystemInit()`, starts CM7_0 and CM7_1, initializes the board LED, and idles forever. Neither its main source nor its project configuration registers an MT9V03X acquisition service.

This is not proof by itself that the reference image is the cause; it establishes a single discriminating hypothesis and the exact safe loader experiment needed to test it.

## Phase 1 checkpoint

**Hypothesis:** Task 2 Gate 1 produced no `camera_finish_callback` events because the tested workflow separately downloaded the generic CM0+ image, replacing the source-matched reference CM0+ capture service expected by the MT9V03X driver contract.

**Evidence-supported next action:** perform one controlled CM7_0-originated download using the unchanged reference `.ewx` as enabled extra image 1 and the current fresh CM7_1 HEX as enabled extra image 2; do not separately download generic CM0+. Observe the prescribed producer, finish-flag, and pixel variables for at least 60 seconds while moving a high-contrast target.

**Status at checkpoint:** Phase 1 complete; Phase 2 pending. No production file had been changed and no root-cause claim had been made.

## Phase 2: controlled hardware hypothesis test

The active project was CM7_0. Its temporary IAR download configuration was set to the authoritative E9 `.ewx` as enabled extra image 1 and the current worktree CM7_1 HEX (SHA-256 `65411A213D259094BB69D5EE7415C1C9A060411E0D8E414BE838B19FAE8D4217`) as enabled extra image 2, both with offset zero. No generic CM0+ download followed.

IAR reported `Download completed` at local `2026-07-14 04:18:45`. The target stopped at CM7_0 `main()` with all seven prescribed Live Watch values zero. The run began at `2026-07-14T11:19:26.823Z` and was frozen through **Debug > Break** at `2026-07-14T11:23:18.905Z`, a 232.082-second wall-clock interval.

| Diagnostic | Start | Frozen endpoint |
| --- | ---: | ---: |
| `producer_diag.init_state` | `0x00` | `0x03` |
| `producer_diag.frame_count` | 0 | 11,506 |
| `producer_diag.last_frame_ms` | 0 | 229,986 |
| `producer_diag.frame_valid` | 0 | 1 |
| `mt9v03x_finish_flag` | 0 | 0 |
| `mt9v03x_image[0][0]` | `0x00` | `0x2B` |
| `mt9v03x_image[60][94]` | `0x00` | `0x7D` |

The frame delta was 11,506, or approximately 49.58 frames per wall-clock second. Intermediate samples proved continuous production and pixel response:

```text
frame=813:   pixels=0x26 / 0x7C
frame=4,628: pixels=0x2F / 0x7C
frame=7,008: pixels=0x24 / 0x7C
frame=11,506 pixels=0x2B / 0x7D
```

The finish flag being zero at debugger sample points is expected because CM7_0 service clears it after accounting each completed frame; the continuously increasing `frame_count` and changing pixels distinguish this from the prior zero-callback run.

Frozen timing and safety observations:

- `producer_diag.last_copy_duration_us = 77`
- `producer_diag.max_copy_duration_us = 95`
- `app_safety_fault = 0`
- `actuator_motor_last_left_duty = 0`
- `actuator_motor_last_right_duty = 0`
- Wheel motor power remained operator-confirmed OFF.
- No motion, wheel, servo, or `LXY` command was issued.

### Causal conclusion

The controlled run changed one loader variable while retaining the same M7 firmware: it restored the source-matched reference CM0+ image and did not overwrite it with generic CM0+. The prior workflow produced zero frames for 118.600 seconds; the controlled workflow produced 11,506 frames with changing pixels. Therefore the zero-frame root cause is confirmed: **the separately downloaded generic CM0+ image replaced the reference CM0+ MT9V03X capture service required by the fixed-address/TCPWM59 camera contract.**

## Phase 3: formalization

- Imported unchanged image: `project/iar/project_config/mt9v03x_cm0plus_capture_service.ewx`
- Imported image SHA-256: `508CD93B33730384804E55794C3A11819E904740B3EC60519318B989DCF6A299`
- Added provenance and required launch workflow: `docs/mt9v03x-capture-service-provenance.md`
- Formal CM7_0 extra image 1: repository capture-service image, enabled, offset zero.
- Formal CM7_0 extra image 2: current CM7_1 HEX, enabled, offset zero.
- Added `tools/test_camera_capture_service_static.ps1` to verify the exact hash, Intel HEX checksum/payload/range/entry point, four fixed camera constants, provenance, and loader configuration.

The static test was first observed RED for the missing repository image, missing provenance, and non-repository loader paths. After the minimal import/configuration/documentation change it passed GREEN.

Fresh builds, regression checks, formal Hardware Gate 1, and Hardware Gate 2 are recorded below as they are completed. Task 3 remains out of scope.

### Formal build and regression verification

The shared camera control ABI was advanced to version 2 so that CM7_1 can publish a debugger-visible observation into the existing non-cacheable control block after a successful release. The commit marker is `consumer_last_sequence`: age, two samples, and validity are written first, followed by a DMB, the sequence write, and a final DMB. The control block remains exactly 256 bytes, the complete shared layout remains exactly 8192 bytes, and compile-time checks fix the sequence and age offsets at 172 and 176.

The mirror tests were observed RED before implementation: the static contract reported seven missing version/layout/API/order/call-site requirements and the host test could not compile against the absent fields/API. After the minimal implementation, both `intercore_camera_handoff_test` and the camera static contract passed GREEN. A fresh IAR 9.40.1 clean/build then passed for all three cores:

| Core | Result | Fresh map timestamp (local) |
| --- | --- | --- |
| CM0+ | 0 errors, 0 warnings | `2026-07-14T05:24:30.6028780-07:00` |
| CM7_0 | 0 errors, 3 unchanged pre-existing `Pe550` warnings | `2026-07-14T05:26:25.3015628-07:00` |
| CM7_1 | 0 errors, 0 warnings | `2026-07-14T05:27:52.1159834-07:00` |

The fresh CM7_0 map retains the MT9V03X implementation and fixed image buffer. The fresh CM7_1 map contains `camera_frame_consumer` and `intercore_camera_consumer_publish_observation`, contains no MT9V03X driver/producer symbols, and retains camera/control exports at `0x28060000`/`0x10000` and `0x28080000`/`0x2000`. Camera host, foundation host, capture-service static, camera static, UART ownership, IMU numeric, servo 300 Hz, leg-zero, IK-height, and `git diff --check` all passed.

### Formal Hardware Gate 1

The formal repository loader configuration was downloaded from the active CM7_0 project only. IAR reported `Download completed` at local `2026-07-14 04:52:22`; no separate generic CM0+ download followed. The target ran from `2026-07-14T11:53:08.251Z` to `2026-07-14T11:55:11.839Z`, 123.588 seconds. It produced 6,075 frames (about 49.16 FPS), `init_state=0x03`, `frame_valid=1`, changing samples, `last/max_copy_duration_us=81/96`, `app_safety_fault=0`, and left/right duty 0. The finish flag was zero when frozen because the producer clears each handled event.

### Formal Hardware Gate 2 with shared consumer mirror

After the version-2 mirror build, the debugger-input reload performed one CM7_0-originated download. IAR reported `Download completed` at local `2026-07-14 05:32:34`; the repository capture service and fresh CM7_1 HEX were the two enabled extra images, and no generic CM0+ download followed.

An initial frozen sample proved that the fresh CM7_1 image committed the non-cacheable mirror after release: sequence 2,596, age 0 ms, samples `0x29`/`0xBE`, and valid 1. The formal accounting window then ran from `2026-07-14T12:39:32.772Z` to `2026-07-14T12:41:24.867Z`, 112.095 seconds:

| Diagnostic | Baseline | Endpoint | Delta |
| --- | ---: | ---: | ---: |
| producer frame / shared captured | 2,600 | 8,205 | 5,605 |
| producer publish / shared published | 513 | 1,621 | 1,108 |
| shared consumed | 513 | 1,621 | 1,108 |
| producer period drop | 2,087 | 6,584 | 4,497 |
| producer no-free / invalid / timeout | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 |
| shared no-free / stale / invalid / timeout | 0 / 0 / 0 / 0 | 0 / 0 / 0 / 0 | 0 / 0 / 0 / 0 |
| mirror sequence | 2,596 | 8,204 | 5,608 |
| mirror age | 0 ms | 0 ms | 0 ms |
| mirror samples / valid | `0x29` / `0xBE` / 1 | `0x2E` / `0xBF` / 1 | changing / valid |
| producer heartbeat | 51,982 | 164,000 | 112,018 ms |
| consumer heartbeat | 341,443 | 532,016 | 190,573 ticks |

The accounting closes exactly: cumulative `8205 = 1621 + 6584 + 0 + 0`, and over the formal window `5605 = 1108 + 4497 + 0 + 0`. Published delta equals consumed delta, both slots were `FREE` at the endpoint, copy last/max was 77/94 us, safety fault remained 0, and left/right motor duty remained 0. The 1,108 publishes over 112.095 seconds match the configured 100 ms display period. The mirror sequence is the most recent captured frame sequence, not the publish count; its sequence-last commit proves the associated age/sample/valid fields belong to a successfully released frame.

### Disposition

**PASS — MT9V03X zero-frame root cause fixed and CM7_0 capture / CM7_1 latest-ready handoff verified.** The generic CM0+ overwrite was the causal fault. The imported, hash-locked capture service plus the formal CM7_0 loader workflow restores continuous capture and clean cross-core accounting. Task 3 was not entered. Wheel motor power remained operator-confirmed OFF, and no motion, wheel, servo, or `LXY` command was issued.

## Independent-review documentation and loader-test correction

No hardware was rerun for this documentation/test correction. The EWD contract now uses XML parsing rather than raw-text matching: each of the seven required option names must occur exactly once, contain exactly one state, and match the approved path/use/offset value. Both parsed image paths must resolve to existing files. In-memory negative self-tests reject swapped image slots, either nonzero offset, and `OCDownloadExtraImage=0`; separate temporary-file mutations were observed to exit 1 for all four cases before the authoritative EWD passed. The test and loader do not reference or require an IAR `.sim` sidecar.

The complete 35,821-byte GPLv3 text was copied byte-for-byte from `D:\smartcar\CYT4BB7_Library\LICENSE` to `libraries/doc/GPL-3.0.txt`; both hash to `0B383D5A63DA644F628D99C33976EA6487ED89AAA59F0B3257992DEAC1171E6B`. Provenance now limits the vendor `.ewx` to current local research, records that the supplied reference lacks Corresponding Source/build instructions for the binary, and requires Seekfree source/written-offer/additional-authorization material plus project-owner confirmation before any push or external distribution. It makes no repository-wide license claim.

The approved handoff design and implementation plan now match camera-control version 2: mirror fields consume the former reserve, `consumer_last_sequence`/age remain at offsets 172/176, trailing reserve is 72 bytes, and observation publication is permitted only after successful release with data fields → DMB → sequence → DMB. Capture static, camera static, both host tests, and `git diff --check` are the required software regression gate for this correction.
