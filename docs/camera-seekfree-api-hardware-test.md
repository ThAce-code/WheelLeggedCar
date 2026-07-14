# MT9V03X Seekfree API Hardware Evidence

This record gates camera integration against the untouched Seekfree E9_01 reference. `PASS` means direct evidence was observed during this run or an operator confirmation is explicitly identified as such. `NOT RUN` includes unavailable hardware, tooling, or UI state; details must explain the gap. Wheel power must remain disabled throughout the reference test.

> Runtime loader requirement: camera builds must be launched from active CM7_0. Its debugger loads `project/iar/project_config/mt9v03x_cm0plus_capture_service.ewx` as extra image 1 and the current CM7_1 HEX as extra image 2. Do not separately download the generic CM0+ project; doing so replaces the capture service at `0x10000000` and returns the system to the confirmed init-success/zero-frame failure. Exact provenance and address checks are in `docs/mt9v03x-capture-service-provenance.md`.

## Board and wiring

| Result | Status | Evidence / observation |
| --- | --- | --- |
| Wheel power disabled | PASS | Operator-confirmed after the camera test: wheel motor power was OFF. This was not independently measured. |
| Robot legs in the validated all-90-degree reference pose, if the chassis is used | NOT RUN | The physical host and leg pose were not observable. |
| Camera wiring follows the planned pin map | PASS | Operator-confirmed: SCL `P17_1`, SDA `P17_2`, PCLK `P06_5`, VSYNC `P06_6`, and D0..D7 `P18_0..P18_7`. This records pin-map confirmation, not a continuity-meter measurement. |
| MT9V03X `P17_1` SCL continuity with power removed | NOT RUN | Operator confirmed the planned pin mapping; no continuity instrument measurement was performed. |
| MT9V03X `P17_2` SDA continuity with power removed | NOT RUN | Operator confirmed the planned pin mapping; no continuity instrument measurement was performed. |
| MT9V03X `P06_5` PCLK continuity with power removed | NOT RUN | Operator confirmed the planned pin mapping; no continuity instrument measurement was performed. |
| MT9V03X `P06_6` VSYNC continuity with power removed | NOT RUN | Operator confirmed the planned pin mapping; no continuity instrument measurement was performed. |
| MT9V03X `P18_0..P18_7` D0..D7 continuity with power removed | NOT RUN | Operator confirmed the planned pin mapping; no continuity instrument measurement was performed. |
| WiFi-SPI `P02_0` MISO continuity with power removed | NOT RUN | No continuity measurement was available. |
| WiFi-SPI `P02_1` MOSI continuity with power removed | NOT RUN | No continuity measurement was available. |
| WiFi-SPI `P02_2` SCK continuity with power removed | NOT RUN | No continuity measurement was available. |
| WiFi-SPI `P02_3` CS continuity with power removed | NOT RUN | No continuity measurement was available. |
| WiFi-SPI `P02_4` INT continuity with power removed | NOT RUN | No continuity measurement was available. |
| WiFi-SPI `P23_0` RST continuity with power removed | NOT RUN | No continuity measurement was available. |

Board / camera / WiFi module identifiers:

- Board: not identified from the available UI
- Camera: MT9V03X inferred from the loaded, untouched reference and live symbols; module marking not observed
- WiFi module: Seekfree WiFi-SPI V2, firmware `V2.0.0`, MAC `9C:13:9E:C6:4D:1C`
- Debug probe: CMSIS-DAP (IAR debug log)
- Continuity instrument: not available

## Reference build

Authoritative reference (read-only):

`D:\smartcar\CYT4BB7_Library\Example\Motherboard_Demo\E9_seekfree_assistant\E9_01_seekfree_assistant_mt9v03x_demo`

| Result | Status | Warnings | Errors | Evidence / generated map path |
| --- | --- | ---: | ---: | --- |
| IAR Embedded Workbench 9.40.1 (or compatible) identified | PASS | — | — | Window title and debug log identify IAR Embedded Workbench 9.40.1. |
| Untouched reference source verified before build | PASS | — | — | `user/main_cm7_0.c` SHA-256 before and after: `8D2A8208217EA5500F2C455F941E08F4EAAE6FE29E9738E78C859ACC160C8E97`. No source edit was made. |
| `cyt4bb7_cm_0_plus` build | NOT RUN | — | — | The supplied E9_01 workspace contains only CM7_0 and CM7_1 projects; no CM0+ project is supplied by this reference. |
| `cyt4bb7_cm_7_0` set Active, then built | PASS | 0 | 0 | IAR: `Build succeeded`. Map: `iar/Debug_m7_0/List/cyt4bb7_cm_7_0.map`. |
| `cyt4bb7_cm_7_1` build | PASS | 0 | 0 | IAR: `Build succeeded`. Map: `iar/Debug_m7_1/List/cyt4bb7_cm_7_1.map`. CM7_0 remained Active for the later debug session. |

## UART image result

Reference flow: `SEEKFREE_ASSISTANT_DEBUG_UART`.

| Result | Status | Evidence / observation |
| --- | --- | --- |
| CM7_0 download and debug connection | PASS | CMSIS-DAP download completed, target reset completed, and IAR stopped at `main()`. |
| Reference firmware progresses past camera initialization | PASS | After Go, Live Watch showed a live finish flag and nonzero changing image samples; the program was not trapped in the initialization-error blink loop. Physical LED behavior was not observed. |
| `mt9v03x_finish_flag` observed in IAR Live Watch or Watch | PASS | Readable as `1` during the running session at multiple observation points. |
| Live frame-buffer samples change during the 78.4-second run | PASS | `[0][0]`: `0x67` -> `0x87` -> `0x67`; `[60][94]`: `0xE1` -> `0xE0` -> `0xDF`. This proves buffer activity, not controlled-scene response. |
| `mt9v03x_image[0][0]` changes with a moved high-contrast object | PASS | The operator explicitly reported that the hand/object had been moved. The Assistant changed from the pre-movement close-up diagonal-edge scene to a wide desk/window scene containing the hand and a dark rectangular object. Live Watch changed from pre-movement `0x77` to post-movement `0xB4`. |
| `mt9v03x_image[60][94]` changes with a moved high-contrast object | PASS | Under the same operator-confirmed movement and clearly changed Assistant scene, Live Watch changed from pre-movement `0xDF` to post-movement `0xFF`. |
| Assistant shows repeated changing 188 x 120 grayscale frames for at least 60 seconds | PASS | Seekfree Assistant V2.0.0.6 beta connected to COM6 at 115200 baud, 8N1, no flow control. Its live-image view held 188 x 120 for 75.000 seconds, with screen FPS 1 and approximately 11,399-11,520 bytes/second. The first, middle, and final camera-region screenshots differed (grayscale mean absolute differences 1.174, 0.784, and 1.159), consistent with the already observed changing frame-buffer samples. |

## WiFi result

Gate B used an isolated copy of the authoritative E9_01 reference at
`D:\smartcar\bringup\camera_wifi_spi_gate_b_20260713\E9_01_seekfree_assistant_mt9v03x_demo`.
The authoritative reference remained read-only. The copied application retained the E9_01 camera initialization, one
`image_copy`, `memcpy`, camera/boundary configuration, and Assistant camera/boundary send calls. Only the transport
initialization/interface selection and a no-queue 100 ms send gate were added. Temporary volatile stage markers were
also added to the copied WiFi-SPI library to locate an initial association timeout; they did not alter command flow or
return values. For completion, collapse, and drop quantification, the external temporary copy also declared
`mt9v03x_completion_count` in `zf_device_mt9v03x.h` and incremented it in the camera-finish callback in
`zf_device_mt9v03x.c`. This counter-only diagnostic did not change camera capture or Assistant send behavior, and the
authoritative camera library remained unchanged.

| Result | Status | Evidence / observation |
| --- | --- | --- |
| Isolated copy used; authoritative E9_01 unchanged | PASS | Authoritative `user/main_cm7_0.c` SHA-256 remained `8D2A8208217EA5500F2C455F941E08F4EAAE6FE29E9738E78C859ACC160C8E97` after Gate B. No authoritative or robot source was edited. |
| Copied CM7_0 WiFi-SPI build | PASS | IAR Embedded Workbench 9.40.1 reported `Build succeeded`, 0 errors and 0 warnings. |
| Hotspot and Assistant endpoint | PASS | Windows hotspot SSID `SEEKFREE`, WPA2 password `SEEKFREE123`, band fixed to 2.4 GHz. Seekfree Assistant V2.0.0.6 beta used Network / TCP Server, bind `0.0.0.0`, port `8086`. Firmware used remote `192.168.137.1:8086` and local port `6666`. |
| WiFi-SPI identity and association | PASS | Module firmware `V2.0.0`, MAC `9C:13:9E:C6:4D:1C`; Windows reported one hotspot client at `192.168.137.206`. With hotspot band `Auto`, identity reads succeeded but `WIFI_SPI_SET_WIFI_INFORMATION` timed out after its write. Changing only the hotspot band to 2.4 GHz made the same image associate successfully. |
| `wifi_spi_init()` | PASS | Live Watch `wifi_init_result = 0`; diagnostic command completed with stage/reply `0x80` and return state `0`. |
| `wifi_spi_socket_connect()` | PASS | Live Watch `wifi_socket_result = 0`. Windows held `192.168.137.206:6666 -> 192.168.137.1:8086` in `Established`, owned by the Assistant process, while `0.0.0.0:8086` remained `Listen`. |
| Existing Assistant packet reused over WiFi-SPI | PASS | Application selected `SEEKFREE_ASSISTANT_WIFI_SPI` and continued to call the existing `seekfree_assistant_camera_boundary_send()` and `seekfree_assistant_camera_send()` APIs; it did not call `wifi_spi_send_buffer()` directly or change Assistant framing. |
| One-copy, latest-frame, no-queue 100 ms gate | PASS | A single `frame_pending` flag decoupled frame arrival from send scheduling. A completion makes the newest frame pending; a newer completion replaces an older unsent pending frame, and no queue or custom protocol was introduced. The main loop independently sends only when elapsed time since the previous send start is at least 100 ms, copies the latest completed `mt9v03x_image` into the existing `image_copy`, clears pending at send start, then preserves the existing boundary-before-camera Assistant send flow. |
| Assistant shows changing 188 x 120 grayscale frames during the measured run | PASS | During the formal 185.908-second counter window, Assistant remained connected and displayed 188 x 120 imagery. Foreground captures at `22:24:46.360` and `22:25:02.180 -07:00` reported FPS 9 and 11, and 237,540 and 213,764 bytes/s; their encoded screenshots differed in 79,818 characters. |
| Effective display rate, strict interval proof, and throughput | PASS | Exact frozen Live Watch window `22:23:04.599` to `22:26:10.507 -07:00` (185.908 seconds): completion `7936 -> 17238` (+9302), ready `7695 -> 16937` (+9242), send `1545 -> 3401` (+1856, 9.983 FPS), total drop `6391 -> 13837` (+7446), collapsed `241 -> 301` (+60), gate-closed arrivals `7681 -> 16908` (+9227), deferred batches `1542 -> 3395` (+1853), and pending replacements `6149 -> 13535` (+7386). Send-interval histogram deltas were `<100 ms = 0`, `100-109 ms = 1852`, `110-119 ms = 0`, and `>=120 ms = 4`; all 1,856 sends were therefore at least 100 ms apart. In a concurrent 12.4847-second hotspot window, 3,092,989 bytes transferred at 247,742.9 bytes/s (1.890 Mbit/s). |
| 60-second stability, stalls, and protocol errors | PASS | The 185.908-second formal window exceeded the 60-second gate. TCP remained `Established` from `192.168.137.206:6666` to `192.168.137.1:8086` at both throughput snapshots while `0.0.0.0:8086` remained `Listen`; Assistant imagery continued changing and no disconnect or protocol error was observed. Counter identities held exactly: completion minus send = total drop (+7446), completion minus ready = collapsed (+60), and ready minus send = pending replacements (+7386) because pending state was the same at both frozen endpoints. This is reference transport evidence, not a robot real-time-safety claim. |

## CM7_1 portability

| Result | Status | Evidence / observation |
| --- | --- | --- |
| Camera-only application initializes on CM7_1 | PASS | All three cores were downloaded, CM7_1 was set Active and run, and IAR Live Watch showed `camera_debug_diag.init_state = 0x04` (`CAMERA_DEBUG_INIT_OK`). WiFi and Assistant transport were disabled. |
| Unchanged MT9V03X driver completes frames on CM7_1 | FAIL | Target reset/download occurred at local `2026-07-13 23:53:36 -07:00`. At `2026-07-14 00:01:04.479 -07:00`, after approximately 448.479 seconds, Live Watch still showed `frame_count = 0`, `last_frame_ms = 0`, `frame_valid = 0`, and `mt9v03x_finish_flag = 0`. IAR Debug state confirmed the CPU was running rather than stopped at a breakpoint. |
| 100 ms snapshot scheduler runs at approximately 10 FPS | FAIL | `snapshot_count` remained `0` because no completed frame reached the scheduler. This is a capture failure, not a scheduler-rate measurement. |
| Sample pixels respond to a controlled scene change | NOT RUN | No completed CM7_1 frame was available. Watched samples `image_copy[0][0]` and `image_copy[60][94]` both remained `0`. |
| Hard wheel-duty lock in camera-debug mode | PASS | `APP_CAMERA_DEBUG_ONLY` is locked to `1U`; the sole `actuator_motor_send_duty()` choke point overwrites both requested duties with zero before `bldc_foc_uart_set_duty()`. The static safety contract passed and wheel motor power remained operator-confirmed OFF. |
| Motion-command logical safety check | NOT RUN | No motion command was sent after the no-frame condition triggered STOP B. No wheel motion or `LXY` command was attempted. |
| Servos remain at the all-90-degree reference pose | NOT RUN | The physical servo pose was not independently observable during this run; no servo command was sent. |
| Core-dependent reference architecture reviewed | PASS | The unchanged driver had already produced repeated changing frames on CM7_0 during Gate A. The supplied `E8_09_mt9v03x_uart_seekfree_assistant_cross_ram_m7_1_demo` likewise keeps `mt9v03x_init()` and capture/copy ownership on CM7_0; CM7_1 only reads cross-RAM and sends the Assistant image. This supports the planned core-dependent STOP B classification without modifying DMA/GPIO/TCPWM. |

## Three-core robot build

| Result | Status | Evidence / observation |
| --- | --- | --- |
| Source-matched MT9V03X and Assistant files | PASS | Imported only `zf_device_mt9v03x.c/.h` and `seekfree_assistant.c/.h` from the approved E9 library. SHA-256 values are respectively `33C9B8C1D5641CB48933B4E6796788BB247C813909B4A3F16D963A6B40D7E7E0`, `0C489FABA1851C861E33B44E222D7D72B331EEC851B09F93DC694A20FA17DED2`, `6C6BABD379FAFCCBB64F4DE27A8E837EC3C73EF4E7ECD169656CF80B0C13A28B`, and `FA6FC8DD75323AF03A49FB48BBA63029AD2CB36C1C8484F4B9596835EADEBBB5`, matching the approved hashes enforced by `tools/test_camera_seekfree_api_static.ps1`. Existing Assistant interface, WiFi-SPI, and device-config files were not replaced. |
| Hash-locked source checkout and whitespace policy | PASS | Repository `core.autocrlf=true`, while the approved upstream Assistant bytes include trailing whitespace. `.gitattributes` therefore limits `-text -whitespace` to exactly the four hash-locked source paths: checkout cannot translate line endings, and `git diff --check` does not require altering approved upstream bytes. The static test verifies all four exact attribute rows. |
| Static provenance, ownership, pin, API, and linker contract | PASS | `powershell -ExecutionPolicy Bypass -File tools/test_camera_seekfree_api_static.ps1` reported `PASS`. The test was first observed RED before the four imports, again RED before the contiguous heap/stack linker correction, and again RED before the CM7_1 linker retained the three fixed symbols. |
| Integrated robot CM0+ build | PASS | Task 4 fresh IAR ARM compiler 9.40.1.364 build: 0 errors, 0 warnings in 83.0 seconds. Map: `project/iar/Debug_m0_plus/List/cyt4bb7_cm_0_plus.map`. |
| Integrated robot CM7_0 build | PASS | Task 4 fresh IAR ARM compiler 9.40.1.364 build: 0 errors, 3 pre-existing `Pe550` warnings for unused `control_leg_height_cmd`, `control_leg_pitch_cmd`, and `control_leg_roll_cmd` in 103.7 seconds. Map: `project/iar/Debug_m7_0/List/cyt4bb7_cm_7_0.map`. |
| Integrated robot CM7_1 build | PASS | Task 4 fresh IAR ARM compiler 9.40.1.364 build: 0 errors, 0 warnings in 84.3 seconds. PIT_CH10..CH21 clear-only handlers were required by the real link and added without application work. The linker retains the three fixed camera symbols. Map: `project/iar/Debug_m7_1/List/cyt4bb7_cm_7_1.map`. |

## Map ranges

| Result | Status | Evidence / observation |
| --- | --- | --- |
| Reference CM0+ map memory ranges reviewed | NOT RUN | No CM0+ project/map is supplied by E9_01. |
| Reference CM7_0 map memory ranges reviewed | NOT RUN | Map generated at `iar/Debug_m7_0/List/cyt4bb7_cm_7_0.map`; ranges were not part of Gate A and were not reviewed. |
| Reference CM7_1 map memory ranges reviewed | NOT RUN | Map generated at `iar/Debug_m7_1/List/cyt4bb7_cm_7_1.map`; ranges were not part of Gate A and were not reviewed. |
| `mt9v03x_h_num` fixed placement | PASS | Integrated CM7_1 map lists `.noinit` and symbol `mt9v03x_h_num` at `0x28006BF0`, size `0x2`; exclusive end `0x28006BF2`. |
| `mt9v03x_w_num` fixed placement | PASS | Integrated CM7_1 map lists `.noinit` and symbol `mt9v03x_w_num` at `0x28006BF2`, size `0x2`; exclusive end `0x28006BF4`. |
| `mt9v03x_image_temp` fixed placement | PASS | Integrated CM7_1 map lists `.noinit` and symbol `mt9v03x_image_temp` at `0x28026024`, size `0x5820`; exclusive end `0x2802B844` (inclusive last byte `0x2802B843`). |
| CM0+ ordinary SRAM excludes camera absolute words | PASS | CM0+ map placement summary restricts ordinary read-write placement to `0x28000800-0x28006BEF` union `0x28006BF4-0x2801FFFF`; ordinary allocation cannot occupy `0x28006BF0-0x28006BF3`. Heap/stack is placed at the end of the contiguous post-hole range. |
| CM7_0 ordinary SRAM excludes camera scratch image and data plane | PASS | Task 1 CM7_0 map placement summary restricts ordinary read-write placement to `0x28020000-0x28026023`, `0x2802B844-0x2805FFFF`, and `0x28070000-0x2807FFFF`; ordinary allocation cannot occupy `0x28026024-0x2802B843` or the camera data plane at `0x28060000-0x2806FFFF`. Heap/stack is confined to the final range. |
| Shared SRAM remains `0x28080000-0x28081FFF` | PASS | All three integrated maps export `__intercore_shared_sram_base = 0x28080000` and `__intercore_shared_sram_size = 0x2000`. |
| CM7_1 ordinary SRAM begins at `0x28082000` | PASS | Integrated CM7_1 placement summary is `0x28082000-0x280BFFFF`; the first initialized data block begins at `0x28082000`. |
| Integrated three-core map ranges checked for overlap | PASS | Revalidated after the Task 1 fresh builds. CM0+ ordinary SRAM is `0x28000800-0x28006BEF` union `0x28006BF4-0x2801FFFF`; CM7_0 ordinary SRAM is `0x28020000-0x28026023`, `0x2802B844-0x2805FFFF`, and `0x28070000-0x2807FFFF`; CM7_1 ordinary SRAM is `0x28082000-0x280BFFFF`. The fixed camera objects remain at `0x28006BF0`, `0x28006BF2`, and `0x28026024`; camera data is `0x28060000` size `0x10000`; shared control remains `0x28080000` size `0x2000`. |

## Task 1 cross-core camera handoff evidence

| Result | Status | Evidence / observation |
| --- | --- | --- |
| Host state-machine RED | PASS | After adding `project/tests/intercore_camera_handoff_test.c` and before production implementation, the required GCC command exited `1` with `fatal error: intercore_camera.h: No such file or directory` and `fatal error: project/code/intercore_camera.c: No such file or directory`. |
| Host state-machine GREEN | PASS | GCC C11 with `-Wall -Wextra -Werror -DINTERCORE_HOST_TEST` compiled the camera handoff test; `intercore_camera_handoff_test: PASS`. The existing foundation regression also reported `intercore_control_foundation_test: PASS`. |
| Linker/MPU static RED and GREEN | PASS | The extended static contract first failed on exactly 14 new camera linker, MPU, project, getter, and notify assertions. After implementation, `powershell -ExecutionPolicy Bypass -File tools/test_camera_seekfree_api_static.ps1` reported `PASS`; `git diff --check` also exited `0`. |
| Independent-review ABI RED and GREEN | PASS | The review regression first failed compilation on the absent approved `INTERCORE_CAMERA_NO_FRAME`, `transport.control`, `last_consumed_sequence`, and `view.width/height/stride` interface. Static RED reported exactly six ABI/order/state-contract failures. After the fix, the host compile/API assertions and static contract both pass with the exact approved enum values and Task 2-facing structure fields. |
| Ownership memory ordering | PASS | Static and source review confirm camera magic is followed by an acquire DMB before other control fields are read; publish remains fields → DMB → `READY` → DMB; release is DMB → `FREE` → DMB. |
| Invalid layout/state rejection | PASS | Table-driven host coverage rejects wrong magic, version, format, width, height, stride, slot count, frame bytes, producer epoch, and either slot state above `READING`, incrementing `invalid_layout_count`. Illegal slot states also block producer record/claim/publish and consumer acquire/release. Consumer restart explicitly preserves `WRITING` and `READY` while freeing stale `READING`. |
| Task 1 CM0+ fresh build | PASS | Rebuilt after independent-review fixes with IAR ARM compiler 9.40.1.364: 0 errors, 0 warnings. |
| Task 1 CM7_0 fresh build | PASS | Rebuilt after independent-review fixes with IAR ARM compiler 9.40.1.364: 0 errors, 3 pre-existing `Pe550` warnings for unused `control_leg_height_cmd`, `control_leg_pitch_cmd`, and `control_leg_roll_cmd`. No linker overlap was reported. |
| Task 1 CM7_1 fresh build | PASS | Rebuilt after independent-review fixes with IAR ARM compiler 9.40.1.364: 0 errors, 0 warnings. Both CM7 projects compile `intercore_camera.c/.h`. |
| Camera data-plane reservation | PASS | All three maps export `__camera_shared_sram_base = 0x28060000` and `__camera_shared_sram_size = 0x10000`. CM7_0 ordinary read-write placement is `0x28020000-0x28026023`, `0x2802B844-0x2805FFFF`, and `0x28070000-0x2807FFFF`, leaving the full 64 KiB data plane unallocated. |
| CM7_0 heap/stack placement | PASS | CM7_0 places `HEAP_STACK` at `0x2807E000-0x2807FFFF`, wholly inside the dedicated `0x28070000-0x2807FFFF` heap/stack region. |
| Shared control and CM7_1 ordinary SRAM | PASS | All three maps retain `__intercore_shared_sram_base = 0x28080000` and `__intercore_shared_sram_size = 0x2000`; CM7_1 ordinary read-write placement remains `0x28082000-0x280BFFFF`. |
| Fixed MT9V03X objects after reservation | PASS | CM7_1 map retains `mt9v03x_h_num` at `0x28006BF0` size `0x2`, `mt9v03x_w_num` at `0x28006BF2` size `0x2`, and `mt9v03x_image_temp` at `0x28026024` size `0x5820`. |

## Timing

| Result | Status | Evidence / observation |
| --- | --- | --- |
| Camera initialization timing recorded | NOT RUN | Initialization completed, but elapsed initialization time was not measured. |
| Live Watch camera-buffer observation held for 60 seconds | PASS | UTC `2026-07-14T02:43:05.588Z` to `2026-07-14T02:44:23.976Z` (78.388 seconds). |
| UART Assistant image observation held for 60 seconds | PASS | UTC `2026-07-14T03:09:20.250Z` to `2026-07-14T03:10:35.250Z` (75.000 seconds), continuously connected on COM6 with 188 x 120 live imagery and approximately 11.4 kB/s receive rate. |
| WiFi-SPI Assistant image observation held for 60 seconds | PASS | Frozen formal window local `2026-07-13 22:23:04.599` to `22:26:10.507 -07:00` (185.908 seconds), continuously connected over TCP with changing 188 x 120 imagery. Firmware measured 9.983 FPS with zero `<100 ms` intervals; concurrent throughput was 247,742.9 bytes/s (1.890 Mbit/s). |
| CM7_1 portability Live Watch held beyond 60 seconds | FAIL | Local `2026-07-13 23:53:36 -07:00` to `2026-07-14 00:01:04.479 -07:00` (approximately 448.479 seconds). CPU remained running, initialization stayed OK, but `frame_count`, `snapshot_count`, and both watched copy-buffer samples remained zero. |
| Integrated scheduler/control timing impact checked | NOT RUN | STOP B occurred before a valid frame or snapshot cadence existed, so no runtime scheduling-impact claim is made. |

## Final disposition

| Gate | Status | Disposition |
| --- | --- | --- |
| Gate A — untouched E9_01 supplied-project build plus 60-second debug-UART camera proof | PASS | Both supplied E9_01 projects build with 0 errors/0 warnings, the untouched CM7_0 reference downloads and runs, Assistant displays changing 188 x 120 imagery for 75.000 seconds, the operator-confirmed hand/object movement changes the scene and both watched samples, wheel motor power was operator-confirmed OFF, and camera wiring was operator-confirmed against the planned pin map. Individual continuity-meter measurements and the conditional all-90-degree leg-pose observation remain accurately `NOT RUN` and are not claimed. |
| Gate B — existing Assistant camera packet over WiFi-SPI in an isolated E9_01 copy | PASS | The copied CM7_0 project built with 0 errors/0 warnings; WiFi init and TCP socket connect succeeded on a forced-2.4 GHz Windows hotspot; and the single-pending latest-frame scheduler reused the existing Assistant packets without a queue. Over a frozen 185.908-second window it delivered 1,856 sends (9.983 FPS), every measured send interval was at least 100 ms, TCP stayed connected, and Assistant displayed changing 188 x 120 frames. The isolated scheduler remains reference evidence only and is not integrated robot real-time-safety evidence. |
| Gate C - unchanged-driver CM7_1 camera-only portability | FAIL | CM7_1 initialization succeeded, but no frame completed during approximately 448.479 seconds of a running target: `frame_count = 0`, `snapshot_count = 0`, `last_frame_ms = 0`, `frame_valid = 0`, and `mt9v03x_finish_flag = 0`. This meets the plan's STOP B condition. No DMA/GPIO/TCPWM rewrite, motion command, wheel motion, or `LXY` test was performed. |
| Overall camera integration evidence | FAIL / STOP B | Gates A and B reference proofs remain PASS, and the Task 4 three-core builds, map audit, camera-debug hard wheel lock, and P06_5 ADC exclusion pass. The required CM7_1 capture gate fails with the unchanged driver, so integrated runtime/display acceptance is not claimed and work stops before bottom-driver redesign. |

## Task 2 CM7_0 capture and cross-core handoff

### Static, host, build, and ownership evidence

| Result | Status | Evidence / observation |
| --- | --- | --- |
| Task 2 ownership/static RED | PASS | Before implementation, `tools/test_camera_seekfree_api_static.ps1` failed on exactly 21 new Task 2 assertions covering producer/consumer ownership, CM7 main/ISR wiring, TCPWM59-only masking, project membership, and removal of the old CM7_1 camera app. No pre-existing provenance, linker, MPU, or safety assertion failed. |
| Task 1 release-accounting RED/GREEN | PASS | A host test first proved that a matching release left `last_consume_ms` at zero. The minimal transport correction now increments `consumed_count` and records `consumer_heartbeat_ms` as `last_consume_ms` only after a matching `READING` state and sequence release succeeds; a mismatched release leaves both fields unchanged. |
| Task 2 static and host GREEN | PASS | Camera/static ownership contract, camera handoff host test, and existing inter-core foundation host test passed. The camera producer masks only `tcpwm_0_interrupts_59_IRQn`, copies exactly `MT9V03X_IMAGE_SIZE`, and publishes on the 100 ms latest-frame gate; the CM7_1 consumer performs no camera init, local frame copy, WiFi, or Assistant work. |
| Fresh three-core IAR build | PASS | IAR Embedded Workbench 9.40.1 fresh clean/build succeeded for all cores. CM0+ reported 0 errors/0 warnings; CM7_0 reported 0 errors and the same 3 pre-existing `Pe550` unused-control-command warnings; CM7_1 reported 0 errors/0 warnings. Fresh map timestamps were local 02:30:38, 02:32:11, and 02:33:23 respectively. |
| CM7_0 camera ownership map | PASS | The CM7_0 map contains `camera_capture_producer.o`, `zf_device_mt9v03x.o`, `camera_finish_callback`, `mt9v03x_init`, the three retained fixed camera objects, and `producer_diag`. Fixed objects remain `mt9v03x_h_num=0x28006BF0`, `mt9v03x_w_num=0x28006BF2`, and `mt9v03x_image_temp=0x28026024` size `0x5820`. |
| CM7_1 no-camera/no-WiFi consumer map | PASS | The CM7_1 map contains `camera_frame_consumer.o` and no linked `zf_device_mt9v03x.o`, camera callback, `mt9v03x_*` driver symbol, `wifi_spi_init`, `wifi_spi_socket_connect`, or `seekfree_assistant_camera_send`. |
| Memory ranges | PASS | All maps retain camera data at `0x28060000` size `0x10000` and shared control at `0x28080000` size `0x2000`. CM7_0 `HEAP_STACK` is `0x2807E000-0x2807FFFF`; CM7_1 ordinary placement starts at `0x28082000`. No overlap was found. |

### Hardware Gate 1 - CM7_0 capture

| Result | Status | Evidence / observation |
| --- | --- | --- |
| Three fresh images downloaded | PASS | IAR GUI `Download and Debug` completed and stopped at each core's `main()`: CM0+ local 02:37:49, CM7_0 02:42:09, and CM7_1 02:43:47. Each session displayed Errors 0 / Warnings 0. |
| 60-second minimum runtime | PASS | CM7_0 Live Watch ran from UTC `2026-07-14T09:54:03.479Z` to `2026-07-14T09:56:02.079Z`, exactly 118.600 seconds. |
| Camera initialization | PASS | Start and end `producer_diag.init_state` were `0x03`, which is `CAMERA_CAPTURE_INIT_OK`. |
| Completed frames | FAIL | Start and end `producer_diag.frame_count` were `0`; `last_frame_ms=0`, `frame_valid=0`, and `mt9v03x_finish_flag=0` throughout. The unchanged driver reported successful init but no camera-finish event reached the application. |
| Pixel response / controlled scene change | NOT RUN | With no completed frame, `mt9v03x_image[0][0]` and `mt9v03x_image[60][94]` remained `0`. A controlled high-contrast target movement could not produce meaningful evidence and was not claimed. |
| Safety state | PASS WITH LIMITATION | Wheel motor power was operator-confirmed OFF before the run; camera wiring and the servo all-90-degree reference were also operator-confirmed. No motion command, wheel test, servo command, or `LXY` command was issued. Safety/control fault variables were not independently watched in Live Watch, so no runtime no-fault claim is made. |

### Hardware Gate 2 - two-slot handoff without WiFi

| Result | Status | Evidence / observation |
| --- | --- | --- |
| Gate 2 execution | NOT RUN | The plan requires Gate 2 to stop when Gate 1 produces no frames. Because `frame_count` remained zero for 118.600 seconds, no consumer/handoff timing window was started. No acquired/released accounting, frame-age, slot endpoint, copy-duration, or changing CM7_1 sample claim is made. |
| WiFi/Assistant | NOT RUN | Task 2 neither initialized nor tested WiFi or Assistant transport. |

### Task 2 disposition

Task 2 source ownership, transport integration, host/static regressions, fresh IAR builds, and map ownership checks pass. Hardware Gate 1 fails because no CM7_0 frame completes in the integrated robot image despite successful camera initialization. Per the approved gate, Gate 2 was not run. The implementation is recorded as `DONE_WITH_CONCERNS`; integrated cross-core runtime acceptance remains unproven.

### Task 2 independent-review correction evidence

| Review item | Status | Evidence / observation |
| --- | --- | --- |
| Single-domain frame age | PASS | Consumer age now uses a snapshot of CM7_0 `producer_heartbeat_ms` against CM7_0 `capture_ms`, not CM7_1 `consumer_ms`. Host cases prove 150 ms normal age, no unsigned underflow when the producer timestamp is briefly behind capture, and a correct 32 ms age across 32-bit wrap. The producer heartbeat is refreshed to the capture timestamp immediately before the slot is published `READY`. |
| Failed-publish rollback | PASS | The public abort frees only an attached CM7_0 transport's still-`WRITING` slot when both metadata and camera-control producer epochs match the transport. Tests prove same-epoch invalid-layout failure recovery, non-`WRITING` rejection, and that an old epoch's failed publish/abort cannot alter a new producer's `WRITING` slot. The producer restores TCPWM59 before conditional abort and invalid accounting. |
| Attach startup competition | PASS | Unpublished camera magic and a producer epoch not yet matching the current boot epoch return not-ready without increasing `invalid_layout_count`. After magic and epoch match, invalid version, format, width, height, stride, slot count, frame bytes, and slot state remain hard invalid cases and each increments the counter. |
| Release-time accounting | PASS | Release-at-time validates the matching `READING` state/sequence, refreshes the consumer heartbeat with the current consumer tick immediately before release, and records that tick as `last_consume_ms`. A simulated time advance records 475 ms instead of the earlier 100 ms; a mismatched view leaves heartbeat and accounting unchanged. |
| Strengthened static contract | PASS | The PowerShell test strips C comments, constrains the ordered producer service path `Disable -> memcpy -> heartbeat catch-up -> publish -> Enable -> conditional abort`, rejects cross-domain subtraction, and requires a fresh tick at release. |
| Full software regression | PASS | Camera static, camera host, inter-core foundation host, CM7 UART ownership, IMU numeric, servo 300 Hz, leg IK zero, IK height, and `git diff --check` all passed. |
| Fresh three-core review-fix build | PASS | IAR 9.40.1 clean/build: CM0+ 0 errors/0 warnings; CM7_0 0 errors/3 unchanged pre-existing `Pe550` warnings; CM7_1 0 errors/0 warnings. Fresh map timestamps were local `03:33:55.894`, `03:35:20.443`, and `03:36:32.519`. CM7_0 still contains the producer, MT9V03X driver/callback/init/fixed objects; CM7_1 contains the consumer and no MT9V03X implementation, WiFi, or Assistant symbols. Camera/control reservations remain `0x28060000`/`0x10000` and `0x28080000`/`0x2000`, with no ordinary-placement overlap. |
| Hardware disposition after review | `FAIL` / `NOT RUN` | Hardware was not rerun. Gate 1 remains `FAIL`: initialization succeeded but no finish event/completed frame appeared in 118.600 seconds. Gate 2 remains `NOT RUN` by the zero-frame rule. No root cause or cross-core runtime acceptance is claimed. |

Review-fix disposition remains `DONE_WITH_CONCERNS`.

#### Final P2 attach-classification correction

| Result | Status | Evidence / observation |
| --- | --- | --- |
| Camera magic classification RED/GREEN | PASS | The host regression first failed because a nonzero incorrect camera magic returned not-ready without incrementing `invalid_layout_count`. After the minimal fix, `magic == 0` remains a silent startup retry, nonzero incorrect magic increments the invalid counter and fails, and correct magic with producer-epoch mismatch remains a silent not-ready retry. |
| Focused regression and build | PASS | Camera host/static, foundation host, and `git diff --check` passed. Incremental IAR builds passed for CM7_0 with 0 errors/3 unchanged pre-existing `Pe550` warnings and CM7_1 with 0 errors/0 warnings. No hardware was run; Gate 1 remains `FAIL` and Gate 2 remains `NOT RUN`. |

## MT9V03X zero-frame diagnosis and formal correction

This section supersedes the earlier Task 2 `FAIL`/`NOT RUN` hardware disposition. A controlled same-M7 comparison established that a separate generic CM0+ download overwrote the reference MT9V03X capture service. The generic image produced zero frames for 118.600 seconds; restoring the source-matched CM0+ service produced 11,506 changing frames over 232.082 seconds (about 49.58 FPS). The unchanged service image is now stored as `project/iar/project_config/mt9v03x_cm0plus_capture_service.ewx`, hash-locked to SHA-256 `508CD93B33730384804E55794C3A11819E904740B3EC60519318B989DCF6A299`, and loaded only through the CM7_0 debugger configuration together with the fresh CM7_1 HEX.

| Result | Status | Evidence / observation |
| --- | --- | --- |
| Capture-service static contract | PASS | Exact hash, Intel HEX checksums/range/entry point, four fixed camera constants, provenance text, enabled loader slots, zero offsets, and resolved CM7_1 path all pass. No `.sim` sidecar is required. |
| Shared consumer mirror RED/GREEN | PASS | RED reported seven absent version/layout/API/order/call-site requirements. GREEN retains a 256-byte camera control block and 8192-byte shared layout at protocol version 2, with sequence/age offsets 172/176 and sequence written last after DMB. Camera host and static tests pass. |
| Fresh three-core build | PASS | CM0+ 0 errors/0 warnings; CM7_0 0 errors/3 unchanged `Pe550` warnings; CM7_1 0 errors/0 warnings. Fresh map timestamps were local `05:24:30.603`, `05:26:25.302`, and `05:27:52.116`. |
| Full regression | PASS | Camera and foundation host tests, capture-service/camera/UART static checks, IMU numeric, servo 300 Hz, leg-zero, IK-height, and `git diff --check` all pass. |
| Formal Gate 1 | PASS | `2026-07-14T11:53:08.251Z` to `11:55:11.839Z` (123.588 s): 6,075 frames, about 49.16 FPS, init OK, valid changing pixels, last/max copy 81/96 us, safety 0, duty 0/0. |
| Formal Gate 2 | PASS | `2026-07-14T12:39:32.772Z` to `12:41:24.867Z` (112.095 s): captured 2,600→8,205; published/consumed 513→1,621; period-drop 2,087→6,584; no-free/stale/invalid/timeout all 0; slots `FREE/FREE`. Accounting closes: `5605 = 1108 + 4497 + 0 + 0`. |
| CM7_1 shared observation | PASS | Sequence 2,596→8,204; age 0→0 ms; samples `0x29`/`0xBE`→`0x2E`/`0xBF`; valid 1→1. The fields reside in the non-cacheable shared control region and are committed only after successful release. |
| Safety boundary | PASS | Wheel motor power remained operator-confirmed OFF. Safety fault and left/right motor duty remained 0. No motion, wheel, servo, or `LXY` command was issued; Task 3 was not entered. |

**Corrected disposition: PASS — MT9V03X capture service restored and CM7_0 capture / CM7_1 latest-ready handoff verified.**

### Independent-review static/provenance correction

| Result | Status | Evidence / observation |
| --- | --- | --- |
| EWD XML binding RED/GREEN | PASS | The loader test now parses XML and requires exactly one option/state for each Path1/Use1/Offset1/Path2/Use2/Offset2/ExtraImage setting. Temporary mutations for swapped slots, Offset1=1, Offset2=1, and ExtraImage=0 each exited 1 with the expected option-specific failure; the authoritative EWD then passed. Both parsed files must exist, and no `.sim` file is referenced or required. |
| GPLv3 source record | PASS | `libraries/doc/GPL-3.0.txt` is the byte-identical 35,821-byte copy of `D:\smartcar\CYT4BB7_Library\LICENSE`; SHA-256 is `0B383D5A63DA644F628D99C33976EA6487ED89AAA59F0B3257992DEAC1171E6B`. The artifact provenance limits the `.ewx` to current local research and records the source/written-offer/authorization plus project-owner gate before push or external distribution without assigning a whole-repository license. |
| Approved ABI documents | PASS | The handoff design and implementation plan now state camera-control version 2, mirror fields with a 72-byte trailing reserve, offsets 172/176, and successful-release-only data → DMB → sequence → DMB publication. |
| Hardware scope | NOT RERUN | This review correction changed static validation, licensing/provenance documentation, and approved design/plan text only. The prior formal Gate 1/Gate 2 hardware evidence remains unchanged; Task 3 was not entered. |
