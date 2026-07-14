# MT9V03X Seekfree API Hardware Evidence

This record gates camera integration against the untouched Seekfree E9_01 reference. `PASS` means direct evidence was observed during this run or an operator confirmation is explicitly identified as such. `NOT RUN` includes unavailable hardware, tooling, or UI state; details must explain the gap. Wheel power must remain disabled throughout the reference test.

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
return values.

| Result | Status | Evidence / observation |
| --- | --- | --- |
| Isolated copy used; authoritative E9_01 unchanged | PASS | Authoritative `user/main_cm7_0.c` SHA-256 remained `8D2A8208217EA5500F2C455F941E08F4EAAE6FE29E9738E78C859ACC160C8E97` after Gate B. No authoritative or robot source was edited. |
| Copied CM7_0 WiFi-SPI build | PASS | IAR Embedded Workbench 9.40.1 reported `Build succeeded`, 0 errors and 0 warnings. |
| Hotspot and Assistant endpoint | PASS | Windows hotspot SSID `SEEKFREE`, WPA2 password `SEEKFREE123`, band fixed to 2.4 GHz. Seekfree Assistant V2.0.0.6 beta used Network / TCP Server, bind `0.0.0.0`, port `8086`. Firmware used remote `192.168.137.1:8086` and local port `6666`. |
| WiFi-SPI identity and association | PASS | Module firmware `V2.0.0`, MAC `9C:13:9E:C6:4D:1C`; Windows reported one hotspot client at `192.168.137.206`. With hotspot band `Auto`, identity reads succeeded but `WIFI_SPI_SET_WIFI_INFORMATION` timed out after its write. Changing only the hotspot band to 2.4 GHz made the same image associate successfully. |
| `wifi_spi_init()` | PASS | Live Watch `wifi_init_result = 0`; diagnostic command completed with stage/reply `0x80` and return state `0`. |
| `wifi_spi_socket_connect()` | PASS | Live Watch `wifi_socket_result = 0`. Windows held `192.168.137.206:6666 -> 192.168.137.1:8086` in `Established`, owned by the Assistant process, while `0.0.0.0:8086` remained `Listen`. |
| Existing Assistant packet reused over WiFi-SPI | PASS | Application selected `SEEKFREE_ASSISTANT_WIFI_SPI` and continued to call the existing `seekfree_assistant_camera_boundary_send()` and `seekfree_assistant_camera_send()` APIs; it did not call `wifi_spi_send_buffer()` directly or change Assistant framing. |
| One-copy, no-queue 100 ms gate | PASS | Send was attempted only after `mt9v03x_finish_flag`, only when the timer reached at least 100 ms, and the same single `image_copy` was refreshed immediately before the synchronous Assistant sends. No frame queue was introduced. |
| Assistant shows changing 188 x 120 grayscale frames for at least 60 seconds | PASS | From Go at local `2026-07-13 21:21:41 -07:00` through the measured window ending `21:26:14.752` (at least 273 seconds), Assistant continuously showed 188 x 120 imagery. Two foreground captures 3 seconds apart visibly changed from a near-horizontal dark edge to a curved high-contrast object; encoded screenshots differed in 84,228 characters. |
| Effective display rate and throughput | PASS | Assistant reported 6-7 FPS and 170,060-179,808 bytes/s. In an exact 10.175-second Live Watch window, `wifi_display_send_count` rose `2026 -> 2092` (+66, 6.49 FPS) and the most recent interval remained 139 ms. Independently, the hotspot adapter received 1,444,170 bytes in 10.104 seconds: 142,926 bytes/s (1.09 Mbit/s). The configured 100 ms gate therefore capped the requested rate, while synchronous transport overhead reduced the observed rate below the nominal 10 FPS. |
| 60-second stability, stalls, and protocol errors | PASS | Send count continued increasing past 2,000 with no disconnect, protocol error, or stopped display. Init/socket results stayed `0`; diagnostic reply/stage stayed successful (`0x80`) and return state stayed `0`. Synchronous sending did visibly limit the loop to about 139 ms / 6-7 FPS, so this is transport-reuse evidence only and not a robot real-time-safety claim. |

## CM7_1 portability

| Result | Status | Evidence / observation |
| --- | --- | --- |
| Camera path ported to and validated on CM7_1 | NOT RUN | Outside Gate A; no portability claim made. |

## Three-core robot build

| Result | Status | Evidence / observation |
| --- | --- | --- |
| Integrated robot CM0+ build | NOT RUN | Outside Gate A. |
| Integrated robot CM7_0 build | NOT RUN | Outside Gate A. |
| Integrated robot CM7_1 build | NOT RUN | Outside Gate A. |

## Map ranges

| Result | Status | Evidence / observation |
| --- | --- | --- |
| Reference CM0+ map memory ranges reviewed | NOT RUN | No CM0+ project/map is supplied by E9_01. |
| Reference CM7_0 map memory ranges reviewed | NOT RUN | Map generated at `iar/Debug_m7_0/List/cyt4bb7_cm_7_0.map`; ranges were not part of Gate A and were not reviewed. |
| Reference CM7_1 map memory ranges reviewed | NOT RUN | Map generated at `iar/Debug_m7_1/List/cyt4bb7_cm_7_1.map`; ranges were not part of Gate A and were not reviewed. |
| Integrated three-core map ranges checked for overlap | NOT RUN | Outside Gate A. |

## Timing

| Result | Status | Evidence / observation |
| --- | --- | --- |
| Camera initialization timing recorded | NOT RUN | Initialization completed, but elapsed initialization time was not measured. |
| Live Watch camera-buffer observation held for 60 seconds | PASS | UTC `2026-07-14T02:43:05.588Z` to `2026-07-14T02:44:23.976Z` (78.388 seconds). |
| UART Assistant image observation held for 60 seconds | PASS | UTC `2026-07-14T03:09:20.250Z` to `2026-07-14T03:10:35.250Z` (75.000 seconds), continuously connected on COM6 with 188 x 120 live imagery and approximately 11.4 kB/s receive rate. |
| WiFi-SPI Assistant image observation held for 60 seconds | PASS | Local `2026-07-13 21:21:41 -07:00` to at least `2026-07-13 21:26:14.752 -07:00` (at least 273 seconds), continuously connected over TCP with changing 188 x 120 imagery. A 10.175-second counter window measured 6.49 FPS; Assistant reported 6-7 FPS and approximately 170-180 kB/s. |
| Integrated scheduler/control timing impact checked | NOT RUN | Outside Gate A. |

## Final disposition

| Gate | Status | Disposition |
| --- | --- | --- |
| Gate A — untouched E9_01 supplied-project build plus 60-second debug-UART camera proof | PASS | Both supplied E9_01 projects build with 0 errors/0 warnings, the untouched CM7_0 reference downloads and runs, Assistant displays changing 188 x 120 imagery for 75.000 seconds, the operator-confirmed hand/object movement changes the scene and both watched samples, wheel motor power was operator-confirmed OFF, and camera wiring was operator-confirmed against the planned pin map. Individual continuity-meter measurements and the conditional all-90-degree leg-pose observation remain accurately `NOT RUN` and are not claimed. |
| Gate B — existing Assistant camera packet over WiFi-SPI in an isolated E9_01 copy | PASS | The copied CM7_0 project built with 0 errors/0 warnings; WiFi init and TCP socket connect succeeded on a 2.4 GHz Windows hotspot; Assistant displayed changing 188 x 120 frames continuously for at least 273 seconds; and the 100 ms no-queue gate delivered 6-7 FPS at about 170-180 kB/s without disconnect or protocol error. The measured 139 ms synchronous interval is recorded as a capture-loop stall/real-time risk, not hidden as a nominal 10 FPS result. |
| Overall camera integration evidence | NOT RUN | Gates A and B reference proofs are PASS. CM7_1 portability and integrated three-core robot gates remain `NOT RUN`; no robot real-time-safety claim is made. |
