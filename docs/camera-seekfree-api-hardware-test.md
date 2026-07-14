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
- WiFi module:
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

| Result | Status | Evidence / observation |
| --- | --- | --- |
| WiFi-SPI path exercised | NOT RUN | Outside Gate A unless explicitly tested later. |

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
| Integrated scheduler/control timing impact checked | NOT RUN | Outside Gate A. |

## Final disposition

| Gate | Status | Disposition |
| --- | --- | --- |
| Gate A — untouched E9_01 supplied-project build plus 60-second debug-UART camera proof | PASS | Both supplied E9_01 projects build with 0 errors/0 warnings, the untouched CM7_0 reference downloads and runs, Assistant displays changing 188 x 120 imagery for 75.000 seconds, the operator-confirmed hand/object movement changes the scene and both watched samples, wheel motor power was operator-confirmed OFF, and camera wiring was operator-confirmed against the planned pin map. Individual continuity-meter measurements and the conditional all-90-degree leg-pose observation remain accurately `NOT RUN` and are not claimed. |
| Overall camera integration evidence | NOT RUN | Gate A reference proof is PASS. WiFi-SPI transport, CM7_1 portability, and integrated three-core gates remain outside Task 1 and are still `NOT RUN`. |
