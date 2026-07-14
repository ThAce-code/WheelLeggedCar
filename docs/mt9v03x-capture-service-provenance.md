# MT9V03X CM0+ capture-service image

## Artifact identity

Repository artifact:

```text
project/iar/project_config/mt9v03x_cm0plus_capture_service.ewx
```

The file is an unchanged vendor Intel HEX image. It is 56,964 file bytes and its SHA-256 is:

```text
508CD93B33730384804E55794C3A11819E904740B3EC60519318B989DCF6A299
```

It was copied byte-for-byte from the source-matched Seekfree CYT4BB7 examples supplied with the vendor library. These two authoritative sources were independently hashed and are identical:

```text
D:\smartcar\CYT4BB7_Library\Example\Motherboard_Demo\E8_camera\E8_09_mt9v03x_uart_seekfree_assistant_cross_ram_m7_1_demo\iar\project_config\cyt4bb7_cm_7_0.ewx
D:\smartcar\CYT4BB7_Library\Example\Motherboard_Demo\E9_seekfree_assistant\E9_01_seekfree_assistant_mt9v03x_demo\iar\project_config\cyt4bb7_cm_7_0.ewx
```

The vendor library's GPL3 licensing and permission statement remain in `libraries/LICENSE` and `libraries/doc/GPL3_permission_statement.txt`. This import adds only the exact camera-support image required by the source-matched vendor driver; it does not alter the image or expand it into unrelated firmware functionality.

## Address contract

Decoded Intel HEX payload:

- Payload bytes: 20,234
- Inclusive code-flash range: `0x10000000-0x10004F09`
- Start linear address: `0x10004C09`
- CM7_0 code flash remains at `0x10080000` and above.
- CM7_1 code flash remains at `0x10280000` and above.

The image is wholly inside the CM0+ code-flash reservation and does not overlap either M7 image. It contains the required little-endian camera-contract constants:

| Contract | Value | Reference image location |
| --- | --- | --- |
| Height word | `0x28006BF0` | `0x10003758` |
| Width word | `0x28006BF2` | `0x10003754` |
| Temporary pixel buffer | `0x28026024` | `0x10003798` |
| TCPWM59 register block | `0x40581D80` | `0x1000378C` |

The repository static check rejects a hash, payload range, entry point, fixed constant, or loader configuration change.

## Required IAR launch workflow

Camera runtime must be launched with **active CM7_0** using `Download and Debug`. The CM7_0 debugger configuration loads:

1. `mt9v03x_cm0plus_capture_service.ewx` as enabled extra image 1; and
2. the current `$PROJ_DIR$\..\Debug_m7_1\Exe\cyt4bb7_cm_7_1.hex` as enabled extra image 2 (`$PROJ_DIR$` is `project/iar/project_config`).

Do not separately download the generic CM0+ project after this operation. The generic CM0+ image occupies the same `0x10000000` flash region and replaces the camera capture service; its source only initializes the system, starts the M7 cores, drives the board LED, and idles.

## Causal hardware evidence

Task 2 Gate 1 previously ran 118.600 seconds with camera initialization success but zero finish callbacks, zero frames, and zero pixels after the generic CM0+ workflow. A controlled one-variable run using this capture-service image plus the same current CM7_0/CM7_1 firmware produced 11,506 frames over 232.082 wall-clock seconds (about 49.58 FPS), with changing sampled pixels. This distinguishes the missing/overwritten capture service from the unchanged M7 camera code as the cause of the zero-frame failure.
