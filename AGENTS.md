# Repository Guidelines

## Project Structure & Module Organization

This is an embedded C project for a CYT4BB/Traveo II target using IAR Embedded Workbench. Application code lives in `project/code`, with `.c`/`.h` pairs such as `app.c`, `motor.c`, `sensor_imu.c`, and `telemetry.c`. Core entry points and interrupt files are in `project/user` (`main_cm7_0.c`, `main_cm7_1.c`, `main_cm0plus.c`, `*_isr.c`). IAR workspace files are under `project/iar`; per-core project files are in `project/iar/project_config`. Vendor, SDK, driver, and device support code is under `libraries`; avoid changing it unless the task is a platform or driver update.

Generated IAR outputs appear under directories such as `project/iar/Debug_m0_plus` and `project/iar/Debug_m7_0`. Treat these as disposable, not source.

## Build, Test, and Development Commands

- Open `project/iar/cyt4bb7.eww` in IAR Embedded Workbench 9.40.1 or compatible, then build the needed core projects: `cyt4bb7_cm_0_plus`, `cyt4bb7_cm_7_0`, `cyt4bb7_cm_7_1`.
- From `project/iar`, run `.\删除临时文件IAR.bat` to remove generated IAR output and temporary debug/config files before packaging or sharing a clean tree.
- Use `rg "symbol_name" project/code project/user libraries/zf_*` for fast source lookup.

## Coding Style & Naming Conventions

Follow the existing C style: 4-space indentation, function braces on a new line, project typedefs such as `uint8` and `uint32`, and `0U`/`1U` suffixes for unsigned constants. Keep modules lowercase with underscores where needed (`app_scheduler.c`, `board_gpio.h`). Header guards use lowercase names wrapped with underscores, for example `_app_config_h_`; match local style when adding headers.

Prefer short comments that explain hardware assumptions, timing, or safety behavior. Keep public APIs in the matching header and core-specific code in `project/user`.

## Testing Guidelines

There is no standalone automated test framework. Validate changes by building all affected IAR projects and running a hardware smoke test on the target board. For firmware changes, record the tested core, board behavior, and relevant peripherals, for example: IMU init result, LED heartbeat, motor output disabled/enabled, UART telemetry, and fault behavior.

## Commit & Pull Request Guidelines

This checkout does not include Git history, so no project-specific convention can be inferred. Use concise imperative subjects such as `Add IMU telemetry timeout handling` or `Fix CM7_0 heartbeat timing`. Pull requests should describe changed modules, target core(s), hardware tested, build result, and risks to motor control, sensor timing, or pin assignments.

## Safety & Configuration Tips

Do not change pin mappings, scheduler periods, or motor-control limits casually. Configuration constants are centralized in `project/code/app_config.h` and `project/code/bldc_config.h`; document hardware-dependent changes in the PR description.
