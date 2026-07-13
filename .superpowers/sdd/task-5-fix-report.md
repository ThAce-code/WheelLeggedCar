# Task 5 EWP duplicate-source fix report

## Status

Removed the obsolete nested `intercore_notify` groups from the CM7_0 and CM7_1 IAR project files. The Task 5 `intercore` groups remain the single source of `intercore_notify.c/.h` and `intercore_notify_port.c/.h`; CM7_0 retains the router and router-port files, while CM7_1 has none.

## Static verification

- Baseline checker against `HEAD` failed as expected: each CM7 EWP listed the four notify paths twice (`x2`).
- Post-fix duplicate-path checker passed for both EWP files: no duplicate source paths.
- PowerShell `[xml]` parsing passed for both files. Each required inter-core source path has count `1`; CM7_0 router/router-port paths each have count `1`; CM7_1 router/router-port paths each have count `0`.
- `git diff --check` passed.

No files under `libraries` were changed. IAR builds were not run because `iarbuild.exe` is unavailable in this environment.
