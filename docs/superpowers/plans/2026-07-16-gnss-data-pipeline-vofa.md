# TAU1201 GNSS Data Pipeline and VOFA+ Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a testable TAU1201 data path from CM7_1 UART2 through validated local coordinates and versioned inter-core transport to a fixed-layout CM7_0 VOFA+ telemetry profile.

**Architecture:** CM7_1 owns TAU1201 reception and converts WGS-84 fixes to a start-relative local plane. A dedicated double-buffered GNSS record reuses 512 bytes of the currently unused inter-core event-ring allocation without changing the 8 KiB shared layout. CM7_0 caches the newest valid record and exposes it through a 50 Hz GNSS VOFA+ profile; UART0 remains CM7_0-only and no raw NMEA bytes enter the JustFloat stream.

**Tech Stack:** Embedded C11-style C for CYT4BB7/IAR 9.40.1, SEEKFREE UART/GNSS library, host GCC tests through PowerShell, IAR command-line builds, VOFA+ JustFloat.

## Global Constraints

- TAU1201 uses `UART_2`, `UART2_TX_P10_1`, `UART2_RX_P10_0`, 115200 baud, and 10 Hz updates.
- UART0 remains owned by CM7_0 for VOFA+ and maintenance commands; CM7_1 must not initialize or transmit through UART0.
- GPS data cannot directly enable motion in this phase.
- Runtime control and VOFA+ use local meter coordinates; absolute latitude and longitude remain `double` and never become a single `float`.
- GNSS parsing runs in the CM7_1 main loop; the UART2 ISR only drains bytes through `gnss_uart_callback()`.
- Shared-memory records contain no pointers, remain fixed-size, use CRC, and preserve `sizeof(intercore_shared_layout_struct) == 8192U`.
- A GNSS fix is usable only when age is at most 300 ms, satellite count is at least 8, HDOP is at most 2.0, and both RMC and GGA quality are valid.
- `position_sigma_m` is `-1.0F` until a site-specific error model is calibrated; HDOP is dimensionless and must not be reported as meters.
- Recovery requires at least 3 consecutive usable fixes; use 5 as the initial implementation value.
- Preserve the current untracked `tmp/` directory and unrelated user changes.

## Scope Split

This plan implements only Phase 1 of the approved design. Later plans remain separate so every phase produces independently testable firmware:

1. **This plan:** TAU1201 parsing, local coordinates, inter-core GNSS snapshot, VOFA+.
2. **Later plan:** white-frame calibration and offline/live vision.
3. **Later plan:** one-mine approach, centering, 750-degree rotation, and boundary safety.
4. **Later plan:** fixed multi-mine map, route state machine, turn zone, and return.

## File Structure

### New files

- `project/code/gnss_types.h`: pointer-free GNSS application contracts shared by CM7_1 logic and tests.
- `project/code/local_position.c/.h`: WGS-84 origin and local east/north projection; no hardware dependencies.
- `project/code/sensor_gnss.c/.h`: TAU1201 lifecycle, parse service, quality gate, origin capture, and immutable snapshot.
- `project/tests/gnss_local_position_test.c`: host numeric projection tests.
- `project/tests/gnss_intercore_test.c`: host GNSS double-buffer, CRC, duplicate, and epoch tests.
- `tools/test_gnss_local_position_host.ps1`: GCC runner for projection tests.
- `tools/test_gnss_intercore_host.ps1`: GCC runner for transport tests.
- `tools/test_intercore_control_foundation.ps1`: GCC runner for the existing control-foundation regression test.
- `tools/test_gnss_driver_static.ps1`: driver field and parser wiring assertions.
- `tools/test_gnss_integration_static.ps1`: core ownership, main-loop parsing, IAR membership, and telemetry-layout assertions.

### Modified files

- `libraries/zf_device/zf_device_gnss.h`: add GGA quality fields and make the ISR-to-main flag volatile.
- `libraries/zf_device/zf_device_gnss.c`: parse GGA Fix Quality and HDOP and reset public state at initialization.
- `project/code/intercore_protocol.h`: add compact GNSS payload/slots, sequence metadata, and protocol version 4.
- `project/code/intercore_transport.c/.h`: publish/read GNSS records.
- `project/code/intercore_control.c/.h`: cache the newest CM7_1 GNSS snapshot for CM7_0 consumers.
- `project/code/app_config.h`: define fixed telemetry profiles and select the GNSS profile for bring-up.
- `project/code/telemetry.c`: emit the fixed 20-float GNSS profile without changing UART ownership.
- `project/user/main_cm7_1.c`: initialize and service GNSS outside the ISR and publish new snapshots.
- `project/iar/project_config/cyt4bb7_cm_7_1.ewp`: add the new CM7_1 GNSS modules.
- `project/tests/intercore_control_foundation_test.c`: keep existing navigation tests compatible with protocol version 4.

---

### Task 1: Extend the SEEKFREE GGA Parser with Quality Fields

**Files:**
- Create: `tools/test_gnss_driver_static.ps1`
- Modify: `libraries/zf_device/zf_device_gnss.h:74-106`
- Modify: `libraries/zf_device/zf_device_gnss.c:56-69,294-309,557-617`

**Interfaces:**
- Consumes: existing `gnss_info_struct`, `get_parameter_index()`, and `get_float_number()`.
- Produces: `gnss.fix_quality`, `gnss.hdop`, and `volatile gnss_flag` for `sensor_gnss` in Task 3.

- [ ] **Step 1: Write the failing static contract test**

Create `tools/test_gnss_driver_static.ps1`:

```powershell
$ErrorActionPreference = 'Stop'
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$header = Get-Content -Raw -LiteralPath (Join-Path $repoRoot 'libraries\zf_device\zf_device_gnss.h')
$source = Get-Content -Raw -LiteralPath (Join-Path $repoRoot 'libraries\zf_device\zf_device_gnss.c')
$failures = [System.Collections.Generic.List[string]]::new()

function Assert-Match([string]$Text, [string]$Pattern, [string]$Message)
{
    if($Text -notmatch $Pattern) { $script:failures.Add($Message) }
}

Assert-Match $header 'uint8\s+fix_quality\s*;' 'gnss_info_struct lacks GGA fix_quality'
Assert-Match $header 'float\s+hdop\s*;' 'gnss_info_struct lacks GGA hdop'
Assert-Match $header 'extern\s+volatile\s+uint8\s+gnss_flag\s*;' 'gnss_flag is not volatile across ISR/main'
Assert-Match $source 'get_parameter_index\(6,\s*buf\)' 'GGA field 6 Fix Quality is not parsed'
Assert-Match $source 'get_parameter_index\(8,\s*buf\)' 'GGA field 8 HDOP is not parsed'
Assert-Match $source 'memset\(&gnss,\s*0,\s*sizeof\(gnss\)\)' 'gnss public state is not reset at init'
Assert-Match $source 'return_state\s*=\s*1U?\s*;\s*\}\s*else\s*\{\s*gps_gnrmc_parse' 'bad RMC checksum can leave parsing stuck'
Assert-Match $source 'return_state\s*=\s*1U?\s*;\s*\}\s*else\s*\{\s*gps_gngga_parse' 'bad GGA checksum can leave parsing stuck'

if(0 -ne $failures.Count)
{
    $failures | ForEach-Object { Write-Host "FAIL: $_" -ForegroundColor Red }
    exit 1
}
Write-Host 'PASS: TAU1201 GGA quality parser contract'
```

- [ ] **Step 2: Run the test and verify that it fails**

Run:

```powershell
& .\tools\test_gnss_driver_static.ps1
```

Expected: FAIL for missing `fix_quality`, `hdop`, volatile flag, GGA fields 6/8, GNSS state reset, and checksum-error recovery.

- [ ] **Step 3: Add the quality fields and volatile flag**

In `zf_device_gnss.h`, place these fields after `state` and update the extern:

```c
    uint8       state;
    uint8       fix_quality;
    float       hdop;

extern gnss_info_struct gnss;
extern volatile uint8   gnss_flag;
```

In `zf_device_gnss.c`, update the definition and GGA parser:

```c
volatile uint8 gnss_flag = 0U;

static uint8 gps_gngga_parse(char *line, gnss_info_struct *gnss)
{
    char *buf = line;
    uint8 fix_quality;

    fix_quality = (uint8)get_int_number(&buf[get_parameter_index(6, buf)]);
    gnss->fix_quality = fix_quality;
    gnss->satellite_used = (uint8)get_int_number(
        &buf[get_parameter_index(7, buf)]);
    gnss->hdop = get_float_number(&buf[get_parameter_index(8, buf)]);
    gnss->height = get_float_number(&buf[get_parameter_index(9, buf)]) +
                   get_float_number(&buf[get_parameter_index(11, buf)]);
    return (0U != fix_quality) ? 1U : 0U;
}
```

At the beginning of `gnss_init()` add:

```c
    memset(&gnss, 0, sizeof(gnss));
    gnss_flag = 0U;
```

In `gnss_data_parse()`, replace the RMC and GGA checksum branches so a bad sentence is rejected but the receive state is always released. Apply this exact shape to both sentences:

```c
            if(bbc_xor_calculation != bbc_xor_origin)
            {
                return_state = 1U;
            }
            else
            {
                gps_gnrmc_parse((char *)gps_rmc_buffer, &gnss);
            }
        }
        gnss_rmc_state = GPS_STATE_RECEIVING;
```

and:

```c
            if(bbc_xor_calculation != bbc_xor_origin)
            {
                return_state = 1U;
            }
            else
            {
                gps_gngga_parse((char *)gps_gga_buffer, &gnss);
            }
        }
        gnss_gga_state = GPS_STATE_RECEIVING;
```

Do not `break` or return from either checksum-error branch. This lets the other buffered sentence parse in the same service call and prevents one corrupt sentence from permanently leaving its state at `GPS_STATE_PARSING`.

- [ ] **Step 4: Run the contract test**

Run:

```powershell
& .\tools\test_gnss_driver_static.ps1
```

Expected: `PASS: TAU1201 GGA quality parser contract`.

- [ ] **Step 5: Commit the driver extension**

```powershell
git add tools/test_gnss_driver_static.ps1 libraries/zf_device/zf_device_gnss.c libraries/zf_device/zf_device_gnss.h
git commit -m "Expose GNSS fix quality and HDOP"
```

---

### Task 2: Add Host-Tested Local Coordinate Projection

**Files:**
- Create: `project/code/gnss_types.h`
- Create: `project/code/local_position.c`
- Create: `project/code/local_position.h`
- Create: `project/tests/gnss_local_position_test.c`
- Create: `tools/test_gnss_local_position_host.ps1`

**Interfaces:**
- Consumes: WGS-84 latitude/longitude in degrees.
- Produces: `local_position_set_origin(double, double)`, `local_position_project(double, double, float *, float *)`, and `gnss_snapshot_struct`.

- [ ] **Step 1: Write the failing numeric host test**

Create `project/tests/gnss_local_position_test.c`:

```c
#include <math.h>
#include <stdio.h>
#include "local_position.h"

static unsigned failures;
#define CHECK_NEAR(actual, expected, tolerance) do { \
    if(fabs((double)(actual) - (double)(expected)) > (tolerance)) { \
        printf("FAIL:%d actual=%.9f expected=%.9f\n", __LINE__, \
               (double)(actual), (double)(expected)); failures++; \
    } \
}while(0)

int main(void)
{
    float east_m = 0.0F;
    float north_m = 0.0F;

    local_position_reset();
    if(0U != local_position_project(30.0, 120.0, &east_m, &north_m)) failures++;
    if(0U == local_position_set_origin(30.0, 120.0)) failures++;
    if(0U == local_position_project(30.0, 120.0, &east_m, &north_m)) failures++;
    CHECK_NEAR(east_m, 0.0, 0.001);
    CHECK_NEAR(north_m, 0.0, 0.001);

    local_position_project(30.00001, 120.0, &east_m, &north_m);
    CHECK_NEAR(north_m, 1.11195, 0.01);
    local_position_project(30.0, 120.00001, &east_m, &north_m);
    CHECK_NEAR(east_m, 0.96298, 0.01);

    if(0U != failures) return 1;
    puts("gnss_local_position_test: PASS");
    return 0;
}
```

Create `tools/test_gnss_local_position_host.ps1`:

```powershell
$ErrorActionPreference = 'Stop'
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$binary = Join-Path $env:TEMP 'gnss_local_position_test.exe'
try
{
    $env:PATH = "C:\msys64\ucrt64\bin;$env:PATH"
    & gcc -std=c11 -Wall -Wextra -Werror `
        -I (Join-Path $repoRoot 'project\code') `
        -I (Join-Path $repoRoot 'libraries\zf_common') `
        (Join-Path $repoRoot 'project\tests\gnss_local_position_test.c') `
        (Join-Path $repoRoot 'project\code\local_position.c') `
        -lm -o $binary
    if($LASTEXITCODE -ne 0) { throw 'host compile failed' }
    & $binary
    if($LASTEXITCODE -ne 0) { throw 'host test failed' }
}
finally
{
    Remove-Item -LiteralPath $binary -Force -ErrorAction SilentlyContinue
}
```

- [ ] **Step 2: Run the host test and verify that it fails**

```powershell
& .\tools\test_gnss_local_position_host.ps1
```

Expected: GCC fails because `local_position.h` and `local_position.c` do not exist.

- [ ] **Step 3: Define the GNSS application contract**

Create `project/code/gnss_types.h`:

```c
#ifndef _gnss_types_h_
#define _gnss_types_h_

#include "zf_common_typedef.h"

#define GNSS_SNAPSHOT_MAX_AGE_MS (300U)

typedef struct
{
    uint32 sequence;
    uint32 timestamp_ms;
    double latitude_deg;
    double longitude_deg;
    float local_x_m;
    float local_y_m;
    float speed_mps;
    float course_deg;
    float hdop;
    float position_sigma_m;
    uint32 checksum_error_count;
    uint32 timeout_count;
    uint16 satellite_count;
    uint8 fix_valid;
    uint8 fix_quality;
    uint8 origin_valid;
    uint8 reserved;
}gnss_snapshot_struct;

#endif
```

- [ ] **Step 4: Implement the projection**

Create `project/code/local_position.h`:

```c
#ifndef _local_position_h_
#define _local_position_h_

#include "zf_common_typedef.h"

void local_position_reset(void);
uint8 local_position_set_origin(double latitude_deg, double longitude_deg);
uint8 local_position_has_origin(void);
uint8 local_position_project(double latitude_deg, double longitude_deg,
                             float *east_m, float *north_m);

#endif
```

Create `project/code/local_position.c`:

```c
#include "local_position.h"
#include <math.h>
#include <stddef.h>

#define LOCAL_POSITION_PI             (3.14159265358979323846)
#define LOCAL_POSITION_EARTH_RADIUS_M (6378137.0)

static double origin_latitude_rad;
static double origin_longitude_rad;
static double origin_cos_latitude;
static uint8 origin_valid;

void local_position_reset(void)
{
    origin_latitude_rad = 0.0;
    origin_longitude_rad = 0.0;
    origin_cos_latitude = 1.0;
    origin_valid = 0U;
}

uint8 local_position_set_origin(double latitude_deg, double longitude_deg)
{
    if((latitude_deg < -90.0) || (latitude_deg > 90.0) ||
       (longitude_deg < -180.0) || (longitude_deg > 180.0)) return 0U;
    origin_latitude_rad = latitude_deg * LOCAL_POSITION_PI / 180.0;
    origin_longitude_rad = longitude_deg * LOCAL_POSITION_PI / 180.0;
    origin_cos_latitude = cos(origin_latitude_rad);
    origin_valid = 1U;
    return 1U;
}

uint8 local_position_has_origin(void) { return origin_valid; }

uint8 local_position_project(double latitude_deg, double longitude_deg,
                             float *east_m, float *north_m)
{
    double latitude_rad;
    double longitude_rad;
    if((0U == origin_valid) || (NULL == east_m) || (NULL == north_m)) return 0U;
    latitude_rad = latitude_deg * LOCAL_POSITION_PI / 180.0;
    longitude_rad = longitude_deg * LOCAL_POSITION_PI / 180.0;
    *east_m = (float)((longitude_rad - origin_longitude_rad) *
                      origin_cos_latitude * LOCAL_POSITION_EARTH_RADIUS_M);
    *north_m = (float)((latitude_rad - origin_latitude_rad) *
                       LOCAL_POSITION_EARTH_RADIUS_M);
    return 1U;
}
```

- [ ] **Step 5: Run the numeric host test**

```powershell
& .\tools\test_gnss_local_position_host.ps1
```

Expected: `gnss_local_position_test: PASS`.

- [ ] **Step 6: Commit the projection unit**

```powershell
git add project/code/gnss_types.h project/code/local_position.c project/code/local_position.h project/tests/gnss_local_position_test.c tools/test_gnss_local_position_host.ps1
git commit -m "Add GNSS local coordinate projection"
```

---

### Task 3: Wrap TAU1201 as an Immutable CM7_1 Sensor

**Files:**
- Create: `project/code/sensor_gnss.c`
- Create: `project/code/sensor_gnss.h`
- Create: `tools/test_gnss_integration_static.ps1`

**Interfaces:**
- Consumes: `gnss`, `gnss_flag`, `gnss_init(TAU1201)`, `gnss_data_parse()`, and Task 2 projection.
- Produces: `sensor_gnss_init()`, `sensor_gnss_service(uint32)`, `sensor_gnss_take_snapshot()`, and `sensor_gnss_set_origin()`.

- [ ] **Step 1: Write the failing integration ownership test**

Create `tools/test_gnss_integration_static.ps1` with these exact assertions:

```powershell
$ErrorActionPreference = 'Stop'
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$sensorPath = Join-Path $repoRoot 'project\code\sensor_gnss.c'
$isrPath = Join-Path $repoRoot 'project\user\cm7_1_isr.c'
$failures = [System.Collections.Generic.List[string]]::new()
function Assert-True([bool]$Condition, [string]$Message)
{
    if(-not $Condition) { $script:failures.Add($Message) }
}

Assert-True (Test-Path -LiteralPath $sensorPath) 'sensor_gnss.c is missing'
if(Test-Path -LiteralPath $sensorPath)
{
    $sensor = Get-Content -Raw -LiteralPath $sensorPath
    Assert-True ($sensor -match 'gnss_init\(TAU1201\)') 'TAU1201 is not initialized'
    Assert-True ($sensor -match 'gnss_data_parse\(\)') 'GNSS parsing is not serviced'
    Assert-True ($sensor -notmatch 'UART_0|debug_info_init') 'sensor_gnss claims UART0'
}
$isr = Get-Content -Raw -LiteralPath $isrPath
Assert-True ($isr -match 'gnss_uart_callback\(\)') 'UART2 ISR no longer feeds GNSS bytes'
Assert-True ($isr -notmatch 'gnss_data_parse\(\)') 'GNSS parsing incorrectly runs in ISR'
if(0 -ne $failures.Count) { $failures | ForEach-Object { "FAIL: $_" }; exit 1 }
'PASS: GNSS integration contract'
```

- [ ] **Step 2: Run the static test and verify that it fails**

```powershell
& .\tools\test_gnss_integration_static.ps1
```

Expected: FAIL because `sensor_gnss.c` does not exist yet.

- [ ] **Step 3: Define the sensor interface and quality constants**

Create `project/code/sensor_gnss.h`:

```c
#ifndef _sensor_gnss_h_
#define _sensor_gnss_h_

#include "gnss_types.h"

#define SENSOR_GNSS_MIN_SATELLITES           (8U)
#define SENSOR_GNSS_MAX_HDOP                 (2.0F)
#define SENSOR_GNSS_RECOVERY_FIX_COUNT       (5U)
#define SENSOR_GNSS_AUTO_ORIGIN_SAMPLE_COUNT (50U)

uint8 sensor_gnss_init(void);
void sensor_gnss_service(uint32 now_ms);
uint8 sensor_gnss_take_snapshot(gnss_snapshot_struct *snapshot);
uint8 sensor_gnss_set_origin(double latitude_deg, double longitude_deg);

#endif
```

- [ ] **Step 4: Implement nonblocking service and stable boot origin**

Create `project/code/sensor_gnss.c` with these state transitions:

```c
#include "sensor_gnss.h"
#include "local_position.h"
#include "zf_device_gnss.h"
#include <string.h>

static gnss_snapshot_struct latest;
static uint32 publish_sequence;
static uint32 checksum_error_count;
static uint32 timeout_count;
static uint32 last_sample_ms;
static uint8 new_snapshot;
static uint8 consecutive_usable;
static uint8 timeout_latched;
static uint16 origin_sample_count;
static double origin_latitude_sum;
static double origin_longitude_sum;

static uint8 sample_is_usable(void)
{
    return ((0U != gnss.state) && (0U != gnss.fix_quality) &&
            (SENSOR_GNSS_MIN_SATELLITES <= gnss.satellite_used) &&
            (0.0F < gnss.hdop) && (SENSOR_GNSS_MAX_HDOP >= gnss.hdop)) ? 1U : 0U;
}

uint8 sensor_gnss_init(void)
{
    memset(&latest, 0, sizeof(latest));
    publish_sequence = 0U;
    checksum_error_count = 0U;
    timeout_count = 0U;
    last_sample_ms = 0U;
    new_snapshot = 0U;
    consecutive_usable = 0U;
    timeout_latched = 0U;
    origin_sample_count = 0U;
    origin_latitude_sum = 0.0;
    origin_longitude_sum = 0.0;
    local_position_reset();
    gnss_init(TAU1201);
    return 1U;
}

uint8 sensor_gnss_set_origin(double latitude_deg, double longitude_deg)
{
    origin_sample_count = 0U;
    origin_latitude_sum = 0.0;
    origin_longitude_sum = 0.0;
    return local_position_set_origin(latitude_deg, longitude_deg);
}

void sensor_gnss_service(uint32 now_ms)
{
    uint8 usable;
    if((0U != last_sample_ms) &&
       (GNSS_SNAPSHOT_MAX_AGE_MS < (now_ms - last_sample_ms)) &&
       (0U == timeout_latched))
    {
        timeout_count++;
        timeout_latched = 1U;
    }
    if(0U == gnss_flag) return;
    gnss_flag = 0U;
    if(0U != gnss_data_parse()) { checksum_error_count++; return; }
    last_sample_ms = now_ms;
    timeout_latched = 0U;

    usable = sample_is_usable();
    consecutive_usable = usable ? (uint8)(consecutive_usable +
        ((consecutive_usable < SENSOR_GNSS_RECOVERY_FIX_COUNT) ? 1U : 0U)) : 0U;

    if((0U == local_position_has_origin()) && (0U != usable))
    {
        origin_latitude_sum += gnss.latitude;
        origin_longitude_sum += gnss.longitude;
        origin_sample_count++;
        if(SENSOR_GNSS_AUTO_ORIGIN_SAMPLE_COUNT <= origin_sample_count)
        {
            local_position_set_origin(
                origin_latitude_sum / (double)origin_sample_count,
                origin_longitude_sum / (double)origin_sample_count);
        }
    }

    latest.sequence = ++publish_sequence;
    latest.timestamp_ms = now_ms;
    latest.latitude_deg = gnss.latitude;
    latest.longitude_deg = gnss.longitude;
    latest.speed_mps = gnss.speed / 3.6F;
    latest.course_deg = gnss.direction;
    latest.hdop = gnss.hdop;
    latest.position_sigma_m = -1.0F;
    latest.satellite_count = gnss.satellite_used;
    latest.fix_quality = gnss.fix_quality;
    latest.origin_valid = local_position_has_origin();
    latest.fix_valid = (SENSOR_GNSS_RECOVERY_FIX_COUNT <= consecutive_usable) ? 1U : 0U;
    latest.checksum_error_count = checksum_error_count;
    latest.timeout_count = timeout_count;
    if(0U == local_position_project(latest.latitude_deg, latest.longitude_deg,
                                    &latest.local_x_m, &latest.local_y_m))
    {
        latest.local_x_m = 0.0F;
        latest.local_y_m = 0.0F;
    }
    new_snapshot = 1U;
}

uint8 sensor_gnss_take_snapshot(gnss_snapshot_struct *snapshot)
{
    if((NULL == snapshot) || (0U == new_snapshot)) return 0U;
    memcpy(snapshot, &latest, sizeof(*snapshot));
    new_snapshot = 0U;
    return 1U;
}
```

Keep the sensor nonblocking after initialization. Do not add delay calls to `sensor_gnss_service()`.

- [ ] **Step 5: Run the sensor ownership test**

Run the test now:

```powershell
& .\tools\test_gnss_integration_static.ps1
```

Expected: `PASS: GNSS integration contract`.

- [ ] **Step 6: Commit the sensor wrapper**

```powershell
git add project/code/sensor_gnss.c project/code/sensor_gnss.h tools/test_gnss_integration_static.ps1
git commit -m "Wrap TAU1201 as a GNSS sensor"
```

---

### Task 4: Add a Versioned GNSS Inter-Core Snapshot

**Files:**
- Modify: `project/code/intercore_protocol.h:8-34,86-142,245-285`
- Modify: `project/code/intercore_transport.c`
- Modify: `project/code/intercore_transport.h`
- Create: `project/tests/gnss_intercore_test.c`
- Create: `tools/test_gnss_intercore_host.ps1`
- Create: `tools/test_intercore_control_foundation.ps1`
- Modify: `project/tests/intercore_control_foundation_test.c`

**Interfaces:**
- Consumes: Task 3 local-meter snapshot fields.
- Produces: `intercore_transport_publish_gnss()` and `intercore_transport_read_gnss()` with a 48-byte pointer-free payload.

- [ ] **Step 1: Write the failing GNSS transport test**

Create `project/tests/gnss_intercore_test.c` using the same `TEST_CHECK` macro as `perception_intercore_test.c`. The test body must include:

```c
static _Alignas(32) intercore_shared_layout_struct shared;
static intercore_transport_struct sender;
static intercore_transport_struct receiver;

static void test_gnss_round_trip_and_crc(void)
{
    intercore_gnss_payload_struct sent = {0};
    intercore_gnss_payload_struct received = {0};
    uint32 source_ms = 0U;
    uint32 record_sequence = 0U;

    memset(&shared, 0, sizeof(shared));
    TEST_CHECK(1U == intercore_transport_cm7_0_init(&receiver, &shared));
    TEST_CHECK(1U == intercore_transport_cm7_1_attach(&sender, &shared));
    sent.local_x_m = 1.25F;
    sent.local_y_m = -0.50F;
    sent.hdop = 0.85F;
    sent.satellite_count = 12U;
    sent.fix_valid = 1U;
    sent.origin_valid = 1U;

    TEST_CHECK(1U == intercore_transport_publish_gnss(&sender, &sent, 100U));
    TEST_CHECK(INTERCORE_TRANSPORT_OK == intercore_transport_read_gnss(
        &receiver, &received, &source_ms, &record_sequence));
    TEST_CHECK(1.25F == received.local_x_m);
    TEST_CHECK(-0.50F == received.local_y_m);
    TEST_CHECK(12U == received.satellite_count);
    TEST_CHECK(100U == source_ms);
    TEST_CHECK(INTERCORE_TRANSPORT_NO_DATA == intercore_transport_read_gnss(
        &receiver, &received, &source_ms, &record_sequence));

    receiver.last_gnss_sequence = 0U;
    shared.gnss[shared.metadata.gnss_active_index].payload.local_x_m += 1.0F;
    TEST_CHECK(INTERCORE_TRANSPORT_INVALID == intercore_transport_read_gnss(
        &receiver, &received, &source_ms, &record_sequence));
}
```

The test `main()` must also assert:

```c
TEST_CHECK(48U == sizeof(intercore_gnss_payload_struct));
TEST_CHECK(256U == sizeof(intercore_gnss_slot_struct));
TEST_CHECK(8192U == sizeof(intercore_shared_layout_struct));
TEST_CHECK(0x900U == offsetof(intercore_shared_layout_struct, gnss));
TEST_CHECK(0xD00U == offsetof(intercore_shared_layout_struct, perception));
```

Create `tools/test_gnss_intercore_host.ps1`:

```powershell
$ErrorActionPreference = 'Stop'
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$binary = Join-Path $env:TEMP 'gnss_intercore_host_test.exe'
try
{
    $env:PATH = "C:\msys64\ucrt64\bin;$env:PATH"
    & gcc -std=c11 -Wall -Wextra -Werror -DINTERCORE_HOST_TEST `
        -I (Join-Path $repoRoot 'project\code') `
        -I (Join-Path $repoRoot 'libraries\zf_common') `
        (Join-Path $repoRoot 'project\tests\gnss_intercore_test.c') `
        (Join-Path $repoRoot 'project\code\intercore_protocol.c') `
        (Join-Path $repoRoot 'project\code\intercore_transport.c') `
        -o $binary
    if($LASTEXITCODE -ne 0) { throw 'GNSS intercore host compile failed' }
    & $binary
    if($LASTEXITCODE -ne 0) { throw 'GNSS intercore host test failed' }
}
finally
{
    Remove-Item -LiteralPath $binary -Force -ErrorAction SilentlyContinue
}
```

In `gnss_intercore_test.c`, add one isolated fixture reset before each corruption case. Assert these exact results in addition to the round trip above:

- set `metadata.gnss_active_index = 2U` -> `INTERCORE_TRANSPORT_INVALID`;
- change the active slot header `type`, `version`, or `size` one at a time -> `INTERCORE_TRANSPORT_INVALID`;
- flip one payload byte after publication -> `INTERCORE_TRANSPORT_INVALID` and `health.crc_error_count == 1U`;
- read a successfully consumed sequence again -> `INTERCORE_TRANSPORT_NO_DATA`;
- seed both last-sequence fields, increment `metadata.boot_epoch` after attachment, and call either reader -> `INTERCORE_TRANSPORT_EPOCH_CHANGED`, then verify both `last_navigation_sequence == 0U` and `last_gnss_sequence == 0U`.

Create `tools/test_intercore_control_foundation.ps1`:

```powershell
$ErrorActionPreference = 'Stop'
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$binary = Join-Path $env:TEMP 'intercore_control_foundation_test.exe'
try
{
    $env:PATH = "C:\msys64\ucrt64\bin;$env:PATH"
    & gcc -std=c11 -Wall -Wextra -Werror -DINTERCORE_HOST_TEST `
        -I (Join-Path $repoRoot 'libraries\zf_common') `
        -I (Join-Path $repoRoot 'project\code') `
        (Join-Path $repoRoot 'project\tests\intercore_control_foundation_test.c') `
        (Join-Path $repoRoot 'project\code\intercore_protocol.c') `
        (Join-Path $repoRoot 'project\code\intercore_transport.c') `
        (Join-Path $repoRoot 'project\code\intercore_notify.c') `
        (Join-Path $repoRoot 'project\code\motion_command_router.c') `
        (Join-Path $repoRoot 'project\code\intercore_control.c') `
        -o $binary
    if($LASTEXITCODE -ne 0) { throw 'control-foundation host compile failed' }
    & $binary
    if($LASTEXITCODE -ne 0) { throw 'control-foundation host test failed' }
}
finally
{
    Remove-Item -LiteralPath $binary -Force -ErrorAction SilentlyContinue
}
```

- [ ] **Step 2: Run the transport test and verify that it fails**

```powershell
& .\tools\test_gnss_intercore_host.ps1
```

Expected: GCC reports missing GNSS payload, slots, fields, and APIs.

- [ ] **Step 3: Extend the protocol without changing the shared size**

In `intercore_protocol.h`:

```c
#define INTERCORE_PROTOCOL_VERSION (4U)
#define INTERCORE_EVENT_COUNT      (8U)

typedef enum
{
    INTERCORE_RECORD_NAVIGATION = 1,
    INTERCORE_RECORD_CONTROL_STATUS = 2,
    INTERCORE_RECORD_PERCEPTION_POSE = 3,
    INTERCORE_RECORD_PERCEPTION_SNAPSHOT = 4,
    INTERCORE_RECORD_GNSS = 5
}intercore_record_type_enum;

typedef struct
{
    float local_x_m;
    float local_y_m;
    float speed_mps;
    float course_deg;
    float hdop;
    float position_sigma_m;
    uint32 checksum_error_count;
    uint32 timeout_count;
    uint16 satellite_count;
    uint8 fix_valid;
    uint8 fix_quality;
    uint8 origin_valid;
    uint8 reserved0;
    uint8 reserved[10];
}intercore_gnss_payload_struct;

typedef struct
{
    intercore_header_struct header;
    intercore_gnss_payload_struct payload;
    uint8 reserved[184];
}intercore_gnss_slot_struct;
```

Replace 8 bytes of metadata reserve:

```c
    uint32 gnss_active_index;
    uint32 gnss_sequence;
    uint8 reserved[204];
```

Replace the 16-event allocation while keeping 1024 total bytes:

```c
    intercore_event_struct events[INTERCORE_EVENT_COUNT];
    intercore_gnss_slot_struct gnss[2];
```

Add checks:

```c
INTERCORE_LAYOUT_CHECK(gnss_payload_size,
                       sizeof(intercore_gnss_payload_struct) == 48U);
INTERCORE_LAYOUT_CHECK(gnss_slot_size,
                       sizeof(intercore_gnss_slot_struct) == 256U);
INTERCORE_LAYOUT_CHECK(gnss_offset,
                       offsetof(intercore_shared_layout_struct, gnss) == 0x900U);
INTERCORE_LAYOUT_CHECK(shared_size,
                       sizeof(intercore_shared_layout_struct) == 8192U);
```

- [ ] **Step 4: Implement publish/read APIs**

Add `last_gnss_sequence` to `intercore_transport_struct` and initialize/reset it beside `last_navigation_sequence`.

Because navigation and GNSS share one `boot_epoch`, add one helper and use it from both read APIs when the epoch changes:

```c
static void intercore_transport_accept_epoch(
    intercore_transport_struct *transport)
{
    transport->last_navigation_sequence = 0U;
    transport->last_gnss_sequence = 0U;
    transport->boot_epoch = transport->shared->metadata.boot_epoch;
}
```

Replace the current navigation epoch branch's individual assignments with `intercore_transport_accept_epoch(transport)`. This prevents whichever reader notices a CM7_0 reboot first from leaving the other channel's duplicate filter stale.

Add declarations:

```c
uint8 intercore_transport_publish_gnss(
    intercore_transport_struct *transport,
    const intercore_gnss_payload_struct *payload,
    uint32 source_ms);
intercore_transport_result_enum intercore_transport_read_gnss(
    intercore_transport_struct *transport,
    intercore_gnss_payload_struct *payload,
    uint32 *source_ms,
    uint32 *record_sequence);
```

Implement the publisher in `intercore_transport.c`:

```c
uint8 intercore_transport_publish_gnss(
    intercore_transport_struct *transport,
    const intercore_gnss_payload_struct *payload,
    uint32 source_ms)
{
    intercore_gnss_slot_struct local_slot = {0};
    uint32 inactive_index;

    if((NULL == transport) || (NULL == payload) ||
       (INTERCORE_ROLE_CM7_1 != transport->role) ||
       (0U == transport->attached) || (NULL == transport->shared) ||
       (0U == intercore_metadata_is_valid(transport->shared)) ||
       (transport->boot_epoch != transport->shared->metadata.boot_epoch))
    {
        return 0U;
    }

    intercore_record_prepare(
        &local_slot.header,
        INTERCORE_RECORD_GNSS,
        intercore_next_sequence(transport->shared->metadata.gnss_sequence),
        source_ms,
        payload,
        sizeof(*payload));
    memcpy(&local_slot.payload, payload, sizeof(*payload));

    inactive_index =
        (1U == transport->shared->metadata.gnss_active_index) ? 0U : 1U;
    intercore_copy_to_volatile(
        (volatile uint8 *)&transport->shared->gnss[inactive_index],
        (const uint8 *)&local_slot,
        sizeof(local_slot));
    INTERCORE_DMB();
    transport->shared->metadata.gnss_active_index = inactive_index;
    transport->shared->metadata.gnss_sequence = local_slot.header.sequence;
    INTERCORE_DMB();
    transport->shared->health.cm7_1_publish_count++;
    return 1U;
}
```

Implement the reader with the same stable double-read and epoch rules as navigation:

```c
intercore_transport_result_enum intercore_transport_read_gnss(
    intercore_transport_struct *transport,
    intercore_gnss_payload_struct *payload,
    uint32 *source_ms,
    uint32 *record_sequence)
{
    intercore_gnss_slot_struct local_slot;
    uint32 active_before;
    uint32 active_after;
    uint32 sequence_before;
    uint32 sequence_after;

    if((NULL == transport) || (NULL == payload) || (NULL == source_ms) ||
       (NULL == record_sequence) ||
       (INTERCORE_ROLE_CM7_0 != transport->role) ||
       (0U == transport->attached) || (NULL == transport->shared))
    {
        return INTERCORE_TRANSPORT_INVALID;
    }
    if(0U == intercore_metadata_is_valid(transport->shared))
    {
        transport->shared->health.version_error_count++;
        return INTERCORE_TRANSPORT_INVALID;
    }
    if(transport->boot_epoch != transport->shared->metadata.boot_epoch)
    {
        transport->shared->health.boot_epoch_change_count++;
        intercore_transport_accept_epoch(transport);
        return INTERCORE_TRANSPORT_EPOCH_CHANGED;
    }

    active_before = transport->shared->metadata.gnss_active_index;
    sequence_before = transport->shared->metadata.gnss_sequence;
    if(1U < active_before)
    {
        return INTERCORE_TRANSPORT_INVALID;
    }
    if((0U == sequence_before) ||
       (sequence_before == transport->last_gnss_sequence))
    {
        return INTERCORE_TRANSPORT_NO_DATA;
    }

    intercore_copy_from_volatile(
        (uint8 *)&local_slot,
        (const volatile uint8 *)&transport->shared->gnss[active_before],
        sizeof(local_slot));
    INTERCORE_DMB();
    active_after = transport->shared->metadata.gnss_active_index;
    sequence_after = transport->shared->metadata.gnss_sequence;
    if((active_before != active_after) || (sequence_before != sequence_after))
    {
        return INTERCORE_TRANSPORT_NO_DATA;
    }
    if(local_slot.header.sequence != sequence_before)
    {
        return INTERCORE_TRANSPORT_INVALID;
    }
    if((INTERCORE_PROTOCOL_MAGIC != local_slot.header.magic) ||
       (INTERCORE_PROTOCOL_VERSION != local_slot.header.version) ||
       ((uint16)INTERCORE_RECORD_GNSS != local_slot.header.type) ||
       (sizeof(local_slot.payload) != local_slot.header.size))
    {
        transport->shared->health.version_error_count++;
        return INTERCORE_TRANSPORT_INVALID;
    }
    if(0U == intercore_record_validate(&local_slot.header,
                                       INTERCORE_RECORD_GNSS,
                                       &local_slot.payload,
                                       sizeof(local_slot.payload)))
    {
        transport->shared->health.crc_error_count++;
        return INTERCORE_TRANSPORT_INVALID;
    }

    memcpy(payload, &local_slot.payload, sizeof(*payload));
    *source_ms = local_slot.header.source_ms;
    *record_sequence = local_slot.header.sequence;
    transport->last_gnss_sequence = local_slot.header.sequence;
    transport->shared->health.cm7_0_consume_count++;
    return INTERCORE_TRANSPORT_OK;
}
```

Initialize/reset `last_gnss_sequence` beside `last_navigation_sequence` in both attach paths. The host test covers invalid active indices, wrong type/size/version, CRC mismatch, duplicate sequence, and epoch changes across both channels.

- [ ] **Step 5: Update existing protocol-version fixtures**

In `intercore_control_foundation_test.c`, keep fixture initialization symbolic:

```c
shared.metadata.version = INTERCORE_PROTOCOL_VERSION;
shared.metadata.layout_size = (uint16)sizeof(shared);
```

Do not hard-code version 3 or the old event count anywhere.

- [ ] **Step 6: Run all inter-core host tests**

```powershell
& .\tools\test_gnss_intercore_host.ps1
& .\tools\test_perception_intercore_host.ps1
& .\tools\test_intercore_control_foundation.ps1
```

Expected: all three print `PASS`; perception remains at offset `0xD00` and shared size remains 8192.

- [ ] **Step 7: Commit the transport**

```powershell
git add project/code/intercore_protocol.h project/code/intercore_transport.c project/code/intercore_transport.h project/tests/gnss_intercore_test.c project/tests/intercore_control_foundation_test.c tools/test_gnss_intercore_host.ps1 tools/test_intercore_control_foundation.ps1
git commit -m "Add GNSS intercore snapshot transport"
```

---

### Task 5: Integrate GNSS into the CM7_1 Main Loop

**Files:**
- Modify: `project/user/main_cm7_1.c:45-68`
- Modify: `project/iar/project_config/cyt4bb7_cm_7_1.ewp:1116-1143`
- Test: `tools/test_gnss_integration_static.ps1`

**Interfaces:**
- Consumes: Task 3 sensor snapshots and Task 4 publish API.
- Produces: one compact inter-core GNSS record for every new parsed snapshot.

- [ ] **Step 1: Extend the integration test for main-loop and IAR membership**

Insert these checks immediately before the script's final `if(0 -ne $failures.Count)` block, then run it and expect failure:

```powershell
$main = Get-Content -Raw -LiteralPath (Join-Path $repoRoot 'project\user\main_cm7_1.c')
$ewp = Get-Content -Raw -LiteralPath (Join-Path $repoRoot 'project\iar\project_config\cyt4bb7_cm_7_1.ewp')
Assert-True ($main -match 'sensor_gnss_service') 'CM7_1 main loop does not service GNSS'
Assert-True ($main -match 'intercore_transport_cm7_1_attach') 'CM7_1 never attaches the GNSS publisher'
Assert-True ($main -match 'gnss_transport_attached\s*=\s*0U') 'GNSS publisher has no retry state'
Assert-True ($ewp -match 'code\\sensor_gnss\.c') 'CM7_1 IAR project omits sensor_gnss.c'
Assert-True ($ewp -match 'code\\local_position\.c') 'CM7_1 IAR project omits local_position.c'
```

- [ ] **Step 2: Add GNSS source files to the CM7_1 IAR project**

Inside the existing `<group><name>code</name>` block add:

```xml
<file><name>$PROJ_DIR$\..\..\code\gnss_types.h</name></file>
<file><name>$PROJ_DIR$\..\..\code\local_position.c</name></file>
<file><name>$PROJ_DIR$\..\..\code\local_position.h</name></file>
<file><name>$PROJ_DIR$\..\..\code\sensor_gnss.c</name></file>
<file><name>$PROJ_DIR$\..\..\code\sensor_gnss.h</name></file>
```

- [ ] **Step 3: Wire sensor service and publication into CM7_1**

Add includes and state to `main_cm7_1.c`:

```c
#include "sensor_gnss.h"
#include "intercore_transport.h"

static intercore_transport_struct gnss_transport;
static uint8 gnss_transport_attached;
```

After `camera_frame_consumer_init()` initialize the sensor but do not block waiting for CM7_0:

```c
    gnss_transport_attached = 0U;
    if(0U == sensor_gnss_init())
    {
        while(true) { }
    }
```

Declare the loop-local structures first in the `while(true)` body, then run camera and GNSS service:

```c
        gnss_snapshot_struct snapshot;
        intercore_gnss_payload_struct payload = {0};
        uint32 now_ms;

        camera_frame_consumer_service();
        now_ms = camera_frame_consumer_now_ms();
        if(0U == gnss_transport_attached)
        {
            gnss_transport_attached = intercore_transport_cm7_1_attach(
                &gnss_transport, intercore_memory_get_layout());
        }
        sensor_gnss_service(now_ms);
        if(0U != sensor_gnss_take_snapshot(&snapshot))
        {
            payload.local_x_m = snapshot.local_x_m;
            payload.local_y_m = snapshot.local_y_m;
            payload.speed_mps = snapshot.speed_mps;
            payload.course_deg = snapshot.course_deg;
            payload.hdop = snapshot.hdop;
            payload.position_sigma_m = snapshot.position_sigma_m;
            payload.checksum_error_count = snapshot.checksum_error_count;
            payload.timeout_count = snapshot.timeout_count;
            payload.satellite_count = snapshot.satellite_count;
            payload.fix_valid = snapshot.fix_valid;
            payload.fix_quality = snapshot.fix_quality;
            payload.origin_valid = snapshot.origin_valid;
            if((0U != gnss_transport_attached) &&
               (0U == intercore_transport_publish_gnss(
                           &gnss_transport, &payload, snapshot.timestamp_ms)))
            {
                gnss_transport_attached = 0U;
            }
        }
```

Do not leave the original `camera_frame_consumer_service()` call above this snippet; the loop must service the camera exactly once per iteration. The retry state is mandatory because CM7_1 may start before CM7_0 publishes valid metadata, and a later CM7_0 reboot changes the shared boot epoch.

- [ ] **Step 4: Run ownership and integration tests**

```powershell
& .\tools\test_gnss_driver_static.ps1
& .\tools\test_gnss_local_position_host.ps1
& .\tools\test_gnss_intercore_host.ps1
& .\tools\test_gnss_integration_static.ps1
```

Expected: all print `PASS`. Confirm `cm7_1_isr.c` contains only `gnss_uart_callback()` and never `gnss_data_parse()`.

- [ ] **Step 5: Build CM7_1**

```powershell
& 'D:\IAR\common\bin\IarBuild.exe' .\project\iar\project_config\cyt4bb7_cm_7_1.ewp -build Debug -log all
```

Expected: exit code 0 with no new warnings from `sensor_gnss`, `local_position`, protocol, or transport.

- [ ] **Step 6: Commit the complete CM7_1 sensor integration**

```powershell
git add project/user/main_cm7_1.c project/iar/project_config/cyt4bb7_cm_7_1.ewp tools/test_gnss_integration_static.ps1
git commit -m "Integrate TAU1201 on CM7_1"
```

---

### Task 6: Cache GNSS on CM7_0 and Add a Fixed VOFA+ Profile

**Files:**
- Modify: `project/code/intercore_control.c`
- Modify: `project/code/intercore_control.h`
- Modify: `project/code/app_config.h:24-30`
- Modify: `project/code/telemetry.c`
- Modify: `tools/test_gnss_integration_static.ps1`

**Interfaces:**
- Consumes: Task 4 `intercore_transport_read_gnss()`.
- Produces: `intercore_control_get_latest_gnss()` and a 20-float, 50 Hz VOFA+ GNSS profile.

- [ ] **Step 1: Extend the static test with fixed telemetry contracts**

Insert these checks immediately before the script's final `if(0 -ne $failures.Count)` block:

```powershell
$telemetry = Get-Content -Raw -LiteralPath (Join-Path $repoRoot 'project\code\telemetry.c')
$config = Get-Content -Raw -LiteralPath (Join-Path $repoRoot 'project\code\app_config.h')
Assert-True ($config -match 'APP_TELEMETRY_PROFILE_GNSS') 'GNSS telemetry profile is undefined'
Assert-True ($config -match 'APP_TELEMETRY_PERIOD_MS\s+\(20U\)') 'GNSS telemetry is not 50 Hz'
Assert-True ($telemetry -match 'static\s+float\s+vofa_data\[20\]') 'GNSS profile is not fixed at 20 floats'
Assert-True ($telemetry -match 'gps_age_ms') 'GNSS profile omits data age'
Assert-True ($telemetry -notmatch 'GNGGA|GNRMC|\$GN') 'raw NMEA appears in JustFloat telemetry'
```

Run it and expect the new checks to fail.

- [ ] **Step 2: Cache the newest GNSS record in intercore_control**

Add state to `intercore_control.c`:

```c
static intercore_gnss_payload_struct intercore_latest_gnss;
static uint32 intercore_latest_gnss_source_ms;
static uint8 intercore_latest_gnss_available;
```

Zero it in `intercore_control_init()`. In `intercore_control_update()` read GNSS after navigation processing:

```c
    intercore_gnss_payload_struct gnss_payload;
    uint32 gnss_source_ms = 0U;
    uint32 gnss_sequence = 0U;
    intercore_transport_result_enum gnss_result;

    gnss_result = intercore_transport_read_gnss(
        &intercore_transport, &gnss_payload, &gnss_source_ms, &gnss_sequence);
    if(INTERCORE_TRANSPORT_OK == gnss_result)
    {
        intercore_latest_gnss = gnss_payload;
        intercore_latest_gnss_source_ms = gnss_source_ms;
        intercore_latest_gnss_available = 1U;
    }
    else if(INTERCORE_TRANSPORT_EPOCH_CHANGED == gnss_result)
    {
        intercore_latest_gnss_available = 0U;
    }
```

Expose a copying getter in the header and source:

```c
uint8 intercore_control_get_latest_gnss(
    intercore_gnss_payload_struct *payload, uint32 *source_ms)
{
    if((NULL == payload) || (NULL == source_ms) ||
       (0U == intercore_latest_gnss_available)) return 0U;
    *payload = intercore_latest_gnss;
    *source_ms = intercore_latest_gnss_source_ms;
    return 1U;
}
```

- [ ] **Step 3: Add explicit telemetry profiles**

In `app_config.h`, replace the single telemetry period definition with:

```c
#define APP_TELEMETRY_PROFILE_BALANCE (0U)
#define APP_TELEMETRY_PROFILE_GNSS    (1U)
#define APP_TELEMETRY_PROFILE         (APP_TELEMETRY_PROFILE_GNSS)

#if (APP_TELEMETRY_PROFILE == APP_TELEMETRY_PROFILE_GNSS)
#define APP_TELEMETRY_PERIOD_MS       (20U)
#else
#define APP_TELEMETRY_PERIOD_MS       (10U)
#endif
```

Keep the current balance layout unchanged under the balance profile.

- [ ] **Step 4: Implement the 20-float GNSS JustFloat layout**

Include the CM7_0 cache API and shared age contract. Refactor the buffer selection without changing either existing layout:

```c
#include "intercore_control.h"
#include "gnss_types.h"

#if (APP_TELEMETRY_PROFILE == APP_TELEMETRY_PROFILE_GNSS)
static float vofa_data[20];
#elif APP_TELEMETRY_BALANCE_ENABLE
static float vofa_data[55];
#else
static float vofa_data[8];
#endif
```

In `telemetry_update()`, put the GNSS declarations and assignments in the first profile branch, before the existing balance branch:

```c
#if (APP_TELEMETRY_PROFILE == APP_TELEMETRY_PROFILE_GNSS)
    intercore_gnss_payload_struct gps = {0};
    uint32 gps_source_ms = 0U;
    uint32 gps_age_ms = 0xFFFFFFFFUL;
    uint8 gps_available;
#elif APP_TELEMETRY_BALANCE_ENABLE
    const balance_diag_struct *balance;
    const leg_diag_struct *leg;
    const imu_state_struct *imu;
#endif
```

Populate the fields in this fixed order:

```c
gps_available = intercore_control_get_latest_gnss(&gps, &gps_source_ms);
if(0U != gps_available) gps_age_ms = now_ms - gps_source_ms;

vofa_data[0]  = (float)now_ms;
vofa_data[1]  = (float)telemetry_frame_sequence;
vofa_data[2]  = (float)telemetry_drop_count;
vofa_data[3]  = (float)((0U != gps_available) && (0U != gps.fix_valid) &&
                        (GNSS_SNAPSHOT_MAX_AGE_MS >= gps_age_ms));
vofa_data[4]  = (float)gps.origin_valid;
vofa_data[5]  = gps.local_x_m;
vofa_data[6]  = gps.local_y_m;
vofa_data[7]  = (float)gps.satellite_count;
vofa_data[8]  = gps.hdop;
vofa_data[9]  = (float)gps_age_ms;
vofa_data[10] = gps.speed_mps;
vofa_data[11] = gps.course_deg;
vofa_data[12] = gps.position_sigma_m;
vofa_data[13] = (float)gps.fix_quality;
vofa_data[14] = (float)gps.checksum_error_count;
vofa_data[15] = (float)gps.timeout_count;
vofa_data[16] = (float)app_scheduler_get_missed_tick_count();
vofa_data[17] = (float)app_scheduler_get_max_gap_ms();
vofa_data[18] = (float)(wheel->online && wheel->left_online && wheel->right_online);
vofa_data[19] = (float)rpm_diag->mode;
telemetry_frame_sequence++;
```

Wrap the existing 55-float assignments in `#elif APP_TELEMETRY_BALANCE_ENABLE` and leave its field order byte-for-byte unchanged; keep the existing 8-float assignments in the final `#else`. The common nonblocking transmit path after the assignments remains shared by all three layouts.

Do not include `sensor_gnss.h` in CM7_0. Use the `GNSS_SNAPSHOT_MAX_AGE_MS (300U)` contract already defined in `gnss_types.h` in both sensor and telemetry.

- [ ] **Step 5: Run static and host tests**

```powershell
& .\tools\test_gnss_integration_static.ps1
& .\tools\test_gnss_intercore_host.ps1
& .\tools\test_perception_intercore_host.ps1
& .\tools\test_intercore_control_foundation.ps1
```

Expected: all print `PASS`.

- [ ] **Step 6: Build CM7_0**

```powershell
& 'D:\IAR\common\bin\IarBuild.exe' .\project\iar\project_config\cyt4bb7_cm_7_0.ewp -build Debug -log all
```

Expected: exit code 0; the GNSS profile frame is 84 bytes (`20 * 4 + 4`) and occupies about 1.82 ms at 460800 baud every 20 ms.

- [ ] **Step 7: Commit CM7_0 telemetry support**

```powershell
git add project/code/gnss_types.h project/code/intercore_control.c project/code/intercore_control.h project/code/app_config.h project/code/telemetry.c tools/test_gnss_integration_static.ps1
git commit -m "Stream GNSS diagnostics through VOFA"
```

---

### Task 7: Full Build and Motor-Disabled Hardware Gate

**Files:**
- Verify only; do not change source unless a test or build exposes a defect.

**Interfaces:**
- Consumes: Tasks 1–6.
- Produces: a verified GNSS diagnostic pipeline ready for the white-frame vision phase.

- [ ] **Step 1: Run the complete automated verification set**

```powershell
& .\tools\test_gnss_driver_static.ps1
& .\tools\test_gnss_local_position_host.ps1
& .\tools\test_gnss_intercore_host.ps1
& .\tools\test_gnss_integration_static.ps1
& .\tools\test_perception_intercore_host.ps1
& .\tools\test_intercore_control_foundation.ps1
& .\tools\test_cm7_uart_ownership_static.ps1
& .\tools\test_iar_warning_cleanup.ps1
```

Expected: every script exits 0 and prints PASS; no test modifies tracked files.

- [ ] **Step 2: Build all three affected IAR projects**

```powershell
& 'D:\IAR\common\bin\IarBuild.exe' .\project\iar\project_config\cyt4bb7_cm_0_plus.ewp -build Debug -log all
& 'D:\IAR\common\bin\IarBuild.exe' .\project\iar\project_config\cyt4bb7_cm_7_0.ewp -build Debug -log all
& 'D:\IAR\common\bin\IarBuild.exe' .\project\iar\project_config\cyt4bb7_cm_7_1.ewp -build Debug -log all
```

Expected: all exit 0. Record code size, data size, and warnings for each core in the task handoff.

- [ ] **Step 3: Verify TAU1201 directly before firmware integration**

With motors disabled and the antenna outdoors:

1. Connect TAU1201 TX to a 3.3 V USB-TTL RX, share ground, and power the carrier from 5 V.
2. Open Satrack at 115200 8N1.
3. Confirm RMC and GGA arrive, RMC status becomes `A`, GGA Fix Quality becomes nonzero, and at least 8 satellites participate.
4. Leave the antenna static for 5 minutes and save the NMEA log.
5. Confirm cold-start acquisition is finite and no checksum-error burst repeats continuously.

Expected: a stable fix and a log suitable for comparing firmware-parsed latitude, longitude, satellite count, and HDOP.

- [ ] **Step 4: Flash and verify the integrated pipeline with motor outputs disabled**

1. Flash CM0+, CM7_0, and CM7_1 Debug images.
2. Keep the chassis command disabled and wheels off the ground or mechanically restrained.
3. Open VOFA+ on UART0 at 460800 and select JustFloat.
4. Configure 20 channels in the exact Task 6 order.
5. Wait for 50 usable samples; confirm `origin_valid` changes from 0 to 1.
6. Confirm `gps_valid` becomes 1 only after five consecutive quality-approved fixes.
7. Keep the antenna static for 5 minutes and export VOFA data.
8. Move the antenna approximately 1 m east and then 1 m north; confirm signs and approximate magnitudes of `local_x_m/local_y_m`.
9. Disconnect the antenna; confirm `gps_valid` clears after age exceeds 300 ms without disturbing CM7_0 heartbeat, IMU, or scheduler.
10. Reconnect; confirm motion remains disabled and GPS validity returns only after five usable fixes.

Expected: no UART0 framing corruption, no telemetry drops caused by the 20-float profile, scheduler maximum gap no greater than 2 ms, and no actuator enable caused by GNSS.

- [ ] **Step 5: Verify repository cleanliness and commit boundaries**

```powershell
git status --short --branch
git log --oneline -6
git diff --check HEAD~6..HEAD
```

Expected: only the pre-existing untracked `tmp/` remains; the implementation is split into the driver, projection, sensor wrapper, transport, CM7_1 integration, and VOFA commits described above.

## Completion Gate

Phase 1 is complete only when all automated tests pass, all three IAR projects build, direct Satrack output agrees with the parsed VOFA fields, local-axis movement is correct, stale GNSS clears within the 300 ms contract, UART0 remains CM7_0-only, and GNSS cannot enable motion. Do not start the white-frame vision plan before this gate passes.
