#include <math.h>
#include <stdio.h>
#include <string.h>

#include "sensor_gnss.h"
#include "zf_device_gnss.h"

typedef enum
{
    MOCK_SENTENCE_NONE,
    MOCK_SENTENCE_RMC,
    MOCK_SENTENCE_GGA,
}mock_sentence_enum;

typedef struct
{
    mock_sentence_enum sentence;
    uint8 state;
    uint8 fix_quality;
    uint8 satellite_used;
    float hdop;
    double latitude;
    double longitude;
    float speed;
    float direction;
    uint32 utc_ms;
}mock_event_struct;

gnss_info_struct gnss;
volatile uint8 gnss_flag;

static mock_event_struct pending_event;
static uint32 now_ms;
static unsigned failures;

#define CHECK(condition) do { \
    if(!(condition)) { \
        printf("FAIL:%s:%d: %s\n", __func__, __LINE__, #condition); \
        failures++; \
    } \
}while(0)

#define CHECK_NEAR(actual, expected, tolerance) do { \
    if(fabs((double)(actual) - (double)(expected)) > (tolerance)) { \
        printf("FAIL:%s:%d: actual=%.9f expected=%.9f\n", __func__, __LINE__, \
               (double)(actual), (double)(expected)); \
        failures++; \
    } \
}while(0)

static uint32 next_sequence(uint32 sequence)
{
    sequence++;
    return (0U != sequence) ? sequence : 1U;
}

void gnss_init(gps_device_enum gps_device)
{
    CHECK(TAU1201 == gps_device);
    memset(&gnss, 0, sizeof(gnss));
    memset(&pending_event, 0, sizeof(pending_event));
    gnss_flag = 0U;
}

uint8 gnss_data_parse(void)
{
    if(MOCK_SENTENCE_RMC == pending_event.sentence)
    {
        gnss.state = pending_event.state;
        gnss.latitude = pending_event.latitude;
        gnss.longitude = pending_event.longitude;
        gnss.speed = pending_event.speed;
        gnss.direction = pending_event.direction;
        gnss.rmc_utc_ms = pending_event.utc_ms;
        gnss.rmc_sequence = next_sequence(gnss.rmc_sequence);
    }
    else if(MOCK_SENTENCE_GGA == pending_event.sentence)
    {
        gnss.fix_quality = pending_event.fix_quality;
        gnss.satellite_used = pending_event.satellite_used;
        gnss.hdop = pending_event.hdop;
        gnss.gga_utc_ms = pending_event.utc_ms;
        gnss.gga_sequence = next_sequence(gnss.gga_sequence);
    }
    pending_event.sentence = MOCK_SENTENCE_NONE;
    return 0U;
}

static void reset_sensor(void)
{
    now_ms = 0U;
    CHECK(0U != sensor_gnss_init());
}

static void emit_rmc_at(uint32 utc_ms, uint8 state, double latitude, double longitude)
{
    pending_event.sentence = MOCK_SENTENCE_RMC;
    pending_event.state = state;
    pending_event.latitude = latitude;
    pending_event.longitude = longitude;
    pending_event.speed = 36.0F;
    pending_event.direction = 123.0F;
    pending_event.utc_ms = utc_ms;
    gnss_flag = 1U;
    now_ms += 10U;
    sensor_gnss_service(now_ms);
}

static void emit_rmc(uint8 state, double latitude, double longitude)
{
    emit_rmc_at(1000U, state, latitude, longitude);
}

static void queue_rmc(uint8 state, double latitude, double longitude)
{
    pending_event.sentence = MOCK_SENTENCE_RMC;
    pending_event.state = state;
    pending_event.latitude = latitude;
    pending_event.longitude = longitude;
    pending_event.speed = 36.0F;
    pending_event.direction = 123.0F;
    pending_event.utc_ms = 1000U;
    gnss_flag = 1U;
}

static void emit_gga_at(uint32 utc_ms, uint8 fix_quality, uint8 satellite_used, float hdop)
{
    pending_event.sentence = MOCK_SENTENCE_GGA;
    pending_event.fix_quality = fix_quality;
    pending_event.satellite_used = satellite_used;
    pending_event.hdop = hdop;
    pending_event.utc_ms = utc_ms;
    gnss_flag = 1U;
    now_ms += 10U;
    sensor_gnss_service(now_ms);
}

static void emit_gga(uint8 fix_quality, uint8 satellite_used, float hdop)
{
    emit_gga_at(1000U, fix_quality, satellite_used, hdop);
}

static gnss_snapshot_struct emit_complete_fix(uint8 state, uint8 fix_quality,
                                               uint8 satellite_used, float hdop,
                                               double latitude, double longitude)
{
    gnss_snapshot_struct snapshot;

    memset(&snapshot, 0, sizeof(snapshot));
    emit_rmc(state, latitude, longitude);
    CHECK(0U == sensor_gnss_take_snapshot(&snapshot));
    emit_gga(fix_quality, satellite_used, hdop);
    CHECK(0U != sensor_gnss_take_snapshot(&snapshot));
    CHECK(0U == sensor_gnss_take_snapshot(&snapshot));
    return snapshot;
}

static gnss_snapshot_struct emit_usable_fix(double latitude, double longitude)
{
    return emit_complete_fix(1U, 1U, SENSOR_GNSS_MIN_SATELLITES, 1.0F,
                             latitude, longitude);
}

static void test_requires_both_sentence_sequences(void)
{
    gnss_snapshot_struct snapshot;

    reset_sensor();
    emit_rmc(1U, 30.0, 120.0);
    CHECK(0U == sensor_gnss_take_snapshot(&snapshot));
    emit_rmc(1U, 31.0, 121.0);
    CHECK(0U == sensor_gnss_take_snapshot(&snapshot));
    emit_gga(1U, 10U, 1.0F);
    CHECK(0U != sensor_gnss_take_snapshot(&snapshot));
    CHECK_NEAR(snapshot.latitude_deg, 31.0, 0.0000001);

    emit_gga(1U, 10U, 1.0F);
    CHECK(0U == sensor_gnss_take_snapshot(&snapshot));
    emit_rmc(1U, 32.0, 122.0);
    CHECK(0U != sensor_gnss_take_snapshot(&snapshot));
    CHECK_NEAR(snapshot.latitude_deg, 32.0, 0.0000001);
}

static void test_requires_matching_utc_epochs_and_resynchronizes(void)
{
    gnss_snapshot_struct snapshot;

    reset_sensor();
    emit_rmc_at(1000U, 1U, 30.0, 120.0);
    emit_gga_at(2000U, 1U, 10U, 1.0F);
    CHECK(0U == sensor_gnss_take_snapshot(&snapshot));
    emit_rmc_at(2000U, 1U, 31.0, 121.0);
    CHECK(0U != sensor_gnss_take_snapshot(&snapshot));
    CHECK_NEAR(snapshot.latitude_deg, 31.0, 0.0000001);

    emit_gga_at(3000U, 1U, 10U, 1.0F);
    emit_rmc_at(2500U, 1U, 32.0, 122.0);
    CHECK(0U == sensor_gnss_take_snapshot(&snapshot));
    emit_rmc_at(3000U, 1U, 33.0, 123.0);
    CHECK(0U != sensor_gnss_take_snapshot(&snapshot));
    CHECK_NEAR(snapshot.latitude_deg, 33.0, 0.0000001);

    emit_rmc_at(4000U, 1U, 34.0, 124.0);
    emit_rmc_at(5000U, 1U, 35.0, 125.0);
    emit_gga_at(5000U, 1U, 10U, 1.0F);
    CHECK(0U != sensor_gnss_take_snapshot(&snapshot));
    CHECK_NEAR(snapshot.latitude_deg, 35.0, 0.0000001);

    emit_gga_at(6000U, 1U, 10U, 1.0F);
    emit_gga_at(7000U, 1U, 10U, 1.0F);
    emit_rmc_at(7000U, 1U, 36.0, 126.0);
    CHECK(0U != sensor_gnss_take_snapshot(&snapshot));
    CHECK_NEAR(snapshot.latitude_deg, 36.0, 0.0000001);
}

static void test_utc_epoch_matching_handles_midnight_wrap(void)
{
    gnss_snapshot_struct snapshot;

    reset_sensor();
    emit_rmc_at(86399900U, 1U, -30.0, -120.0);
    emit_gga_at(100U, 1U, 10U, 1.0F);
    CHECK(0U == sensor_gnss_take_snapshot(&snapshot));
    emit_rmc_at(100U, 1U, -31.0, -121.0);
    CHECK(0U != sensor_gnss_take_snapshot(&snapshot));
    CHECK_NEAR(snapshot.latitude_deg, -31.0, 0.0000001);
    CHECK_NEAR(snapshot.longitude_deg, -121.0, 0.0000001);
}

static void test_invalid_coordinates_never_recover_or_seed_origin(void)
{
    static const double invalid_latitudes[] = {NAN, INFINITY, -91.0, 91.0};
    static const double invalid_longitudes[] = {NAN, -INFINITY, -181.0, 181.0};
    gnss_snapshot_struct snapshot;
    unsigned index;

    for(index = 0U; index < sizeof(invalid_latitudes) / sizeof(invalid_latitudes[0]); index++)
    {
        reset_sensor();
        snapshot = emit_complete_fix(1U, 1U, 10U, 1.0F,
                                     invalid_latitudes[index], 120.0);
        CHECK(0U == snapshot.fix_valid);
        CHECK(0U == snapshot.origin_valid);
        CHECK(isfinite(snapshot.latitude_deg));
        CHECK(isfinite(snapshot.longitude_deg));
    }

    for(index = 0U; index < sizeof(invalid_longitudes) / sizeof(invalid_longitudes[0]); index++)
    {
        reset_sensor();
        snapshot = emit_complete_fix(1U, 1U, 10U, 1.0F,
                                     30.0, invalid_longitudes[index]);
        CHECK(0U == snapshot.fix_valid);
        CHECK(0U == snapshot.origin_valid);
        CHECK(isfinite(snapshot.latitude_deg));
        CHECK(isfinite(snapshot.longitude_deg));
    }
}

static void test_recovers_after_five_complete_usable_fixes(void)
{
    gnss_snapshot_struct snapshot;
    uint8 index;

    reset_sensor();
    for(index = 1U; index <= SENSOR_GNSS_RECOVERY_FIX_COUNT; index++)
    {
        snapshot = emit_usable_fix(30.0 + (double)index * 0.000001, 120.0);
        CHECK(((index == SENSOR_GNSS_RECOVERY_FIX_COUNT) ? 1U : 0U) ==
              snapshot.fix_valid);
    }
}

static void test_timeout_publishes_invalid_copy_and_resets_recovery(void)
{
    gnss_snapshot_struct snapshot;
    gnss_snapshot_struct timeout_snapshot;
    uint32 measurement_timestamp;
    uint32 measurement_sequence;
    uint8 index;

    reset_sensor();
    for(index = 0U; index < SENSOR_GNSS_RECOVERY_FIX_COUNT; index++)
    {
        snapshot = emit_usable_fix(30.0, 120.0);
    }
    CHECK(1U == snapshot.fix_valid);
    measurement_timestamp = snapshot.timestamp_ms;
    measurement_sequence = snapshot.sequence;

    now_ms = measurement_timestamp + GNSS_SNAPSHOT_MAX_AGE_MS + 1U;
    sensor_gnss_service(now_ms);
    CHECK(0U != sensor_gnss_take_snapshot(&timeout_snapshot));
    CHECK(0U == timeout_snapshot.fix_valid);
    CHECK(1U == timeout_snapshot.timeout_count);
    CHECK(measurement_timestamp == timeout_snapshot.timestamp_ms);
    CHECK(measurement_sequence != timeout_snapshot.sequence);

    sensor_gnss_service(now_ms + 100U);
    CHECK(0U == sensor_gnss_take_snapshot(&timeout_snapshot));

    emit_rmc(1U, 30.0, 120.0);
    CHECK(0U == sensor_gnss_take_snapshot(&snapshot));
    emit_gga(1U, SENSOR_GNSS_MIN_SATELLITES, 1.0F);
    CHECK(0U == sensor_gnss_take_snapshot(&snapshot));
    emit_rmc(1U, 30.0, 120.0);
    CHECK(0U != sensor_gnss_take_snapshot(&snapshot));
    CHECK(0U == snapshot.fix_valid);
    CHECK(1U == snapshot.timeout_count);
}

static void test_timeout_discards_a_pre_timeout_partial_pair(void)
{
    gnss_snapshot_struct snapshot;
    uint32 measurement_timestamp;
    uint8 index;

    reset_sensor();
    snapshot = emit_usable_fix(30.0, 120.0);
    measurement_timestamp = snapshot.timestamp_ms;

    emit_rmc(1U, 30.1, 120.1);
    CHECK(0U == sensor_gnss_take_snapshot(&snapshot));

    now_ms = measurement_timestamp + GNSS_SNAPSHOT_MAX_AGE_MS + 1U;
    sensor_gnss_service(now_ms);
    CHECK(0U != sensor_gnss_take_snapshot(&snapshot));
    CHECK(0U == snapshot.fix_valid);
    CHECK(measurement_timestamp == snapshot.timestamp_ms);

    emit_gga(1U, 10U, 1.0F);
    CHECK(0U == sensor_gnss_take_snapshot(&snapshot));
    emit_rmc(1U, 30.2, 120.2);
    CHECK(0U == sensor_gnss_take_snapshot(&snapshot));
    emit_gga(1U, 10U, 1.0F);
    CHECK(0U != sensor_gnss_take_snapshot(&snapshot));
    CHECK(0U == snapshot.fix_valid);

    for(index = 2U; index <= SENSOR_GNSS_RECOVERY_FIX_COUNT; index++)
    {
        snapshot = emit_usable_fix(30.2, 120.2);
        CHECK(((index == SENSOR_GNSS_RECOVERY_FIX_COUNT) ? 1U : 0U) ==
              snapshot.fix_valid);
    }
}

static void test_timeout_discards_a_buffered_sentence_parsed_during_timeout(void)
{
    gnss_snapshot_struct snapshot;
    uint32 measurement_timestamp;

    reset_sensor();
    snapshot = emit_usable_fix(30.0, 120.0);
    measurement_timestamp = snapshot.timestamp_ms;

    queue_rmc(1U, 30.1, 120.1);
    now_ms = measurement_timestamp + GNSS_SNAPSHOT_MAX_AGE_MS + 1U;
    sensor_gnss_service(now_ms);
    CHECK(0U != sensor_gnss_take_snapshot(&snapshot));
    CHECK(0U == snapshot.fix_valid);
    CHECK(measurement_timestamp == snapshot.timestamp_ms);

    emit_gga(1U, 10U, 1.0F);
    CHECK(0U == sensor_gnss_take_snapshot(&snapshot));
    emit_rmc(1U, 30.2, 120.2);
    CHECK(0U != sensor_gnss_take_snapshot(&snapshot));
    CHECK(0U == snapshot.fix_valid);
}

static void test_quality_thresholds_reset_recovery(void)
{
    static const struct
    {
        uint8 state;
        uint8 fix_quality;
        uint8 satellite_used;
        float hdop;
        uint8 expected_valid;
    }cases[] =
    {
        {1U, 1U, SENSOR_GNSS_MIN_SATELLITES, SENSOR_GNSS_MAX_HDOP, 1U},
        {0U, 1U, SENSOR_GNSS_MIN_SATELLITES, 1.0F, 0U},
        {1U, 0U, SENSOR_GNSS_MIN_SATELLITES, 1.0F, 0U},
        {1U, 1U, SENSOR_GNSS_MIN_SATELLITES - 1U, 1.0F, 0U},
        {1U, 1U, SENSOR_GNSS_MIN_SATELLITES, 0.0F, 0U},
        {1U, 1U, SENSOR_GNSS_MIN_SATELLITES, SENSOR_GNSS_MAX_HDOP + 0.01F, 0U},
    };
    gnss_snapshot_struct snapshot;
    unsigned case_index;
    uint8 index;

    for(case_index = 0U; case_index < (sizeof(cases) / sizeof(cases[0])); case_index++)
    {
        reset_sensor();
        for(index = 1U; index < SENSOR_GNSS_RECOVERY_FIX_COUNT; index++)
        {
            snapshot = emit_usable_fix(30.0, 120.0);
            CHECK(0U == snapshot.fix_valid);
        }
        snapshot = emit_complete_fix(cases[case_index].state,
                                     cases[case_index].fix_quality,
                                     cases[case_index].satellite_used,
                                     cases[case_index].hdop, 30.0, 120.0);
        CHECK(cases[case_index].expected_valid == snapshot.fix_valid);
    }
}

static void test_snapshot_is_copied(void)
{
    gnss_snapshot_struct snapshot;

    reset_sensor();
    emit_rmc(1U, 30.5, 120.5);
    emit_gga(1U, 10U, 1.25F);
    gnss.latitude = 0.0;
    gnss.longitude = 0.0;
    gnss.hdop = 99.0F;
    CHECK(0U != sensor_gnss_take_snapshot(&snapshot));
    CHECK_NEAR(snapshot.latitude_deg, 30.5, 0.0000001);
    CHECK_NEAR(snapshot.longitude_deg, 120.5, 0.0000001);
    CHECK_NEAR(snapshot.hdop, 1.25, 0.0001);
    CHECK(0U == sensor_gnss_take_snapshot(&snapshot));
    CHECK(0U == sensor_gnss_take_snapshot(NULL));
}

static void test_origin_uses_fifty_complete_usable_fixes(void)
{
    gnss_snapshot_struct snapshot;
    uint16 index;

    reset_sensor();
    for(index = 1U; index <= SENSOR_GNSS_AUTO_ORIGIN_SAMPLE_COUNT; index++)
    {
        snapshot = emit_usable_fix(30.0 + (double)index * 0.000001,
                                   120.0 + (double)index * 0.000001);
        CHECK(((index == SENSOR_GNSS_AUTO_ORIGIN_SAMPLE_COUNT) ? 1U : 0U) ==
              snapshot.origin_valid);
    }
}

int main(void)
{
    test_requires_both_sentence_sequences();
    test_requires_matching_utc_epochs_and_resynchronizes();
    test_utc_epoch_matching_handles_midnight_wrap();
    test_invalid_coordinates_never_recover_or_seed_origin();
    test_recovers_after_five_complete_usable_fixes();
    test_timeout_publishes_invalid_copy_and_resets_recovery();
    test_timeout_discards_a_pre_timeout_partial_pair();
    test_timeout_discards_a_buffered_sentence_parsed_during_timeout();
    test_quality_thresholds_reset_recovery();
    test_snapshot_is_copied();
    test_origin_uses_fifty_complete_usable_fixes();

    if(0U != failures)
    {
        printf("sensor_gnss_test: FAIL (%u assertions)\n", failures);
        return 1;
    }
    puts("sensor_gnss_test: PASS");
    return 0;
}
