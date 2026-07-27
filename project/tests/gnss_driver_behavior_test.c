#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define GNSS_HOST_TEST (1)

typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned int uint32;
typedef signed char int8;

#define _zf_common_function_h_
#define _zf_common_fifo_h_
#define _zf_driver_delay_h_
#define _zf_driver_uart_h_
#define _zf_device_gnss_h_

#define GNSS_UART (2)
#define GNSS_RX   (0)
#define GNSS_TX   (0)
#define GNSS_PI   (3.1415926535898)
#define ANGLE_TO_RAD(x) ((x) * GNSS_PI / 180.0)
#define RAD_TO_ANGLE(x) ((x) * 180.0 / GNSS_PI)
#define FIFO_DATA_8BIT (0)
#define FIFO_READ_ONLY (0)
#define FIFO_READ_AND_CLEAN (1)

typedef struct { uint8 unused; } fifo_struct;
typedef enum { TAU1201 = 1, GN42A = 1, GN43RFA = 2 } gps_device_enum;
typedef enum { GPS_STATE_RECEIVING, GPS_STATE_RECEIVED, GPS_STATE_PARSING } gps_state_enum;
typedef struct
{
    uint16 year;
    uint8 month;
    uint8 day;
    uint8 hour;
    uint8 minute;
    uint8 second;
}gps_time_struct;
typedef struct
{
    gps_time_struct time;
    uint8 state;
    uint8 fix_quality;
    float hdop;
    uint32 rmc_sequence;
    uint32 gga_sequence;
    uint32 rmc_utc_ms;
    uint32 gga_utc_ms;
    uint16 latitude_degree;
    uint16 latitude_cent;
    uint16 latitude_second;
    uint16 longitude_degree;
    uint16 longitude_cent;
    uint16 longitude_second;
    double latitude;
    double longitude;
    int8 ns;
    int8 ew;
    float speed;
    float direction;
    uint8 antenna_direction_state;
    float antenna_direction;
    uint8 satellite_used;
    float height;
}gnss_info_struct;

static int func_str_to_int(char *text) { return (int)strtol(text, NULL, 10); }
static double func_str_to_double(char *text) { return strtod(text, NULL); }
static void fifo_init(fifo_struct *fifo, int type, uint8 *buffer, uint32 size)
    { (void)fifo; (void)type; (void)buffer; (void)size; }
static void fifo_clear(fifo_struct *fifo) { (void)fifo; }
static uint32 fifo_used(fifo_struct *fifo) { (void)fifo; return 0U; }
static void fifo_write_buffer(fifo_struct *fifo, uint8 *data, uint32 length)
    { (void)fifo; (void)data; (void)length; }
static void fifo_read_buffer(fifo_struct *fifo, uint8 *data, uint32 *length, int mode)
    { (void)fifo; (void)data; (void)length; (void)mode; }
static uint8 uart_query_byte(int uart, uint8 *data) { (void)uart; (void)data; return 0U; }
static void uart_init(int uart, uint32 baud, int rx, int tx)
    { (void)uart; (void)baud; (void)rx; (void)tx; }
static void uart_write_buffer(int uart, uint8 *data, uint32 length)
    { (void)uart; (void)data; (void)length; }
static void uart_rx_interrupt(int uart, uint8 enable) { (void)uart; (void)enable; }
static void system_delay_ms(uint32 time_ms) { (void)time_ms; }

uint8 gnss_host_parse_sentence(const char *sentence, uint32 length,
                               gnss_info_struct *parsed);

#include "../../libraries/zf_device/zf_device_gnss.c"

static unsigned failures;

#define CHECK(condition) do { if(!(condition)) { \
    printf("FAIL:%s:%d: %s\n", __func__, __LINE__, #condition); fflush(stdout); failures++; \
} }while(0)

#define CHECK_NEAR(actual, expected, tolerance) do { \
    if(fabs((double)(actual) - (double)(expected)) > (tolerance)) { \
        printf("FAIL:%s:%d actual=%.9f expected=%.9f\n", __func__, __LINE__, \
               (double)(actual), (double)(expected)); failures++; \
    } \
}while(0)

static uint32 make_sentence(const char *body, char *sentence, uint32 capacity)
{
    static const char hex[] = "0123456789ABCDEF";
    uint8 checksum = 0U;
    uint32 index;
    uint32 body_length = (uint32)strlen(body);

    CHECK(body_length + 6U <= capacity);
    memcpy(sentence, body, body_length);
    for(index = 1U; index < body_length; index++) checksum ^= (uint8)body[index];
    sentence[body_length] = '*';
    sentence[body_length + 1U] = hex[checksum >> 4U];
    sentence[body_length + 2U] = hex[checksum & 0x0FU];
    sentence[body_length + 3U] = '\r';
    sentence[body_length + 4U] = '\n';
    sentence[body_length + 5U] = '\0';
    return body_length + 5U;
}

static uint8 production_parse(const char *body)
{
    char sentence[128];
    uint32 length = make_sentence(body, sentence, sizeof(sentence));
    uint8 *buffer;
    uint32 *captured_length;
    uint8 *truncated;
    gps_state_enum *state;

    if(0 == strncmp(&sentence[3], "RMC", 3))
    {
        buffer = gps_rmc_buffer;
        captured_length = &gps_rmc_length;
        truncated = &gps_rmc_truncated;
        state = &gnss_rmc_state;
    }
    else if(0 == strncmp(&sentence[3], "GGA", 3))
    {
        buffer = gps_gga_buffer;
        captured_length = &gps_gga_length;
        truncated = &gps_gga_truncated;
        state = &gnss_gga_state;
    }
    else
    {
        buffer = gps_ths_buffer;
        captured_length = &gps_ths_length;
        truncated = &gps_ths_truncated;
        state = &gnss_ths_state;
    }
    memset(buffer, 0, GNSS_BUFFER_SIZE);
    memcpy(buffer, sentence, length);
    *captured_length = length;
    *truncated = 0U;
    *state = GPS_STATE_RECEIVED;
    return gnss_data_parse();
}

static void test_rmc_signs_fractional_utc_and_invalid_fields(void)
{
    char sentence[128];
    gnss_info_struct parsed;
    uint32 length;

    memset(&parsed, 0, sizeof(parsed));
    length = make_sentence("$GNRMC,123519.250,A,4807.038,S,01131.000,W,0.0,0.0,230394,,,A",
                           sentence, sizeof(sentence));
    CHECK(0U != gnss_host_parse_sentence(sentence, length, &parsed));
    CHECK_NEAR(parsed.latitude, -48.1173, 0.0000001);
    CHECK_NEAR(parsed.longitude, -11.5166666667, 0.0000001);
    CHECK(45319250U == parsed.rmc_utc_ms);

    length = make_sentence("$GNRMC,123519.250,A,4807.038,N,01131.000,E,0.0,0.0,230394,,,A",
                           sentence, sizeof(sentence));
    CHECK(0U != gnss_host_parse_sentence(sentence, length, &parsed));
    CHECK_NEAR(parsed.latitude, 48.1173, 0.0000001);
    CHECK_NEAR(parsed.longitude, 11.5166666667, 0.0000001);

    length = make_sentence("$GNRMC,123519.250,A,4807.038,N,01131.000,E,123456789012345678901234567890,0.0,230394,,,A",
                           sentence, sizeof(sentence));
    CHECK(0U == gnss_host_parse_sentence(sentence, length, &parsed));

    length = make_sentence("$GNRMC,123519.251,A,4807.038,X,01131.000,E,0.0,0.0,230394,,,A",
                           sentence, sizeof(sentence));
    CHECK(0U == gnss_host_parse_sentence(sentence, length, &parsed));
    length = make_sentence("$GNRMC,123519.252,A,4807.038,N,01131.000,Q,0.0,0.0,230394,,,A",
                           sentence, sizeof(sentence));
    CHECK(0U == gnss_host_parse_sentence(sentence, length, &parsed));
    length = make_sentence("$GNRMC,123519.253,A,9100.000,N,01131.000,E,0.0,0.0,230394,,,A",
                           sentence, sizeof(sentence));
    CHECK(0U == gnss_host_parse_sentence(sentence, length, &parsed));
    length = make_sentence("$GNRMC,123519.254,A,nan,N,01131.000,E,0.0,0.0,230394,,,A",
                           sentence, sizeof(sentence));
    CHECK(0U == gnss_host_parse_sentence(sentence, length, &parsed));
}

static void test_gga_fractional_utc(void)
{
    char sentence[128];
    gnss_info_struct parsed;
    uint32 length;

    memset(&parsed, 0, sizeof(parsed));
    length = make_sentence("$GNGGA,235959.999,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,",
                           sentence, sizeof(sentence));
    CHECK(0U != gnss_host_parse_sentence(sentence, length, &parsed));
    CHECK(86399999U == parsed.gga_utc_ms);
    CHECK(1U == parsed.fix_quality);
    CHECK(8U == parsed.satellite_used);
}

static void test_bounded_checksum_rejections(void)
{
    char sentence[160];
    gnss_info_struct parsed;
    uint32 length;

    memset(&parsed, 0, sizeof(parsed));
    strcpy(sentence, "$GNRMC,123519.000,V,,,,,,,230394,,,N\r\n");
    CHECK(0U == gnss_host_parse_sentence(sentence, (uint32)strlen(sentence), &parsed));
    strcpy(sentence, "$GNRMC,123519.000,V,,,,,,,230394,,,N*A\r\n");
    CHECK(0U == gnss_host_parse_sentence(sentence, (uint32)strlen(sentence), &parsed));
    strcpy(sentence, "$GNRMC,123519.000,V,,,,,,,230394,,,N*GG\r\n");
    CHECK(0U == gnss_host_parse_sentence(sentence, (uint32)strlen(sentence), &parsed));

    length = make_sentence("$GNRMC,123519.000,V,,,,,,,230394,,,N",
                           sentence, sizeof(sentence));
    sentence[length - 4U] ^= 1;
    CHECK(0U == gnss_host_parse_sentence(sentence, length, &parsed));

    memset(sentence, 'A', 128U);
    sentence[0] = '$';
    CHECK(0U == gnss_host_parse_sentence(sentence, 128U, &parsed));
}

static void test_production_parse_is_transactional_on_semantic_failure(void)
{
    gnss_info_struct before;

    memset(&gnss, 0x5A, sizeof(gnss));
    before = gnss;
    CHECK(0U != production_parse(
        "$GNRMC,123519.250,A,4807.038,N,01131.000,E,0.0,0.0,23X394,,,A"));
    CHECK(0 == memcmp(&before, &gnss, sizeof(gnss)));

    before = gnss;
    CHECK(0U != production_parse(
        "$GNRMC,123519.250,A,123456789012345678901234567890,N,01131.000,E,0.0,0.0,230394,,,A"));
    CHECK(0 == memcmp(&before, &gnss, sizeof(gnss)));

    before = gnss;
    CHECK(0U != production_parse(
        "$GNGGA,123519.250,4807.038,N,01131.000,E,1,08,123456789012345678901234567890,545.4,M,46.9,M,,"));
    CHECK(0 == memcmp(&before, &gnss, sizeof(gnss)));
}

static void test_production_no_fix_gga_clears_quality(void)
{
    memset(&gnss, 0, sizeof(gnss));
    CHECK(0U == production_parse(
        "$GNGGA,123519.250,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,"));
    CHECK(1U == gnss.fix_quality);
    CHECK(8U == gnss.satellite_used);
    CHECK(0U == production_parse(
        "$GNGGA,123520.250,,,,,0,00,99.9,0.0,M,0.0,M,,"));
    CHECK(0U == gnss.fix_quality);
    CHECK(0U == gnss.satellite_used);
    CHECK_NEAR(gnss.hdop, 99.9, 0.001);
}

static void test_production_ths_invalid_state_commits(void)
{
    gnss_info_struct before;

    memset(&gnss, 0, sizeof(gnss));
    CHECK(0U == production_parse("$GNTHS,123.4,A"));
    CHECK(1U == gnss.antenna_direction_state);
    CHECK_NEAR(gnss.antenna_direction, 123.4, 0.001);

    CHECK(0U == production_parse("$GNTHS,123.4,V"));
    CHECK(0U == gnss.antenna_direction_state);

    before = gnss;
    CHECK(0U != production_parse("$GNTHS,123.4,X"));
    CHECK(0 == memcmp(&before, &gnss, sizeof(gnss)));
}

int main(void)
{
    test_rmc_signs_fractional_utc_and_invalid_fields();
    test_gga_fractional_utc();
    test_bounded_checksum_rejections();
    test_production_parse_is_transactional_on_semantic_failure();
    test_production_no_fix_gga_clears_quality();
    test_production_ths_invalid_state_commits();

    if(0U != failures)
    {
        printf("gnss_driver_behavior_test: FAIL (%u assertions)\n", failures);
        return 1;
    }
    puts("gnss_driver_behavior_test: PASS");
    return 0;
}
