#include <stdio.h>
#include <string.h>

#include "camera_frame_consumer.h"
#include "dl1b_safety.h"
#include "intercore_memory.h"
#include "intercore_transport.h"
#include "perception_intercore.h"
#include "single_gap_app.h"

#define TEST_WIDTH  (188U)
#define TEST_HEIGHT (120U)

static uint32 failure_count;
static uint32 publish_count;
static uint32 mock_now_ms;
static uint32 navigation_attach_count;
static uint32 pose_attach_count;
static uint32 tof_init_count;
static uint8 pixels[TEST_WIDTH * TEST_HEIGHT];
static camera_frame_handler_fn registered_handler;
static navigation_command_struct published_command;
static perception_pose_snapshot_struct mock_pose;
static single_gap_tof_snapshot_struct mock_tof;
static volatile intercore_shared_layout_struct mock_shared;

#define TEST_CHECK(condition)                                                        \
    do                                                                               \
    {                                                                                \
        if(!(condition))                                                             \
        {                                                                            \
            failure_count++;                                                         \
            printf("FAIL:%u: %s\n", (unsigned)__LINE__, #condition);               \
        }                                                                            \
    } while(0)

#if !defined(TEST_RPM_LIMIT) && !defined(TEST_FEATURE_DISABLED)
static uint8 float_near(float expected, float actual, float tolerance)
{
    float error = expected - actual;
    if(0.0f > error)
    {
        error = -error;
    }
    return (tolerance >= error) ? 1U : 0U;
}
#endif

volatile intercore_shared_layout_struct *intercore_memory_get_layout(void)
{
    return &mock_shared;
}

uint8 intercore_transport_cm7_1_attach(intercore_transport_struct *transport,
                                       volatile intercore_shared_layout_struct *shared)
{
    (void)transport;
    (void)shared;
    navigation_attach_count++;
    return 1U;
}

uint8 perception_intercore_cm7_1_attach(perception_intercore_transport_struct *transport,
                                        volatile intercore_shared_layout_struct *shared)
{
    (void)transport;
    (void)shared;
    pose_attach_count++;
    return 1U;
}

uint8 perception_intercore_read_pose(perception_intercore_transport_struct *transport,
                                     perception_pose_snapshot_struct *snapshot)
{
    (void)transport;
    *snapshot = mock_pose;
    return 1U;
}

uint8 intercore_transport_publish_navigation(intercore_transport_struct *transport,
                                             const navigation_command_struct *command,
                                             uint32 source_ms)
{
    (void)transport;
    (void)source_ms;
    published_command = *command;
    publish_count++;
    return 1U;
}

uint8 dl1b_safety_init(uint32 now_ms)
{
    (void)now_ms;
    tof_init_count++;
    return 1U;
}

void dl1b_safety_update(uint32 now_ms)
{
    (void)now_ms;
}

single_gap_tof_snapshot_struct dl1b_safety_get_snapshot(void)
{
    return mock_tof;
}

void camera_frame_consumer_set_handler(camera_frame_handler_fn handler)
{
    registered_handler = handler;
}

uint32 camera_frame_consumer_now_ms(void)
{
    return mock_now_ms;
}

static void reset_inputs(uint32 now_ms)
{
    memset((void *)&mock_shared, 0, sizeof(mock_shared));
    memset(&published_command, 0, sizeof(published_command));
    memset(&mock_pose, 0, sizeof(mock_pose));
    memset(&mock_tof, 0, sizeof(mock_tof));
    memset(pixels, 50, sizeof(pixels));
    publish_count = 0U;
    mock_now_ms = now_ms;
    navigation_attach_count = 0U;
    pose_attach_count = 0U;
    tof_init_count = 0U;
    registered_handler = NULL;
    mock_pose.sequence = 1U;
    mock_pose.timestamp_us = now_ms * 1000U;
    mock_pose.validity_flags = PERCEPTION_POSE_VALID_ODOMETRY;
    mock_tof.initialized = 1U;
    mock_tof.valid = 1U;
    mock_tof.sample_ms = now_ms;
    mock_tof.distance_mm = 1000U;
}

static camera_vision_frame_view_struct make_frame(uint32 sequence,
                                                  uint32 capture_ms,
                                                  uint32 frame_age_ms)
{
    camera_vision_frame_view_struct frame;

    memset(&frame, 0, sizeof(frame));
    frame.sequence = sequence;
    frame.capture_ms = capture_ms;
    frame.frame_age_ms = frame_age_ms;
    frame.width = TEST_WIDTH;
    frame.height = TEST_HEIGHT;
    frame.stride = TEST_WIDTH;
    frame.frame_bytes = sizeof(pixels);
    frame.pixels = pixels;
    return frame;
}

#if defined(TEST_FEATURE_DISABLED)
static void test_disabled_feature_touches_no_hardware_or_transport(void)
{
    camera_vision_frame_view_struct frame;

    reset_inputs(40U);
    TEST_CHECK(0U == single_gap_app_init());
    frame = make_frame(1U, 1000U, 0U);
    single_gap_app_on_frame(&frame);
    single_gap_app_service(40U);
    TEST_CHECK(NULL == registered_handler);
    TEST_CHECK(0U == navigation_attach_count);
    TEST_CHECK(0U == pose_attach_count);
    TEST_CHECK(0U == tof_init_count);
    TEST_CHECK(0U == publish_count);
}
#elif defined(TEST_MOTION_ENABLED)
static void draw_cone(uint16 center_x,
                      uint16 top_y,
                      uint16 bottom_y,
                      uint16 bottom_width)
{
    uint16 height = bottom_y - top_y;
    uint16 left;
    uint16 right;
    uint16 width;
    uint16 x;
    uint16 y;

    for(y = top_y; y <= bottom_y; y++)
    {
        width = (uint16)(3U + ((bottom_width - 3U) *
                               (y - top_y)) / height);
        left = center_x - width / 2U;
        right = center_x + width / 2U;
        for(x = left; x <= right; x++)
        {
            pixels[y * TEST_WIDTH + x] = 210U;
        }
        if((y >= (top_y + (height * 2U) / 5U)) &&
           (y <= (top_y + (height * 3U) / 5U)))
        {
            for(x = left; x <= right; x++)
            {
                pixels[y * TEST_WIDTH + x] = 255U;
            }
        }
    }
}

static void test_motion_build_requires_five_valid_frames(void)
{
    camera_vision_frame_view_struct frame;
    uint32 index;
    uint32 now_ms;

    reset_inputs(0U);
    draw_cone(65U, 38U, 104U, 53U);
    draw_cone(123U, 38U, 104U, 53U);
    TEST_CHECK(0U == single_gap_app_init());
    for(index = 1U; index <= 5U; index++)
    {
        now_ms = index * 40U;
        mock_now_ms = now_ms;
        mock_tof.sample_ms = now_ms;
        mock_pose.sequence = index;
        frame = make_frame(index, 1000U + index, 0U);
        single_gap_app_on_frame(&frame);
        single_gap_app_service(now_ms);
    }
    TEST_CHECK(5U == publish_count);
    TEST_CHECK(1U == published_command.enable);
    TEST_CHECK(NAVIGATION_STOP_NONE == published_command.stop_reason);
    TEST_CHECK(float_near(60.0f, published_command.forward_rpm, 0.001f));
}
#elif !defined(TEST_RPM_LIMIT)
static void test_sensing_only_publishes_disabled_command(void)
{
    camera_vision_frame_view_struct frame;

    reset_inputs(40U);
    TEST_CHECK(0U == single_gap_app_init());
    TEST_CHECK(NULL != registered_handler);
    frame = make_frame(1U, 1000U, 0U);
    registered_handler(&frame);
    single_gap_app_service(40U);
    TEST_CHECK(1U == publish_count);
    TEST_CHECK(NAVIGATION_SOURCE_VISION == published_command.source);
    TEST_CHECK(NAVIGATION_MODE_VISION_ASSIST == published_command.mode);
    TEST_CHECK(100U == published_command.valid_for_ms);
    TEST_CHECK(0U != published_command.source_sequence);
    TEST_CHECK(0U == published_command.enable);
    TEST_CHECK(NAVIGATION_STOP_DISABLED == published_command.stop_reason);
}

static void test_stale_frame_and_near_tof_publish_explicit_stops(void)
{
    camera_vision_frame_view_struct frame;

    reset_inputs(200U);
    TEST_CHECK(0U == single_gap_app_init());
    frame = make_frame(2U, 1000U, 200U);
    single_gap_app_on_frame(&frame);
    single_gap_app_service(200U);
    TEST_CHECK(1U == publish_count);
    TEST_CHECK(0U == published_command.enable);
    TEST_CHECK(NAVIGATION_STOP_STALE == published_command.stop_reason);

    reset_inputs(40U);
    mock_tof.distance_mm = 350U;
    TEST_CHECK(0U == single_gap_app_init());
    frame = make_frame(3U, 1000U, 0U);
    single_gap_app_on_frame(&frame);
    single_gap_app_service(40U);
    TEST_CHECK(1U == publish_count);
    TEST_CHECK(0U == published_command.enable);
    TEST_CHECK(NAVIGATION_STOP_EMERGENCY == published_command.stop_reason);
}

static void test_speed_conversion(void)
{
    TEST_CHECK(float_near(60.0f,
                          single_gap_speed_mps_to_rpm(0.20f, 0.20f),
                          0.001f));
    TEST_CHECK(0.0f == single_gap_speed_mps_to_rpm(0.20f, 0.0f));
}
#else
static void test_configured_rpm_above_limit_is_invalid(void)
{
    camera_vision_frame_view_struct frame;

    reset_inputs(40U);
    TEST_CHECK(0U == single_gap_app_init());
    frame = make_frame(1U, 1000U, 0U);
    single_gap_app_on_frame(&frame);
    single_gap_app_service(40U);
    TEST_CHECK(1U == publish_count);
    TEST_CHECK(0U == published_command.enable);
    TEST_CHECK(NAVIGATION_STOP_INVALID == published_command.stop_reason);
    TEST_CHECK(0.0f == published_command.forward_rpm);
}
#endif

int main(void)
{
#if defined(TEST_FEATURE_DISABLED)
    test_disabled_feature_touches_no_hardware_or_transport();
#elif defined(TEST_MOTION_ENABLED)
    test_motion_build_requires_five_valid_frames();
#elif !defined(TEST_RPM_LIMIT)
    test_sensing_only_publishes_disabled_command();
    test_stale_frame_and_near_tof_publish_explicit_stops();
    test_speed_conversion();
#else
    test_configured_rpm_above_limit_is_invalid();
#endif

    if(0U != failure_count)
    {
        printf("single_gap_app_test: %u failure(s)\n", (unsigned)failure_count);
        return 1;
    }
    puts("single_gap_app_test: PASS");
    return 0;
}
