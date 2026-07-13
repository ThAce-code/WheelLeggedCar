#include <math.h>
#include <stdio.h>
#include <string.h>
#include "intercore_protocol.h"

static uint32 test_failure_count = 0U;

#define TEST_CHECK(condition)                                                   \
    do                                                                          \
    {                                                                           \
        if(!(condition))                                                        \
        {                                                                       \
            printf("FAIL:%s:%d: %s\n", __FILE__, __LINE__, #condition);         \
            test_failure_count++;                                               \
        }                                                                       \
    }while(0)

static void test_protocol_crc_and_sizes(void)
{
    static const uint8 sample[] = "123456789";

    TEST_CHECK(0xCBF43926UL == intercore_crc32(sample, 9U));
    TEST_CHECK(24U == sizeof(intercore_header_struct));
    TEST_CHECK(24U == sizeof(navigation_command_struct));
    TEST_CHECK(8192U == sizeof(intercore_shared_layout_struct));
}

static void test_navigation_structural_validation(void)
{
    navigation_command_struct command = {0};

    command.forward_rpm = 20.0f;
    command.turn_rate_dps = -10.0f;
    command.confidence = 0.75f;
    command.source_sequence = 1U;
    command.valid_for_ms = 200U;
    command.enable = 1U;
    command.source = NAVIGATION_SOURCE_VISION;
    command.mode = NAVIGATION_MODE_VISION_ASSIST;
    TEST_CHECK(1U == intercore_navigation_is_structurally_valid(&command));

    command.forward_rpm = NAN;
    TEST_CHECK(0U == intercore_navigation_is_structurally_valid(&command));
    command.forward_rpm = 20.0f;
    command.valid_for_ms = 201U;
    TEST_CHECK(0U == intercore_navigation_is_structurally_valid(&command));
    command.valid_for_ms = 200U;
    command.confidence = 1.1f;
    TEST_CHECK(0U == intercore_navigation_is_structurally_valid(&command));
}

static void test_record_rejects_corruption(void)
{
    intercore_header_struct header = {0};
    navigation_command_struct command = {0};

    command.forward_rpm = 15.0f;
    command.turn_rate_dps = 5.0f;
    command.confidence = 0.8f;
    command.source_sequence = 4U;
    command.valid_for_ms = 200U;
    command.enable = 1U;
    command.source = NAVIGATION_SOURCE_VISION;
    command.mode = NAVIGATION_MODE_VISION_ASSIST;

    intercore_record_prepare(&header,
                             INTERCORE_RECORD_NAVIGATION,
                             9U,
                             50U,
                             &command,
                             sizeof(command));
    TEST_CHECK(1U == intercore_record_validate(&header,
                                               INTERCORE_RECORD_NAVIGATION,
                                               &command,
                                               sizeof(command)));
    ((uint8 *)&command)[0] ^= 0x01U;
    TEST_CHECK(0U == intercore_record_validate(&header,
                                               INTERCORE_RECORD_NAVIGATION,
                                               &command,
                                               sizeof(command)));
}

int main(void)
{
    test_protocol_crc_and_sizes();
    test_navigation_structural_validation();
    test_record_rejects_corruption();

    if(0U != test_failure_count)
    {
        printf("intercore_control_foundation_test: %u failure(s)\n",
               (unsigned int)test_failure_count);
        return 1;
    }

    printf("intercore_control_foundation_test: PASS\n");
    return 0;
}
