#include <stdio.h>

#include "dl1b_safety.h"
#include "dl1b_safety_port.h"

static uint32 failure_count;
static uint8 mock_init_result;
static uint8 mock_read_valid;
static uint16 mock_distance_mm;
static uint32 mock_read_count;

#define TEST_CHECK(condition)                                                        \
    do                                                                               \
    {                                                                                \
        if(!(condition))                                                             \
        {                                                                            \
            failure_count++;                                                         \
            printf("FAIL:%u: %s\n", (unsigned)__LINE__, #condition);               \
        }                                                                            \
    } while(0)

uint8 dl1b_safety_port_init(void)
{
    return mock_init_result;
}

uint8 dl1b_safety_port_read(uint16 *distance_mm)
{
    mock_read_count++;
    if((0U == mock_read_valid) || (NULL == distance_mm))
    {
        return 0U;
    }
    *distance_mm = mock_distance_mm;
    return 1U;
}

static void reset_mock(void)
{
    mock_init_result = 0U;
    mock_read_valid = 0U;
    mock_distance_mm = 8192U;
    mock_read_count = 0U;
}

static void test_init_failure_is_invalid(void)
{
    single_gap_tof_snapshot_struct sample;

    reset_mock();
    TEST_CHECK(0U == dl1b_safety_init(0U));
    sample = dl1b_safety_get_snapshot();
    TEST_CHECK(0U == sample.initialized);
    TEST_CHECK(0U == sample.valid);
    TEST_CHECK(0U == mock_read_count);
}

static void test_poll_period_and_valid_sample(void)
{
    single_gap_tof_snapshot_struct sample;

    reset_mock();
    mock_init_result = 1U;
    mock_read_valid = 1U;
    mock_distance_mm = 500U;
    TEST_CHECK(1U == dl1b_safety_init(0U));
    dl1b_safety_update(49U);
    TEST_CHECK(0U == mock_read_count);
    TEST_CHECK(0U == dl1b_safety_get_snapshot().valid);

    dl1b_safety_update(50U);
    sample = dl1b_safety_get_snapshot();
    TEST_CHECK(1U == mock_read_count);
    TEST_CHECK(1U == sample.initialized);
    TEST_CHECK(1U == sample.valid);
    TEST_CHECK(500U == sample.distance_mm);
    TEST_CHECK(50U == sample.sample_ms);

    dl1b_safety_update(99U);
    TEST_CHECK(1U == mock_read_count);
}

static void test_invalid_read_clears_validity(void)
{
    reset_mock();
    mock_init_result = 1U;
    mock_read_valid = 1U;
    mock_distance_mm = 350U;
    TEST_CHECK(1U == dl1b_safety_init(0U));
    dl1b_safety_update(50U);
    TEST_CHECK(1U == dl1b_safety_get_snapshot().valid);
    TEST_CHECK(350U == dl1b_safety_get_snapshot().distance_mm);

    mock_read_valid = 0U;
    dl1b_safety_update(100U);
    TEST_CHECK(0U == dl1b_safety_get_snapshot().valid);
    TEST_CHECK(50U == dl1b_safety_get_snapshot().sample_ms);
}

int main(void)
{
    test_init_failure_is_invalid();
    test_poll_period_and_valid_sample();
    test_invalid_read_clears_validity();

    if(0U != failure_count)
    {
        printf("single_gap_dl1b_test: %u failure(s)\n", (unsigned)failure_count);
        return 1;
    }
    puts("single_gap_dl1b_test: PASS");
    return 0;
}
