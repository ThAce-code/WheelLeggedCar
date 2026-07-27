#include <stdio.h>
#include <string.h>

#include "single_gap_detector.h"

#define TEST_WIDTH  (188U)
#define TEST_HEIGHT (120U)

static uint32 failure_count;
static uint8 image[TEST_WIDTH * TEST_HEIGHT];

#define TEST_CHECK(condition)                                                        \
    do                                                                               \
    {                                                                                \
        if(!(condition))                                                             \
        {                                                                            \
            failure_count++;                                                         \
            printf("FAIL:%u: %s\n", (unsigned)__LINE__, #condition);               \
        }                                                                            \
    } while(0)

static void clear_image(uint8 value)
{
    memset(image, value, sizeof(image));
}

static void draw_cone(uint16 center_x,
                      uint16 top_y,
                      uint16 bottom_y,
                      uint16 bottom_width)
{
    uint16 half_width;
    uint16 height;
    uint16 left;
    uint16 right;
    uint16 width;
    uint16 x;
    uint16 y;

    half_width = bottom_width / 2U;
    height = bottom_y - top_y;
    for(y = top_y; y <= bottom_y; y++)
    {
        width = (uint16)(3U + ((bottom_width - 3U) * (y - top_y)) / height);
        left = center_x - width / 2U;
        right = center_x + width / 2U;
        for(x = left; x <= right; x++)
        {
            image[y * TEST_WIDTH + x] = 210U;
        }
        if((y >= (top_y + (height * 2U) / 5U)) &&
           (y <= (top_y + (height * 3U) / 5U)))
        {
            for(x = left; x <= right; x++)
            {
                image[y * TEST_WIDTH + x] = 255U;
            }
        }
    }
    (void)half_width;
}

static void test_centered_pair_produces_gap(void)
{
    single_gap_observation_struct observation;

    clear_image(50U);
    draw_cone(65U, 38U, 104U, 53U);
    draw_cone(123U, 38U, 104U, 53U);
    TEST_CHECK(1U == single_gap_detector_process(image,
                                                  TEST_WIDTH,
                                                  TEST_HEIGHT,
                                                  TEST_WIDTH,
                                                  1U,
                                                  20U,
                                                  &observation));
    TEST_CHECK(1U == observation.valid);
    TEST_CHECK(0U == observation.ambiguous);
    TEST_CHECK(2U == observation.accepted_count);
    TEST_CHECK(94U == observation.gap_center_x);
    TEST_CHECK(58U == observation.gap_width_px);
    TEST_CHECK(104U == observation.left_bottom_y);
    TEST_CHECK(104U == observation.right_bottom_y);
}

static void test_one_and_three_cones_are_not_a_gap(void)
{
    single_gap_observation_struct observation;

    clear_image(50U);
    draw_cone(65U, 38U, 104U, 53U);
    TEST_CHECK(1U == single_gap_detector_process(image, 188U, 120U, 188U,
                                                  2U, 60U, &observation));
    TEST_CHECK(0U == observation.valid);
    TEST_CHECK(1U == observation.accepted_count);

    clear_image(50U);
    draw_cone(35U, 38U, 104U, 39U);
    draw_cone(94U, 38U, 104U, 39U);
    draw_cone(153U, 38U, 104U, 39U);
    TEST_CHECK(1U == single_gap_detector_process(image, 188U, 120U, 188U,
                                                  3U, 100U, &observation));
    TEST_CHECK(0U == observation.valid);
    TEST_CHECK(1U == observation.ambiguous);
    TEST_CHECK(3U == observation.accepted_count);
}

static void test_flat_and_wrong_shape_are_rejected(void)
{
    single_gap_observation_struct observation;

    clear_image(30U);
    TEST_CHECK(1U == single_gap_detector_process(image, 188U, 120U, 188U,
                                                  4U, 140U, &observation));
    TEST_CHECK(0U == observation.valid);
    TEST_CHECK(0U == observation.accepted_count);

    TEST_CHECK(0U == single_gap_detector_process(image, 187U, 120U, 188U,
                                                  5U, 180U, &observation));
    TEST_CHECK(0U == observation.valid);
}

int main(void)
{
    test_centered_pair_produces_gap();
    test_one_and_three_cones_are_not_a_gap();
    test_flat_and_wrong_shape_are_rejected();

    if(0U != failure_count)
    {
        printf("single_gap_detector_test: %u failure(s)\n", (unsigned)failure_count);
        return 1;
    }
    puts("single_gap_detector_test: PASS");
    return 0;
}
