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

    if(0U != failures)
    {
        return 1;
    }

    puts("gnss_local_position_test: PASS");
    return 0;
}
