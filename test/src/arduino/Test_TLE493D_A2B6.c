// std includes
#include <stdbool.h>

// project c includes
#include "sensor_types.h"
#include "sensors_common.h"
// std includes
#include <stdbool.h>

// project c includes
// common to all sensors
#include "sensor_types.h"
#include "sensors_common_defines.h"
#include "sensors_common.h"

// common to same generation of sensors
#include "sensors_gen_2_common_defines.h"
#include "sensors_gen_2_common.h"

// sensor specicifc includes
#include "TLE493D_A2B6_defines.h"
#include "TLE493D_A2B6.h"


// Unity c includes
#include "unity.h"
#include "unity_fixture.h"


TEST_GROUP(TLE493D_A2B6);


extern Sensor_ts dut;


TEST_SETUP(TLE493D_A2B6)
{
  //This is run before each test
    (void) TLE493D_A2B6_init(&dut);
}


TEST_TEAR_DOWN(TLE493D_A2B6)
{
    (void) TLE493D_A2B6_deinit(&dut);
}


TEST(TLE493D_A2B6, calculateTemperature)
{
    float temperature = 0.0;
    TLE493D_A2B6_calculateTemperature(&dut, &temperature);
    TEST_ASSERT_FLOAT_WITHIN( 1.0, -GEN_2_TEMP_OFFSET * GEN_2_TEMP_MULT + GEN_2_TEMP_REF, temperature );
}


TEST(TLE493D_A2B6, dummy)
{
    TEST_ASSERT( true == !false );
}
