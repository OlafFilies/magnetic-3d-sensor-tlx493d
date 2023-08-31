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


TEST_GROUP(TLE493D_A2B6_needsSensor);


extern Sensor_ts dut;


//This is run before EACH TEST
TEST_SETUP(TLE493D_A2B6_needsSensor)
{
    // 'main' initializes at startup, so either init everything or nothing at all, otherwise communication will be lost !
    // (void) TLE493D_A2B6_init(&dut);
    // initI2CComLibIF(&dut, Wire);
    // setDefaultConfig(&dut);
}


TEST_TEAR_DOWN(TLE493D_A2B6_needsSensor)
{
    // If deinitializing here make sure to reinit in 'TEST_SETUP' or communication will be lost !
    // (void) TLE493D_A2B6_deinit(&dut);
}


// Check if setDefaultConfig worked properly and data can be read and expected values are set.
TEST(TLE493D_A2B6_needsSensor, defaultConfig)
{
    float temperature = 0.0;
    TEST_ASSERT_EQUAL( true, TLE493D_A2B6_getTemperature(&dut, &temperature) );
    TEST_ASSERT_FLOAT_WITHIN( 20.0, 25.0, temperature );

    TEST_ASSERT_EQUAL_HEX( 0x94, dut.regMap[0x11] );
}
