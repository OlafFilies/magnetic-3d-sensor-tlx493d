// std includes
#include <string.h>

// test includes
#include "Test_includes.h"
#include "Test_utils.h"


// variables used in the tests below that have to be accessed in the setup and tear down methods
static Sensor_ts dut;


// define test group name
TEST_GROUP(TLE493D_A2B6);


// Setup method called before every individual test defined for this test group
TEST_SETUP(TLE493D_A2B6)
{
    for(auto c : "\nTEST_SETUP(TLE493D_A2B6) ...\n\n")
        putCharacter(c);

    // 'main' initializes at startup, so either init everything or nothing at all, otherwise communication will be lost !
    (void) TLE493D_A2B6_init(&dut);

    memset(dut.regMap, 0, dut.regMapSize);
}


// Tear down method called before every individual test defined for this test group
TEST_TEAR_DOWN(TLE493D_A2B6)
{
    for(auto c : "TEST_TEAR_DOWN(TLE493D_A2B6) ...\n\n")
        putCharacter(c);

    // If deinitializing here make sure to reinit in 'TEST_SETUP' or communication will be lost !
    (void) TLE493D_A2B6_deinit(&dut);
}


// Define all relevant tests for the sensor device

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


// Bundle all tests to be executed for this test group
TEST_GROUP_RUNNER(TLE493D_A2B6)
{
    RUN_TEST_CASE(TLE493D_A2B6, calculateTemperature);
    RUN_TEST_CASE(TLE493D_A2B6, dummy);
}
