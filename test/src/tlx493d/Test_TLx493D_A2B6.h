// std includes
#include <string.h>

// test includes
#include "Test_includes.h"
// #include "Test_utils.h"
#include "Test_tlx493d_common.h"
#include "Test_tlx493d_gen_2_common.h"


void TLx493D_A2B6_suiteSetUp(void);
void TLx493D_A2B6_suiteTearDown(void);


// variables used in the tests below that have to be accessed in the setup and tear down methods
static Sensor_ts dut;


// define test group name
TEST_GROUP(TLx493D_A2B6);


// Setup method called before every individual test defined for this test group
TEST_SETUP(TLx493D_A2B6)
{
    // for(auto c : "\nTEST_SETUP(TLx493D_A2B6) ...\n\n")
    //     putCharacter(c);

    // 'main' initializes at startup, so either init everything or nothing at all, otherwise communication will be lost !
    (void) TLx493D_A2B6_init(&dut);

    memset(dut.regMap, 0, dut.regMapSize);
}


// Tear down method called before every individual test defined for this test group
TEST_TEAR_DOWN(TLx493D_A2B6)
{
    // for(auto c : "TEST_TEAR_DOWN(TLx493D_A2B6) ...\n\n")
    //     putCharacter(c);

    // If deinitializing here make sure to reinit in 'TEST_SETUP' or communication will be lost !
    (void) TLx493D_A2B6_deinit(&dut);
}


// Define all relevant tests for the sensor device

TEST(TLx493D_A2B6, calculateTemperature)
{
    double temperature = 0.0;
    TLx493D_A2B6_calculateTemperature(&dut, &temperature);
    TEST_ASSERT_FLOAT_WITHIN( 1.0, -GEN_2_TEMP_OFFSET * GEN_2_TEMP_MULT + GEN_2_TEMP_REF, temperature );
}


TEST(TLx493D_A2B6, dummy)
{
    TEST_ASSERT( true == !false );
}


// Bundle all tests to be executed for this test group
TEST_GROUP_RUNNER(TLx493D_A2B6)
{
    TLx493D_A2B6_suiteSetUp();

    RUN_TEST_CASE(TLx493D_A2B6, calculateTemperature);
    RUN_TEST_CASE(TLx493D_A2B6, dummy);

    // run gen 2 common functions tests
    RUN_TEST_GROUP(SensorsCommon);
    RUN_TEST_GROUP(SensorsGen2Common);

    TLx493D_A2B6_suiteTearDown();
}
