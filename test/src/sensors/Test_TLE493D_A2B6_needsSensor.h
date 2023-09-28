// test includes
#include "Test_includes.h"
// #include "Test_utils.h"


// variables used in the tests below that have to be accessed in the setup and tear down methods
static Sensor_ts dut;


// define test group name
TEST_GROUP(TLE493D_A2B6_needsSensor);


// Setup method called before every individual test defined for this test group
TEST_SETUP(TLE493D_A2B6_needsSensor)
{
    // for(auto c : "\nTEST_SETUP(TLE493D_A2B6_needsSensor) ...\n\n")
    //     putCharacter(c);

    // // 'main' initializes at startup, so either init everything or nothing at all, otherwise communication will be lost !
    // (void) TLE493D_A2B6_init(&dut);
    // initI2CComLibIF(&dut, Wire);
    // setDefaultConfig(&dut);
}


// Tear down method called before every individual test defined for this test group
TEST_TEAR_DOWN(TLE493D_A2B6_needsSensor)
{
    // for(auto c : "TEST_TEAR_DOWN(TLE493D_A2B6_needsSensor) ...\n\n")
    //     putCharacter(c);

    // // If deinitializing here make sure to reinit in 'TEST_SETUP' or communication will be lost !
    // (void) TLE493D_A2B6_deinit(&dut);
}


// Define all relevant tests for the sensor device

TEST(TLE493D_A2B6_needsSensor, readRegisters)
{
    TEST_ASSERT_EQUAL( true, gen_2_readRegisters(&dut) );
//    TEST_ASSERT_EQUAL( true, readRegisters(&dut) );
}


TEST(TLE493D_A2B6_needsSensor, getTemperature)
{
    float temperature = 0.0;

    TEST_ASSERT_EQUAL( true, dut.functions->getTemperature(&dut, &temperature));
    TEST_ASSERT_FLOAT_WITHIN( 20.0, 25.0, temperature );
}


// Check if setDefaultConfig worked properly and data can be read and expected values are set.
TEST(TLE493D_A2B6_needsSensor, defaultConfig)
{
    TEST_ASSERT_EQUAL_HEX( 0x94, dut.regMap[dut.commonRegisters.MOD1] );
}


TEST(TLE493D_A2B6_needsSensor, dummy)
{
    TEST_ASSERT( true == !false );
}


// Bundle all tests to be executed for this test group
TEST_GROUP_RUNNER(TLE493D_A2B6_needsSensor)
{
    RUN_TEST_CASE(TLE493D_A2B6_needsSensor, readRegisters);
    RUN_TEST_CASE(TLE493D_A2B6_needsSensor, getTemperature);
    RUN_TEST_CASE(TLE493D_A2B6_needsSensor, defaultConfig);
    RUN_TEST_CASE(TLE493D_A2B6_needsSensor, dummy);
}
