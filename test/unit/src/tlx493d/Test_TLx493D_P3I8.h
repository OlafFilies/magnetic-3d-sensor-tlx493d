// std includes
#include <string.h>

// test includes
#include "Test_includes.h"


void TLx493D_P3I8_suiteSetUp(void);
void TLx493D_P3I8_suiteTearDown(void);


// variables used in the tests below that have to be accessed in the setup and tear down methods
static TLx493D_t dut;


// test includes that may require dut
#include "Test_tlx493d_common.h"
#include "Test_tlx493d_gen_3_common.h"


// define test group name
TEST_GROUP(TLx493D_P3I8);


// Setup method called before every individual test defined for this test group
TEST_SETUP(TLx493D_P3I8)
{
}


// Tear down method called before every individual test defined for this test group
TEST_TEAR_DOWN(TLx493D_P3I8)
{
}


// Define all relevant tests for the sensor device
TEST(TLx493D_P3I8, dummy)
{
    TEST_ASSERT( true == !false );
}


// Bundle all tests to be executed for this test group
TEST_GROUP_RUNNER(TLx493D_P3I8)
{
    TLx493D_P3I8_suiteSetUp();

    RUN_TEST_CASE(TLx493D_P3I8, dummy);

    // run gen 2 common functions tests
    RUN_TEST_GROUP(SensorsCommon);
    RUN_TEST_GROUP(SensorsGen2Common);

    TLx493D_P3I8_suiteTearDown();
}
