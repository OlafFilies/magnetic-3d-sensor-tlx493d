#ifndef TEST_TLX493D_COMMON_NEEDS_SENSOR_H
#define TEST_TLX493D_COMMON_NEEDS_SENSOR_H


// test includes
#include "Test_includes.h"


// Variables used in the tests below that have to be accessed in the setup and tear down methods.
// The "dut" variable is taken from the respective sensor that the common functions are applied to.


// define test group name
TEST_GROUP(SensorsCommon_needsSensor);


// Setup method called before every individual test defined for this test group
static TEST_SETUP(SensorsCommon_needsSensor)
{
}


// Tear down method called before every individual test defined for this test group
static TEST_TEAR_DOWN(SensorsCommon_needsSensor)
{
}


TEST_IFX(SensorsCommon_needsSensor, checkGetMagneticFieldAndTemperature)
{
    double  x, y, z, t;
    double  xl, xh, yl, yh, zl, zh;
    int16_t xl_i, xh_i, yl_i, yh_i, zl_i, zh_i;

    TEST_ASSERT( tlx493d_common_getTemperature(&dut, &t) == true );
    TEST_ASSERT_FLOAT_WITHIN( 20.0, 25.0, t );

    tlx493d_common_getMagneticField(&dut, &x, &y, &z);
    TEST_ASSERT_FLOAT_WITHIN( 1.0, 0.0, x );
    TEST_ASSERT_FLOAT_WITHIN( 1.0, 0.0, y );
    TEST_ASSERT_FLOAT_WITHIN( 1.0, 0.0, z );

    x = 0.0;
    y = 0.0;
    z = 0.0;
    t = 0.0;
    tlx493d_common_getMagneticFieldAndTemperature(&dut, &x, &y, &z, &t);
    TEST_ASSERT_FLOAT_WITHIN( 20.0, 25.0, t );
    TEST_ASSERT_FLOAT_WITHIN( 1.0, 0.0, x );
    TEST_ASSERT_FLOAT_WITHIN( 1.0, 0.0, y );
    TEST_ASSERT_FLOAT_WITHIN( 1.0, 0.0, z );
}


// Bundle all tests to be executed for this test group
static TEST_GROUP_RUNNER(SensorsCommon_needsSensor)
{
    RUN_TEST_CASE(SensorsCommon_needsSensor, checkGetMagneticFieldAndTemperature);
}


#endif // TEST_TLX493D_COMMON_NEEDS_SENSOR_H
