
#include "Test_includes.hpp"


extern "C" {
    TEST_GROUP(T1_CPP);


    static Sensor_ts dut;


    TEST_SETUP(T1_CPP)
    {
        // 'main' initializes at startup, so either init everything or nothing at all, otherwise communication will be lost !
        (void) TLE493D_A2B6_init(&dut);
        initI2CComLibIF(&dut, Wire);
        setDefaultConfig(&dut);
    }


    TEST_TEAR_DOWN(T1_CPP)
    {
        // If deinitializing here make sure to reinit in 'TEST_SETUP' or communication will be lost !
        (void) TLE493D_A2B6_deinit(&dut);
    }


    TEST(T1_CPP, calculateTemperature)
    {
        float temperature = 0.0;
        // TEST_ASSERT_EQUAL( true, gen_2_readRegisters(&dut) );
        // TEST_ASSERT_EQUAL( true, readRegisters(&dut) );

        TEST_ASSERT_EQUAL( true, TLE493D_A2B6_getTemperature(&dut, &temperature) );
        TEST_ASSERT_FLOAT_WITHIN( 20.0, 25.0, temperature );

        TEST_ASSERT_EQUAL_HEX( 0x94, dut.regMap[0x11] );

        // TLE493D_A2B6_calculateTemperature(&dut, &temperature);
        // TEST_ASSERT_FLOAT_WITHIN( 1.0, -GEN_2_TEMP_OFFSET * GEN_2_TEMP_MULT + GEN_2_TEMP_REF, temperature );
    }


    TEST(T1_CPP, dummy)
    {
        TEST_ASSERT( true == !false );
    }


    TEST_GROUP_RUNNER(T1_CPP)
    {
        RUN_TEST_CASE(T1_CPP, calculateTemperature);
        RUN_TEST_CASE(T1_CPP, dummy);
    }
}