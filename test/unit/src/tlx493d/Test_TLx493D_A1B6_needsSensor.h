// test includes
#include "Test_includes.h"


void TLx493D_A1B6_needsSensor_suiteSetup(void);
void TLx493D_A1B6_needsSensor_suiteTearDown(void);
void TLx493D_A1B6_atReset_suiteSetup(void);
void TLx493D_A1B6_atReset_suiteTearDown(void);


// variables used in the tests below that have to be accessed in the setup and tear down methods
static TLx493D_t dut;


#include "Test_tlx493d_commonFunctions_needsSensor.h"
#include "Test_tlx493d_common_needsSensor.h"
#include "Test_tlx493d_common.h"
#include "Test_tlx493d_gen_1_common_needsSensor.h"
// #include "Test_tlx493d_gen_1_common.h"


// define test group name
TEST_GROUP(TLx493D_A1B6_needsSensor);


// Setup method called before every individual test defined for this test group
TEST_SETUP(TLx493D_A1B6_needsSensor)
{
}


// Tear down method called before every individual test defined for this test group
TEST_TEAR_DOWN(TLx493D_A1B6_needsSensor)
{
}


TEST(TLx493D_A1B6_needsSensor, readRegisters)
{
    TEST_ASSERT_EQUAL( true, TLx493D_A1B6_readRegisters(&dut) );
}

TEST(TLx493D_A1B6_needsSensor, getTemperature)
{
    double temperature = 0.0;

    TEST_ASSERT_EQUAL( true, TLx493D_A1B6_getTemperature(&dut, &temperature));
    TEST_ASSERT_FLOAT_WITHIN( 5.0, 25.0, temperature ); // Environmental temp is around 25 - 30 deg C
}

TEST(TLx493D_A1B6_needsSensor, getMagneticField)
{
    double x = 0.0, y = 0.0, z = 0.0;

    TEST_ASSERT_EQUAL( true, TLx493D_A1B6_getMagneticField(&dut, &x, &y, &z) );
    TEST_ASSERT_FLOAT_WITHIN( 1.0, 0.0, x ); // Residual mag field stays within 0-1 mT
    TEST_ASSERT_FLOAT_WITHIN( 1.0, 0.0, y );
    TEST_ASSERT_FLOAT_WITHIN( 1.0, 0.0, z );
}

// define test group name
TEST_GROUP(TLx493D_A1B6_TempDisable);

// Setup method called before every individual test defined for this test group
TEST_SETUP(TLx493D_A1B6_TempDisable)
{
    TLx493D_A1B6_disableTemperatureMeasurement(&dut);
    // dut.functions->disableTemperatureMeasurement(&dut);
}


// Tear down method called before every individual test defined for this test group
TEST_TEAR_DOWN(TLx493D_A1B6_TempDisable)
{
    TLx493D_A1B6_enableTemperatureMeasurement(&dut);
    // dut.functions->enableTemperatureMeasurement(&dut);
}

TEST(TLx493D_A1B6_TempDisable, TempDisable)
{
    // CH cannot be 0b11 when temperature measurement is disabled
    // TEST_ASSERT_NOT_EQUAL ( 0x3, ( dut.regMap[dut.commonBitfields.CH] & dut.regDef[dut.commonBitfields.CH].mask) >> dut.regDef[dut.commonBitfields.CH].offset );
    
    double temperature = 0.0;
    TEST_ASSERT_EQUAL( true, TLx493D_A1B6_getTemperature(&dut, &temperature));
    
    // When temperature measurement is disabled in runtime, the last measured value remains in the register
    // sample 50 values of temperature after temp disable and check if they are equal to the value measured above.
    double temp_sum = 0.0;

    for(uint8_t i=0; i<50; i++)
    {
        double temp_temp = 0.0;
        TLx493D_A1B6_getTemperature(&dut, &temp_temp);
        temp_sum += temp_temp;
    }

    TEST_ASSERT_EQUAL_FLOAT(temp_sum/50, temperature);
}

// define test group name
TEST_GROUP(TLx493D_A1B6_ParityCheck);

// Setup method called before every individual test defined for this test group
TEST_SETUP(TLx493D_A1B6_ParityCheck)
{
}


// Tear down method called before every individual test defined for this test group
TEST_TEAR_DOWN(TLx493D_A1B6_ParityCheck)
{
    //TLx493D_A1B6_enableParityTest(dut);
}

TEST(TLx493D_A1B6_ParityCheck, SetWrongParity_ParityCheckDisabled)
{
    // // first, disable Parity check.
    // TEST_ASSERT_EQUAL( true, TLx493D_A1B6_disableParityTest(&dut) );
    
    // // here, changine a Write Register bitField and not recalculating parity and not setting
    // // it in the bitField.P, should give an error
    // TLx493D_A1B6_setBitfield(&dut, dut.commonBitfields.Temp_NEN,1);
    // TEST_ASSERT_EQUAL( true, TLx493D_A1B6_transferWriteRegisters(&dut));

    // // the readRegisters() throws NO error due to wrong parity
    // TEST_ASSERT_EQUAL( true, TLx493D_A1B6_readRegisters(&dut) );
    
    // // enable partity check at the end, to restore default state for other tests
    // TEST_ASSERT_EQUAL( true, TLx493D_A1B6_enableParityTest(&dut) );
}

TEST(TLx493D_A1B6_ParityCheck, SetWrongParity_ParityCheckEnabled)
{
    // TODO: setup anew
    // // parity check is enabled, by default
    
    // // here, changine a Write Register bitField and not recalculating parity and not setting
    // // it in the bitField.P, should give an error
    // TLx493D_A1B6_setBitfield(&dut, dut.commonBitfields.Temp_NEN,0);
    // TEST_ASSERT_EQUAL( true, TLx493D_A1B6_transferWriteRegisters(&dut) );

    // // the readRegisters() throws error due to wrong parity
    // TEST_ASSERT_EQUAL( false, TLx493D_A1B6_readRegisters(&dut) );
}



// define test group name
TEST_GROUP(TLx493D_A1B6_atReset);

// Setup method called before every individual test defined for this test group
TEST_SETUP(TLx493D_A1B6_atReset)
{
    TLx493D_A1B6_readRegisters(&dut);
}


// Tear down method called before every individual test defined for this test group
TEST_TEAR_DOWN(TLx493D_A1B6_atReset)
{
}

TEST(TLx493D_A1B6_atReset, regMapatReset_MeasurementBitfields)
{
    // TODO: setup anew
    // TEST_ASSERT_EQUAL_UINT8 ( 0x00, ( dut.regMap[dut.commonBitfields.BX_MSB] & dut.regDef[dut.commonBitfields.BX_MSB].mask) >> dut.regDef[dut.commonBitfields.BX_MSB].offset  );
    // TEST_ASSERT_EQUAL_UINT8 ( 0x00, ( dut.regMap[dut.commonBitfields.BY_MSB] & dut.regDef[dut.commonBitfields.BY_MSB].mask) >> dut.regDef[dut.commonBitfields.BY_MSB].offset  );
    // TEST_ASSERT_EQUAL_UINT8 ( 0x00, (dut.regMap[dut.commonBitfields.BZ_MSB] & dut.regDef[dut.commonBitfields.BZ_MSB].mask) >> dut.regDef[dut.commonBitfields.BZ_MSB].offset  );
    // TEST_ASSERT_EQUAL_UINT8 ( 0x01, (dut.regMap[dut.commonBitfields.TEMP_MSB] & dut.regDef[dut.commonBitfields.TEMP_MSB].mask) >> dut.regDef[dut.commonBitfields.TEMP_MSB].offset );
    
    // TEST_ASSERT_EQUAL_UINT8 ( 0x5, ( dut.regMap[dut.commonBitfields.BX_LSB] & dut.regDef[dut.commonBitfields.BX_LSB].mask) >> dut.regDef[dut.commonBitfields.BX_LSB].offset );
    // TEST_ASSERT_EQUAL_UINT8 ( 0x3, ( dut.regMap[dut.commonBitfields.BY_LSB] & dut.regDef[dut.commonBitfields.BY_LSB].mask) >> dut.regDef[dut.commonBitfields.BY_LSB].offset );

    // TEST_ASSERT_EQUAL_UINT8 ( 0x3 , ( dut.regMap[dut.commonBitfields.BZ_LSB] & dut.regDef[dut.commonBitfields.BZ_LSB].mask) >> dut.regDef[dut.commonBitfields.BZ_LSB].offset );
    // TEST_ASSERT_EQUAL_UINT8 ( 0x0 , ( dut.regMap[dut.commonBitfields.TEMP_LSB] & dut.regDef[dut.commonBitfields.TEMP_LSB].mask) >> dut.regDef[dut.commonBitfields.TEMP_LSB].offset );
}

TEST(TLx493D_A1B6_atReset, regMapatReset_nonMeasurementBitfields)
{
    // TODO: setup anew
    // TEST_ASSERT_EQUAL_UINT8 ( 0x0, ( dut.regMap[dut.commonBitfields.FRM] & dut.regDef[dut.commonBitfields.FRM].mask) >> dut.regDef[dut.commonBitfields.FRM].offset );
    // TEST_ASSERT_UINT8_WITHIN ( 0x3, 0x0, ( dut.regMap[dut.commonBitfields.CH] & dut.regDef[dut.commonBitfields.CH].mask) >> dut.regDef[dut.commonBitfields.CH].offset );
    // TEST_ASSERT_EQUAL_UINT8 ( 0x0, ( dut.regMap[dut.commonBitfields.T] & dut.regDef[dut.commonBitfields.T].mask) >> dut.regDef[dut.commonBitfields.T].offset );
    // TEST_ASSERT_EQUAL_UINT8 ( 0x1, ( dut.regMap[dut.commonBitfields.FF] & dut.regDef[dut.commonBitfields.FF].mask) >> dut.regDef[dut.commonBitfields.FF].offset );
    // TEST_ASSERT_EQUAL_UINT8 ( 0x0, ( dut.regMap[dut.commonBitfields.PD] & dut.regDef[dut.commonBitfields.PD].mask) >> dut.regDef[dut.commonBitfields.PD].offset );
}

// Bundle all tests to be executed for this test group
TEST_GROUP_RUNNER(TLx493D_A1B6_needsSensor)
{
    TLx493D_A1B6_needsSensor_suiteSetup();

    RUN_TEST_CASE(TLx493D_A1B6_needsSensor, readRegisters);
    RUN_TEST_CASE(TLx493D_A1B6_needsSensor, getTemperature);
    RUN_TEST_CASE(TLx493D_A1B6_needsSensor, getMagneticField);

    //RUN_TEST_GROUP(TLx493D_A1B6_TempDisable);
    RUN_TEST_CASE(TLx493D_A1B6_TempDisable, TempDisable);
    RUN_TEST_CASE(TLx493D_A1B6_ParityCheck, SetWrongParity_ParityCheckDisabled);
    
    
    
    // run common functions tests
    RUN_TEST_GROUP(SensorsCommonFunctions);

    // run gen 1 common functions tests
    RUN_TEST_GROUP(SensorsCommon);
    RUN_TEST_GROUP(SensorsCommon_needsSensor);
    // RUN_TEST_GROUP(SensorsGen1Common);
    // RUN_TEST_GROUP(SensorsGen1Common_needsSensor);


    // This test is run at the end since the sensor cannot recover from a wrong parity error without hard reset
    // So tests will fail in the loop second time onwards. First time everything will pass. 
    //RUN_TEST_CASE(TLx493D_A1B6_ParityCheck, SetWrongParity_ParityCheckEnabled);
    TLx493D_A1B6_needsSensor_suiteTearDown();

    TLx493D_A1B6_atReset_suiteSetup();

    // This test is commented out since we dont check measurement bitfields (BX,BY,BZ,TEMP) at reset.
    // TODO: Remove
    //RUN_TEST_CASE(TLx493D_A1B6_atReset, regMapatReset_MeasurementBitfields);
    RUN_TEST_CASE(TLx493D_A1B6_atReset, regMapatReset_nonMeasurementBitfields);

    TLx493D_A1B6_atReset_suiteTearDown();

}
