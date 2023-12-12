// test includes
#include "Test_includes.h"


void TLx493D_A1B6_needsSensor_suiteSetup(void);
void TLx493D_A1B6_needsSensor_suiteTearDown(void);
void TLx493D_A1B6_atReset_suiteSetup(void);
void TLx493D_A1B6_atReset_suiteTearDown(void);


// variables used in the tests below that have to be accessed in the setup and tear down methods
static TLx493D_t dut;


// test includes that may require dut
#include "Test_tlx493d_commonFunctions_needsSensor.h"
#include "Test_tlx493d_common_needsSensor.h"
#include "Test_tlx493d_common.h"


// define test group name
TEST_GROUP(TLx493D_A1B6_needsSensor);
TEST_GROUP(TLx493D_A1B6_needsSensorInternal);
TEST_GROUP(TLx493D_A1B6_TempDisableInternal);
TEST_GROUP(TLx493D_A1B6_ParityCheckInternal);
// TEST_GROUP(TLx493D_A1B6_atResetInternal);


// static double  x, y, z, t;
// static double  xl, xh, yl, yh, zl, zh;
// static int16_t xl_i, xh_i, yl_i, yh_i, zl_i, zh_i;


// Setup method called before every individual test defined for this test group
static TEST_SETUP(TLx493D_A1B6_needsSensorInternal)
{
    // x = 0.0;
    // y = 0.0;
    // z = 0.0;
    // t = 0.0;

    // xl = 0.0;
    // xh = 0.0;
    // yl = 0.0;
    // yh = 0.0;
    // zl = 0.0;
    // zh = 0.0;

    // xl_i = 0;
    // xh_i = 0;
    // yl_i = 0;
    // yh_i = 0;
    // zl_i = 0;
    // zh_i = 0;
}


// Tear down method called before every individual test defined for this test group
static TEST_TEAR_DOWN(TLx493D_A1B6_needsSensorInternal)
{
}


// Setup method called before every individual test defined for this test group
static TEST_SETUP(TLx493D_A1B6_TempDisableInternal)
{
    TLx493D_A1B6_disableTemperatureMeasurement(&dut);
    // dut.functions->disableTemperatureMeasurement(&dut);
}


// Tear down method called before every individual test defined for this test group
static TEST_TEAR_DOWN(TLx493D_A1B6_TempDisableInternal)
{
    TLx493D_A1B6_enableTemperatureMeasurement(&dut);
    // dut.functions->enableTemperatureMeasurement(&dut);
}


static TEST_SETUP(TLx493D_A1B6_ParityCheckInternal)
{
}


// Tear down method called before every individual test defined for this test group
static TEST_TEAR_DOWN(TLx493D_A1B6_ParityCheckInternal)
{
    //TLx493D_A1B6_enableParityTest(dut);
}


// static TEST_SETUP(TLx493D_A1B6_atResetInternal)
// {
//     TLx493D_A1B6_readRegisters(&dut);
// }


// // Tear down method called before every individual test defined for this test group
// static TEST_TEAR_DOWN(TLx493D_A1B6_atResetInternal)
// {
// }


/**
 * Define tests for unsupported common functionality.
 */
TEST_IFX(TLx493D_A1B6_needsSensorInternal, checkUnsupportedFunctionality)
{
}


/**
 * Define tests for supported common functionality.
 * Requires that the registers have been read once, in setDefaultConfig.
 */

TEST_IFX(TLx493D_A1B6_needsSensorInternal, checkSupportedFunctionality)
{
    // // while( dut.functions->readRegisters(&dut) == false ) ;
    // TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    // // TEST_ASSERT( dut.functions->hasValidData(&dut) == true ); // fails sometimes
    // // TEST_ASSERT( dut.functions->hasValidBusParity(&dut) == true ); // fails sometimes

    // TEST_ASSERT( dut.functions->isFunctional(&dut) == true );

    // TEST_ASSERT( dut.functions->hasValidFuseParity(&dut) == true );
    // TEST_ASSERT( dut.functions->hasValidConfigurationParity(&dut) == true );

    // TEST_ASSERT( dut.functions->hasValidTBit(&dut) == true );
}


TEST_IFX(TLx493D_A1B6_needsSensorInternal, checkGetMagneticFieldAndTemperature)
{
    // TEST_ASSERT( dut.functions->getTemperature(&dut, &t) == true );
    // TEST_ASSERT_FLOAT_WITHIN( 20.0, 25.0, t );

    // dut.functions->calculateMagneticField(&dut, &x, &y, &z);
    // TEST_ASSERT_FLOAT_WITHIN( 1.0, 0.0, x );
    // TEST_ASSERT_FLOAT_WITHIN( 1.0, 0.0, y );
    // TEST_ASSERT_FLOAT_WITHIN( 1.0, 0.0, z );

    // t = 0.0;
    // x = 0.0;
    // y = 0.0;
    // z = 0.0;
    // dut.functions->calculateMagneticFieldAndTemperature(&dut, &x, &y, &z, &t);
    // TEST_ASSERT_FLOAT_WITHIN( 20.0, 25.0, t );
    // TEST_ASSERT_FLOAT_WITHIN( 1.0, 0.0, x );
    // TEST_ASSERT_FLOAT_WITHIN( 1.0, 0.0, y );
    // TEST_ASSERT_FLOAT_WITHIN( 1.0, 0.0, z );
}


TEST_IFX(TLx493D_A1B6_needsSensorInternal, checkBasicFunctionality)
{
    // TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    // TEST_ASSERT( dut.functions->readRegisters(&dut) == true );
}


TEST_IFX(TLx493D_A1B6_needsSensorInternal, checkConfigMeasurementFunctionality)
{
    // TLx493D_Register_t *dt   = &dut.regDef[A1B6_DT_e];
    // TLx493D_Register_t *am   = &dut.regDef[A1B6_AM_e];
    // TLx493D_Register_t *bzLSBS   = &dut.regDef[A1B6_BZ_LSBS_e];
    // TLx493D_Register_t *tempLSBS = &dut.regDef[A1B6_TEMP_LSBS_e];

    // // Supported
    // // TLx493D_BxBy_e
    // TEST_ASSERT( dut.functions->setMeasurement(&dut, TLx493D_BxBy_e) == true );
    // TEST_ASSERT( dut.functions->readRegisters(&dut) == true);

    // TEST_ASSERT( (dut.regMap[A1B6_CONFIG_REG_e] & dt->mask) == dt->mask ); // DT
    // TEST_ASSERT( (dut.regMap[A1B6_CONFIG_REG_e] & am->mask) == am->mask ); // AM

    // TEST_ASSERT( dut.regMap[0x02] == 0x80 ); // Bz MSBS
    // TEST_ASSERT( dut.regMap[0x03] == 0x80 ); // TEMP MSBS
    // TEST_ASSERT( (dut.regMap[A1B6_TEMP2_REG_e] & (tempLSBS->mask | bzLSBS->mask)) == 0x00 ); // TEMP and Bz LSBS


    // // TLx493D_BxByBz_e
    // TEST_ASSERT( dut.functions->setMeasurement(&dut, TLx493D_BxByBz_e) == true );
    // TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    // TEST_ASSERT( (dut.regMap[A1B6_CONFIG_REG_e] & dt->mask) == dut.regDef[A1B6_DT_e].mask ); // DT
    // TEST_ASSERT( (dut.regMap[A1B6_CONFIG_REG_e] & am->mask) == 0x00 ); // AM is 0x00

    // TEST_ASSERT( dut.regMap[0x03] == 0x80 ); // TEMP MSB
    // TEST_ASSERT( (dut.regMap[A1B6_TEMP2_REG_e] & tempLSBS->mask) == 0x00 ); // TEMP


    // // TLx493D_BxByBzTemp_e
    // TEST_ASSERT( dut.functions->setMeasurement(&dut, TLx493D_BxByBzTemp_e) == true );
    // TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    // TEST_ASSERT( (dut.regMap[A1B6_CONFIG_REG_e] & 0x80) == 0x00 ); // DT
    // TEST_ASSERT( (dut.regMap[A1B6_CONFIG_REG_e] & 0x40) == 0x00 ); // AM


    // // Unsupported
    // TEST_ASSERT( dut.functions->setMeasurement(&dut, TLx493D_VHall_Bias_e) == false );
    // TEST_ASSERT( dut.functions->setMeasurement(&dut, TLx493D_Spintest_e) == false );
    // TEST_ASSERT( dut.functions->setMeasurement(&dut, TLx493D_SAT_test_e) == false );
    // TEST_ASSERT( dut.functions->setMeasurement(&dut, TLx493D_BxTemp_e) == false );
    // TEST_ASSERT( dut.functions->setMeasurement(&dut, TLx493D_BzTemp_e) == false );
}


TEST_IFX(TLx493D_A1B6_needsSensorInternal, checkConfigTriggerFunctionality)
{
    // // switch to LPM
    // TEST_ASSERT( dut.functions->setPowerMode(&dut, TLx493D_LOW_POWER_MODE_e) == true );
    // TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    // TEST_ASSERT( (dut.regMap[A1B6_MOD1_REG_e] & 0x03) == 0x00 );

    // // Low-power mode only supports this trigger
    // TEST_ASSERT( dut.functions->setTrigger(&dut, TLx493D_NO_ADC_ON_READ_e) == true );
    // TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    // TEST_ASSERT( (dut.regMap[A1B6_CONFIG_REG_e] & 0x30) == 0x00 );


    // // MCM supports other modes, so enable MCM first
    // TEST_ASSERT( dut.functions->setPowerMode(&dut, TLx493D_MASTER_CONTROLLED_MODE_e) == true );
    // TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    // TEST_ASSERT( (dut.regMap[A1B6_MOD1_REG_e] & 0x03) == 0x01 );

    // // // try triggers
    // TEST_ASSERT( dut.functions->setTrigger(&dut, TLx493D_ADC_ON_READ_AFTER_REG_05_e) == true );
    // TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    // TEST_ASSERT( ((dut.regMap[A1B6_CONFIG_REG_e] & 0x30) == 0x20) || ((dut.regMap[A1B6_CONFIG_REG_e] & 0x30) == 0x30) );

    // // Not to be used with our default config CA = 0, INT = 1 !
    // // TEST_ASSERT( dut.functions->setTrigger(&dut, TLx493D_ADC_ON_READ_BEFORE_FIRST_MSB_e) == true );
    // // TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    // // // printRegisters(&dut);
    // // TEST_ASSERT( (dut.regMap[A1B6_CONFIG_REG_e] & 0x30) == 0x10 );

    // TEST_ASSERT( dut.functions->setTrigger(&dut, TLx493D_NO_ADC_ON_READ_e) == true );
    // TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    // TEST_ASSERT( (dut.regMap[A1B6_CONFIG_REG_e] & 0x30) == 0x00 );


    // // switch back to LPM
    // TEST_ASSERT( dut.functions->setPowerMode(&dut, TLx493D_LOW_POWER_MODE_e) == true );
    // TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    // TEST_ASSERT( (dut.regMap[A1B6_MOD1_REG_e] & 0x03) == 0x00 );
}


TEST_IFX(TLx493D_A1B6_needsSensorInternal, checkConfigSensitivityFunctionality)
{
    // double sf;

    // // unsupported
    // TEST_ASSERT( dut.functions->setSensitivity(&dut, TLx493D_EXTRA_SHORT_RANGE_e) == false );


    // // supported
    // TEST_ASSERT( dut.functions->setSensitivity(&dut, TLx493D_SHORT_RANGE_e) == true );
    // TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    // TEST_ASSERT( (dut.regMap[A1B6_CONFIG_REG_e] & 0x08) == 0x08 );

    // dut.functions->getSensitivityScaleFactor(&dut, &sf);
    // TEST_ASSERT_EQUAL_FLOAT( 2.0, sf );


    // TEST_ASSERT( dut.functions->setSensitivity(&dut, TLx493D_FULL_RANGE_e) == true );
    // TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    // TEST_ASSERT( (dut.regMap[A1B6_CONFIG_REG_e] & 0x08) == 0x00 );

    // dut.functions->getSensitivityScaleFactor(&dut, &sf);
    // TEST_ASSERT_EQUAL_FLOAT( 1.0, sf );
}


// Check if setDefaultConfig worked properly and data can be read and expected values are set.
TEST_IFX(TLx493D_A1B6_needsSensorInternal, checkModeDefaultConfigFunctionality)
{
    // TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    // TEST_ASSERT_EQUAL_HEX( 0x00, dut.regMap[A1B6_CONFIG_REG_e] ); // all defaults
    // TEST_ASSERT_EQUAL_HEX( 0x94, dut.regMap[A1B6_MOD1_REG_e] ); // FP on, INT off, PR on
}


TEST_IFX(TLx493D_A1B6_needsSensorInternal, checkModeIICAddressFunctionality)
{
    // // printRegisters(&dut);
    // // print("addr : %x", dut.comLibIFParams.i2c_params.address << 1);

    // TEST_ASSERT( dut.functions->setIICAddress(&dut, TLx493D_IIC_ADDR_A3_e) == true );
    // TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    // TEST_ASSERT( (dut.regMap[A1B6_MOD1_REG_e] & 0x60) == 0x60 );
    // TEST_ASSERT( TLx493D_A1B6_hasValidIICadr(&dut) == true );

    // TEST_ASSERT( dut.functions->setIICAddress(&dut, TLx493D_IIC_ADDR_A2_e) == true );
    // TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    // TEST_ASSERT( (dut.regMap[A1B6_MOD1_REG_e] & 0x60) == 0x40 );
    // TEST_ASSERT( TLx493D_A1B6_hasValidIICadr(&dut) == true );

    // TEST_ASSERT( dut.functions->setIICAddress(&dut, TLx493D_IIC_ADDR_A1_e) == true );
    // TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    // TEST_ASSERT( (dut.regMap[A1B6_MOD1_REG_e] & 0x60) == 0x20 );
    // TEST_ASSERT( TLx493D_A1B6_hasValidIICadr(&dut) == true );

    // TEST_ASSERT( dut.functions->setIICAddress(&dut, TLx493D_IIC_ADDR_A0_e) == true );
    // TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    // TEST_ASSERT( (dut.regMap[A1B6_MOD1_REG_e] & 0x60) == 0x00 );
    // TEST_ASSERT( TLx493D_A1B6_hasValidIICadr(&dut) == true );
}


TEST_IFX(TLx493D_A1B6_needsSensorInternal, checkModeCollisionAvoidanceFunctionality)
{
    // TEST_ASSERT( dut.functions->disableCollisionAvoidance(&dut) == true );
    // TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    // TEST_ASSERT( (dut.regMap[A1B6_MOD1_REG_e] & 0x08) == 0x08 );

    // TEST_ASSERT( dut.functions->enableCollisionAvoidance(&dut) == true );
    // TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    // TEST_ASSERT( (dut.regMap[A1B6_MOD1_REG_e] & 0x08) == 0x00 );
    // TEST_ASSERT( dut.functions->enableCollisionAvoidance(&dut) == true );
}


TEST_IFX(TLx493D_A1B6_needsSensorInternal, checkModeInterruptFunctionality)
{
    // // TEST_ASSERT( dut.functions->enableInterrupt(&dut) == true );
    // // TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    // // TEST_ASSERT( (dut.regMap[A1B6_MOD1_REG_e] & 0x04) == 0x00 );

    // TEST_ASSERT( dut.functions->disableInterrupt(&dut) == true );
    // TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    // TEST_ASSERT( (dut.regMap[A1B6_MOD1_REG_e] & 0x04) == 0x04 );
}


TEST_IFX(TLx493D_A1B6_needsSensorInternal, checkModePowerModeFunctionality)
{
    // TEST_ASSERT( dut.functions->setPowerMode(&dut, TLx493D_FAST_MODE_e) == true );
    // TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    // TEST_ASSERT( (dut.regMap[A1B6_MOD1_REG_e] & 0x03) == 0x03 );

    // TEST_ASSERT( dut.functions->setPowerMode(&dut, TLx493D_MASTER_CONTROLLED_MODE_e) == true );
    // TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    // TEST_ASSERT( (dut.regMap[A1B6_MOD1_REG_e] & 0x03) == 0x01 );

    // // forbidden
    // TEST_ASSERT( (dut.regMap[A1B6_MOD1_REG_e] & 0x03) != 0x10 );

    // TEST_ASSERT( dut.functions->setPowerMode(&dut, TLx493D_LOW_POWER_MODE_e) == true );
    // TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    // TEST_ASSERT( (dut.regMap[A1B6_MOD1_REG_e] & 0x03) == 0x00 );
}


TEST_IFX(TLx493D_A1B6_needsSensorInternal, checkModeUpdateRateFunctionality)
{
    // TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_SLOW_e) == true );
    // TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    // TEST_ASSERT( (dut.regMap[A1B6_MOD2_REG_e] & 0x80) == 0x80 );


    // TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_FAST_e) == true );
    // TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    // TEST_ASSERT( (dut.regMap[A1B6_MOD2_REG_e] & 0x80) == 0x00 );


    // // Unsupported
    // TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_770_HZ_e) == false );
    // TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_97_HZ_e) == false );
    // TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_24_HZ_e) == false );
    // TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_12_HZ_e) == false );
    // TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_6_HZ_e) == false );
    // TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_3_HZ_e) == false );
    // TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_0_4_HZ_e) == false );
    // TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_0_05_HZ_e) == false );
    // TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_1000_HZ_e) == false );
    // TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_125_HZ_e) == false );
    // TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_31_HZ_e) == false );  
    // TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_16_HZ_e) == false );  
}

TEST_IFX(TLx493D_A1B6_needsSensorInternal, getTemperature)
{
    double temperature = 0.0;

    TEST_ASSERT_EQUAL( true, TLx493D_A1B6_getTemperature(&dut, &temperature));
    TEST_ASSERT_FLOAT_WITHIN( 10.0, 25.0, temperature ); // Environmental temp is around 25 - 30 deg C
}

TEST_IFX(TLx493D_A1B6_needsSensorInternal, getMagneticField)
{
    double x = 0.0, y = 0.0, z = 0.0;

    TEST_ASSERT_EQUAL( true, TLx493D_A1B6_getMagneticField(&dut, &x, &y, &z) );
    TEST_ASSERT_FLOAT_WITHIN( 1.0, 0.0, x ); // Residual mag field stays within 0-1 mT
    TEST_ASSERT_FLOAT_WITHIN( 1.0, 0.0, y );
    TEST_ASSERT_FLOAT_WITHIN( 1.0, 0.0, z );
}


TEST(TLx493D_A1B6_needsSensorInternal, readRegisters)
{
    TEST_ASSERT_EQUAL( true, TLx493D_A1B6_readRegisters(&dut) );
}


TEST_IFX(TLx493D_A1B6_TempDisableInternal, TempDisable)
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


TEST_IFX(TLx493D_A1B6_ParityCheckInternal, SetWrongParity_ParityCheckDisabled)
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

TEST_IFX(TLx493D_A1B6_ParityCheckInternal, SetWrongParity_ParityCheckEnabled)
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


// TEST_IFX(TLx493D_A1B6_atResetInternal, regMapatReset_MeasurementBitfields)
// {
//     // TODO: setup anew
//     // TEST_ASSERT_EQUAL_UINT8 ( 0x00, ( dut.regMap[dut.commonBitfields.BX_MSB] & dut.regDef[dut.commonBitfields.BX_MSB].mask) >> dut.regDef[dut.commonBitfields.BX_MSB].offset  );
//     // TEST_ASSERT_EQUAL_UINT8 ( 0x00, ( dut.regMap[dut.commonBitfields.BY_MSB] & dut.regDef[dut.commonBitfields.BY_MSB].mask) >> dut.regDef[dut.commonBitfields.BY_MSB].offset  );
//     // TEST_ASSERT_EQUAL_UINT8 ( 0x00, (dut.regMap[dut.commonBitfields.BZ_MSB] & dut.regDef[dut.commonBitfields.BZ_MSB].mask) >> dut.regDef[dut.commonBitfields.BZ_MSB].offset  );
//     // TEST_ASSERT_EQUAL_UINT8 ( 0x01, (dut.regMap[dut.commonBitfields.TEMP_MSB] & dut.regDef[dut.commonBitfields.TEMP_MSB].mask) >> dut.regDef[dut.commonBitfields.TEMP_MSB].offset );
    
//     // TEST_ASSERT_EQUAL_UINT8 ( 0x5, ( dut.regMap[dut.commonBitfields.BX_LSB] & dut.regDef[dut.commonBitfields.BX_LSB].mask) >> dut.regDef[dut.commonBitfields.BX_LSB].offset );
//     // TEST_ASSERT_EQUAL_UINT8 ( 0x3, ( dut.regMap[dut.commonBitfields.BY_LSB] & dut.regDef[dut.commonBitfields.BY_LSB].mask) >> dut.regDef[dut.commonBitfields.BY_LSB].offset );

//     // TEST_ASSERT_EQUAL_UINT8 ( 0x3 , ( dut.regMap[dut.commonBitfields.BZ_LSB] & dut.regDef[dut.commonBitfields.BZ_LSB].mask) >> dut.regDef[dut.commonBitfields.BZ_LSB].offset );
//     // TEST_ASSERT_EQUAL_UINT8 ( 0x0 , ( dut.regMap[dut.commonBitfields.TEMP_LSB] & dut.regDef[dut.commonBitfields.TEMP_LSB].mask) >> dut.regDef[dut.commonBitfields.TEMP_LSB].offset );
// }

// TEST_IFX(TLx493D_A1B6_atResetInternal, regMapatReset_nonMeasurementBitfields)
// {
//     // TODO: setup anew
//     // TEST_ASSERT_EQUAL_UINT8 ( 0x0, ( dut.regMap[dut.commonBitfields.FRM] & dut.regDef[dut.commonBitfields.FRM].mask) >> dut.regDef[dut.commonBitfields.FRM].offset );
//     // TEST_ASSERT_UINT8_WITHIN ( 0x3, 0x0, ( dut.regMap[dut.commonBitfields.CH] & dut.regDef[dut.commonBitfields.CH].mask) >> dut.regDef[dut.commonBitfields.CH].offset );
//     // TEST_ASSERT_EQUAL_UINT8 ( 0x0, ( dut.regMap[dut.commonBitfields.T] & dut.regDef[dut.commonBitfields.T].mask) >> dut.regDef[dut.commonBitfields.T].offset );
//     // TEST_ASSERT_EQUAL_UINT8 ( 0x1, ( dut.regMap[dut.commonBitfields.FF] & dut.regDef[dut.commonBitfields.FF].mask) >> dut.regDef[dut.commonBitfields.FF].offset );
//     // TEST_ASSERT_EQUAL_UINT8 ( 0x0, ( dut.regMap[dut.commonBitfields.PD] & dut.regDef[dut.commonBitfields.PD].mask) >> dut.regDef[dut.commonBitfields.PD].offset );
// }


static TEST_GROUP_RUNNER(TLx493D_A1B6_needsSensorInternal)
{
    RUN_TEST_CASE(TLx493D_A1B6_needsSensorInternal, checkUnsupportedFunctionality);
    RUN_TEST_CASE(TLx493D_A1B6_needsSensorInternal, checkSupportedFunctionality);

    RUN_TEST_CASE(TLx493D_A1B6_needsSensorInternal, checkBasicFunctionality);
    RUN_TEST_CASE(TLx493D_A1B6_needsSensorInternal, checkGetMagneticFieldAndTemperature);

    RUN_TEST_CASE(TLx493D_A1B6_needsSensorInternal, checkConfigMeasurementFunctionality);
    RUN_TEST_CASE(TLx493D_A1B6_needsSensorInternal, checkConfigTriggerFunctionality);
    RUN_TEST_CASE(TLx493D_A1B6_needsSensorInternal, checkConfigSensitivityFunctionality);

    RUN_TEST_CASE(TLx493D_A1B6_needsSensorInternal, checkModeDefaultConfigFunctionality);
    RUN_TEST_CASE(TLx493D_A1B6_needsSensorInternal, checkModeIICAddressFunctionality);
    RUN_TEST_CASE(TLx493D_A1B6_needsSensorInternal, checkModeCollisionAvoidanceFunctionality);
    RUN_TEST_CASE(TLx493D_A1B6_needsSensorInternal, checkModeInterruptFunctionality);
    RUN_TEST_CASE(TLx493D_A1B6_needsSensorInternal, checkModePowerModeFunctionality);

    // MOD2
    RUN_TEST_CASE(TLx493D_A1B6_needsSensorInternal, checkModeUpdateRateFunctionality);


    RUN_TEST_CASE(TLx493D_A1B6_needsSensorInternal, readRegisters);
    RUN_TEST_CASE(TLx493D_A1B6_needsSensorInternal, getTemperature);
    RUN_TEST_CASE(TLx493D_A1B6_needsSensorInternal, getMagneticField);

    // RUN_TEST_GROUP(TLx493D_A1B6_TempDisable);
    RUN_TEST_CASE(TLx493D_A1B6_TempDisableInternal, TempDisable);
    RUN_TEST_CASE(TLx493D_A1B6_ParityCheckInternal, SetWrongParity_ParityCheckDisabled);

    // This test is run at the end since the sensor cannot recover from a wrong parity error without hard reset
    // So tests will fail in the loop second time onwards. First time everything will pass. 
    //RUN_TEST_CASE(TLx493D_A1B6_ParityCheck, SetWrongParity_ParityCheckEnabled);
}


// Bundle all tests to be executed for this test group
TEST_GROUP_RUNNER(TLx493D_A1B6_needsSensor)
{
    TLx493D_A1B6_needsSensor_suiteSetup();  
    
    // run common functions tests
    RUN_TEST_GROUP(SensorsCommonFunctions);

    // run gen 1 common functions tests
    RUN_TEST_GROUP(SensorsCommon);
    RUN_TEST_GROUP(SensorsCommon_needsSensor);

    RUN_TEST_GROUP(TLx493D_A1B6_needsSensorInternal);

    TLx493D_A1B6_needsSensor_suiteTearDown();


    // TODO: No software reset support for gen 1 and 2, only gen 3 and hardware support for all. Therefore remove.
    // TLx493D_A1B6_atReset_suiteSetup();

    // // This test is commented out since we dont check measurement bitfields (BX,BY,BZ,TEMP) at reset.
    // // TODO: Remove
    // //RUN_TEST_CASE(TLx493D_A1B6_atReset, regMapatReset_MeasurementBitfields);
    // RUN_TEST_CASE(TLx493D_A1B6_atResetInternal, regMapatReset_nonMeasurementBitfields);

    // TLx493D_A1B6_atReset_suiteTearDown();
}
