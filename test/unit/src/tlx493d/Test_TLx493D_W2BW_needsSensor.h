// test includes
#include "Test_includes.h"


void TLx493D_W2BW_needsSensor_suiteSetup(void);
void TLx493D_W2BW_needsSensor_suiteTearDown(void);


// variables used in the tests below that have to be accessed in the setup and tear down methods
static TLx493D_t dut;


// test includes that may require dut
#include "Test_tlx493d_commonFunctions_needsSensor.h"
#include "Test_tlx493d_common_needsSensor.h"
#include "Test_tlx493d_common.h"
#include "Test_tlx493d_gen_2_common_needsSensor.h"
#include "Test_tlx493d_gen_2_common.h"


// define test group name
TEST_GROUP(TLx493D_W2BW_needsSensor);
TEST_GROUP(TLx493D_W2BW_needsSensorInternal);


static double  x, y, z, t;


// Setup method called before every individual test defined for this test group
static TEST_SETUP(TLx493D_W2BW_needsSensorInternal)
{
    x = 0.0;
    y = 0.0;
    z = 0.0;
    t = 0.0;
}


// Tear down method called before every individual test defined for this test group
static TEST_TEAR_DOWN(TLx493D_W2BW_needsSensorInternal)
{
}


// Define all relevant tests for the sensor device
/**
 * Define tests for unsupported common functionality.
 */
TEST_IFX(TLx493D_W2BW_needsSensorInternal, checkUnsupportedFunctionality)
{
}


/**
 * Define tests for supported common functionality.
 * Requires that the registers have been read once, in setDefaultConfig.
 */

TEST_IFX(TLx493D_W2BW_needsSensorInternal, checkSupportedFunctionality)
{
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    // printRegisters(&dut);
 
    // TEST_ASSERT( dut.functions->hasValidData(&dut) == true ); // fails sometimes
    // TEST_ASSERT( dut.functions->hasValidBusParity(&dut) == true ); // fails sometimes

    TEST_ASSERT( dut.functions->isFunctional(&dut) == true );

    TEST_ASSERT( dut.functions->hasValidFuseParity(&dut) == true );
    TEST_ASSERT( dut.functions->hasValidConfigurationParity(&dut) == true );

    TEST_ASSERT( dut.functions->hasValidTBit(&dut) == true );
}


TEST_IFX(TLx493D_W2BW_needsSensorInternal, checkGetMagneticFieldAndTemperature)
{
    TEST_ASSERT( dut.functions->getTemperature(&dut, &t) == true );
    TEST_ASSERT_FLOAT_WITHIN( 20.0, 25.0, t );

    dut.functions->calculateMagneticField(&dut, &x, &y, &z);
    TEST_ASSERT_FLOAT_WITHIN( 1.0, 0.0, x );
    TEST_ASSERT_FLOAT_WITHIN( 1.0, 0.0, y );
    TEST_ASSERT_FLOAT_WITHIN( 1.0, 0.0, z );

    t = 0.0;
    x = 0.0;
    y = 0.0;
    z = 0.0;
    dut.functions->calculateMagneticFieldAndTemperature(&dut, &x, &y, &z, &t);
    TEST_ASSERT_FLOAT_WITHIN( 20.0, 25.0, t );
    TEST_ASSERT_FLOAT_WITHIN( 1.0, 0.0, x );
    TEST_ASSERT_FLOAT_WITHIN( 1.0, 0.0, y );
    TEST_ASSERT_FLOAT_WITHIN( 1.0, 0.0, z );


    // TLx493D_FULL_RANGE_e
    int16_t xr, yr, zr, tr;
    dut.functions->calculateRawMagneticFieldAndTemperature(&dut, &xr, &yr, &zr, &tr);

    int16_t xr2, yr2, zr2;
    dut.functions->calculateRawMagneticFieldAtTemperature(&dut, tr, TLx493D_FULL_RANGE_e, x, y, z, &xr2, &yr2, &zr2);

    TEST_ASSERT_INT16_WITHIN( 2, xr, xr2 );
    TEST_ASSERT_INT16_WITHIN( 2, yr, yr2 );
    TEST_ASSERT_INT16_WITHIN( 2, zr, zr2 );


    // TLx493D_EXTRA_SHORT_RANGE_e
    TEST_ASSERT( dut.functions->setSensitivity(&dut, TLx493D_EXTRA_SHORT_RANGE_e) == true );
    TEST_ASSERT( dut.functions->getTemperature(&dut, &t) == true );

    t = 0.0;
    x = 0.0;
    y = 0.0;
    z = 0.0;
    dut.functions->calculateMagneticFieldAndTemperature(&dut, &x, &y, &z, &t);

    tr = 0;
    xr = 0;
    yr = 0;
    zr = 0;
    dut.functions->calculateRawMagneticFieldAndTemperature(&dut, &xr, &yr, &zr, &tr);

    xr2 = 0;
    yr2 = 0;
    zr2 = 0;
    dut.functions->calculateRawMagneticFieldAtTemperature(&dut, tr, TLx493D_EXTRA_SHORT_RANGE_e, x, y, z, &xr2, &yr2, &zr2);

    TEST_ASSERT_INT16_WITHIN( 2, xr, xr2 );
    TEST_ASSERT_INT16_WITHIN( 2, yr, yr2 );
    TEST_ASSERT_INT16_WITHIN( 2, zr, zr2 );


    // TLx493D_SHORT_RANGE_e
    TEST_ASSERT( dut.functions->setSensitivity(&dut, TLx493D_SHORT_RANGE_e) == true );
    TEST_ASSERT( dut.functions->getTemperature(&dut, &t) == true );

    t = 0.0;
    x = 0.0;
    y = 0.0;
    z = 0.0;
    dut.functions->calculateMagneticFieldAndTemperature(&dut, &x, &y, &z, &t);

    tr = 0;
    xr = 0;
    yr = 0;
    zr = 0;
    dut.functions->calculateRawMagneticFieldAndTemperature(&dut, &xr, &yr, &zr, &tr);

    xr2 = 0;
    yr2 = 0;
    zr2 = 0;
    dut.functions->calculateRawMagneticFieldAtTemperature(&dut, tr, TLx493D_SHORT_RANGE_e, x, y, z, &xr2, &yr2, &zr2);

    TEST_ASSERT_INT16_WITHIN( 2, xr, xr2 );
    TEST_ASSERT_INT16_WITHIN( 2, yr, yr2 );
    TEST_ASSERT_INT16_WITHIN( 2, zr, zr2 );
    

    // back to TLx493D_FULL_RANGE_e
    TEST_ASSERT( dut.functions->setSensitivity(&dut, TLx493D_FULL_RANGE_e) == true );


    dut.functions->calculateRawMagneticFieldAtTemperature(&dut, tr, TLx493D_FULL_RANGE_e, 0.5, 0.5, 0.5, &xr, &yr, &zr);
    dut.functions->calculateRawMagneticFieldAtTemperature(&dut, tr, TLx493D_SHORT_RANGE_e, 0.5, 0.5, 0.5, &xr2, &yr2, &zr2);

    TEST_ASSERT_INT16_WITHIN( 2, xr, xr2 );
    TEST_ASSERT_INT16_WITHIN( 2, yr, yr2 );
    TEST_ASSERT_INT16_WITHIN( 2, zr, zr2 );


    dut.functions->calculateRawMagneticFieldAtTemperature(&dut, tr, TLx493D_EXTRA_SHORT_RANGE_e, 0.5, 0.5, 0.5, &xr, &yr, &zr);

    TEST_ASSERT_INT16_WITHIN( 2, xr, xr2 );
    TEST_ASSERT_INT16_WITHIN( 2, yr, yr2 );
    TEST_ASSERT_INT16_WITHIN( 2, zr, zr2 );
}


TEST_IFX(TLx493D_W2BW_needsSensorInternal, checkBasicFunctionality)
{
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
}


TEST_IFX(TLx493D_W2BW_needsSensorInternal, checkConfigMeasurementFunctionality)
{
    TLx493D_Register_t *dt       = &dut.regDef[W2BW_DT_e];
    TLx493D_Register_t *am       = &dut.regDef[W2BW_AM_e];
    TLx493D_Register_t *bzLSBS   = &dut.regDef[W2BW_BZ_LSBS_e];
    TLx493D_Register_t *tempLSBS = &dut.regDef[W2BW_TEMP_LSBS_e];


    // Unsupported
    TEST_ASSERT( dut.functions->setMeasurement(&dut, TLx493D_VHall_Bias_e) == false );
    TEST_ASSERT( dut.functions->setMeasurement(&dut, TLx493D_Spintest_e) == false );
    TEST_ASSERT( dut.functions->setMeasurement(&dut, TLx493D_SAT_test_e) == false );
    TEST_ASSERT( dut.functions->setMeasurement(&dut, TLx493D_BxTemp_e) == false );
    TEST_ASSERT( dut.functions->setMeasurement(&dut, TLx493D_BzTemp_e) == false );


    // Supported
    // TLx493D_BxBy_e
    TEST_ASSERT( dut.functions->setMeasurement(&dut, TLx493D_BxBy_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);

    TEST_ASSERT( (dut.regMap[W2BW_CONFIG_REG_e] & dt->mask) == dt->mask ); // DT
    TEST_ASSERT( (dut.regMap[W2BW_CONFIG_REG_e] & am->mask) == am->mask ); // AM

    TEST_ASSERT( dut.regMap[0x02] == 0x80 ); // Bz MSBS
    TEST_ASSERT( dut.regMap[0x03] == 0x80 ); // TEMP MSBS
    TEST_ASSERT( (dut.regMap[W2BW_TEMP2_REG_e] & (tempLSBS->mask | bzLSBS->mask)) == 0x00 ); // TEMP and Bz LSBS


    // TLx493D_BxByBz_e
    TEST_ASSERT( dut.functions->setMeasurement(&dut, TLx493D_BxByBz_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT( (dut.regMap[W2BW_CONFIG_REG_e] & dt->mask) == dt->mask ); // DT
    TEST_ASSERT( (dut.regMap[W2BW_CONFIG_REG_e] & am->mask) == 0x00 ); // AM is 0x00

    TEST_ASSERT( dut.regMap[0x03] == 0x80 ); // TEMP MSB
    TEST_ASSERT( (dut.regMap[W2BW_TEMP2_REG_e] & tempLSBS->mask) == 0x00 ); // TEMP


    // TLx493D_BxByBzTemp_e
    TEST_ASSERT( dut.functions->setMeasurement(&dut, TLx493D_BxByBzTemp_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT( (dut.regMap[W2BW_CONFIG_REG_e] & 0x80) == 0x00 ); // DT
    TEST_ASSERT( (dut.regMap[W2BW_CONFIG_REG_e] & 0x40) == 0x00 ); // AM
}


TEST_IFX(TLx493D_W2BW_needsSensorInternal, checkConfigTriggerFunctionality)
{
    // switch to LPM
    TEST_ASSERT( dut.functions->setPowerMode(&dut, TLx493D_LOW_POWER_MODE_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT( (dut.regMap[W2BW_MOD1_REG_e] & 0x03) == 0x00 );

    // Low-power mode only supports this trigger
    TEST_ASSERT( dut.functions->setTrigger(&dut, TLx493D_NO_ADC_ON_READ_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT( (dut.regMap[W2BW_CONFIG_REG_e] & 0x30) == 0x00 );


    // MCM supports other modes, so enable MCM first
    TEST_ASSERT( dut.functions->setPowerMode(&dut, TLx493D_MASTER_CONTROLLED_MODE_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT( (dut.regMap[W2BW_MOD1_REG_e] & 0x03) == 0x01 );

    // try triggers
    TEST_ASSERT( dut.functions->setTrigger(&dut, TLx493D_ADC_ON_READ_AFTER_REG_05_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT( ((dut.regMap[W2BW_CONFIG_REG_e] & 0x30) == 0x20) || ((dut.regMap[W2BW_CONFIG_REG_e] & 0x30) == 0x30) );

    // Not to be used with our default config CA = 0, INT = 1 !
    // TEST_ASSERT( dut.functions->setTrigger(&dut, TLx493D_ADC_ON_READ_BEFORE_FIRST_MSB_e) == true );
    // // while( dut.functions->readRegisters(&dut) == false ) ;
    // TEST_ASSERT( (dut.regMap[W2BW_CONFIG_REG_e] & 0x30) == 0x10 );

    TEST_ASSERT( dut.functions->setTrigger(&dut, TLx493D_NO_ADC_ON_READ_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT( (dut.regMap[W2BW_CONFIG_REG_e] & 0x30) == 0x00 );


    // switch back to LPM
    TEST_ASSERT( dut.functions->setPowerMode(&dut, TLx493D_LOW_POWER_MODE_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT( (dut.regMap[W2BW_MOD1_REG_e] & 0x03) == 0x00 );
}


TEST_IFX(TLx493D_W2BW_needsSensorInternal, checkConfigSensitivityFunctionality)
{
    double sf;

    // supported
    TEST_ASSERT( dut.functions->setSensitivity(&dut, TLx493D_EXTRA_SHORT_RANGE_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT( tlx493d_common_returnBitfield(&dut, W2BW_X2_e) == 0x01 );
    TEST_ASSERT( tlx493d_common_returnBitfield(&dut, W2BW_X4_READABLE_e) == 0x01 );

    sf = dut.functions->getSensitivityScaleFactor(&dut);
    TEST_ASSERT_EQUAL_FLOAT( 4.0, sf );


    TEST_ASSERT( dut.functions->setSensitivity(&dut, TLx493D_SHORT_RANGE_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT( tlx493d_common_returnBitfield(&dut, W2BW_X2_e) == 0x01 );
    TEST_ASSERT( tlx493d_common_returnBitfield(&dut, W2BW_X4_READABLE_e) == 0x00 );

    sf = dut.functions->getSensitivityScaleFactor(&dut);
    TEST_ASSERT_EQUAL_FLOAT( 2.0, sf );
   

    TEST_ASSERT( dut.functions->setSensitivity(&dut, TLx493D_FULL_RANGE_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT( tlx493d_common_returnBitfield(&dut, W2BW_X2_e) == 0x00 );
    TEST_ASSERT( tlx493d_common_returnBitfield(&dut, W2BW_X4_READABLE_e) == 0x00 );

    sf = dut.functions->getSensitivityScaleFactor(&dut);
    TEST_ASSERT_EQUAL_FLOAT( 1.0, sf );
}


// Check if setDefaultConfig worked properly and data can be read and expected values are set.
TEST_IFX(TLx493D_W2BW_needsSensorInternal, checkModeDefaultConfigFunctionality)
{
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    printRegisters(&dut);

    // TEST_ASSERT_EQUAL_HEX( dut.regMap[W2BW_CONFIG_REG_e], 0x00 );
    TEST_ASSERT_EQUAL_HEX( dut.regMap[W2BW_MOD1_REG_e], 0x94 ); // PR on, CA on
    TEST_ASSERT_EQUAL_HEX( dut.regMap[W2BW_MOD2_REG_e], 0x01 );
}


TEST_IFX(TLx493D_W2BW_needsSensorInternal, checkModeIICAddressFunctionality)
{
    // printRegisters(&dut);
    // print("addr : %x", dut.comLibIFParams.i2c_params.address << 1);

    TEST_ASSERT( dut.functions->setIICAddress(&dut, TLx493D_IIC_ADDR_A3_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true );
    TEST_ASSERT( (dut.regMap[W2BW_MOD1_REG_e] & 0x60) == 0x60 );
    TEST_ASSERT( TLx493D_W2BW_hasValidIICadr(&dut) == true );

    TEST_ASSERT( dut.functions->setIICAddress(&dut, TLx493D_IIC_ADDR_A2_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true );
    TEST_ASSERT( (dut.regMap[W2BW_MOD1_REG_e] & 0x60) == 0x40 );
    TEST_ASSERT( TLx493D_W2BW_hasValidIICadr(&dut) == true );

    TEST_ASSERT( dut.functions->setIICAddress(&dut, TLx493D_IIC_ADDR_A1_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true );
    TEST_ASSERT( (dut.regMap[W2BW_MOD1_REG_e] & 0x60) == 0x20 );
    TEST_ASSERT( TLx493D_W2BW_hasValidIICadr(&dut) == true );

    TEST_ASSERT( dut.functions->setIICAddress(&dut, TLx493D_IIC_ADDR_A0_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true );
    TEST_ASSERT( (dut.regMap[W2BW_MOD1_REG_e] & 0x60) == 0x00 );
    TEST_ASSERT( TLx493D_W2BW_hasValidIICadr(&dut) == true );
}


TEST_IFX(TLx493D_W2BW_needsSensorInternal, checkModeCollisionAvoidanceFunctionality)
{
    TEST_ASSERT( dut.functions->disableCollisionAvoidance(&dut) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT( (dut.regMap[W2BW_MOD1_REG_e] & 0x08) == 0x08 );

    TEST_ASSERT( dut.functions->enableCollisionAvoidance(&dut) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT( (dut.regMap[W2BW_MOD1_REG_e] & 0x08) == 0x00 );
    TEST_ASSERT( dut.functions->enableCollisionAvoidance(&dut) == true );
}


TEST_IFX(TLx493D_W2BW_needsSensorInternal, checkModeInterruptFunctionality)
{
    // TEST_ASSERT( dut.functions->enableInterrupt(&dut) == true );
    // TEST_ASSERT( (dut.regMap[W2BW_MOD1_REG_e] & 0x04) == 0x00 );

    TEST_ASSERT( dut.functions->disableInterrupt(&dut) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT( (dut.regMap[W2BW_MOD1_REG_e] & 0x04) == 0x04 );
}


TEST_IFX(TLx493D_W2BW_needsSensorInternal, checkModePowerModeFunctionality)
{
    TEST_ASSERT( dut.functions->setPowerMode(&dut, TLx493D_FAST_MODE_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT( (dut.regMap[W2BW_MOD1_REG_e] & 0x03) == 0x03 );

    TEST_ASSERT( dut.functions->setPowerMode(&dut, TLx493D_MASTER_CONTROLLED_MODE_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT( (dut.regMap[W2BW_MOD1_REG_e] & 0x03) == 0x01 );

    // forbidden
    TEST_ASSERT( (dut.regMap[W2BW_MOD1_REG_e] & 0x03) != 0x10 );

    TEST_ASSERT( dut.functions->setPowerMode(&dut, TLx493D_LOW_POWER_MODE_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT( (dut.regMap[W2BW_MOD1_REG_e] & 0x03) == 0x00 );
}


TEST_IFX(TLx493D_W2BW_needsSensorInternal, checkModeUpdateRateFunctionality)
{
    // Supported
    TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_97_HZ_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT_EQUAL_HEX( 0b001, tlx493d_common_returnBitfield(&dut, W2BW_PRD_e) );

    TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_24_HZ_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT_EQUAL_HEX( 0b010, tlx493d_common_returnBitfield(&dut, W2BW_PRD_e) );


    TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_12_HZ_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT_EQUAL_HEX( 0b011, tlx493d_common_returnBitfield(&dut, W2BW_PRD_e) );


    TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_6_HZ_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT_EQUAL_HEX( 0b100, tlx493d_common_returnBitfield(&dut, W2BW_PRD_e) );


    TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_3_HZ_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT_EQUAL_HEX( 0b101, tlx493d_common_returnBitfield(&dut, W2BW_PRD_e) );


    TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_0_4_HZ_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT_EQUAL_HEX( 0b110, tlx493d_common_returnBitfield(&dut, W2BW_PRD_e) );


    TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_0_05_HZ_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT_EQUAL_HEX( 0b111, tlx493d_common_returnBitfield(&dut, W2BW_PRD_e) );


    TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_770_HZ_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT_EQUAL_HEX( 0b000, tlx493d_common_returnBitfield(&dut, W2BW_PRD_e) );


    // Unsupported
    TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_1000_HZ_e) == false );
    TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_125_HZ_e) == false );
    TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_31_HZ_e) == false );
    TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_16_HZ_e) == false );

    TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_SLOW_e) == false );
    TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_FAST_e) == false );
}


TEST_IFX(TLx493D_W2BW_needsSensorInternal, checkWakeUpSettingsFunctionality)
{
    TEST_ASSERT( dut.functions->disableWakeUpMode(&dut) == true );
    TEST_ASSERT( dut.functions->isWakeUpEnabled(&dut) == false );

    TEST_ASSERT( dut.functions->enableWakeUpMode(&dut) == true );
    TEST_ASSERT( dut.functions->isWakeUpEnabled(&dut) == true );

    TEST_ASSERT( dut.functions->disableWakeUpMode(&dut) == true );
}


TEST_IFX(TLx493D_W2BW_needsSensorInternal, checkWakeUpThresholdFunctionality)
{
    // pos. numbers
    TEST_ASSERT( dut.functions->setWakeUpThresholdsAsInteger(&dut, 0x0ABC, 0x00BC, 0x000C, 0x0FBC, 0x0F0C, 0x0F00) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);

    // MSBs
    TEST_ASSERT( dut.regMap[0x07] == (0x0ABC >> 4));
    TEST_ASSERT( dut.regMap[0x08] == (0x00BC >> 4));
    TEST_ASSERT( dut.regMap[0x09] == (0x000C >> 4));
    TEST_ASSERT( dut.regMap[0x0A] == (0x0FBC >> 4));
    TEST_ASSERT( dut.regMap[0x0B] == (0x0F0C >> 4));
    TEST_ASSERT( dut.regMap[0x0C] == (0x0F00 >> 4));

    // LSBs
    TEST_ASSERT( (dut.regMap[0x0D] & 0x07) == ((0x0ABC >> 1) & 0x07));
    TEST_ASSERT( ((dut.regMap[0x0D] >> 3) & 0x07) == ((0x00BC >> 1) & 0x07));
    TEST_ASSERT( (dut.regMap[0x0E] & 0x07) == ((0x000C >> 1) & 0x07));
    TEST_ASSERT( ((dut.regMap[0x0E] >> 3) & 0x07) == ((0x0FBC >> 1) & 0x07));
    TEST_ASSERT( (dut.regMap[0x0F] & 0x07) == ((0x0F0C >> 1) & 0x07));
    TEST_ASSERT( ((dut.regMap[0x0F] >> 3) & 0x07) == ((0x0F00 >> 1) & 0x07));


    // neg. numbers
    TEST_ASSERT( dut.functions->setWakeUpThresholdsAsInteger(&dut, 0x8ABC, 0x80BC, 0x800C, 0x8FBC, 0x8F0C, 0x8F00) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);

    // MSBs
    TEST_ASSERT( dut.regMap[0x07] == (0xABC >> 4));
    TEST_ASSERT( dut.regMap[0x08] == (0x0BC >> 4));
    TEST_ASSERT( dut.regMap[0x09] == (0x00C >> 4));
    TEST_ASSERT( dut.regMap[0x0A] == (0xFBC >> 4));
    TEST_ASSERT( dut.regMap[0x0B] == (0xF0C >> 4));
    TEST_ASSERT( dut.regMap[0x0C] == (0xF00 >> 4));

    // LSBs
    TEST_ASSERT( (dut.regMap[0x0D] & 0x07) == ((0xABC >> 1) & 0x07));
    TEST_ASSERT( ((dut.regMap[0x0D] >> 3) & 0x07) == ((0x0BC >> 1) & 0x07));
    TEST_ASSERT( (dut.regMap[0x0E] & 0x07) == ((0x00C >> 1) & 0x07));
    TEST_ASSERT( ((dut.regMap[0x0E] >> 3) & 0x07) == ((0xFBC >> 1) & 0x07));
    TEST_ASSERT( (dut.regMap[0x0F] & 0x07) == ((0xF0C >> 1) & 0x07));
    TEST_ASSERT( ((dut.regMap[0x0F] >> 3) & 0x07) == ((0xF00 >> 1) & 0x07));


    TEST_ASSERT( dut.functions->setWakeUpThresholdsAsInteger(&dut, -1, -2, -16, -100, -256, -1024) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);

    // MSBs
    TEST_ASSERT( dut.regMap[0x07] == ((-1 >> 4) & 0xFF));
    TEST_ASSERT( dut.regMap[0x08] == ((-2 >> 4) & 0xFF));
    TEST_ASSERT( dut.regMap[0x09] == ((-16 >> 4) & 0xFF));
    TEST_ASSERT( dut.regMap[0x0A] == ((-100 >> 4) & 0xFF));
    TEST_ASSERT( dut.regMap[0x0B] == ((-256 >> 4) & 0xFF));
    TEST_ASSERT( dut.regMap[0x0C] == ((-1024 >> 4) & 0xFF));

    // LSBs
    TEST_ASSERT( (dut.regMap[0x0D] & 0x07) == ((-1 >> 1) & 0x07));
    TEST_ASSERT( ((dut.regMap[0x0D] >> 3) & 0x07) == ((-2 >> 1) & 0x07));
    TEST_ASSERT( (dut.regMap[0x0E] & 0x07) == ((-16 >> 1) & 0x07));
    TEST_ASSERT( ((dut.regMap[0x0E] >> 3) & 0x07) == ((-100 >> 1) & 0x07));
    TEST_ASSERT( (dut.regMap[0x0F] & 0x07) == ((-256 >> 1) & 0x07));
    TEST_ASSERT( ((dut.regMap[0x0F] >> 3) & 0x07) == ((-1024 >> 1) & 0x07));
}


static TEST_GROUP_RUNNER(TLx493D_W2BW_needsSensorInternal)
{
    // First test default config applied in runner setup method 
    RUN_TEST_CASE(TLx493D_W2BW_needsSensorInternal, checkModeDefaultConfigFunctionality);

    RUN_TEST_CASE(TLx493D_W2BW_needsSensorInternal, checkUnsupportedFunctionality);
    RUN_TEST_CASE(TLx493D_W2BW_needsSensorInternal, checkSupportedFunctionality);

    RUN_TEST_CASE(TLx493D_W2BW_needsSensorInternal, checkBasicFunctionality);
    RUN_TEST_CASE(TLx493D_W2BW_needsSensorInternal, checkGetMagneticFieldAndTemperature);

    RUN_TEST_CASE(TLx493D_W2BW_needsSensorInternal, checkConfigMeasurementFunctionality);
    RUN_TEST_CASE(TLx493D_W2BW_needsSensorInternal, checkConfigTriggerFunctionality);
    RUN_TEST_CASE(TLx493D_W2BW_needsSensorInternal, checkConfigSensitivityFunctionality);

    // RUN_TEST_CASE(TLx493D_W2BW_needsSensorInternal, checkModeDefaultConfigFunctionality);
    RUN_TEST_CASE(TLx493D_W2BW_needsSensorInternal, checkModeIICAddressFunctionality);
    RUN_TEST_CASE(TLx493D_W2BW_needsSensorInternal, checkModeCollisionAvoidanceFunctionality);
    RUN_TEST_CASE(TLx493D_W2BW_needsSensorInternal, checkModeInterruptFunctionality);
    RUN_TEST_CASE(TLx493D_W2BW_needsSensorInternal, checkModePowerModeFunctionality);

    // MOD2
    RUN_TEST_CASE(TLx493D_W2BW_needsSensorInternal, checkModeUpdateRateFunctionality);

    // WakeUp functionality
    RUN_TEST_CASE(TLx493D_W2BW_needsSensorInternal, checkWakeUpSettingsFunctionality);
    RUN_TEST_CASE(TLx493D_W2BW_needsSensorInternal, checkWakeUpThresholdFunctionality);
}


// Bundle all tests to be executed for this test group
TEST_GROUP_RUNNER(TLx493D_W2BW_needsSensor)
{
    TLx493D_W2BW_needsSensor_suiteSetup();

    RUN_TEST_GROUP(TLx493D_W2BW_needsSensorInternal);
    
    // // run common functions tests
    RUN_TEST_GROUP(SensorsCommonFunctions);

    // // run gen 2 common functions tests
    RUN_TEST_GROUP(SensorsGen2Common_needsSensor);
    RUN_TEST_GROUP(SensorsGen2Common);
    RUN_TEST_GROUP(SensorsCommon);

    TLx493D_W2BW_needsSensor_suiteTearDown();
}
