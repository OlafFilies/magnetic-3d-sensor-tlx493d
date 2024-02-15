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

    TEST_ASSERT_EQUAL_HEX8( 0x01, tlx493d_common_returnBitfield(&dut, W2BW_DT_e) ); // DT
    TEST_ASSERT_EQUAL_HEX8( 0x01, tlx493d_common_returnBitfield(&dut, W2BW_AM_e) ); // AM
    TEST_ASSERT_EQUAL_HEX8( 0x80, tlx493d_common_returnBitfield(&dut, W2BW_BZ_MSBS_e) ); // Bz MSBS
    TEST_ASSERT_EQUAL_HEX8( 0x80, tlx493d_common_returnBitfield(&dut, W2BW_TEMP_MSBS_e) ); // TEMP MSBS
    TEST_ASSERT_EQUAL_HEX8( 0x00, tlx493d_common_returnBitfield(&dut, W2BW_TEMP_LSBS_e) ); // TEMP LSBS
    TEST_ASSERT_EQUAL_HEX8( 0x00, tlx493d_common_returnBitfield(&dut, W2BW_BZ_LSBS_e) ); // Bz LSBS


    // TLx493D_BxByBz_e
    TEST_ASSERT( dut.functions->setMeasurement(&dut, TLx493D_BxByBz_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT_EQUAL_HEX8( 0x01, tlx493d_common_returnBitfield(&dut, W2BW_DT_e) ); // DT
    TEST_ASSERT_EQUAL_HEX8( 0x00, tlx493d_common_returnBitfield(&dut, W2BW_AM_e) ); // AM
    TEST_ASSERT_EQUAL_HEX8( 0x80, tlx493d_common_returnBitfield(&dut, W2BW_TEMP_MSBS_e) ); // TEMP MSBS
    TEST_ASSERT_EQUAL_HEX8( 0x00, tlx493d_common_returnBitfield(&dut, W2BW_TEMP_LSBS_e) ); // TEMP LSBS


    // TLx493D_BxByBzTemp_e
    TEST_ASSERT( dut.functions->setMeasurement(&dut, TLx493D_BxByBzTemp_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT_EQUAL_HEX8( 0x00, tlx493d_common_returnBitfield(&dut, W2BW_DT_e) ); // DT
    TEST_ASSERT_EQUAL_HEX8( 0x00, tlx493d_common_returnBitfield(&dut, W2BW_AM_e) ); // AM
}


TEST_IFX(TLx493D_W2BW_needsSensorInternal, checkConfigTriggerFunctionality)
{
    // switch to LPM
    TEST_ASSERT( dut.functions->setPowerMode(&dut, TLx493D_LOW_POWER_MODE_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT_EQUAL_HEX8( 0x00, tlx493d_common_returnBitfield(&dut, W2BW_MODE_e) );

    // Low-power mode only supports this trigger
    TEST_ASSERT( dut.functions->setTrigger(&dut, TLx493D_NO_ADC_ON_READ_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT_EQUAL_HEX8( 0x00, tlx493d_common_returnBitfield(&dut, W2BW_TRIG_e) );


    // MCM supports other modes, so enable MCM first
    TEST_ASSERT( dut.functions->setPowerMode(&dut, TLx493D_MASTER_CONTROLLED_MODE_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT_EQUAL_HEX8( 0x01, tlx493d_common_returnBitfield(&dut, W2BW_MODE_e) );

    // try triggers
    TEST_ASSERT( dut.functions->setTrigger(&dut, TLx493D_ADC_ON_READ_AFTER_REG_05_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT_GREATER_OR_EQUAL_INT8( 0b10, tlx493d_common_returnBitfield(&dut, W2BW_TRIG_e) );
    TEST_ASSERT_LESS_OR_EQUAL_INT8( 0b11, tlx493d_common_returnBitfield(&dut, W2BW_TRIG_e) );
    // TEST_ASSERT( ((dut.regMap[W2BW_CONFIG_REG_e] & 0x30) == 0x20) || ((dut.regMap[W2BW_CONFIG_REG_e] & 0x30) == 0x30) );

    // Not to be used with our default config CA = 0, INT = 1 !
    // TEST_ASSERT( dut.functions->setTrigger(&dut, TLx493D_ADC_ON_READ_BEFORE_FIRST_MSB_e) == true );
    // // while( dut.functions->readRegisters(&dut) == false ) ;
    // TEST_ASSERT_EQUAL_HEX8( 0x01, tlx493d_common_returnBitfield(&dut, W2BW_TRIG_e) );

    TEST_ASSERT( dut.functions->setTrigger(&dut, TLx493D_NO_ADC_ON_READ_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT_EQUAL_HEX8( 0x00, tlx493d_common_returnBitfield(&dut, W2BW_TRIG_e) );


    // switch back to LPM
    TEST_ASSERT( dut.functions->setPowerMode(&dut, TLx493D_LOW_POWER_MODE_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT_EQUAL_HEX8( 0x00, tlx493d_common_returnBitfield(&dut, W2BW_MODE_e) );
}


TEST_IFX(TLx493D_W2BW_needsSensorInternal, checkConfigSensitivityFunctionality)
{
    double sf;

    // supported
    TEST_ASSERT( dut.functions->setSensitivity(&dut, TLx493D_EXTRA_SHORT_RANGE_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT_EQUAL_HEX8( 0x01, tlx493d_common_returnBitfield(&dut, W2BW_X2_e) );
    TEST_ASSERT_EQUAL_HEX8( 0x01, tlx493d_common_returnBitfield(&dut, W2BW_X4_READWRITE_e) );

    sf = dut.functions->getSensitivityScaleFactor(&dut);
    TEST_ASSERT_EQUAL_FLOAT( 4.0, sf );


    TEST_ASSERT( dut.functions->setSensitivity(&dut, TLx493D_SHORT_RANGE_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT_EQUAL_HEX8( 0x01, tlx493d_common_returnBitfield(&dut, W2BW_X2_e) );
    TEST_ASSERT_EQUAL_HEX8( 0x00, tlx493d_common_returnBitfield(&dut, W2BW_X4_READWRITE_e) );

    sf = dut.functions->getSensitivityScaleFactor(&dut);
    TEST_ASSERT_EQUAL_FLOAT( 2.0, sf );
   

    TEST_ASSERT( dut.functions->setSensitivity(&dut, TLx493D_FULL_RANGE_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT_EQUAL_HEX8( 0x00, tlx493d_common_returnBitfield(&dut, W2BW_X2_e) );
    TEST_ASSERT_EQUAL_HEX8( 0x00, tlx493d_common_returnBitfield(&dut, W2BW_X4_READWRITE_e) );

    sf = dut.functions->getSensitivityScaleFactor(&dut);
    TEST_ASSERT_EQUAL_FLOAT( 1.0, sf );
}


// Check if setDefaultConfig worked properly and data can be read and expected values are set.
TEST_IFX(TLx493D_W2BW_needsSensorInternal, checkModeDefaultConfigFunctionality)
{
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    printRegisters(&dut);

    TEST_ASSERT_EQUAL_HEX8( 0x00, dut.regMap[W2BW_CONFIG_REG_e] & 0xFE );
    TEST_ASSERT_EQUAL_HEX8( 0x94, dut.regMap[W2BW_MOD1_REG_e] ); // PR on, CA on
    TEST_ASSERT_EQUAL_HEX8( 0x01, dut.regMap[W2BW_MOD2_REG_e] );
}


TEST_IFX(TLx493D_W2BW_needsSensorInternal, checkModeIICAddressFunctionality)
{
    // printRegisters(&dut);
    // print("addr : %x", dut.comLibIFParams.i2c_params.address << 1);

    TEST_ASSERT( dut.functions->setIICAddress(&dut, TLx493D_IIC_ADDR_A3_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true );
    TEST_ASSERT_EQUAL_HEX8( 0x03, tlx493d_common_returnBitfield(&dut, W2BW_IICADR_e) );
    TEST_ASSERT( TLx493D_W2BW_hasValidIICadr(&dut) == true );

    TEST_ASSERT( dut.functions->setIICAddress(&dut, TLx493D_IIC_ADDR_A2_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true );
    TEST_ASSERT_EQUAL_HEX8( 0x02, tlx493d_common_returnBitfield(&dut, W2BW_IICADR_e) );
    TEST_ASSERT( TLx493D_W2BW_hasValidIICadr(&dut) == true );

    TEST_ASSERT( dut.functions->setIICAddress(&dut, TLx493D_IIC_ADDR_A1_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true );
    TEST_ASSERT_EQUAL_HEX8( 0x01, tlx493d_common_returnBitfield(&dut, W2BW_IICADR_e) );
    TEST_ASSERT( TLx493D_W2BW_hasValidIICadr(&dut) == true );

    TEST_ASSERT( dut.functions->setIICAddress(&dut, TLx493D_IIC_ADDR_A0_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true );
    TEST_ASSERT_EQUAL_HEX8( 0x00, tlx493d_common_returnBitfield(&dut, W2BW_IICADR_e) );
    TEST_ASSERT( TLx493D_W2BW_hasValidIICadr(&dut) == true );
}


TEST_IFX(TLx493D_W2BW_needsSensorInternal, checkModeCollisionAvoidanceFunctionality)
{
    TEST_ASSERT( dut.functions->disableCollisionAvoidance(&dut) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT_EQUAL_HEX8( 0x01, tlx493d_common_returnBitfield(&dut, W2BW_CA_e) );

    TEST_ASSERT( dut.functions->enableCollisionAvoidance(&dut) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT_EQUAL_HEX8( 0x00, tlx493d_common_returnBitfield(&dut, W2BW_CA_e) );
    // TEST_ASSERT( dut.functions->enableCollisionAvoidance(&dut) == true );
}


TEST_IFX(TLx493D_W2BW_needsSensorInternal, checkModeInterruptFunctionality)
{
    // TEST_ASSERT( dut.functions->enableInterrupt(&dut) == true );
    // TEST_ASSERT_EQUAL_HEX8( (dut.regMap[W2BW_MOD1_REG_e] & 0x04) == 0x00 );

    TEST_ASSERT( dut.functions->disableInterrupt(&dut) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT_EQUAL_HEX8( 0x01, tlx493d_common_returnBitfield(&dut, W2BW_INT_e) );
}


TEST_IFX(TLx493D_W2BW_needsSensorInternal, checkModePowerModeFunctionality)
{
    TEST_ASSERT( dut.functions->setPowerMode(&dut, TLx493D_FAST_MODE_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT_EQUAL_HEX8( 0x03, tlx493d_common_returnBitfield(&dut, W2BW_MODE_e) );

    TEST_ASSERT( dut.functions->setPowerMode(&dut, TLx493D_MASTER_CONTROLLED_MODE_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT_EQUAL_HEX8( 0x01, tlx493d_common_returnBitfield(&dut, W2BW_MODE_e) );

    // forbidden
    TEST_ASSERT_NOT_EQUAL( 0x10, tlx493d_common_returnBitfield(&dut, W2BW_MODE_e) );

    TEST_ASSERT( dut.functions->setPowerMode(&dut, TLx493D_LOW_POWER_MODE_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT_EQUAL_HEX8( 0x00, tlx493d_common_returnBitfield(&dut, W2BW_MODE_e) );
}


TEST_IFX(TLx493D_W2BW_needsSensorInternal, checkModeUpdateRateFunctionality)
{
    // Supported
    TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_97_HZ_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT_EQUAL_HEX8( 0b001, tlx493d_common_returnBitfield(&dut, W2BW_PRD_e) );

    TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_24_HZ_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT_EQUAL_HEX8( 0b010, tlx493d_common_returnBitfield(&dut, W2BW_PRD_e) );


    TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_12_HZ_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT_EQUAL_HEX8( 0b011, tlx493d_common_returnBitfield(&dut, W2BW_PRD_e) );


    TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_6_HZ_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT_EQUAL_HEX8( 0b100, tlx493d_common_returnBitfield(&dut, W2BW_PRD_e) );


    TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_3_HZ_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT_EQUAL_HEX8( 0b101, tlx493d_common_returnBitfield(&dut, W2BW_PRD_e) );


    TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_0_4_HZ_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT_EQUAL_HEX8( 0b110, tlx493d_common_returnBitfield(&dut, W2BW_PRD_e) );


    TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_0_05_HZ_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT_EQUAL_HEX8( 0b111, tlx493d_common_returnBitfield(&dut, W2BW_PRD_e) );


    TEST_ASSERT( dut.functions->setUpdateRate(&dut, TLx493D_UPDATE_RATE_770_HZ_e) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);
    TEST_ASSERT_EQUAL_HEX8( 0b000, tlx493d_common_returnBitfield(&dut, W2BW_PRD_e) );


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

    // threshold 11 bits (+ 1 implicit  set to 0)
    // MSBs
    TEST_ASSERT_EQUAL_HEX8( 0x0ABC >> 4, tlx493d_common_returnBitfield(&dut, W2BW_XL_MSBS_e) );
    TEST_ASSERT_EQUAL_HEX8( 0x00BC >> 4, tlx493d_common_returnBitfield(&dut, W2BW_XH_MSBS_e) );
    TEST_ASSERT_EQUAL_HEX8( 0x000C >> 4, tlx493d_common_returnBitfield(&dut, W2BW_YL_MSBS_e) );
    TEST_ASSERT_EQUAL_HEX8( 0x0FBC >> 4, tlx493d_common_returnBitfield(&dut, W2BW_YH_MSBS_e) );
    TEST_ASSERT_EQUAL_HEX8( 0x0F0C >> 4, tlx493d_common_returnBitfield(&dut, W2BW_ZL_MSBS_e) );
    TEST_ASSERT_EQUAL_HEX8( 0x0F00 >> 4, tlx493d_common_returnBitfield(&dut, W2BW_ZH_MSBS_e) );

    // LSBs
    TEST_ASSERT_EQUAL_HEX8( (0x0ABC >> 1) & 0x07, tlx493d_common_returnBitfield(&dut, W2BW_XL_LSBS_e) );
    TEST_ASSERT_EQUAL_HEX8( (0x00BC >> 1) & 0x07, tlx493d_common_returnBitfield(&dut, W2BW_XH_LSBS_e) );
    TEST_ASSERT_EQUAL_HEX8( (0x000C >> 1) & 0x07, tlx493d_common_returnBitfield(&dut, W2BW_YL_LSBS_e) );
    TEST_ASSERT_EQUAL_HEX8( (0x0FBC >> 1) & 0x07, tlx493d_common_returnBitfield(&dut, W2BW_YH_LSBS_e) );
    TEST_ASSERT_EQUAL_HEX8( (0x0F0C >> 1) & 0x07, tlx493d_common_returnBitfield(&dut, W2BW_ZL_LSBS_e) );
    TEST_ASSERT_EQUAL_HEX8( (0x0F00 >> 1) & 0x07, tlx493d_common_returnBitfield(&dut, W2BW_ZH_LSBS_e) );


    // neg. numbers
    TEST_ASSERT( dut.functions->setWakeUpThresholdsAsInteger(&dut, 0x8ABC, 0x80BC, 0x800C, 0x8FBC, 0x8F0C, 0x8F00) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);

    // MSBs
    TEST_ASSERT_EQUAL_HEX8( (((int16_t) 0x8ABC) >> 4) & 0xFF, tlx493d_common_returnBitfield(&dut, W2BW_XL_MSBS_e) );
    TEST_ASSERT_EQUAL_HEX8( (((int16_t) 0x80BC) >> 4) & 0xFF, tlx493d_common_returnBitfield(&dut, W2BW_XH_MSBS_e) );
    TEST_ASSERT_EQUAL_HEX8( (((int16_t) 0x800C) >> 4) & 0xFF, tlx493d_common_returnBitfield(&dut, W2BW_YL_MSBS_e) );
    TEST_ASSERT_EQUAL_HEX8( (((int16_t) 0x8FBC) >> 4) & 0xFF, tlx493d_common_returnBitfield(&dut, W2BW_YH_MSBS_e) );
    TEST_ASSERT_EQUAL_HEX8( (((int16_t) 0x8F0C) >> 4) & 0xFF, tlx493d_common_returnBitfield(&dut, W2BW_ZL_MSBS_e) );
    TEST_ASSERT_EQUAL_HEX8( (((int16_t) 0x8F00) >> 4) & 0xFF, tlx493d_common_returnBitfield(&dut, W2BW_ZH_MSBS_e) );

    // LSBs
    TEST_ASSERT_EQUAL_HEX8( (((int16_t) 0x8ABC) >> 1) & 0x07, tlx493d_common_returnBitfield(&dut, W2BW_XL_LSBS_e) );
    TEST_ASSERT_EQUAL_HEX8( (((int16_t) 0x80BC) >> 1) & 0x07, tlx493d_common_returnBitfield(&dut, W2BW_XH_LSBS_e) );
    TEST_ASSERT_EQUAL_HEX8( (((int16_t) 0x800C) >> 1) & 0x07, tlx493d_common_returnBitfield(&dut, W2BW_YL_LSBS_e) );
    TEST_ASSERT_EQUAL_HEX8( (((int16_t) 0x8FBC) >> 1) & 0x07, tlx493d_common_returnBitfield(&dut, W2BW_YH_LSBS_e) );
    TEST_ASSERT_EQUAL_HEX8( (((int16_t) 0x8F0C) >> 1) & 0x07, tlx493d_common_returnBitfield(&dut, W2BW_ZL_LSBS_e) );
    TEST_ASSERT_EQUAL_HEX8( (((int16_t) 0x8F00) >> 1) & 0x07, tlx493d_common_returnBitfield(&dut, W2BW_ZH_LSBS_e) );


    TEST_ASSERT( dut.functions->setWakeUpThresholdsAsInteger(&dut, -1, -2, -16, -100, -256, -1024) == true );
    TEST_ASSERT( dut.functions->readRegisters(&dut) == true);

    // MSBs
    TEST_ASSERT_EQUAL_HEX8( (((int16_t) -1) >> 4) & 0xFF,    tlx493d_common_returnBitfield(&dut, W2BW_XL_MSBS_e) );
    TEST_ASSERT_EQUAL_HEX8( (((int16_t) -2) >> 4) & 0xFF,    tlx493d_common_returnBitfield(&dut, W2BW_XH_MSBS_e) );
    TEST_ASSERT_EQUAL_HEX8( (((int16_t) -16) >> 4) & 0xFF,   tlx493d_common_returnBitfield(&dut, W2BW_YL_MSBS_e) );
    TEST_ASSERT_EQUAL_HEX8( (((int16_t) -100) >> 4) & 0xFF,  tlx493d_common_returnBitfield(&dut, W2BW_YH_MSBS_e) );
    TEST_ASSERT_EQUAL_HEX8( (((int16_t) -256) >> 4) & 0xFF,  tlx493d_common_returnBitfield(&dut, W2BW_ZL_MSBS_e) );
    TEST_ASSERT_EQUAL_HEX8( (((int16_t) -1024) >> 4) & 0xFF, tlx493d_common_returnBitfield(&dut, W2BW_ZH_MSBS_e) );

    // LSBs
    TEST_ASSERT_EQUAL_HEX8( (((int16_t) -1) >> 1) & 0x07,    tlx493d_common_returnBitfield(&dut, W2BW_XL_LSBS_e) );
    TEST_ASSERT_EQUAL_HEX8( (((int16_t) -2) >> 1) & 0x07,    tlx493d_common_returnBitfield(&dut, W2BW_XH_LSBS_e) );
    TEST_ASSERT_EQUAL_HEX8( (((int16_t) -16) >> 1) & 0x07,   tlx493d_common_returnBitfield(&dut, W2BW_YL_LSBS_e) );
    TEST_ASSERT_EQUAL_HEX8( (((int16_t) -100) >> 1) & 0x07,  tlx493d_common_returnBitfield(&dut, W2BW_YH_LSBS_e) );
    TEST_ASSERT_EQUAL_HEX8( (((int16_t) -256) >> 1) & 0x07,  tlx493d_common_returnBitfield(&dut, W2BW_ZL_LSBS_e) );
    TEST_ASSERT_EQUAL_HEX8( (((int16_t) -1024) >> 1) & 0x07, tlx493d_common_returnBitfield(&dut, W2BW_ZH_LSBS_e) );
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
