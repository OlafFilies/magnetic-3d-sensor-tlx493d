// test includes
#include "Test_includes.hpp"


const uint8_t POWER_PIN = LED2;

static ifx::tlx493d::Kit2GoBoardSupport bsc;


extern "C" {
    // project includes
    #include "Test_TLx493D_P2B6_needsSensor.h"
    #include "Test_tlx493d_commonFunctions_needsSensor.h"


    // Method invoked by Unity before a test suite is run 
    void TLx493D_P2B6_needsSensor_suiteSetup() {
        // deinit in TEAR_DOWN will cut communication link, so if deinit is called communication must be reinitialized !
        (void) TLx493D_P2B6_init(&dut);


    // pinMode(POWER_PIN, OUTPUT);
    // digitalWrite(POWER_PIN, HIGH);
    // delayMicroseconds(0);

        // bsc.setPowerPin(POWER_PIN, OUTPUT, HIGH, LOW, 0, 50000);
        // ifx::tlx493d::initBoardSupport(&dut, bsc);
        // bsc.begin();

        ifx::tlx493d::initCommunication(&dut, Wire, TLx493D_IIC_ADDR_A0_e);
        dut.functions->setDefaultConfig(&dut);
    }
    
    
    // Method invoked by Unity after a test suite is run 
    void TLx493D_P2B6_needsSensor_suiteTearDown() {
        ifx::tlx493d::deinitCommunication(&dut);
    // digitalWrite(POWER_PIN, LOW);
    // delayMicroseconds(200000);

        // If deinitializing here make sure to reinit in 'TEST_SETUP' or communication will be lost !
        dut.functions->deinit(&dut);
    }
}
