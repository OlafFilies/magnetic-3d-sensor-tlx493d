// test includes
#include "Test_includes.hpp"


const uint8_t SELECT_PIN = 3;


extern "C" {
    // project includes
    #include "Test_TLx493D_P3I8_needsSensor.h"
    #include "Test_tlx493d_commonFunctions_needsSensor.h"


    // Method invoked by Unity before a test suite is run 
    void TLx493D_P3I8_needsSensor_suiteSetup() {
        // deinit in TEAR_DOWN will cut communication link, so if deinit is called communication must be reinitialized !
        (void) TLx493D_P3I8_init(&dut);

        ifx::tlx493d::Kit2GoBoardSupport bsc;
        bsc.setSelectPin(SELECT_PIN, OUTPUT, LOW, HIGH, 50, 50);
        // bsc.begin();
        ifx::tlx493d::initBoardSupport(&dut, bsc);
        ifx::tlx493d::initCommunication(&dut, SPI);
        bsc.begin();

        dut.functions->setDefaultConfig(&dut);
    }
    
    
    // Method invoked by Unity after a test suite is run 
    void TLx493D_P3I8_needsSensor_suiteTearDown() {
        // If deinitializing here make sure to reinit in 'TEST_SETUP' or communication will be lost !
        dut.functions->deinit(&dut);
    }
}
