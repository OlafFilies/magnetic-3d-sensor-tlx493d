// test includes
#include "Test_includes.hpp"


#define POWER_PIN_LOW 3


extern "C" {
    // project includes
    #include "Test_TLx493D_P3I8_needsSensor.h"
    #include "Test_tlx493d_commonFunctions_needsSensor.h"


    // Method invoked by Unity before a test suite is run 
    void TLx493D_P3I8_needsSensor_suiteSetup() {
        // deinit in TEAR_DOWN will cut communication link, so if deinit is called communication must be reinitialized !
        (void) TLx493D_P3I8_init(&dut);

        Kit2GoBoardSupport bsc;
        // bsc.setPowerPin(LED2, OUTPUT, HIGH, LOW, 50, 50); // done in main !
        bsc.setSelectPin(POWER_PIN_LOW, OUTPUT, LOW, HIGH, 50, 50);
        bsc.begin();
        tlx493d_initBoardSupport(&dut, bsc);
        
        tlx493d_initCommunication(&dut, SPI);
        TLx493D_P3I8_setDefaultConfig(&dut);
    }
    
    
    // Method invoked by Unity after a test suite is run 
    void TLx493D_P3I8_needsSensor_suiteTearDown() {
        // If deinitializing here make sure to reinit in 'TEST_SETUP' or communication will be lost !
        (void) TLx493D_P3I8_deinit(&dut);
    }
}
