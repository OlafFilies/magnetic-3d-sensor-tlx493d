// test includes
#include "Test_includes.hpp"


extern "C" {
    // project includes
    #include "Test_TLx493D_A2B6_needsSensor.h"
    #include "Test_tlx493d_commonFunctions_needsSensor.h"


    // Method invoked by Unity before a test suite is run 
    void TLx493D_A2B6_needsSensor_suiteSetup() {
        // Serial.print("\nTLx493D_A2B6_needsSensor_suiteSetup ...\n");

        // deinit in TEAR_DOWN will cut communication link, so if deinit is called communication must be reinitialized !
        (void) TLx493D_A2B6_init(&dut);
        tlx493d_initCommunication(&dut, Wire, TLx493D_IIC_ADDR_A0_e);
        TLx493D_A2B6_setDefaultConfig(&dut);
    }
    
    
    // Method invoked by Unity after a test suite is run 
    void TLx493D_A2B6_needsSensor_suiteTearDown() {
        // Serial.print("TLx493D_A2B6_needsSensor_suiteTearDown ...\n");

        // If deinitializing here make sure to reinit in 'TEST_SETUP' or communication will be lost !
        (void) TLx493D_A2B6_deinit(&dut);
    }
}
