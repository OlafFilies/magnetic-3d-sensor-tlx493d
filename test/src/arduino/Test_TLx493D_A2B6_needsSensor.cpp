// test includes
#include "Test_includes.hpp"


extern "C" {
    // project includes
    #include "Test_TLx493D_A2B6_needsSensor.h"
    #include "Test_tlx493d_commonFunctions_needsSensor.h"
    

    // Method invoked by Unity before a test suite is run 
    void TLx493D_A2B6_needsSensor_suiteSetup() {
        // print("\nTLx493D_A2B6_needsSensor_suiteSetup ...\n");

        // deinit in TEAR_DOWN will cut communication link, so if deinit is called communication must be reinitialized !
        TLx493D_A2B6_init(&dut);
        TLx493D_initCommunication(&dut, Wire);
        dut.functions->setDefaultConfig(&dut);
    }
    
    
    // Method invoked by Unity after a test suite is run 
    void TLx493D_A2B6_needsSensor_suiteTearDown() {
        // print("\nTLx493D_A2B6_needsSensor_suiteTearDown ...\n");

        // If deinitializing here make sure to reinit in 'TEST_SETUP' or communication will be lost !
        dut.functions->deinit(&dut);
    }
}
