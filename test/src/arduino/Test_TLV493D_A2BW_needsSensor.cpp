// test includes
#include "Test_includes.hpp"


extern "C" {
    // project includes
    #include "Test_TLV493D_A2BW_needsSensor.h"
    #include "Test_sensors_commonFunctions_needsSensor.h"


    // Method invoked by Unity before a test suite is run 
    void TLV493D_A2BW_needsSensor_suiteSetup() {
        // deinit in TEAR_DOWN will cut communication link, so if deinit is called communication must be reinitialized !
        (void) TLV493D_A2BW_init(&dut);
        initI2CComLibIF(&dut, Wire);
        setDefaultConfig(&dut);
    }
    
    
    // Method invoked by Unity after a test suite is run 
    void TLV493D_A2BW_needsSensor_suiteTearDown() {
        // If deinitializing here make sure to reinit in 'TEST_SETUP' or communication will be lost !
        (void) TLV493D_A2BW_deinit(&dut);
    }
}
