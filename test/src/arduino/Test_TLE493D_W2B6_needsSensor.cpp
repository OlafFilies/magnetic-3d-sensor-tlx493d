// test includes
#include "Test_includes.hpp"


extern "C" {
    // project includes
    #include "Test_TLE493D_W2B6_needsSensor.h"
    #include "Test_sensors_commonFunctions_needsSensor.h"


    // Method invoked by Unity before a test suite is run 
    void TLE493D_W2B6_needsSensor_suiteSetup() {
        // deinit in TEAR_DOWN will cut communication link, so if deinit is called communication must be reinitialized !
        (void) TLE493D_A2B6_init(&dut);
        // (void) TLE493D_W2B6_init(&dut);
        initI2CComLibIF(&dut, Wire);
        setDefaultConfig(&dut);
    }
    
    
    // Method invoked by Unity after a test suite is run 
    void TLE493D_W2B6_needsSensor_suiteTearDown() {
        // If deinitializing here make sure to reinit in 'TEST_SETUP' or communication will be lost !
        (void) TLE493D_A2B6_deinit(&dut);
        // (void) TLE493D_W2B6_deinit(&dut);
    }
}
