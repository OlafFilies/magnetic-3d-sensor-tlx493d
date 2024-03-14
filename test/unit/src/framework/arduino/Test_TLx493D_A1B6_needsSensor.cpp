#include "Test_includes.hpp"


const uint8_t POWER_PIN   = LED2;
const uint8_t ADDRESS_PIN = 7;

static ifx::tlx493d::Kit2GoBoardSupport bsc;


extern "C" {
    // project includes
    #include "Test_TLx493D_A1B6_needsSensor.h"
    #include "Test_tlx493d_commonFunctions_needsSensor.h"


    // Method invoked by Unity before a test suite is run 
    void TLx493D_A1B6_needsSensor_suiteSetup() {
        // deinit in TEAR_DOWN will cut communication link, so if deinit is called communication must be reinitialized !
        (void) TLx493D_A1B6_init(&dut);

        bsc.setPowerPin(POWER_PIN, OUTPUT, INPUT, HIGH, LOW, 0, 250000);
        ifx::tlx493d::initBoardSupport(&dut, bsc);
        bsc.init();

        ifx::tlx493d::initCommunication(&dut, Wire, TLx493D_IIC_ADDR_A0_e, true);
        dut.functions->setDefaultConfig(&dut);
    }
    
    
    // Method invoked by Unity after a test suite is run 
    void TLx493D_A1B6_needsSensor_suiteTearDown() {
        // TODO: deinitCommunication in conjunction with bsc.deinit causes issues !
        ifx::tlx493d::deinitCommunication(&dut, false);
        // ifx::tlx493d::deinitCommunication(&dut, true);
        bsc.deinit();

        // If deinitializing here make sure to reinit in 'TEST_SETUP' or communication will be lost !
        dut.functions->deinit(&dut);
    }


    // Method invoked by Unity before a test suite is run 
    void TLx493D_A1B6_atReset_suiteSetup() {
        // deinit in TEAR_DOWN will cut communication link, so if deinit is called communication must be reinitialized !
        (void) TLx493D_A1B6_init(&dut);

        bsc.setPowerPin(POWER_PIN, OUTPUT, INPUT, HIGH, LOW, 0, 250000);
        ifx::tlx493d::initBoardSupport(&dut, bsc);
        bsc.init();

        ifx::tlx493d::initCommunication(&dut, Wire, TLx493D_IIC_ADDR_A0_e, true);
    }
    
    
    // Method invoked by Unity after a test suite is run 
    void TLx493D_A1B6_atReset_suiteTearDown() {
        // TODO: deinitCommunication in conjunction with bsc.deinit causes issues !
        ifx::tlx493d::deinitCommunication(&dut, false);
        // ifx::tlx493d::deinitCommunication(&dut, true);
        bsc.deinit();

        // If deinitializing here make sure to reinit in 'TEST_SETUP' or communication will be lost !
        dut.functions->deinit(&dut);
    }

    // TODO: make it work !
    // Method invoked by Unity before a test suite is run 
    void TLx493D_A1B6_extendedAddresses_suiteSetup() {
        // deinit in TEAR_DOWN will cut communication link, so if deinit is called communication must be reinitialized !
        (void) TLx493D_A1B6_init(&dut);

        // deinit in TEAR_DOWN will cut communication link, so if deinit is called communication must be reinitialized !
        logPrint("\nbsc object used to power on board; check if POWER PIN connected!\ncheck if ADDR pin is connected!\n");
        // power up the board using bsc
        bsc.setPowerPin(POWER_PIN, OUTPUT, INPUT, HIGH, LOW, 0, 250000);
        bsc.setAddressPin(ADDRESS_PIN, OUTPUT, INPUT, HIGH, LOW, 1000, 1000);
        ifx::tlx493d::initBoardSupport(&dut, bsc);
        bsc.init(true, false, true);

        ifx::tlx493d::initCommunication(&dut, Wire, TLx493D_IIC_ADDR_A4_e, true);
        dut.functions->setDefaultConfig(&dut);
    }


    // Method invoked by Unity after a test suite is run 
    void TLx493D_A1B6_extendedAddresses_suiteTearDown() {
        // TODO: deinitCommunication in conjunction with bsc.deinit causes issues !
        ifx::tlx493d::deinitCommunication(&dut, false);
        // ifx::tlx493d::deinitCommunication(&dut, true);
        bsc.deinit();

        // If deinitializing here make sure to reinit in 'TEST_SETUP' or communication will be lost !
        dut.functions->deinit(&dut);
        // (void) TLx493D_A1B6_deinit(&dut);
    }
}