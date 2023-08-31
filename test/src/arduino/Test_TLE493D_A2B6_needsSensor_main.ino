// std includes
#include <stdbool.h>

// Arduino includes
#include <SPI.h>
#include <Arduino.h>

// project cpp includes
#include "arduino_defines.h"


extern "C" {
    // project c includes
    #include "sensor_types.h"
    #include "sensors_common.h"

    // Unity c includes
    #include "unity_fixture.h"


    Sensor_ts dut;


    void RunAllTests(void)
    {
        // TODO: Use flags / defines to determine which groups of tests should be added to the test. Use the '--build-property option for the 'arduino_compile' target
        // to add the flags by defining the respective targets in the makefile.
        // Setup to run all test in a single run !
        // makefile : --build-property "compiler.cpp.extra_flags=\"-D<TEST_SPECIFIER>=1\"" build
        // See also UnityMain options !

        RUN_TEST_GROUP(TLE493D_A2B6_needsSensor);
    }
}


// Arduino setup method
void setup() {
    Serial.begin(115200);

    // required for S2Go
    pinMode(LED2, OUTPUT);
    digitalWrite(LED2, HIGH);

    init(&dut, TLE493D_A2B6_e);
    initI2CComLibIF(&dut, Wire);
    setDefaultConfig(&dut);

    Serial.print("setup done.\n");
}


// Arduino main loop method
void loop() {
    Serial.println("\n");

    // TODO: use other options in order to automate test runs !
    int         argc       = 2;
    const char *argv[argc] = { "", "-v" };

    (void) UnityMain(argc, argv, RunAllTests);
    delay(3000);
}