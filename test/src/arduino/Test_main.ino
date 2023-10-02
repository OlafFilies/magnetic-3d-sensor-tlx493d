// test includes
#include "Test_includes.hpp"


#define POWER_PIN LED2 // 15


extern "C" {
    void RunAllTests(void)
    {
        // TODO: Use flags / defines to determine which groups of tests should be added to the test. Use the '--build-property option for the 'arduino_compile' target
        // to add the flags by defining the respective targets in the makefile.
        // Setup to run all test in a single run !
        // makefile : --build-property "compiler.cpp.extra_flags=\"-D<TEST_SPECIFIER>=1\"" build
        // See also UnityMain options !


// TLE493D_A1B6
#ifdef TEST_TLE493D_A1B6

    RUN_TEST_GROUP(TLE493D_A1B6);

#endif

#ifdef TEST_TLE493D_A1B6_NEEDS_SENSOR

    RUN_TEST_GROUP(TLE493D_A1B6_needsSensor);

#endif


// TLE493D_A2B6
#ifdef TEST_TLE493D_A2B6

    RUN_TEST_GROUP(TLE493D_A2B6);

#endif

#ifdef TEST_TLE493D_A2B6_NEEDS_SENSOR

    RUN_TEST_GROUP(TLE493D_A2B6_needsSensor);

#endif


// TLV493D_A2BW
#ifdef TEST_TLV493D_A2BW

    RUN_TEST_GROUP(TLV493D_A2BW);

#endif

#ifdef TEST_TLV493D_A2BW_NEEDS_SENSOR

    RUN_TEST_GROUP(TLV493D_A2BW_needsSensor);

#endif


// TLE493D_P2B6
#ifdef TEST_TLE493D_P2B6

    RUN_TEST_GROUP(TLE493D_P2B6);

#endif

#ifdef TEST_TLE493D_P2B6_NEEDS_SENSOR

    RUN_TEST_GROUP(TLE493D_P2B6_needsSensor);

#endif


// TLE493D_W2B6
#ifdef TEST_TLE493D_W2B6

    RUN_TEST_GROUP(TLE493D_W2B6);

#endif

#ifdef TEST_TLE493D_W2B6_NEEDS_SENSOR

    RUN_TEST_GROUP(TLE493D_W2B6_needsSensor);

#endif

    }
}


// Arduino setup method
void setup() {
    Serial.begin(115200);
    delay(3000);

    // required for S2Go
    pinMode(POWER_PIN, OUTPUT);
    digitalWrite(POWER_PIN, HIGH);

    delay(500);
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