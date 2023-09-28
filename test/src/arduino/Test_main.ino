// test includes
#include "Test_includes.hpp"


#define POWER_PIN LED2 // 15


extern "C" {
    void TLE493D_A2B6_suiteSetUp(void);
    void TLE493D_A2B6_suiteTearDown(void);
    void TLE493D_A2B6_needsSensor_suiteSetup(void);
    void TLE493D_A2B6_needsSensor_suiteTearDown(void);


    void RunAllTests(void)
    {
        // TODO: Use flags / defines to determine which groups of tests should be added to the test. Use the '--build-property option for the 'arduino_compile' target
        // to add the flags by defining the respective targets in the makefile.
        // Setup to run all test in a single run !
        // makefile : --build-property "compiler.cpp.extra_flags=\"-D<TEST_SPECIFIER>=1\"" build
        // See also UnityMain options !

#ifdef TEST_TLE493D_A2B6

    TLE493D_A2B6_suiteSetUp();
    RUN_TEST_GROUP(TLE493D_A2B6);
    TLE493D_A2B6_suiteTearDown();

#endif

#ifdef TEST_TLE493D_A2B6_NEEDS_SENSOR

    TLE493D_A2B6_needsSensor_suiteSetup();
    RUN_TEST_GROUP(TLE493D_A2B6_needsSensor);
    TLE493D_A2B6_needsSensor_suiteTearDown();

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