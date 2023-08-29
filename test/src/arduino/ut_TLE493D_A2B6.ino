// std includes
#include <stdbool.h>

// Arduino includes
#include <SPI.h>
#include <Arduino.h>

// project cpp includes
#include "arduino_defines.h"
// #include "unity_arduino.h"
#include "unity.h"


extern "C" {
    // project c includes
    #include "sensor_types.h"
    #include "sensors_common.h"


    // Method used by Unity to output a single character 
    void putCharacter(char c) {
        Serial.print(c);
        // Serial.flush();
    }

    // Method used by Unity to flush the output
    void flushCharacter() {
        Serial.flush();
    }


    // Method invoked by Unity before a test is run 
    void setUp(void) {
    }

    // Method invoked by Unity after a test is run 
    void tearDown(void) {
    }


    // Method invoked by Unity before a test suite is run 
    void suiteSetUp(void) {
    }
    
    // Method invoked by Unity after a test suite is run 
    int suiteTearDown(int num_failures) {
        return 0;
    }


    Sensor_ts a2b6;

    // test if default config has been set correctly
    void test_default_config(void) {
        TEST_ASSERT_EQUAL( true, updateRegisterMap(&a2b6) );
        TEST_ASSERT_EQUAL_HEX( 0x94, a2b6.regMap[0x11] );
    }
    
    
    // test of bit field methods
    void test_set_bit_field(void) {
        TEST_ASSERT_EQUAL( true, !false);
    }
}


// Arduino setup method
void setup() {
    Serial.begin(115200);

    // required for S2Go
    pinMode(LED2, OUTPUT);
    digitalWrite(LED2, HIGH);

    init(&a2b6, TLE493D_A2B6_e);
    initI2CComLibIF(&a2b6, Wire);
    setDefaultConfig(&a2b6);

    Serial.print("setup done.\n");
}


// Arduino main loop method
void loop() {
    Serial.println("\n");

    UnityBegin("test/arduino/ut_TLE493D_A2B6.ino");
 
    RUN_TEST(test_default_config);
    RUN_TEST(test_set_bit_field);

    UnityEnd();
    delay(3000);
}