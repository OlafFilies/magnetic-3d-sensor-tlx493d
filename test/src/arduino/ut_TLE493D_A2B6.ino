// std includes
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Arduino includes
#include <SPI.h>
#include <Arduino.h>

// project cpp includes
#include "arduino_defines.h"
#include "ut_arduino_inc.h"


extern "C" {
    #include "sensor_types.h"
    #include "sensors_common.h"

    void putCharacter(char c) {
        Serial.print(c);
        Serial.flush();
    }

    void flushCharacter() {
        Serial.flush();
    }
}


extern "C" void setUp(void) {
}


extern "C" void tearDown(void) {
}


extern "C" void suiteSetUp(void) {
}
    
    
extern "C" int suiteTearDown(int num_failures) {
    return 0;
}


Sensor_ts a2b6;


extern "C" void test_default_config(void) {
    TEST_ASSERT_EQUAL( true, updateRegisterMap(&a2b6) );
    TEST_ASSERT_EQUAL_HEX( 0x94, a2b6.regMap[0x11] );
}
    
    
extern "C" void test_set_bit_field(void) {
}


void setup() {
    Serial.begin(115200);
    delay(100);

    // required for S2Go
    // pinMode(LED2, OUTPUT);
    // digitalWrite(LED2, HIGH);

    init(&a2b6, TLE493D_A2B6_e);
    initI2CComLibIF(&a2b6, Wire);
    setDefaultConfig(&a2b6);

    delay(100);
    Serial.print("setup done.\n");
}


void loop() {
    UnityBegin("test/arduino/ut_TLE493D_A2B6.ino");
 
    RUN_TEST(test_default_config);
    
    UnityEnd();

    delay(3000);
}