// std includes
#include <setjmp.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <SPI.h>
#include <Arduino.h>

// project cpp includes
#include "arduino_defines.h"

extern "C" {
    #include "sensor_types.h"
    #include "sensors_common.h"
    #include "unity.h"
}

Sensor_ts a2bw;

void initTests(void) {
    Serial.begin(115200);
    delay(100);

    // required for S2Go
    // pinMode(LED2, OUTPUT);
    // digitalWrite(LED2, HIGH);

    init(&a2bw, TLV493D_A2BW_e, I2C_e);
    initI2CComLibIF(&a2bw, Wire);
    // reset(&a2b6);
    setDefaultConfig(&a2bw);

    delay(100);
    Serial.print("setup done.\n");
}

void setUp(void) {

}

void tearDown(void) {

}

void test_default_config(void) {
    TEST_ASSERT_EQUAL( true, updateRegisterMap(&a2bw) );
    TEST_ASSERT_EQUAL_HEX( 0x94, a2bw.regMap[0x11] );
}

void test_set_bit_field(void) {

}

void setup() {
    initTests();
}

void loop() {

    UnityBegin("test/arduino/ut_TLV493D_A2BW.ino");
    
    RUN_TEST(test_default_config);

    return UnityEnd();
}
