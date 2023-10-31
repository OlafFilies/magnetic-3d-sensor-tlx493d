// std includes

// Arduino includes
#include <Arduino.h>

// project cpp includes
#include "arduino_defines.h"
#include "Logger.h"


// S2Go boards
#define POWER_PIN_HIGH 15

// SPI chip
#define POWER_PIN_LOW 3


extern "C" {
    #include "tlx493d_types.h"
    #include "tlx493d_gen_2_common_defines.h"
    #include "tlx493d_gen_2_common.h"
    #include "tlx493d_gen_3_common_defines.h"
    #include "tlx493d_gen_3_common.h"

    #include "tlx493d_common.h"
    #include "cInterface.h"

    #include "TLx493D_A1B6.h"

    #include "TLx493D_A2B6.h"
    #include "TLx493D_P2B6.h"
    #include "TLx493D_W2B6.h"
    #include "TLx493D_W2BW.h"

    #include "TLx493D_P3B6.h"
    #include "TLx493D_P3I8.h"
}


TLx493D_ts dut;


TLx493D_ts a1b6;

TLx493D_ts a2b6;
TLx493D_ts p2b6;
TLx493D_ts w2b6;
TLx493D_ts w2bw;

TLx493D_ts p3b6;
TLx493D_ts p3i8;


#define USE_I2C  (1)


void setup() {
    // // required for S2Go
    // pinMode(POWER_PIN_HIGH, OUTPUT);
    // digitalWrite(POWER_PIN_HIGH, HIGH);

    Serial.begin(115200);
    delay(3000);


#ifdef USE_I2C

    // I2C
    // required for S2Go
    pinMode(POWER_PIN_HIGH, OUTPUT);
    digitalWrite(POWER_PIN_HIGH, HIGH);


    // tlx493d_init(&dut, TLx493D_A1B6_e);
    // tlx493d_init(&dut, TLx493D_A2B6_e);
    // tlx493d_init(&dut, TLx493D_P2B6_e);
    tlx493d_init(&dut, TLx493D_W2B6_e);
    // tlx493d_init(&dut, TLx493D_W2BW_e);
    // tlx493d_init(&dut, TLx493D_P3B6_e);

    TLx493D_initCommunication(&dut, Wire);
    tlx493d_setDefaultConfig(&dut);


    // tlx493d_init(&a1b6, TLx493D_A1B6_e);
    tlx493d_init(&a2b6, TLx493D_A2B6_e);
    // tlx493d_init(&p2b6, TLx493D_P2B6_e);
    tlx493d_init(&w2b6, TLx493D_W2B6_e);
    // tlx493d_init(&w2bw, TLx493D_W2BW_e);
    // tlx493d_init(&p3b6, TLx493D_P3B6_e);

    // TLx493D_initCommunication(&a1b6, Wire);
    TLx493D_initCommunication(&a2b6, Wire);
    tlx493d_setDefaultConfig(&a2b6);
    // TLx493D_initCommunication(&p2b6, Wire);
    TLx493D_initCommunication(&w2b6, Wire);
    tlx493d_setDefaultConfig(&w2b6);
    // TLx493D_initCommunication(&w2bw, Wire);
    // TLx493D_initCommunication(&p3b6, Wire);


#else

    pinMode(POWER_PIN_LOW, OUTPUT);
    digitalWrite(POWER_PIN_LOW, HIGH);

    // SPI
    // required for S2Go
    // pinMode(POWER_PIN_LOW, OUTPUT);
    // digitalWrite(POWER_PIN_LOW, LOW);

    tlx493d_init(&dut, TLx493D_P3I8_e);
    TLx493D_initCommunication(&dut, SPI);


    tlx493d_init(&p3i8, TLx493D_P3I8_e);
    TLx493D_initCommunication(&p3i8, SPI);

#endif

    digitalWrite(POWER_PIN_LOW, LOW);
    // tlx493d_setDefaultConfig(&dut);
    digitalWrite(POWER_PIN_LOW, HIGH);
    

    // Serial.println("setDefaultConfig done.");
    // // frameworkReset(&dut);
    // Serial.println("updateRegisterMap ...");
    // readRegisters(&dut);
    // Serial.println("updateRegisterMap done.");
    // setDefaultConfig(&dut);
    // Serial.println("2 setDefaultConfig done.");

    delay(100);
    Serial.println("setup done.");
}


void loop() {
    double temp = 0.0;
    double valX = 0, valY = 0, valZ = 0;
    Serial.println("loop ...");

    // digitalWrite(POWER_PIN_LOW, LOW);
    Serial.print(true == tlx493d_getTemperature(&dut, &temp) ? "getTemperature ok\n" : "getTemperature error\n");
    // digitalWrite(POWER_PIN_LOW, HIGH);

    Serial.print("Temperature is: ");
    Serial.print(temp);
    Serial.println("°C");

    // digitalWrite(POWER_PIN_LOW, LOW);
    Serial.print(true == tlx493d_getMagneticField(&dut, &valX, &valY, &valZ) ? "getMagneticField ok\n" : "getMagneticField error\n");
    // digitalWrite(POWER_PIN_LOW, HIGH);

    Serial.print("Value X is: ");
    Serial.print(valX);
    Serial.println(" mT");
    Serial.print("Value Y is: ");
    Serial.print(valY);
    Serial.println(" mT");
    Serial.print("Value Z is: ");
    Serial.print(valZ);
    Serial.println(" mT");

    Serial.print(true == tlx493d_isFunctional(&dut) ? "isFunctional\n" : "NOT isFunctional\n");
    Serial.print(true == tlx493d_hasValidData(&dut) ? "hasValidData\n" : "NOT hasValidData\n");
    Serial.print(true == tlx493d_hasValidTemperatureData(&dut) ? "hasValidTemperatureData\n" : "NOT hasValidTemperatureData\n");
    Serial.print(true == tlx493d_hasValidMagneticFieldData(&dut) ? "hasValidMagneticFieldData\n" : "NOT hasValidFieldData\n");


    Serial.print(true == TLx493D_A2B6_hasValidIICadr(&dut) ? "TLx493D_A2B6_hasValidIICadr\n" : "NOT TLx493D_A2B6_hasValidIICadr\n");
    Serial.print(true == TLx493D_A2B6_hasWakeup(&dut) ? "TLx493D_A2B6_hasWakeup\n" : "NOT TLx493D_A2B6_hasWakeup\n");


    // tlx493d_setIICAddress(&dut, GEN_2_STD_IIC_ADDR_A1);
    // Serial.print(dut.comLibIFParams.i2c_params.address << 1);
    // Serial.print(true == TLx493D_A2B6_hasValidIICadr(&dut) ? "TLx493D_A2B6_hasValidIICadr\n" : "NOT TLx493D_A2B6_hasValidIICadr\n");

    // tlx493d_setPowerMode(&dut, 0b00);
    // delay(1000);
    // tlx493d_setPowerMode(&dut, 0b01);
    // delay(1000);

    // TLx493D_A2B6_disableCollisionAvoidance(&dut);
    // tlx493d_setPowerMode(&dut, 0b11);
    // delay(1000);
    // TLx493D_A2B6_enableCollisionAvoidance(&dut);

    printRegisters(dut.regMap, dut.regMapSize);
    Serial.print("\n");

    delay(1000);
}