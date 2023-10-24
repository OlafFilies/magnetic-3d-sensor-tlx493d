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
    #include "sensor_types.h"
    #include "sensors_gen_2_common_defines.h"
    #include "sensors_gen_2_common.h"
    // #include "sensors_gen_3_common_defines.h"
    // #include "sensors_gen_3_common.h"
    #include "sensors_common.h"

    #include "TLE493D_A2B6.h"
    // #include "TLE493D_P2B6.h"
    // #include "TLE493D_W2B6.h"

    // #include "TLE493D_P3B6.h"
    // #include "TLE493D_P3I8.h"
}

// extern "C" void frameworkReset(Sensor_ts *sensor);


Sensor_ts dut;


// void printRegMap2(Sensor_ts *sensor) {
//     Serial.print("regMap : "); 
 
//     for(int i = 0; i < sensor->regMapSize; ++i) {
//         Serial.print("  0x");
//         Serial.print(sensor->regMap[i], HEX);
//         // Serial.print(sensor->regMap[i], BIN);
//     }
 
//     Serial.println();
//  }


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


    init(&dut, TLE493D_A2B6_e);
    // init(&dut, TLE493D_P2B6_e);
    // init(&dut, TLE493D_W2B6_e);
    // init(&dut, TLE493D_P3B6_e);

    initComLibIF(&dut, Wire);

#else

    pinMode(POWER_PIN_LOW, OUTPUT);
    digitalWrite(POWER_PIN_LOW, HIGH);

    // SPI
    // required for S2Go
    // pinMode(POWER_PIN_LOW, OUTPUT);
    // digitalWrite(POWER_PIN_LOW, LOW);

    init(&dut, TLE493D_P3I8_e);
    initComLibIF(&dut, SPI);

#endif

    digitalWrite(POWER_PIN_LOW, LOW);
    setDefaultConfig(&dut);
    digitalWrite(POWER_PIN_LOW, HIGH);
    

    // Serial.println("setDefaultConfig done.");
    // printRegMap(&dut);
    // // frameworkReset(&dut);
    // Serial.println("updateRegisterMap ...");
    // readRegisters(&dut);
    // Serial.println("updateRegisterMap done.");
    // printRegMap(&dut);
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
    Serial.print(true == getTemperature(&dut, &temp) ? "getTemperature ok\n" : "getTemperature error\n");
    // digitalWrite(POWER_PIN_LOW, HIGH);

    Serial.print("Temperature is: ");
    Serial.print(temp);
    Serial.println("°C");

    // digitalWrite(POWER_PIN_LOW, LOW);
    Serial.print(true == getMagneticField(&dut, &valX, &valY, &valZ) ? "getMagneticField ok\n" : "getMagneticField error\n");
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

    Serial.print(true == isFunctional(&dut) ? "isFunctional\n" : "NOT isFunctional\n");
    Serial.print(true == hasValidData(&dut) ? "hasValidData\n" : "NOT hasValidData\n");
    Serial.print(true == hasValidTemperatureData(&dut) ? "hasValidTemperatureData\n" : "NOT hasValidTemperatureData\n");
    Serial.print(true == hasValidMagneticFieldData(&dut) ? "hasValidMagneticFieldData\n" : "NOT hasValidFieldData\n");


    Serial.print(true == TLE493D_A2B6_hasValidIICadr(&dut) ? "TLE493D_A2B6_hasValidIICadr\n" : "NOT TLE493D_A2B6_hasValidIICadr\n");
    Serial.print(true == TLE493D_A2B6_hasWakeup(&dut) ? "TLE493D_A2B6_hasWakeup\n" : "NOT TLE493D_A2B6_hasWakeup\n");


    // printRegMap(&dut);
    // Serial.print("\n");

    // setIICAddress(&dut, GEN_2_STD_IIC_ADDR_A1);
    // Serial.print(dut.comLibIFParams.i2c_params.address << 1);
    // Serial.print(true == TLE493D_A2B6_hasValidIICadr(&dut) ? "TLE493D_A2B6_hasValidIICadr\n" : "NOT TLE493D_A2B6_hasValidIICadr\n");

// gen_2_setPowerMode(&dut, 0b00);
//     delay(1000);
// gen_2_setPowerMode(&dut, 0b01);
//     delay(1000);

// TLE493D_A2B6_disableCollisionAvoidance(&dut);
// gen_2_setPowerMode(&dut, 0b11);
//    delay(1000);
// TLE493D_A2B6_enableCollisionAvoidance(&dut);

    // printRegMap2(&dut);
    printRegMap(dut.regMap, dut.regMapSize);
    Serial.print("\n");

    delay(1000);
}