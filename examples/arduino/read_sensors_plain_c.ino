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
    // #include "tlx493d_gen_1_common_defines.h"
    // #include "tlx493d_gen_1_common.h"
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


    // init(&dut, TLx493D_A2B6_e);
    // init(&dut, TLx493D_P2B6_e);
    init(&dut, TLx493D_W2B6_e);
    // init(&dut, TLx493D_P3B6_e);

    initComLibIF(&dut, Wire);

#else

    pinMode(POWER_PIN_LOW, OUTPUT);
    digitalWrite(POWER_PIN_LOW, HIGH);

    // SPI
    // required for S2Go
    // pinMode(POWER_PIN_LOW, OUTPUT);
    // digitalWrite(POWER_PIN_LOW, LOW);

    init(&dut, TLx493D_P3I8_e);
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
// // logMsg("logMsg  INFO : ", 11.11, 1, 2.330, "hallo !");
// // info("%d  0x%x  '%f'  %c  %s\n", 11.11, 1, 2, 2.330, 'n', "hallo !");

// info("'%d'\n", 30);

// info("%d  %d  %c  %s\n", 1, 3, 'l', "hallo !");
// info("%d  '%f'\n", 30, 3.0);
// info("'%f'\n", (double) 3.0);
// info("'%lf'\n", 3.0);
// info("'%lf'\n", (double) 3);
// info("%d  '%e'  %s\n", 1, 2.33, "hallo !");
// warn("%d  0x%x  '%f'  %s\n", 1, 2, 2.330, "hallo !");
// error("%d  0x%x  '%g'  %c  %s\n", 1, 2, 2.330, 'n', "hallo !");

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


    Serial.print(true == TLx493D_A2B6_hasValidIICadr(&dut) ? "TLx493D_A2B6_hasValidIICadr\n" : "NOT TLx493D_A2B6_hasValidIICadr\n");
    Serial.print(true == TLx493D_A2B6_hasWakeup(&dut) ? "TLx493D_A2B6_hasWakeup\n" : "NOT TLx493D_A2B6_hasWakeup\n");


    // printRegMap(&dut);
    // Serial.print("\n");

    // setIICAddress(&dut, GEN_2_STD_IIC_ADDR_A1);
    // Serial.print(dut.comLibIFParams.i2c_params.address << 1);
    // Serial.print(true == TLx493D_A2B6_hasValidIICadr(&dut) ? "TLx493D_A2B6_hasValidIICadr\n" : "NOT TLx493D_A2B6_hasValidIICadr\n");

// gen_2_setPowerMode(&dut, 0b00);
//     delay(1000);
// gen_2_setPowerMode(&dut, 0b01);
//     delay(1000);

// TLx493D_A2B6_disableCollisionAvoidance(&dut);
// gen_2_setPowerMode(&dut, 0b11);
//    delay(1000);
// TLx493D_A2B6_enableCollisionAvoidance(&dut);

    // printRegMap2(&dut);
    printRegisters(dut.regMap, dut.regMapSize);
    Serial.print("\n");

    delay(1000);
}