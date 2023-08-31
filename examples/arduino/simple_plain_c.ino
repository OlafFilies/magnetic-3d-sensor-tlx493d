// std includes
#include <assert.h>

// Arduino includes
#include <SPI.h>
#include <Arduino.h>
#include "xmc_i2c.h"

// project cpp includes
#include "arduino_defines.h"

#define POWER_PIN 15


extern "C" {
    #include "sensor_types.h"
    #include "sensors_common.h"
}

extern "C" void frameworkReset(Sensor_ts *sensor);


Sensor_ts dut;


void printRegMap(Sensor_ts *sensor) {
    Serial.print("regMap : "); 
 
    for(int i = 0; i < sensor->regMapSize; ++i) {
        Serial.print("   ");
        Serial.print(sensor->regMap[i]);
    }
 
    Serial.println();
 }
 

void setup() {
    Serial.begin(115200);
    delay(3000);

    // required for S2Go
    pinMode(POWER_PIN, OUTPUT);
    digitalWrite(POWER_PIN, HIGH);

    init(&dut, TLE493D_A2B6_e);
    initI2CComLibIF(&dut, Wire);
    
    setDefaultConfig(&dut);
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
    float temp = 0.0;
    float valX = 0, valY = 0, valZ = 0;
    Serial.println("loop ...");

    Serial.print(true == getTemperature(&dut, &temp) ? "getTemperature ok\n" : "getTemperature error\n");
    Serial.print("Temperature is: ");
    Serial.print(temp);
    Serial.println("°C");

    Serial.print(true == getFieldValues(&dut, &valX, &valY, &valZ) ? "getFieldValues ok\n" : "getFieldValues error\n");
    Serial.print("Value X is: ");
    Serial.print(valX);
    Serial.println(" mT");
    Serial.print("Value Y is: ");
    Serial.print(valY);
    Serial.println(" mT");
    Serial.print("Value Z is: ");
    Serial.print(valZ);
    Serial.println(" mT");

    delay(1000);
}