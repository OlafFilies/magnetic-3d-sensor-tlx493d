// std includes
#include <assert.h>

// Arduino includes
#include <SPI.h>
#include <Arduino.h>
#include "xmc_i2c.h"
#include "XMC1100.h"

// project cpp includes
#include "arduino_defines.h"

#define POWER_PIN 15


extern "C" {
    #include "sensor_types.h"
    #include "sensors_common.h"
}

extern "C" void frameworkReset(Sensor_ts *sensor);


Sensor_ts a2b6;


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

    init(&a2b6, TLE493D_A2B6_e);
    initI2CComLibIF(&a2b6, Wire);
    
    setDefaultConfig(&a2b6);
    Serial.println("1 setDefaultConfig done.");
    printRegMap(&a2b6);
    // frameworkReset(&a2b6);
    Serial.println("updateRegisterMap ...");
    updateRegisterMap(&a2b6);
    Serial.println("updateRegisterMap done.");
    printRegMap(&a2b6);
    setDefaultConfig(&a2b6);
    Serial.println("2 setDefaultConfig done.");

    delay(100);
    Serial.println("setup done.");
}


void loop() {
    float temp = 0.0;
    float valX = 0, valY = 0, valZ = 0;
    Serial.println("loop ...");

    Serial.print(true == getTemperature(&a2b6, &temp) ? "getTemperature ok\n" : "getTemperature error\n");
    Serial.print("Temperature is: ");
    Serial.print(temp);
    Serial.println("°C");

    Serial.print(true == getFieldValues(&a2b6, &valX, &valY, &valZ) ? "getFieldValues ok\n" : "getFieldValues error\n");
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