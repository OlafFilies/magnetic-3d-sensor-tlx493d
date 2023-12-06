// project cpp includes
#include "TLx493D_inc.hpp"

#define THRESHOLD         (int16_t)200

#define LOWER_THRESHOLD_X  (-THRESHOLD)
#define LOWER_THRESHOLD_Y  (-THRESHOLD)
#define LOWER_THRESHOLD_Z  (-THRESHOLD)

#define UPPER_THRESHOLD_X  (THRESHOLD)
#define UPPER_THRESHOLD_Y  (THRESHOLD)
#define UPPER_THRESHOLD_Z  (THRESHOLD)

#define INTERRUPT_PIN 8
// TLx493D_A1B6 dut(Wire);

// TLx493D_A2B6 dut(Wire);
// TLx493D_P2B6 dut(Wire);
// TLx493D_W2B6 dut(Wire, TLx493D_IIC_ADDR_A0_e);
TLx493D_W2BW dut(Wire, TLx493D_IIC_ADDR_A0_e);

// TLx493D_A2B6 a2b6(Wire);
// TLx493D_P2B6 p2b6(Wire);
// TLx493D_W2B6 w2b6(Wire);
// TLx493D_W2BW w2bw(Wire);

// TLx493D_P3B6 p3b6(Wire);


void setup() {
    delay(3000);
    Serial.begin(115200);
    delay(100);

    pinMode(INTERRUPT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), wakeUp_ISR, FALLING);

    dut.setPowerPin(LED2, OUTPUT, HIGH, LOW, 50, 50);
    dut.begin();
    printRegisters(dut.getSensor());
    // dut.setMeasurement(TLx493D_BxByBzTemp_e);
    // dut.setPowerMode(TLx493D_LOW_POWER_MODE_e);
    dut.setSensitivity(TLx493D_FULL_RANGE_e);
    dut.enableInterrupt();
    dut.enableWakeUpMode();
    printRegisters(dut.getSensor());
    // // dut.disableWakeUpMode();
    dut.setWakeUpThresholdsAsInteger(LOWER_THRESHOLD_X, UPPER_THRESHOLD_X, LOWER_THRESHOLD_Y, UPPER_THRESHOLD_Y, LOWER_THRESHOLD_Z, UPPER_THRESHOLD_Z);
    Serial.print("isWakeUpEnabled : ");
    Serial.println(dut.isWakeUpEnabled() ? "enabled" : "not enabled");
    // printRegisters(dut.getSensor());

    delay(100);
    Serial.print("setup done.\n");
}


void loop() {
    // double temp1 = 0.0, temp2 = 0.0, temp3 = 0.0;
    // double valX1 = 0, valY1 = 0, valZ1 = 0, valX2 = 0, valY2 = 0, valZ2 = 0, valX3 = 0, valY3 = 0, valZ3 = 0;

    // dut.getTemperature(&temp1); 
    // // // dut2.getTemperature(&temp2); 
    // // // dut3.getTemperature(&temp3); 

    // dut.getMagneticField(&valX1, &valY1, &valZ1);
    // // // dut2.getMagneticField(&valX2, &valY2, &valZ2);
    // // // dut3.getMagneticField(&valX3, &valY3, &valZ3);

    // Serial.println("========================================");
    // Serial.print("Temperature of Sensor 1:\t");Serial.print(temp1);Serial.println(" °C");
    // Serial.print("Magnetic X-Value of Sensor 1:\t");Serial.print(valX1);Serial.println(" mT");
    // Serial.print("Magnetic Y-Value of Sensor 1:\t");Serial.print(valY1);Serial.println(" mT");
    // Serial.print("Magnetic Z-Value of Sensor 1:\t");Serial.print(valZ1);Serial.println(" mT");
    // // Serial.println("----------------------------------------");
    // // // Serial.print("Temperature of Sensor 2:\t");Serial.print(temp2);Serial.println(" °C");
    // // // Serial.print("Magnetic X-Value of Sensor 2:\t");Serial.print(valX2);Serial.println(" mT");
    // // // Serial.print("Magnetic Y-Value of Sensor 2:\t");Serial.print(valY2);Serial.println(" mT");
    // // // Serial.print("Magnetic Z-Value of Sensor 2:\t");Serial.print(valZ2);Serial.println(" mT");
    // // // Serial.println("----------------------------------------");
    // // // Serial.print("Temperature of Sensor 3:\t");Serial.print(temp3);Serial.println(" °C");
    // // // Serial.print("Magnetic X-Value of Sensor 3:\t");Serial.print(valX3);Serial.println(" mT");
    // // // Serial.print("Magnetic Y-Value of Sensor 3:\t");Serial.print(valY3);Serial.println(" mT");
    // // // Serial.print("Magnetic Z-Value of Sensor 3:\t");Serial.print(valZ3);Serial.println(" mT");
    // // // Serial.println("========================================\n\n");
    // // Serial.println(dut.isWakeUpEnabled());

    // // dut.readRegisters();
    // printRegisters(dut.getSensor());
    delay(2000);
    // printRegisters(dut.getSensor());
}

void wakeUp_ISR() {
    Serial.println("Interrupt triggered");
   double temp1 = 0.0, temp2 = 0.0, temp3 = 0.0;
    double valX1 = 0, valY1 = 0, valZ1 = 0, valX2 = 0, valY2 = 0, valZ2 = 0, valX3 = 0, valY3 = 0, valZ3 = 0;

    dut.getTemperature(&temp1); 
    // // dut2.getTemperature(&temp2); 
    // // dut3.getTemperature(&temp3); 

    dut.getMagneticField(&valX1, &valY1, &valZ1);
    // // dut2.getMagneticField(&valX2, &valY2, &valZ2);
    // // dut3.getMagneticField(&valX3, &valY3, &valZ3);

    Serial.println("========================================");
    Serial.print("Temperature of Sensor 1:\t");Serial.print(temp1);Serial.println(" °C");
    Serial.print("Magnetic X-Value of Sensor 1:\t");Serial.print(valX1);Serial.println(" mT");
    Serial.print("Magnetic Y-Value of Sensor 1:\t");Serial.print(valY1);Serial.println(" mT");
    Serial.print("Magnetic Z-Value of Sensor 1:\t");Serial.print(valZ1);Serial.println(" mT");
    printRegisters(dut.getSensor());
}