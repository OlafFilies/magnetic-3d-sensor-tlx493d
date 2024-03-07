/** Project CPP includes */ 
#include "TLx493D_inc.hpp"


using namespace ifx::tlx493d;


/** Declaration of three sensor objects with the same default address (A0). */
TLx493D_W2B6 dut1(Wire, TLx493D_IIC_ADDR_A0_e);
TLx493D_W2B6 dut2(Wire, TLx493D_IIC_ADDR_A0_e);
TLx493D_W2B6 dut3(Wire, TLx493D_IIC_ADDR_A0_e);

// TODO(Jens): pull power pin defs out as const uint8_t ...

void setup() {
    Serial.begin(115200);
    delay(3000);

    /** In this example we're using the XMC4700-Relax-Kit. 
     *  Here we're using three GPIOs to power up the sensors, one
     *  after the other. This is done by defining the pins with the
     *  Board Support Class and its functions. The power up pins are
     *  enabled during the SensorClassObject.begin() call. See below. 
    */
    // dut1.setPowerPin(8, OUTPUT, HIGH, LOW, 50, 50);
    // dut2.setPowerPin(9, OUTPUT, HIGH, LOW, 50, 50);
    // dut3.setPowerPin(10, OUTPUT, HIGH, LOW, 50, 50);
    dut1.setPowerPin(8, OUTPUT, HIGH, LOW, 0, 250000);
    dut2.setPowerPin(9, OUTPUT, HIGH, LOW, 0, 250000);
    dut3.setPowerPin(10, OUTPUT, HIGH, LOW, 0, 250000);

    /** Here we're enabling one sensor after the other. This procedure 
     *  is necessary, otherwise it can happen that the initialization of the
     *  sensors fails. In between we change the I2C-Address of the sensor
     *  that was just enabled. This has to be done, because the default 
     *  addresses of the senors are identical. For the last sensor this is not 
     *  required anymore, since the other two have a new address.
     */
    dut1.begin();
    // delay(500);
    dut1.setIICAddress(TLx493D_IIC_ADDR_A2_e);
    // delay(500);
    dut2.begin();
    // delay(500);
    dut2.setIICAddress(TLx493D_IIC_ADDR_A1_e);
    // delay(500);
    dut3.begin();
    // delay(500);

    Serial.print("setup done.\n");
}

/** In the loop we're reading out the temperature values as well as the magnetic values in X, Y, Z-direction 
 *  of all three sensors. After that they're all printed to the serial output.
 */
void loop() {
    double temp1 = 0.0, temp2 = 0.0, temp3 = 0.0;
    double valX1 = 0, valY1 = 0, valZ1 = 0, valX2 = 0, valY2 = 0, valZ2 = 0, valX3 = 0, valY3 = 0, valZ3 = 0;

    dut1.getTemperature(&temp1); 
    dut2.getTemperature(&temp2); 
    dut3.getTemperature(&temp3); 

    dut1.getMagneticField(&valX1, &valY1, &valZ1);
    dut2.getMagneticField(&valX2, &valY2, &valZ2);
    dut3.getMagneticField(&valX3, &valY3, &valZ3);

    Serial.println("========================================");
    Serial.print("Temperature of Sensor 1:\t");Serial.print(temp1);Serial.println(" °C");
    Serial.print("Magnetic X-Value of Sensor 1:\t");Serial.print(valX1);Serial.println(" mT");
    Serial.print("Magnetic Y-Value of Sensor 1:\t");Serial.print(valY1);Serial.println(" mT");
    Serial.print("Magnetic Z-Value of Sensor 1:\t");Serial.print(valZ1);Serial.println(" mT");
    Serial.println("----------------------------------------");
    Serial.print("Temperature of Sensor 2:\t");Serial.print(temp2);Serial.println(" °C");
    Serial.print("Magnetic X-Value of Sensor 2:\t");Serial.print(valX2);Serial.println(" mT");
    Serial.print("Magnetic Y-Value of Sensor 2:\t");Serial.print(valY2);Serial.println(" mT");
    Serial.print("Magnetic Z-Value of Sensor 2:\t");Serial.print(valZ2);Serial.println(" mT");
    Serial.println("----------------------------------------");
    Serial.print("Temperature of Sensor 3:\t");Serial.print(temp3);Serial.println(" °C");
    Serial.print("Magnetic X-Value of Sensor 3:\t");Serial.print(valX3);Serial.println(" mT");
    Serial.print("Magnetic Y-Value of Sensor 3:\t");Serial.print(valY3);Serial.println(" mT");
    Serial.print("Magnetic Z-Value of Sensor 3:\t");Serial.print(valZ3);Serial.println(" mT");
    Serial.println("========================================\n\n");

    delay(2000);
}
