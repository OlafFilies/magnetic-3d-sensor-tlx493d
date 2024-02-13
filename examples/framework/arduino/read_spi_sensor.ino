/** Project CPP includes */
#include "TLx493D_inc.hpp"


using namespace ifx::tlx493d;


const uint8_t POWER_PIN       = LED2;
const uint8_t CHIP_SELECT_PIN = 3;


TLx493D_P3I8 dut(SPI);


void setup() {
    delay(3000);
    Serial.begin(115200);

    /** Setting the power up pin of the Kit2GO as well as the chip select pin for the SPI 
     *  For this we're using the functions of the Board Support Class
     */
    dut.setPowerPin(POWER_PIN, OUTPUT, HIGH, LOW, 50, 50);
    dut.setSelectPin(CHIP_SELECT_PIN, OUTPUT, LOW, HIGH, 50, 50);
    dut.begin();

    Serial.print("setup done.\n");
}

/** In the loop we're reading out the temperature value (in °C) as well as the magnetic fields values in X, Y, Z-direction (in mT)
 *  We're also reading out the raw temperature value (in LSB).
 */
void loop() {
    double temp = 0.0;
    double valX = 0, valY = 0, valZ = 0;
    int16_t tempRaw = 0;

    /** Note that you don't have to toggle any chip select signal, this is done in the background within the transfer function */
    Serial.print(true == dut.getTemperature(&temp) ? "getTemperature ok\n" : "getTemperature error\n");

    Serial.print("Temperature is: ");
    Serial.print(temp);
    Serial.println(" °C");

    dut.getRawTemperature(&tempRaw);

    Serial.print("Raw temperature is: ");
    Serial.print(tempRaw);
    Serial.println(" LSB");

    Serial.print(true == dut.getMagneticField(&valX, &valY, &valZ) ? "getMagneticField ok\n" : "getMagneticField error\n");

    Serial.print("Value X is: ");
    Serial.print(valX);
    Serial.println(" mT");
    Serial.print("Value Y is: ");
    Serial.print(valY);
    Serial.println(" mT");
    Serial.print("Value Z is: ");
    Serial.print(valZ);
    Serial.println(" mT");

    printRegisters(dut.getSensor());
    Serial.print("\n");

    delay(1000);
}
