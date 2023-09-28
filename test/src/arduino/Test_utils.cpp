// std includes
#include <stdbool.h>

// Arduino includes
#include <SPI.h>
#include <Arduino.h>


extern "C" {
    #include "Test_utils.h"

    
    // Method used by Unity to output a single character 
    void putCharacter(char c) {
        Serial.print(c);
        Serial.flush();
    }

    // Method used by Unity to flush the output
    void flushCharacter() {
        Serial.flush();
    }
}
