// std includes
#include <stdbool.h>

// Arduino includes
#include <SPI.h>
#include <Arduino.h>


#include "Logger.h"


extern "C" {
    void printRegMap(uint8_t *rm, uint8_t rmSize) {
        Serial.print("regMap :"); 

        for(uint8_t i = 0; i < rmSize; ++i) {
            Serial.print("  0x");
            Serial.print(rm[i], HEX);
        }

        Serial.println();
    }


    void logMsg(const char *prefix, const char *s) {
        Serial.print(prefix);
        Serial.println(s);
    }


    void logMsgui(const char *prefix, uint8_t ui) {
        Serial.print(prefix);
        Serial.print("0x");
        Serial.println(ui, HEX);
    }


    void info(const char *s) {
        logMsg("INFO : ", s);
        Serial.flush();
    }


    void infoui(uint8_t ui) {
        logMsgui("INFO : ", ui);
        Serial.flush();
    }


    void warn(const char *s) {
        logMsg("WARNING : ", s);
        Serial.flush();
    }


    void warnui(uint8_t ui) {
        logMsgui("INFO : ", ui);
        Serial.flush();
    }


    void error(const char *s) {
        logMsg("ERROR : ", s);
        Serial.flush();
    }


    void errorui(uint8_t ui) {
        logMsgui("INFO : ", ui);
        Serial.flush();
    }


    void flush() {
        Serial.flush();
    }
}
