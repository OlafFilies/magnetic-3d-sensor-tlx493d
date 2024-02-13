// std includes
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

// Arduino includes
#include <Arduino.h>


// project cpp includes

// project c includes
#include "Logger.h"
#include "tlx493d_types.h"


namespace ifx {
    namespace tlx493d {
        const uint16_t LOGGER_BUFFER_SIZE = 512;

        static void logMessage(const char *prefix, const char *format, va_list vaList) {
            char buffer[LOGGER_BUFFER_SIZE];

            size_t prefixSize = strlen(prefix);
            memcpy(buffer, prefix, prefixSize);
            int ret = vsprintf(buffer + prefixSize, format, vaList);

            if( (ret + prefixSize) > LOGGER_BUFFER_SIZE ) {
                Serial.print("FATAL : Buffer overflow (> ");
                Serial.print(LOGGER_BUFFER_SIZE);
                Serial.println(" characters) because message too long !\n");
            }

            Serial.println(buffer);
        }
    }
}


extern "C" {
    void printRegisters(TLx493D_t *sensor) {
        Serial.print("\nregMap :"); 

        for(uint8_t i = 0; i < sensor->regMapSize; ++i) {
            Serial.print("  0x");
            Serial.print(sensor->regMap[i], HEX);
        }

        Serial.println();
    }


    void printDouble(double d) {
        Serial.print(d);
    }


    void print(const char *format, ...) {
        va_list ap;
        va_start(ap, format);
        ifx::tlx493d::logMessage("", format, ap);
        va_end(ap);
    }


    void info(const char *format, ...) {
        va_list ap;
        va_start(ap, format);
        ifx::tlx493d::logMessage("INFO : ", format, ap);
        va_end(ap);
    }


    void warn(const char *format, ...) {
        va_list ap;
        va_start(ap, format);
        ifx::tlx493d::logMessage("WARNING : ", format, ap);
        va_end(ap);
    }


    void error(const char *format, ...) {
        va_list ap;
        va_start(ap, format);
        ifx::tlx493d::logMessage("ERROR : ", format, ap);
        va_end(ap);
    }


    void flush() {
        // USE WITH CAUTION ! DEVICE MAY HANGUP !
        // Serial.flush();
        Serial.println();

    }
}
