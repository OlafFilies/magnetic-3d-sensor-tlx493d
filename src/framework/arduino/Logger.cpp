// std includes
#include <cstdarg>
#include <cstdio>
#include <cstring>

// Arduino includes
#include <Arduino.h>


// project cpp includes

// project c includes
#include "Logger.h"
#include "tlx493d_types.h"


namespace ifx {
    namespace tlx493d {
        const uint16_t LOGGER_BUFFER_SIZE = 512U;

        static void logMessage(const char *prefix, const char *format, va_list vaList) {
            char buffer[LOGGER_BUFFER_SIZE];

            size_t prefixSize = strlen(prefix);
            (void) memcpy(buffer, prefix, prefixSize);
            int ret = vsprintf(buffer + prefixSize, format, vaList);

            if( (ret + prefixSize) > LOGGER_BUFFER_SIZE ) {
                Serial.print("FATAL : Buffer overflow (> ");
                Serial.print(LOGGER_BUFFER_SIZE);
                Serial.println(" characters) because message too long !\n");
            }

            Serial.print(buffer);
        }
    }
}


extern "C" {
    void logPrintRegisters(const TLx493D_t *sensor, const char *headLine = NULL) {
        Serial.println();
        Serial.println(headLine);
        // Serial.println();

        for(uint8_t i = 0; i < sensor->regMapSize; ++i) {
            logPrint("    0x%02X", sensor->regMap[i]);
            // Serial.print("  0x");
            // Serial.print(sensor->regMap[i], HEX);
        }

        Serial.println();
    }


    void logPrintDouble(double d) {
        Serial.print(d);
    }


    void logPrint(const char *format, ...) {
        va_list ap;
        va_start(ap, format);
        ifx::tlx493d::logMessage("", format, ap);
        va_end(ap);
    }


    void logPrintln(const char *format, ...) {
        Serial.println();
        va_list ap;
        va_start(ap, format);
        ifx::tlx493d::logMessage("", format, ap);
        va_end(ap);
        Serial.println();
    }


    void logInfo(const char *format, ...) {
        Serial.println();
        va_list ap;
        va_start(ap, format);
        ifx::tlx493d::logMessage("INFO : ", format, ap);
        va_end(ap);
        Serial.println();
    }


    void logWarn(const char *format, ...) {
        Serial.println();
        va_list ap;
        va_start(ap, format);
        ifx::tlx493d::logMessage("WARNING : ", format, ap);
        va_end(ap);
        Serial.println();
    }


    void logError(const char *format, ...) {
        Serial.println();
        va_list ap;
        va_start(ap, format);
        ifx::tlx493d::logMessage("ERROR : ", format, ap);
        va_end(ap);
        Serial.println();
    }


    void logFlush() {
        // USE WITH CAUTION ! DEVICE MAY HANGUP !
        Serial.flush();
        // Serial.println();
    }
}
