// std includes
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

// Arduino includes
// #include <SPI.h>
#include <Arduino.h>


#include "Logger.h"


#define BUFFER_SIZE  512


extern "C" {
    void printRegisters(uint8_t *rm, uint8_t rmSize) {
        Serial.print("regMap :"); 

        for(uint8_t i = 0; i < rmSize; ++i) {
            Serial.print("  0x");
            Serial.print(rm[i], HEX);
        }

        Serial.println();
    }


    // void logMessage(const char *prefix, const char *format, ...) {
    //     char buffer[BUFFER_SIZE];
    //     size_t prefixSize = strlen(prefix);
    //     memcpy(buffer, prefix, prefixSize);

    //     va_list ap;
    //     va_start(ap, format);      
    //     int ret = vsprintf(buffer + prefixSize, format, ap);
    //     va_end(ap);

    //     if( (ret + prefixSize) > BUFFER_SIZE ) {
    //         Serial.println("FATAL : Buffer overflow (>256 characters) because message too long !");
    //     }

    //     // Serial.print(prefix);
    //     Serial.println(buffer);
    // }


    // void logMessage(const char *prefix, const char *format, va_list vaList) {
    //     char buffer[BUFFER_SIZE];

    //     double f = va_arg(vaList, double);
    //     Serial.print("f : ");
    //     Serial.println(f);

    //     size_t prefixSize = strlen(prefix);
    //     memcpy(buffer, prefix, prefixSize);
    //     int ret = vsprintf(buffer + prefixSize, format, vaList);
        
    //     Serial.print("ret : ");
    //     Serial.println(ret);

    //     if( (ret + prefixSize) > BUFFER_SIZE ) {
    //         Serial.println("FATAL : Buffer overflow (> 512 characters) because message too long !\n");
    //     }

    //     Serial.println(buffer);
    // }


    void logMessage(const char *prefix, const char *format, va_list vaList) {
    // void logMessage(const char *prefix, const char *format, va_list *vaList) {
        char buffer[BUFFER_SIZE];

        size_t prefixSize = strlen(prefix);
        memcpy(buffer, prefix, prefixSize);
        int ret = vsprintf(buffer + prefixSize, format, vaList);

        if( (ret + prefixSize) > BUFFER_SIZE ) {
            Serial.println("FATAL : Buffer overflow (> 512 characters) because message too long !\n");
        }

        Serial.println(buffer);
        // DO NOT USE FLUSH HERE ! DEVICE WILL HANGUP !
        // Serial.flush();
    }


    void info(const char *format, ...) {
        va_list ap;
        va_start(ap, format);
        // Serial.println(formatMsg("INFO : ", format, ap));
        logMessage("INFO : ", format, ap);
        va_end(ap);
    }


    // void warn(const char *format, ...) {
    //     va_list ap;
    //     va_start(ap, format);
    //     logMessage("WARNING : ", format, &ap);
    //     va_end(ap);
    // }


    // void error(const char *format, ...) {
    //     va_list ap;
    //     va_start(ap, format);
    //     logMessage("ERROR : ", format, &ap);
    //     va_end(ap);
    // }


    // void logMsg(const char *prefix, const char *s) {
    //     Serial.print(prefix);
    //     Serial.println(s);
    // }


    // void info(const char *s) {
    //     logMsg("INFO : ", s);
    //     // Serial.flush();
    // }


    // void warn(const char *s) {
    //     logMsg("WARNING : ", s);
    //     Serial.flush();
    // }


    // void error(const char *s) {
    //     logMsg("ERROR : ", s);
    //     Serial.flush();
    // }


    // USE WITH CAUTION ! DEVICE MAY HANGUP !
    void flush() {
        Serial.flush();
    }
}
