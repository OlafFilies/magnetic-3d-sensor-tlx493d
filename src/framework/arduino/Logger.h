#ifndef TLX493D_LOGGER_H
#define TLX493D_LOGGER_H


// std includes
#include <stdarg.h>


#ifdef __cplusplus

extern "C" {

#endif


void printRegisters(uint8_t *rm, uint8_t rmSize);
void print(double d);

void logMessage(const char *prefix, const char *format, va_list vaList);

void info(const char *format, ...);
void warn(const char *format, ...);
void error(const char *format, ...);

void flush();


#ifdef __cplusplus

}

#endif


#endif // TLX493D_LOGGER_H
