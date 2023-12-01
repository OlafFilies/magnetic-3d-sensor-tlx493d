#ifndef TLX493D_LOGGER_H
#define TLX493D_LOGGER_H


// std includes
#include <stdarg.h>


#ifdef __cplusplus

extern "C" {

#endif


typedef struct TLx493D_t  TLx493D_t;


void printRegisters(TLx493D_t *sensor);
void printDouble(double d);

void logMessage(const char *prefix, const char *format, va_list vaList);

void print(const char *format, ...);
void info(const char *format, ...);
void warn(const char *format, ...);
void error(const char *format, ...);

void flush();


#ifdef __cplusplus

}

#endif


#endif // TLX493D_LOGGER_H
