#ifndef TLX493D_LOGGER_H
#define TLX493D_LOGGER_H


// std includes
#include <stdarg.h>


#ifdef __cplusplus

// template<typename T> void logMsg(const char *prefix, T t) {
//     Serial.print(prefix);
//     Serial.println(t);
// }

// template<typename T, typename T1> void logMsg(const char *prefix, T t, T1 t1) {
//     Serial.print(prefix);
//     Serial.print(t);
//     Serial.print(" ");
//     Serial.println(t1);
// }

// template<typename T, typename T1, typename T2> void logMsg(const char *prefix, T t, T1 t1, T2 t2) {
//     Serial.print(prefix);
//     Serial.print(t);
//     Serial.print(" ");
//     Serial.print(t1);
//     Serial.print(" ");
//     Serial.println(t2);
// }

// template<typename T, typename T1, typename T2, typename T3> void logMsg(const char *prefix, T t, T1 t1, T2 t2, T3 t3) {
//     Serial.print(prefix);
//     Serial.print(t);
//     Serial.print(" ");
//     Serial.print(t1);
//     Serial.print(" ");
//     Serial.print(t2);
//     Serial.print(" ");
//     Serial.println(t3);
// }

// template<typename T, typename T1, typename T2, typename T3> void info(const char *prefix, T t) {
//     logMsg(prefix, t);
// }

// template<typename T, typename T1, typename T2, typename T3> void info(const char *prefix, T t, T1 t1) {
//     logMsg(prefix, t, t1);
// }

// template<typename T, typename T1, typename T2, typename T3> void info(const char *prefix, T t, T1 t1, T2 t2) {
//     logMsg(prefix, t, t1, t2);
// }

// template<typename T, typename T1, typename T2, typename T3> void info(const char *prefix, T t, T1 t1, T2 t2, T3 t3) {
//     logMsg(prefix, t, t1, t2, t3);
// }

extern "C" {

#endif


void printRegisters(uint8_t *rm, uint8_t rmSize);

// // void logMessage(const char *prefix, const char *format, ...);
// void logMessage(const char *prefix, const char *format, va_list *vaList);
void logMessage(const char *prefix, const char *format, va_list vaList);

void info(const char *format, ...);
void warn(const char *format, ...);
void error(const char *format, ...);

// void logMsg(const char *prefix, const char *s);
// void info(const char *s);

void flush();


#ifdef __cplusplus

}

#endif


#endif // TLX493D_LOGGER_H
