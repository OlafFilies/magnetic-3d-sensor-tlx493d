#ifndef TLX493D_LOGGER_H
#define TLX493D_LOGGER_H


#ifdef __cplusplus


/** std includes. */
#ifdef __AVR__

    #include <stdarg.h>

#else

    #include <cstdarg>

#endif


extern "C" {

#else

/** std includes. */
#include <stdarg.h>

/** project c includes. */
#include "tlx493d_types.h"

#endif


/**
 * @brief The function `logPrintRegisters` prints out all the internal registers of the
 * passed sensor object.
 * 
 * @param[in] sensor A pointer to a TLx493D_t structure, which represents the TLx493D sensor.
 * @param[in] headLine A string with register names to be printed above the registers for easier reading.
*/
void logPrintRegisters(const TLx493D_t *sensor, const char *headLine);

/**
 * @brief The function `logPrintDouble` prints out a value in the double format. 
 * 
 * @param[in] d Value which will be printed to the serial monitor. 
 */
void logPrintDouble(double d);

/**
 * @brief The function `logPrint` is used to print out a formatted string - without prefix - to the serial output.
 * 
 * @param[in] format Actual string, which should be printed to the serial output. 
 */
void logPrint(const char *format, ...);

/**
 * @brief The function `logPrintln` is used to print out a formatted string - without prefix - to the serial output plus a trailing linefeed.
 * 
 * @param[in] format Actual string, which should be printed to the serial output. 
*/
void logPrintln(const char *format, ...);

/**
 * @brief The function `logInfo` is used to print out an info message to the user.
 * It uses the prefix "INFO : " to directly indicate the origin of the message.
 * It also allows to pass a variable number of arguments to the function (...).
 * 
 * @param[in] format Actual string, which should be printed to the serial output. 
 */
void logInfo(const char *format, ...);

/**
 * @brief The function `logWarn` is used to print out a warning message to the user.
 * It uses the prefix "WARNING : " to directly indicate the origin of the message. 
 * It also allows to pass a variable number of arguments to the function (...).
 * 
 * @param[in] format Actual string, which should be printed to the serial output. 
 */
void logWarn(const char *format, ...);

/**
 * @brief The function `logError` is used to print out an error message to the user.
 * It uses the prefix "ERROR : " to directly indicate the origin of the message. 
 * It also allows to pass a variable number of arguments to the function (...).
 * 
 * @param[in] format Actual string, which should be printed to the serial output.   
 */
void logError(const char *format, ...);

/**
 * @brief The function `flush` is used to send a new line character to ther serial output.
 * 
 */
void logFlush(void);


#ifdef __cplusplus

}

#endif


#endif /** TLX493D_LOGGER_H */
