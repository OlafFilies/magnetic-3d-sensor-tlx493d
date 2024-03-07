#ifndef TLX493D_LOGGER_H
#define TLX493D_LOGGER_H


// std includes
// #include <stdarg.h>


#ifdef __cplusplus

#include <cstdarg>

using TLx493D_t = struct TLx493D_t;

extern "C" {

#else

#include <stdarg.h>

typedef struct TLx493D_t  TLx493D_t;

#endif


// typedef struct TLx493D_t  TLx493D_t;
// using TLx493D_t = struct TLx493D_t;


/**
 * @brief The function `tlx493d_logPrintRegisters` prints out all the internal registers of the
 * passed sensor object.
 * 
 * @param[in,out] sensor A pointer to a TLx493D_t structure, which represents the TLx493D sensor.
 */
void tlx493d_logPrintRegisters(const TLx493D_t *sensor, const char *headLine);

/**
 * @brief The function `tlx493d_logPrintDouble` prints out a value in the double format. 
 * 
 * @param[in] d Value which will be printed to the serial monitor. 
 */
void tlx493d_logPrintDouble(double d);

/**
 * @brief The function `tlx493d_logPrint` is used to tlx493d_logPrint out a formatted string - without prefix - to the serial output.
 * 
 * @param[in] format Actual string, which should be printed to the serial output. 
 */
void tlx493d_logPrint(const char *format, ...);

/**
 * @brief The function `tlx493d_logPrintln` is used to tlx493d_logPrint out a formatted string - without prefix - to the serial output plus a trailing linefeed.
 * 
 * @param[in] format Actual string, which should be printed to the serial output. 
 * @param[in] ... Actual string, which should be printed to the serial output. 
*/
void tlx493d_logPrintln(const char *format, ...);

/**
 * @brief The function `tlx493d_logInfo` is used to tlx493d_logPrint out an tlx493d_logInfo message to the user.
 * It uses the prefix "INFO : " to directly indicate the origin of the message.
 * It also allows to pass a variable number of arguments to the function (...).
 * 
 * @param[in] format Actual string, which should be printed to the serial output. 
 */
void tlx493d_logInfo(const char *format, ...);

/**
 * @brief The function `tlx493d_logWarn` is used to tlx493d_logPrint out a warning message to the user.
 * It uses the prefix "WARNING : " to directly indicate the origin of the message. 
 * It also allows to pass a variable number of arguments to the function (...).
 * 
 * @param[in] format Actual string, which should be printed to the serial output. 
 */
void tlx493d_logWarn(const char *format, ...);

/**
 * @brief The function `tlx493d_logError` is used to tlx493d_logPrint out an tlx493d_logError message to the user.
 * It uses the prefix "ERROR : " to directly indicate the origin of the message. 
 * It also allows to pass a variable number of arguments to the function (...).
 * 
 * @param[in] format Actual string, which should be printed to the serial output.   
 */
void tlx493d_logError(const char *format, ...);

/**
 * @brief The function `flush` is used to send a new line character to ther serial output.
 * 
 */
void tlx493d_logFlush(void);


#ifdef __cplusplus

}

#endif


#endif // TLX493D_LOGGER_H
