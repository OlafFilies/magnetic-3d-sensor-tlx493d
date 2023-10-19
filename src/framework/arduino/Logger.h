#ifndef LOGGER_HPP
#define LOGGER_HPP


// template<typename T> void logmsg(char *prefix, T p) {
//     Serial.print(prefix);
//     Serial.println(p);
// }

#ifdef __cplusplus

extern "C" {

#endif

void printRegMap(uint8_t *rm, uint8_t rmSize);

void logmsg(const char *prefix, const char *s);
void logmsgui(const char *prefix, uint8_t ui);

void info(const char *s);
void infoui(uint8_t ui);

void warn(const char *s);
void warnui(uint8_t ui);

void error(const char *s);
void errorui(uint8_t ui);

void flush();


#ifdef __cplusplus

}

#endif


#endif // LOGGER_HPP
