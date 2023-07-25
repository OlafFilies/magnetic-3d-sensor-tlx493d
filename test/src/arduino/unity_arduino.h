

#define UNITY_OUTPUT_CHAR_HEADER_DECLARATION   putCharacter(char c)
#define UNITY_OUTPUT_FLUSH_HEADER_DECLARATION  flushCharacter()

#define UNITY_OUTPUT_CHAR(a)  putCharacter(a)
#define UNITY_OUTPUT_FLUSH()  flushCharacter()


// Not properly handled by Arduion IDE 1.8
// #define UNITY_OUTPUT_COLOR  (1)


#include "unity.h"
