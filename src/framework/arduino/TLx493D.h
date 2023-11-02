#ifndef TLx493D_H
#define TLx493D_H


// std includes

// Arduino includes
#include <Arduino.h>

// project cpp includes
#include "arduino_defines.h"
#include "cInterface.h"
#include "Logger.h"
#include "S2GoTemplateArduino.hpp"
#include "SPI_Lib.hpp"
#include "TLx493D.hpp"
#include "TwoWire_Lib.hpp"


extern "C" {
    // project c includes
    #include "tlx493d_types.h"

    // #include "tlx493d_gen_2_common.h"
    // #include "tlx493d_gen_3_common.h"
    // #include "tlx493d_common.h"

    #include "TLx493D_A1B6.h"

    #include "TLx493D_A2B6.h"
    #include "TLx493D_P2B6.h"
    #include "TLx493D_W2B6.h"
    #include "TLx493D_W2BW.h"

    #include "TLx493D_P3B6.h"
    #include "TLx493D_P3I8.h"
}

#endif // TLx493D_H
