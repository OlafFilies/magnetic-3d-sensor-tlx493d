#ifndef TLX493D_BOARD_SUPPORT_USING_KIT2GO_HPP
#define TLX493D_BOARD_SUPPORT_USING_KIT2GO_HPP


// std includes
#include <cstdbool>

// Arduino includes
#include <Arduino.h>

// project cpp includes
#include "Kit2GoBoardSupport.hpp"

// project c includes
// common to all sensors
#include "tlx493d_types.h"


namespace ifx {
    namespace tlx493d {
        bool initBoardSupport(TLx493D_t *sensor, Kit2GoBoardSupport &bsc);

        // bool tlx493d_initBoardSupport(TLx493D_t *sensor);
    }
}


#endif // TLX493D_BOARD_SUPPORT_USING_KIT2GO_HPP
