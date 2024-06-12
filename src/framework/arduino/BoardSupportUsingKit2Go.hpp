#ifndef TLX493D_BOARD_SUPPORT_USING_KIT2GO_HPP
#define TLX493D_BOARD_SUPPORT_USING_KIT2GO_HPP


/** std includes. */
#include <cstdbool>

/** Arduino includes. */
#include <Arduino.h>

/** project cpp includes. */
#include "Kit2GoBoardSupport.hpp"

/** project c includes. */
#include "tlx493d_types.h"

/**
 * @brief The `initBoardSupport` function initializes the board support class. It can be used to define power, address and select pins
 * and makes the handling with Kit2GOs easier for the user.
 * 
 * @param[in,out] sensor A pointer to a TLx493D_t struct, which represents the TLx493D sensor.
 * @param[in,out] bsc A pointer to a board support class instance.
 * 
 * @return The function `initBoardSupport` returns a bool value to indicate if the execution was successful.
 * @retval 0 Error.
 * @retval 1 Successful.
 */
namespace ifx {
    namespace tlx493d {
        bool initBoardSupport(TLx493D_t *sensor, Kit2GoBoardSupport &bsc);
    }
}


#endif // TLX493D_BOARD_SUPPORT_USING_KIT2GO_HPP
