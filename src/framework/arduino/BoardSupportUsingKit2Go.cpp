// std includes
#include <cstddef>
#include <cstdlib>

// project c includes
// common to all sensors
#include "tlx493d_types.h"

// common to same generation of sensors

// sensor specific includes

// project cpp includes
#include "types.hpp"
#include "BoardSupportUsingKit2Go.hpp"


namespace ifx {
    namespace tlx493d {
        bool initBoardSupport(TLx493D_t *sensor, Kit2GoBoardSupport &bsc) {
            sensor->boardSupportInterface.boardSupportObj.k2go_obj       = (TLx493D_Kit2GoBoardSupportObject_t *) malloc(sizeof(TLx493D_Kit2GoBoardSupportObject_t));
            sensor->boardSupportInterface.boardSupportObj.k2go_obj->k2go = &bsc;
            return true;
        }
    }
}