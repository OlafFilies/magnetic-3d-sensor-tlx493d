#ifndef TEST_INCLUDES_H
#define TEST_INCLUDES_H


// std includes
#include <stdbool.h>

// project c includes
// common to all sensors
#include "Logger.h"

#include "tlx493d_types.h"
#include "tlx493d_common_defines.h"
#include "tlx493d_common.h"

// common to same generation of sensors
// #include "tlx493d_gen_1_common_defines.h"
// #include "tlx493d_gen_1_common.h"

#include "tlx493d_gen_2_common_defines.h"
#include "tlx493d_gen_2_common.h"

#include "tlx493d_gen_3_common_defines.h"
#include "tlx493d_gen_3_common.h"

// sensor specicifc includes
#include "TLx493D_A1B6_defines.h"
#include "TLx493D_A1B6.h"

#include "TLx493D_A2B6_defines.h"
#include "TLx493D_A2B6.h"

#include "TLx493D_P2B6_defines.h"
#include "TLx493D_P2B6.h"

#include "TLx493D_W2B6_defines.h"
#include "TLx493D_W2B6.h"

#include "TLx493D_W2BW_defines.h"
#include "TLx493D_W2BW.h"

#include "TLx493D_P3B6_defines.h"
#include "TLx493D_P3B6.h"

#include "TLx493D_P3I8_defines.h"
#include "TLx493D_P3I8.h"


// Unity c includes
#include "unity.h"
#include "unity_fixture.h"

// IFX addons
#include "unity_ifx.h"


#endif // TEST_INCLUDES_H