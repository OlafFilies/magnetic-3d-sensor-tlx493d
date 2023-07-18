/**
 * @file TLV493D_W2B6.h
 * @author Infineon Technologies AG
 * @brief Definiton of the complete sensor functionality
 * @copyright Copyright (c) 2023 Infineon Technologies AG
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef TLE493D_W2B6_H
#define TLE493D_W2B6_H

/** std includes*/
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

// project c includes
// common to all sensors
#include "sensor_types.h"
#include "sensors_config_common.h"
#include "sensors_common.h"

// common to same generation of sensors
#include "sensors_gen_2_config_common.h"
#include "sensors_gen_2_common.h"

// sensor specicifc includes
#include "TLE493D_W2B6_config.h"

/**
 * @brief Initializes the XENSIV™ TLE493D-W2B6 magnetic 3D sensor
 * It initializes the sensor structure and checks if the right communication protocol is provided
 *
 * @param[in] sensor Pointer to the XENSIV™ TLV493D-W2B6 magnetic 3D sensor structure
 * @param[in] comLibIF Communication interface type of the sensor
 * @return true - If successful
 * @return false - If unsuccessful
 */
bool TLE493D_W2B6_init(Sensor_ts *sensor, SupportedComLibraryInterfaceTypes_te comLibIF);
bool TLE493D_W2B6_setDefaultConfig(Sensor_ts *sensor);

#endif // TLE493D_W2B6_H
