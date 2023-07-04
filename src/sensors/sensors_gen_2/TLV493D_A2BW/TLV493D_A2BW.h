/**
 * @file TLV493D_A2BW.h
 * @author Infineon Technologies AG
 * @brief Definiton of the complete sensor functionality
 * @copyright Copyright (c) 2023 Infineon Technologies AG
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef TLV493D_A2BW_H_
#define TLV493D_A2BW_H_

/** std includes*/
#include <stdbool.h>
#include <stdint.h>

/** Common to all sensors */
#include "sensor_types.h"
#include "sensors_config_common.h"
#include "sensors_common.h"

/** Common to the same generation of senors */
#include "sensors_gen_2_config_common.h"
#include "sensors_gen_2_common.h"

/** Sensor specific includes */
#include "TLV493D_A2BW_config.h"

typedef struct TLV493D_A2BW_functions_ts {
    InitFuncPtr                         init;
    DeinitFuncPtr                       deinit;
    GetTemperatureFuncPtr               getTemperature;
    UpdateGetTemperatureFuncPtr         updateGetTemperature;
    GetFieldValuesFuncPtr               getFieldValues;
    UpdateGetFieldValuesFuncPtr         updateGetFieldValues;
    ResetFuncPtr                        reset;
    GetDiagnosisFuncPtr                 getDiagnosis;
    CalculateParityFuncPtr              calculateParity;
    UpdateRegistersFuncPtr              updateRegisterMap;
} TLV493D_A2BW_functions_ts;

bool TLV493D_A2BW_init(Sensor_ts *sensor, SupportedComLibraryInterfaceTypes_te comLibIF);
bool TLV493D_A2BW_deinit(Sensor_ts *sensor);

bool TLV493D_A2BW_getTemperature(Sensor_ts *sensor, float *temp);
bool TLV493D_A2BW_updateGetTemperature(Sensor_ts *sensor, float *temp); // TODO: Do we need this function? Why should the user read out an old value? The register update should be included in the getTemp function

bool TLV493D_A2BW_getFieldValues(Sensor_ts *sensor, float *x, float *y, float *z);
bool TLV493D_A2BW_updateGetFieldValues(Sensor_ts *sensor, float *x, float *y, float *z); // TODO: Same here?

bool TLV493D_A2BW_reset(Sensor_ts *sensor);
bool TLV493D_A2BW_getDiagnosis(Sensor_ts *sensor);
bool TLV493D_A2BW_calculateParity(Sensor_ts *sensor);

bool TLV493D_A2BW_setDefaultConfig(Sensor_ts *sensor);
bool TLV493D_A2BW_updateRegisterMap(Sensor_ts *sensor);

// individual functions
void TLV493D_A2BW_get1ByteModeBuffer(uint8_t *buf, uint8_t *bufLen);
void TLV493D_A2BW_getTemperatureMeasurementsBuffer(uint8_t *regMap, uint8_t *buf, uint8_t *bufLen);

#endif /** TLV493D_A2BW_H */