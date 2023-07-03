// std includes
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
#include "sensors_gen_1_config_common.h"
#include "sensors_gen_1_common.h"

// sensor specicifc includes
#include "TLE493D_A1B6_config.h"
#include "TLE493D_A1B6.h"

// structures for holding communication params
extern struct ComLibraryFunctions_ts comLibIF_i2c;
extern void setI2CParameters(ComLibraryParameters_ts *params); //TODO: assign GEN1 addr 
extern void frameworkDelayMicroseconds(uint32_t us);

/*
  Listing of all register names for this sensor.
  Used to index "TLE493D_A2B6_regDef" defined below, so index values must match !
*/
typedef enum {
               BX_MSB = 0,
               BY_MSB,
               BZ_MSB,
               TEMP_MSB,
               BX_LSB,
               BY_LSB,
               TEMP_LSB,
               ID,
               BZ_LSB,
               P,
               FF,
               CF,
               T,
               PD3,
               PD0,
               FRM,
               DT,
               AM,
               TRIG,
               X2,
               TL_mag,
               CP,
               FP,
               IICaddr,
               PR,
               CA,
               INT,
               MODE,
               PRD,
               Type,
               HWV } TLE493D_A1B6_registerNames_te;


