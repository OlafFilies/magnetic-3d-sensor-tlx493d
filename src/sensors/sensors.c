#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "sensors.h"
#include "TLE493D_A1B6/TLE493D_A1B6.h"
#include "TLV493D_A1B6/TLV493D_A1B6.h"
#include "TLE493D_P2B6/TLE493D_P2B6.h"
#include "TLE493D_W2B6/TLE493D_W2B6.h"


// In case a sensor shall be instantiated bypassing the interface
void *createSensor(sensorTypes_t sensorType) {
    switch(sensorType) {
        case TLE493D_A1B6 : {
                                TLE493D_A1B6_t *sensor = malloc(sizeof(TLE493D_A1B6_t));
                                TLE493D_A1B6_initObject(sensor);
                                return sensor;
                                break;
                            }

        case TLV493D_A1B6 : {
                                TLV493D_A1B6_t *sensor = malloc(sizeof(TLV493D_A1B6_t));
                                TLV493D_A1B6_initObject(sensor);
                                return sensor;
                                break;
                            }

        case TLE493D_P2B6 : {
                                TLE493D_P2B6_t *sensor = malloc(sizeof(TLE493D_P2B6_t));
                                TLE493D_P2B6_initObject(sensor);
                                return sensor;
                                break;
                            }

        case TLE493D_W2B6 : {
                                TLE493D_W2B6_t *sensor = malloc(sizeof(TLE493D_W2B6_t));
                                TLE493D_W2B6_initObject(sensor);
                                return sensor;
                                break;
                            }
    }
}


void printType(sensorTypes_t sensorType) {
    switch(sensorType) {
        case TLE493D_A1B6 : printf("sensorType : TLE493D_A1B6\n");
                            break;

        case TLV493D_A1B6 : printf("sensorType : TLV493D_A1B6\n");
                            break;

        case TLE493D_P2B6 : printf("sensorType : TLE493D_P2B6\n");
                            break;

        case TLE493D_W2B6 : printf("sensorType : TLE493D_W2B6\n");
                            break;
    }
}
