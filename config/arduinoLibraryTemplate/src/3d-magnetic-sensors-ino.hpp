/** 
 * @file        3d-magnetic-ino.hpp
 * @brief       XENSIV™ 3D magnetic Arduino API
 * @copyright   Copyright (c) 2023-2023 Infineon Technologies AG
 *              
 * SPDX-License-Identifier: MIT
 */

#ifndef IFX_3D_MAGNETIC_SENSORS_INO_HPP_
#define IFX_3D_MAGNETIC_SENSORS_INO_HPP_

#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>
// #include "3d-magnetic-platf-ino.hpp"
// #include "xensiv_3d_magnetic.h"

/**
 * @addtogroup 3dmagneticinoapi
 * @{
 */

// typedef int32_t Error_t;
// typedef xensiv_3d_magnetic_status_t Diag_t;
// typedef xensiv_3d_magnetic_boc_cfg_t ABOC_t;

class IFX3DMagneticSensorsIno
{
    public:

        static constexpr uint8_t       unusedPin = 0xFFU; /**< Unused pin */        

        IFX3DMagneticSensorsIno (TwoWire * wire = &Wire, uint8_t intPin = unusedPin);
        IFX3DMagneticSensorsIno (HardwareSerial * serial, uint8_t intPin = unusedPin);
       ~IFX3DMagneticSensorsIno();

    //     Error_t begin           ();
    //     Error_t end             ();
    //     Error_t startMeasure    (int16_t  periodInSec = 0, int16_t alarmTh = 0, void (*cback) (void *) = nullptr, bool earlyNotification = false);
    //     Error_t stopMeasure     ();
    //     Error_t getCO2          (int16_t & CO2PPM);
    //     Error_t getDiagnosis    (Diag_t & diagnosis);
    //     Error_t setABOC         (ABOC_t aboc, int16_t abocRef);
    //     Error_t setPressRef     (uint16_t pressRef);
    //     Error_t performForcedCompensation(uint16_t co2Ref);
    //     Error_t clearForcedCompensation  ();
    //     Error_t reset           ();
    //     Error_t getDeviceID     (uint8_t & prodID, uint8_t & revID);

    //     Error_t getRegister     (uint8_t regAddr, uint8_t * data, uint8_t len);
    //     Error_t setRegister     (uint8_t regAddr, const uint8_t * data, uint8_t len);

    private:

        TwoWire         * i2c;          /**< I2C interface*/
        HardwareSerial  * uart;         /**< UART interface */   
        uint8_t           intPin;       /**< Interrupt pin */

        static constexpr uint16_t baudrateBps = 9600;      /**< UART baud rate in bps */
        static constexpr uint32_t freqHz      = 100000;    /**< I2C frequency in Hz*/

    //     xensiv_3D_MAGNETIC_t   dev;          /**< XENSIV™ PAS CO2 corelib object */
};

/** @} */

#endif /** IFX_3D_MAGNETIC_SENSORS_INO_HPP_ **/