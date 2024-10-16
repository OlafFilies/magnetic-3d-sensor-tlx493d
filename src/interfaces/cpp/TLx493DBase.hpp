#ifndef TLX493D_BASE_HPP
#define TLX493D_BASE_HPP


// std includes
#include <stdbool.h>
#include <stdint.h>

// project cpp includes
#include "types.hpp"

// project c includes
#include "tlx493d_types.h"
#include "tlx493d.h"


/***
 * Base class.
*/
class TLx493DBase {

    public:

        ~TLx493DBase() {
        }


        bool readRegisters() {
            return ::tlx493d_readRegisters(&sensor);
        }


        bool getRawTemperature(uint16_t *temperature) {
            return ::tlx493d_getRawTemperature(&sensor, temperature);
        }


        bool getRawMagneticField(uint16_t *x, uint16_t *y, uint16_t *z) {
            return ::tlx493d_getRawMagneticField(&sensor, x, y, z);
        }


        bool getRawMagneticFieldAndTemperature(uint16_t *x, uint16_t *y, uint16_t *z, uint16_t *temp) {
            return ::tlx493d_getRawMagneticFieldAndTemperature(&sensor, x, y, z, temp);
        }


        bool getTemperature(double *temperature) {
            return ::tlx493d_getTemperature(&sensor, temperature);
        }


        bool getMagneticField(double *x, double *y, double *z) {
            return ::tlx493d_getMagneticField(&sensor, x, y, z);
        }


        bool getMagneticFieldAndTemperature(double *x, double *y, double *z, double *temp) {
            return ::tlx493d_getMagneticFieldAndTemperature(&sensor, x, y, z, temp);
        }


        // functions related to the "Config" register
        bool setMeasurement(TLx493D_MeasurementType_t meas) {
            return ::tlx493d_setMeasurement(&sensor, meas);
        }


        bool setTrigger(TLx493D_TriggerType_t bits) {
            return ::tlx493d_setTrigger(&sensor, bits);
        }

        
        bool setSensitivity(TLx493D_SensitivityType_t range) {
            return ::tlx493d_setSensitivity(&sensor, range);
        }


        // functions related to the "Mod1" and "Mod2" registers
        bool setDefaultConfig() {
            return ::tlx493d_setDefaultConfig(&sensor);
        }

        
        bool setIICAddress(TLx493D_IICAddressType_t addr) {
            return ::tlx493d_setIICAddress(&sensor, addr);
        }


        bool enableCollisionAvoidance() {
            return ::tlx493d_enableCollisionAvoidance(&sensor);
        }


        bool disableCollisionAvoidance() {
            return ::tlx493d_disableCollisionAvoidance(&sensor);
        }


        bool enableInterrupt() {
            return ::tlx493d_enableInterrupt(&sensor);
        }


        bool disableInterrupt() {
            return ::tlx493d_disableInterrupt(&sensor);
        }


        bool setPowerMode(TLx493D_PowerModeType_t mode) {
            return ::tlx493d_setPowerMode(&sensor, mode);
        }

        
        bool setUpdateRate(TLx493D_UpdateRateType_t bit) {
            return ::tlx493d_setUpdateRate(&sensor, bit);
        }


        // functions related to the "Diag" register
        bool hasValidData() {
            return ::tlx493d_hasValidData(&sensor);
        }

        bool isFunctional() {
            return ::tlx493d_isFunctional(&sensor);
        }


        // functions available only to a subset of sensors with wake-up functionality
        bool hasWakeUp() {
            return ::tlx493d_hasWakeUp(&sensor);
        }


        bool isWakeUpEnabled() {
            return ::tlx493d_isWakeUpEnabled(&sensor);
        }


        bool enableWakeUpMode() {
            return ::tlx493d_enableWakeUpMode(&sensor);
        }


        bool disableWakeUpMode() {
            return ::tlx493d_disableWakeUpMode(&sensor);
        }


        bool setWakeUpThresholdsAsInteger(int16_t xl_th, int16_t xh_th, int16_t yl_th, int16_t yh_th, int16_t zl_th, int16_t zh_th) {
            return ::tlx493d_setWakeUpThresholdsAsInteger(&sensor, xl_th, xh_th, yl_th, yh_th, zl_th, zh_th);
        }


        bool setWakeUpThresholds(double xl_th, double xh_th, double yl_th, double yh_th, double zl_th, double zh_th) {
            return ::tlx493d_setWakeUpThresholds(&sensor, xl_th, xh_th, yl_th, yh_th, zl_th, zh_th);
        }


        bool softwareReset() {
            return ::tlx493d_softwareReset(&sensor);
        }


        // utilities
        const char *getTypeAsString() {
            return ::tlx493d_getTypeAsString(&sensor);
        }

        
        void calculateRawMagneticFieldAtTemperature(int16_t rawTemp, TLx493D_SensitivityType_t sens, double mT, int16_t *rawMF) {
            return ::tlx493d_calculateRawMagneticFieldAtTemperature(&sensor, rawTemp, sens, mT, rawMF);
        }


        // Attribute getters for TLx493D object
        TLx493D_t *getSensor() {
            return &sensor;
        }


        TLx493D_SupportedSensorType_t getSensorType() {
            return sensor.sensorType;
        }


        TLx493D_SupportedComLibraryInterfaceType_t getComLibIFType() {
            return sensor.comIFType;
        }


        uint8_t *getRegisterMap() {
            return sensor.regMap;
        }


        uint8_t getRegisterMapSize() {
            return sensor.regMapSize;
        }


        uint8_t getI2CAddress() {
            return sensor.comInterface.comLibParams.iic_params.address;
        }


    protected:

        TLx493DBase() {
        }

        TLx493D_t  sensor;
};


/***
 * Generic template class.
*/
template<typename BoardSupportClass, typename BusWrapper, TLx493D_SupportedSensorType_t sensorType>
    class TLx493D : public TLx493DBase {

    private:

        typedef typename BusWrapper::BusType  BusType;
        

        TLx493D();
        TLx493D(BusType &c);
};


#endif // TLX493D_BASE_HPP
