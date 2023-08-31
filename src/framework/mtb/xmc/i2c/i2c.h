/*
*****************************************************************************
* Copyright (C) 2019 Infineon Technologies AG. All rights reserved.
*
* Infineon Technologies AG (INFINEON) is supplying this file for use
* exclusively with Infineon's products. This file can be freely
* distributed within development tools and software supporting such microcontroller
* products.
*
* THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
* OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
* INFINEON SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR DIRECT, INDIRECT, INCIDENTAL,
* ASPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
*
******************************************************************************
*/

/*! \file i2c.h
 * \brief I2C Driver for XMC1100 USIC Module */

#ifndef SRC_PERIPHERALS_I2C_I2C_H_
#define SRC_PERIPHERALS_I2C_I2C_H_

#include <stdint.h>
void wait(uint32_t time_us);

//! Status values returned by the I2C read/write commands
typedef enum {
	I2C_STATUS_OK		= 0,
	I2C_STATUS_TIMEOUT	= 1,
	// Deprecated
	I2C_STATUS_BUSY		= 2,
	I2C_STATUS_NACK		= 3,
} I2C_status_t;

//! Initialize the I2C peripheral
void I2C_init(void);

//! Enable the I2C channel
void I2C_enable(void);

//! Disable the I2C channel
void I2C_disable(void);

//! Connect SDA and SCL pins to the I2C peripheral
void I2C_mode_normal(void);

/*! \brief Switch I2C pins to GPIO mode
 * Disconnect SDA and SCL pins from the I2C peripheral.
 * Configure them as GPIO inputs (in order to avoid unintended
 * input signals to the I2C peripheral that will cause it to freeze) */
void I2C_mode_gpio_input(void);

/*! \brief Recovery procedure according to XMC1100 Errata Sheet */
void I2C_peripheral_recover(void) __attribute__((deprecated));


/*! \brief Read a number of bytes from an I2C device to a <b>uint8_t</b> array
 *
 *  Read <b>count</b> bytes from the device with the given I2C address
 * to the data array
 * \param addr I2C device address.
 * \param data Address where the data should be stored.
 * \param count Number of bytes to be read from the device to the <b>data</b> array */
int32_t I2C_read(uint8_t addr, uint8_t *data, uint8_t count);


/*! \brief Write a number of <b>uint8_t</b> bytes to an I2C device.
 *
 *  Read <b>count</b> bytes from the device with the given I2C address
 * to the data array
 * \param addr I2C device address.
 * \param data Address where the data should be stored.
 * \param count Number of bytes to be read from the device to the <b>data</b> array */
int32_t I2C_write(uint8_t addr, const uint8_t* data, uint8_t count);


/*! \brief Wait for an I2C transmission to end.
 *
 * Blocks the program execution until the last I2C operation is completed. */
void I2C_wait_transmit(void);


//! Write FF (the recover value) to the I2C Bus
void I2C_write_recover(void);

//! Write 00 (the reset value) to the I2C Bus
void I2C_write_reset(void);



#endif /* SRC_PERIPHERALS_I2C_I2C_H_ */
