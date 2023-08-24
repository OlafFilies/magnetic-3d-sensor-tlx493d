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

/*! \file conf_i2c.h
 * \brief Configuration file for the I2C driver.
 *
 * Configure the SDA and SCL pins and communication speed.
 * */

#ifndef SRC_PERIPHERALS_I2C_CONF_I2C_H_
#define SRC_PERIPHERALS_I2C_CONF_I2C_H_

#include <xmc_i2c.h>
#include <xmc_gpio.h>



//! Pin used as SDA
#define CONF_I2C_SDA_PIN 	P2_10

//! Pin used as SCL
#define CONF_I2C_SCL_PIN 	P2_11

//! USIC Channel used for I2C
#define CONF_I2C_CH 		XMC_I2C0_CH1


// SDA as INPUT PULL UP
static const XMC_GPIO_CONFIG_t CONF_I2C_SDA_PIN_INPUT = {
  .mode = XMC_GPIO_MODE_INPUT_PULL_UP,
  .input_hysteresis = XMC_GPIO_INPUT_HYSTERESIS_STANDARD,
};

// SDA as INPUT TRISTATE (disabled)
static const XMC_GPIO_CONFIG_t CONF_I2C_SDA_PIN_DISABLED = {
  .mode = XMC_GPIO_MODE_INPUT_TRISTATE,
  .input_hysteresis = XMC_GPIO_INPUT_HYSTERESIS_STANDARD,
};

// SDA as OPEN-DRAIN OUTPUT I2C
static const XMC_GPIO_CONFIG_t CONF_I2C_SDA_PIN_OUTPUT = {
  .mode = XMC_GPIO_MODE_OUTPUT_OPEN_DRAIN_ALT7 ,
  .output_level = XMC_GPIO_OUTPUT_LEVEL_HIGH,
};


// SCL as /INT input
static const XMC_GPIO_CONFIG_t CONF_I2C_SCL_PIN_INPUT =
{
  .mode = XMC_GPIO_MODE_INPUT_SAMPLING,
  .input_hysteresis = XMC_GPIO_INPUT_HYSTERESIS_STANDARD
};

// SCL as INPUT TRISTATE (disabled)
static const XMC_GPIO_CONFIG_t CONF_I2C_SCL_PIN_DISABLED =
{
  .mode = XMC_GPIO_MODE_INPUT_TRISTATE,
  .input_hysteresis = XMC_GPIO_INPUT_HYSTERESIS_STANDARD
};

// SCL as OPEN DRAIN OUTPUT
static const XMC_GPIO_CONFIG_t CONF_I2C_SCL_PIN_OUTPUT = {
  .mode = XMC_GPIO_MODE_OUTPUT_OPEN_DRAIN_ALT6,
  .output_level = XMC_GPIO_OUTPUT_LEVEL_HIGH,
};




/* USIC configuration */
static const XMC_I2C_CH_CONFIG_t I2C_master_conf = {
	.baudrate = 100000U,
	// .baudrate = 400000U,
};

#endif /* SRC_PERIPHERALS_I2C_CONF_I2C_H_ */
