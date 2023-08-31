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

/*! \file conf_interrupts.h
 * \brief ERU Module Configuration */

#ifndef SRC_GENERAL_CONF_ERU_H_
#define SRC_GENERAL_CONF_ERU_H_

#include <xmc_eru.h>

/* ERU config */
static const XMC_ERU_ETL_CONFIG_t CONF_SCL_PIN_INT_ERU =
{
	// Input Pin P2.11
	.input_b = XMC_ERU_ETL_INPUT_B1,
	// P2.11
	.source = XMC_ERU_ETL_SOURCE_B,
	.edge_detection = XMC_ERU_ETL_EDGE_DETECTION_RISING,
	.status_flag_mode = XMC_ERU_ETL_STATUS_FLAG_MODE_HWCTRL,
	.enable_output_trigger = true,
	.output_trigger_channel = XMC_ERU_ETL_OUTPUT_TRIGGER_CHANNEL0
};

/* ERU_OGU config */
static const XMC_ERU_OGU_CONFIG_t CONF_SCL_PIN_OGU =
{
	.service_request = XMC_ERU_OGU_SERVICE_REQUEST_ON_TRIGGER,
	.enable_pattern_detection = false,
};

#endif /* SRC_GENERAL_CONF_ERU_H_ */
