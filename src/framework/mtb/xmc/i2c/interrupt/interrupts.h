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

// #include "pal.h"

// #ifdef USE_INT


/*! \file interrupts.h
 * \brief ERU Module used for Interrupt Handling */

#ifndef SRC_GENERAL_ERU_H_
#define SRC_GENERAL_ERU_H_

#include <stdbool.h>
//#include <XMC1100.h>

/*! \brief Enable falling edge interrupt on P2.11 and P0.0 (TLI support)
 * Using ERU and CCU4 slice CC40 (0)
 * And pull-up input on P0.0
 * Disable mentioned interrupts
 * */
void INT_init_ext_interrupts(void);

/*! \brief Enable external interrupts from P0.0 and P2.11
 * ERU0_0 and CCU40_0
 * */
static inline void INT_int_ext_enable(void)
{
	NVIC_ClearPendingIRQ(CCU40_0_IRQn);
	NVIC_ClearPendingIRQ(ERU0_0_IRQn);
	NVIC_EnableIRQ(CCU40_0_IRQn);
	NVIC_EnableIRQ(ERU0_0_IRQn);
}

/*! \brief Disable external interrupts from P0.0 and P2.11
 * ERU0_0 and CCU40_0
 * */
static inline void INT_int_ext_disable(void)
{
	NVIC_DisableIRQ(CCU40_0_IRQn);
	NVIC_DisableIRQ(ERU0_0_IRQn);
}

/*! \brief Return value of flag showing TLI detected.
 * Requires sensor be kept in Low Power/Fast Mode until a pulse is detected
 * Requires CCU40_0 interrupt enabled
 * */
bool INT_get_TLI_detected(void);

#endif /* SRC_GENERAL_ERU_H_ */

// #endif
