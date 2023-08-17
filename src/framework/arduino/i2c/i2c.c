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

#include <xmc_i2c.h>

#include "conf_i2c.h"
#include "i2c.h"
// #include "src/misc/misc.h"
// #include "src/debug/debug.h"
// #include "src/xmc1100/gpio/gpio.h"
// #include "src/xmc1100/time/time.h"
// #include "src/xmc1100/interrupt/interrupts.h"

#include "interrupts.h"

// reset address
#define TLE493D_AW2B6_I2C_RESET_ADDR	0x00U
// recovery address
#define TLE493D_AW2B6_I2C_RECOV_ADDR	0xFFU


/* Defines for SysTick */
// multiplication factor for 1us resolution
#define _MULTIPLICATION_FACTOR			(32u)
//systick stop value
#define _SYSTICK_CTRL_DISABLE_Msk		(0UL << SysTick_CTRL_ENABLE_Pos)

extern void myDebug(char *msg);


/* Delay for 'time_us' us + t_call + t_cfg
 * t_call ~= 1.2us
 * t_cfg  ~= 1us
 *
 * As such, the actual delay time will be time_us + 2.2us
 *
 * Should support values up to 67s
 * */
void wait(uint32_t time_us)
{
	// sanity check
	if (!time_us) {
		return;
	}
	SysTick->LOAD  = (uint32_t)((time_us * _MULTIPLICATION_FACTOR) - 1UL);
	SysTick->VAL   = 0UL;
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
	while((SysTick->CTRL & 0x10000) == 0UL) {
	}
	SysTick->CTRL=_SYSTICK_CTRL_DISABLE_Msk;
}


void I2C_init(void)
{
    INT_init_ext_interrupts();


	XMC_I2C_CH_Init(XMC_USIC0_CH1, &I2C_master_conf);

	// I2C pins will be in GPIO input mode, the peripheral will always read HIGH
	// on both pins
	I2C_mode_gpio_input();
	I2C_enable();
}

void I2C_enable(void)
{
	/* Configure GPIO, P2 also needs to be set to digital - mode set is not enough */
	XMC_GPIO_Init(CONF_I2C_SDA_PIN, &CONF_I2C_SDA_PIN_DISABLED);
	XMC_GPIO_Init(CONF_I2C_SCL_PIN, &CONF_I2C_SCL_PIN_DISABLED);

	/* start USIC */
	XMC_I2C_CH_Start(XMC_USIC0_CH1);
}

void I2C_disable(void)
{
	XMC_I2C_CH_Stop(XMC_USIC0_CH1);
}


void I2C_mode_normal(void)
{
	// configure TLx port for I2C output
	WR_REG(PORT2->IOCR8, PORT2_IOCR8_PC11_Msk, PORT2_IOCR8_PC11_Pos, 0x1EU);
	WR_REG(PORT2->IOCR8, PORT2_IOCR8_PC10_Msk, PORT2_IOCR8_PC10_Pos, 0x1FU);

	// I2C peripheral will read the state of the pins
	XMC_USIC0_CH1->DXCR[XMC_I2C_CH_INPUT_SDA] = (XMC_USIC0_CH1->DXCR[XMC_I2C_CH_INPUT_SDA] & ~USIC_CH_DXCR_DSEL_Msk)
						                      | (USIC0_C1_DX0_P2_10 << USIC_CH_DXCR_DSEL_Pos);
	XMC_USIC0_CH1->DXCR[XMC_I2C_CH_INPUT_SCL] = (XMC_USIC0_CH1->DXCR[XMC_I2C_CH_INPUT_SCL] & ~USIC_CH_DXCR_DSEL_Msk)
						                      | (USIC0_C1_DX1_P2_11 << USIC_CH_DXCR_DSEL_Pos);

	// disable and clear interrupt
	INT_int_ext_disable();
}


void I2C_mode_gpio_input(void)
{
	// I2C peripheral will always read 1
	XMC_USIC0_CH1->DXCR[XMC_I2C_CH_INPUT_SDA] |= USIC_CH_DXCR_DSEL_Msk;
	XMC_USIC0_CH1->DXCR[XMC_I2C_CH_INPUT_SCL] |= USIC_CH_DXCR_DSEL_Msk;

	// configure pins for GPIO input
	WR_REG(PORT2->IOCR8, PORT2_IOCR8_PC11_Msk, PORT2_IOCR8_PC11_Pos, 0x0U);
	WR_REG(PORT2->IOCR8, PORT2_IOCR8_PC10_Msk, PORT2_IOCR8_PC10_Pos, 0x0U);

	// enable interrupt
	INT_int_ext_enable();
}


int32_t I2C_write(uint8_t addr, const uint8_t* data, uint8_t count)
{
	int32_t status = I2C_STATUS_OK;

	// check peripheral not already used
	__disable_irq();

myDebug("I2C_write");

// XMC_I2C_CH_ClearStatusFlag(XMC_I2C0_CH1, 0xFFFFFFFF);
	XMC_I2C_CH_ClearStatusFlag(
			XMC_I2C0_CH1,
			XMC_I2C_CH_STATUS_FLAG_RECEIVE_INDICATION
			| XMC_I2C_CH_STATUS_FLAG_ALTERNATIVE_RECEIVE_INDICATION
			| XMC_I2C_CH_STATUS_FLAG_ACK_RECEIVED
			| XMC_I2C_CH_STATUS_FLAG_NACK_RECEIVED
	);
	I2C_mode_normal();

	// send addressing frame
	XMC_I2C_CH_MasterStart(XMC_I2C0_CH1, addr, XMC_I2C_CH_CMD_WRITE);

	// wait for sensor ACK
	int32_t retry_cnt = 1500U;

	while(true) {
		if ((XMC_I2C_CH_GetStatusFlag(XMC_I2C0_CH1) & (XMC_I2C_CH_STATUS_FLAG_ACK_RECEIVED))) {
			break;
		} else if ((XMC_I2C_CH_GetStatusFlag(XMC_I2C0_CH1) & (XMC_I2C_CH_STATUS_FLAG_NACK_RECEIVED))) {
			status = I2C_STATUS_NACK;
			goto _i2c_write_end;
		}
		
		wait(1);
		retry_cnt--;

		if (retry_cnt <= 0U) {
			// myDebug("1");
			goto timeout;
		}
	}

	// send data
	XMC_I2C_CH_ClearStatusFlag(
		XMC_I2C0_CH1,
		XMC_I2C_CH_STATUS_FLAG_ACK_RECEIVED
		| XMC_I2C_CH_STATUS_FLAG_NACK_RECEIVED
	);

	for(uint32_t i = 0; i < count; i++) {
		// transmit
		XMC_I2C_CH_MasterTransmit(XMC_I2C0_CH1, data[i]);
		// wait for ACK
		retry_cnt = 1500U;

		while(!(XMC_I2C_CH_GetStatusFlag(XMC_I2C0_CH1) & XMC_I2C_CH_STATUS_FLAG_ACK_RECEIVED)) {
			wait(1);
			retry_cnt--;

			if (retry_cnt <= 0) {
			// myDebug("2");
				goto timeout;
			}
		}
		XMC_I2C_CH_ClearStatusFlag(XMC_I2C0_CH1, XMC_I2C_CH_STATUS_FLAG_ACK_RECEIVED);
	}


        // while (!XMC_USIC_CH_TXFIFO_IsEmpty(XMC_I2C0_CH1))
        // {
        // }
	// send stop
	// wait(5);

	XMC_I2C_CH_MasterStop(XMC_I2C0_CH1);
	I2C_wait_transmit();
	
	// XMC_I2C_CH_MasterStop(XMC_I2C0_CH1);
	// I2C_wait_transmit();
    // XMC_I2C0_CH1->IN[0U] = (uint32_t)6U << 8U;
    // XMC_I2C0_CH1->TBUF[0] = (uint32_t)6U << 8U;

	// wait for ACK

        // while ( XMC_USIC_CH_GetTransmitBufferStatus(XMC_I2C0_CH1) == XMC_USIC_CH_TBUF_STATUS_BUSY )
		// {
		// }

	retry_cnt = 150000U;
	while(!(XMC_I2C_CH_GetStatusFlag(XMC_I2C0_CH1) & XMC_I2C_CH_STATUS_FLAG_TRANSMIT_BUFFER_INDICATION)) {
		wait(1);
		retry_cnt--;
			// myDebug("4");

		if (retry_cnt <= 0) {
			goto timeout;
		}
	}
// XMC_I2C_CH_ClearStatusFlag(XMC_I2C0_CH1, 0xFFFFFFFF);


_i2c_write_end:
// wait(5);
	I2C_mode_gpio_input();
	XMC_I2C_CH_ClearStatusFlag(
		XMC_I2C0_CH1,
		XMC_I2C_CH_STATUS_FLAG_NACK_RECEIVED
		| XMC_I2C_CH_STATUS_FLAG_TRANSMIT_BUFFER_INDICATION
	);
	__enable_irq();
	return status;

timeout:
	__enable_irq();
	// Reset system
	// Execution ends here
	NVIC_SystemReset();
	return I2C_STATUS_TIMEOUT;
}


int32_t I2C_read(uint8_t addr, uint8_t *data, uint8_t count)
{
	volatile uint8_t aux;
	int32_t status = I2C_STATUS_OK;
// XMC_I2C_CH_ClearStatusFlag(XMC_I2C0_CH1, 0xFFFFFFFF);


	// make method atomic
	__disable_irq();

	// clear status flags
	XMC_I2C0_CH1->PSCR |= XMC_I2C_CH_STATUS_FLAG_RECEIVE_INDICATION
						| XMC_I2C_CH_STATUS_FLAG_ALTERNATIVE_RECEIVE_INDICATION
						| XMC_I2C_CH_STATUS_FLAG_ACK_RECEIVED
						| XMC_I2C_CH_STATUS_FLAG_NACK_RECEIVED;

	// switch to I2C mode
	// (from GPIO edge Interrupt)
	I2C_mode_normal();

	// send addressing frame
	XMC_I2C_CH_MasterStart(XMC_I2C0_CH1, addr, XMC_I2C_CH_CMD_READ);

	// wait for sensor ACK
	int32_t retry_cnt = 1500;

	while(true) {
		if ((XMC_I2C_CH_GetStatusFlag(XMC_I2C0_CH1) & (XMC_I2C_CH_STATUS_FLAG_ACK_RECEIVED))) {
			break;
		} else if ((XMC_I2C_CH_GetStatusFlag(XMC_I2C0_CH1) & (XMC_I2C_CH_STATUS_FLAG_NACK_RECEIVED))) {
			status = I2C_STATUS_NACK;
			goto _i2c_read_end;
		}

		wait(1);
		retry_cnt--;

		if (retry_cnt <= 0U) {
			status = I2C_STATUS_TIMEOUT;
			goto _i2c_timeout;
		}
	}

	// clear status flags
	XMC_I2C0_CH1->PSCR |= XMC_I2C_CH_STATUS_FLAG_RECEIVE_INDICATION
						| XMC_I2C_CH_STATUS_FLAG_ALTERNATIVE_RECEIVE_INDICATION
						| XMC_I2C_CH_STATUS_FLAG_ACK_RECEIVED
						| XMC_I2C_CH_STATUS_FLAG_NACK_RECEIVED;

	// read once from RX buffer to flush it (make sure it is not optimize out)
	aux = XMC_I2C_CH_GetReceivedData(XMC_I2C0_CH1);
	aux = aux;
	// request and receive data
	retry_cnt = 1500;
	for(uint8_t i = 0; i < count; i++) {
		// request data, prepare to send ACK (request more)
		if (i == (count - 1)) {
			XMC_I2C_CH_MasterReceiveNack(XMC_USIC0_CH1);
		} else {
			XMC_I2C_CH_MasterReceiveAck(XMC_I2C0_CH1);
		}

		// wait for data (ACK will be sent when data is received)
		while(!(XMC_I2C_CH_GetStatusFlag(XMC_I2C0_CH1)
					& (XMC_I2C_CH_STATUS_FLAG_RECEIVE_INDICATION
					| XMC_I2C_CH_STATUS_FLAG_ALTERNATIVE_RECEIVE_INDICATION))) {
			wait(1);
			retry_cnt--;
			if(retry_cnt <= 0U) {
				status = I2C_STATUS_TIMEOUT;
				goto _i2c_timeout;
			}
		}
		// read received data
		data[i] = XMC_I2C_CH_GetReceivedData(XMC_I2C0_CH1);
		XMC_I2C_CH_ClearStatusFlag(
			XMC_I2C0_CH1,
			XMC_I2C_CH_STATUS_FLAG_RECEIVE_INDICATION
			| XMC_I2C_CH_STATUS_FLAG_ALTERNATIVE_RECEIVE_INDICATION
		);
	}

	// stop transmission
	XMC_I2C_CH_MasterStop(XMC_I2C0_CH1);

	// wait for transmission
	retry_cnt = 1500;
	while((XMC_I2C0_CH1->PSR_IICMode & XMC_I2C_CH_STATUS_FLAG_TRANSMIT_BUFFER_INDICATION) == 0U) {
		wait(1);
		retry_cnt--;
		if (retry_cnt <= 0U) {
			status = I2C_STATUS_TIMEOUT;
			goto _i2c_timeout;
		}
	}


_i2c_read_end:
	// clear status flags
	XMC_I2C0_CH1->PSCR |= XMC_I2C_CH_STATUS_FLAG_TRANSMIT_BUFFER_INDICATION
						| XMC_I2C_CH_STATUS_FLAG_NACK_RECEIVED;
	I2C_mode_gpio_input();
	__enable_irq();
	return status;

_i2c_timeout:
	__enable_irq();
	// Reset the system
	// Execution ends here
	NVIC_SystemReset();
	return I2C_STATUS_TIMEOUT;
}


void I2C_peripheral_recover()
{
	WR_REG(PORT2->IOCR8, 0xF8000000U, PORT2_IOCR8_PC11_Pos, 0);
	WR_REG(PORT2->IOCR8, PORT2_IOCR8_PC10_Msk, PORT2_IOCR8_PC10_Pos, 0x0U);
	XMC_USIC_CH_TXFIFO_Flush(XMC_USIC0_CH1);
	XMC_USIC_CH_RXFIFO_Flush(XMC_USIC0_CH1);
	XMC_I2C_CH_Stop(XMC_USIC0_CH1);
	I2C_init();
	I2C_enable();
}


void I2C_wait_transmit(void)
{
	uint32_t count = 0;
	while (XMC_USIC_CH_GetTransmitBufferStatus(XMC_USIC0_CH1) == XMC_USIC_CH_TBUF_STATUS_BUSY)
	{
		wait(1);
		// check TDV, wait until TBUF is ready
		count++;
		if(count>100000)
		{
			NVIC_SystemReset();
		}
	}
}


void I2C_write_recover(void)
{
	I2C_mode_normal();
	XMC_I2C_CH_MasterStart(	XMC_USIC0_CH1,
				TLE493D_AW2B6_I2C_RECOV_ADDR,
				XMC_I2C_CH_CMD_READ
	);
	wait(10);
	XMC_I2C_CH_MasterStop(XMC_USIC0_CH1);
	I2C_wait_transmit();
	I2C_mode_gpio_input();
}

void I2C_write_reset(void)
{
	I2C_mode_normal();
	XMC_I2C_CH_MasterStart(
		XMC_USIC0_CH1,
		TLE493D_AW2B6_I2C_RESET_ADDR,
		XMC_I2C_CH_CMD_WRITE
	);
	wait(10);
	XMC_I2C_CH_MasterStop(XMC_USIC0_CH1);
	I2C_wait_transmit();
	I2C_mode_gpio_input();
}
