/****************************************************************************
 *	 $Id:: uarttest.c 3635 2012-10-31 00:31:46Z usb00423					$
 *	 Project: NXP LPC8xx USART example
 *
 *	 Description:
 *	   This file contains USART test modules, main entry, to test USART APIs.
 *
 ****************************************************************************
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * products. This software is supplied "AS IS" without any warranties.
 * NXP Semiconductors assumes no responsibility or liability for the
 * use of the software, conveys no license or title under any patent,
 * copyright, or mask work right to the product. NXP Semiconductors
 * reserves the right to make changes in the software without
 * notification. NXP Semiconductors also make no representation or
 * warranty that such application will be suitable for the specified
 * use without further testing or modification.
 
 * Permission to use, copy, modify, and distribute this software and its 
 * documentation is hereby granted, under NXP Semiconductors' 
 * relevant copyright in the software, without fee, provided that it 
 * is used in conjunction with NXP Semiconductors microcontrollers. This 
 * copyright, permission, and disclaimer notice must appear in all copies of 
 * this code.
 
 ****************************************************************************/
#include "LPC8xx.h"
#include "lpc8xx_clkconfig.h"
#include "lpc8xx_gpio.h"
#include "lpc8xx_uart.h"
#include "mystd.h"

#define SYSTICK_DELAY		(SystemCoreClock/1000)	// interrupt every 1ms
volatile u4 TimeTick = 0;

// sub functions
void SysTick_Handler(void)
{
  TimeTick++;
}

void wait_ms(u4 tick)
{
  u4 timetick;

  SysTick->VAL = 0;						// Enable the SysTick Counter
  SysTick->CTRL |= (0x1<<0);			// Clear SysTick Counter

  timetick = TimeTick;
  while ((TimeTick - timetick) < tick) ;

  SysTick->CTRL &= ~(0x1<<0);			// Disable SysTick Counter
  SysTick->VAL = 0;						// Clear SysTick Counter
}

bool pulled_trigger_p(void)
{
	bool buf[100];
	bool res = TRUE;
	u1 i;

	for(i=0; i<100; i++) buf[i] = 1;

	for(i=0; i<100; i++)
		buf[i] = GPIOGetPinValue(0, 0);

	for(i=0; i<100; i++) {
		if(buf[i] == 1) {				// 0:pulled, 1:released
			res = FALSE;
			break;
		}
	}
	return res;
}

void do_setting_bt_module(void)
{
	UARTSend(LPC_USART0, (uint8_t *)"$$$", 3);
	wait_ms(100);
	UARTSend(LPC_USART0, (uint8_t *)"s~,6\r", 5);				// HID
	wait_ms(100);
	UARTSend(LPC_USART0, (uint8_t *)"sm,6\r", 5);				// pairing mode
	wait_ms(100);
//	UARTSend(LPC_USART0, (uint8_t *)"su,9600\r", 8);			// 9600bps
//	wait_ms(10);
	UARTSend(LPC_USART0, (uint8_t *)"s-,SLIDEARM200\r", 15);	// BT Name
	wait_ms(100);
	UARTSend(LPC_USART0, (uint8_t *)"r,1\r", 4);				// reset
	wait_ms(100);
}

void init_hw(void)
{
	u4 regVal;

	SystemCoreClockUpdate();

	// systick wait
	SysTick_Config(SYSTICK_DELAY);

#if 0
	// Config CLKOUT, mostly used for debugging
	regVal = LPC_SWM->PINASSIGN8;
	regVal &= ~( 0xFF << 16 );
	regVal |= ( 17 << 16 );
	LPC_SWM->PINASSIGN8 = regVal;							/* P0.17 is CLKOUT, ASSIGN(23:16). */
	CLKOUT_Setup( CLKOUTCLK_SRC_MAIN_CLK );
//	CLKOUT_Setup( CLKOUTCLK_SRC_IRC_OSC );
#endif

	// uart
	regVal = LPC_SWM->PINASSIGN0 & ~( 0xFF << 0 );
	LPC_SWM->PINASSIGN0 = regVal | ( 7 << 0 );				/* P0.7 is UART0 TXD, ASSIGN0(7:0) */
	regVal = LPC_SWM->PINASSIGN0 & ~( 0xFF << 8 );
	LPC_SWM->PINASSIGN0 = regVal | ( 8 << 8 );				/* P0.8 is UART0 RXD. ASSIGN0(15:8) */

	UARTInit(LPC_USART0, 115200);
}

// main function
int main (void)
{
	init_hw();

#if 0
	UARTSend(LPC_USART0, (uint8_t *)"C", 1); // 0x43
	delay_ms(10);
	UARTSend(LPC_USART0, (uint8_t *)"a", 1); // 0x61
	delay_ms(10);
#endif

	if(pulled_trigger_p())
		do_setting_bt_module();

	while (1)
	{
		;
	}
}
