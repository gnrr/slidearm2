/****************************************************************************
 *   $Id:: uarttest.c 3635 2012-10-31 00:31:46Z usb00423                    $
 *   Project: NXP LPC8xx USART example
 *
 *   Description:
 *     This file contains USART test modules, main entry, to test USART APIs.
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
#include "keycode.h"

#define SW_FWD		0
#define SW_BACK		1

#define SW_TYPE_A	0		// normal open
#define SW_TYPE_B	1		// normal close
u1 sw_type_back;			// SW_TYPE_A: BERETTA
							// SW_TYPE_B: SIG, GLOCK

u1 keystate[2] = {0, 0};	// 0:initial	1:pressed	 2:released
volatile u4 TimeTick = 0;

// sub functions
void SysTick_Handler(void)
{
    TimeTick++;
}

void wait_ms(u4 tick)
{
    u4 timetick;

    SysTick->VAL = 0;					// Enable the SysTick Counter
    SysTick->CTRL |= (0x1<<0);			// Clear SysTick Counter

    timetick = TimeTick;
    while ((TimeTick - timetick) < tick) ;

    SysTick->CTRL &= ~(0x1<<0);			// Disable SysTick Counter
    SysTick->VAL = 0;					// Clear SysTick Counter
}

bool keep_pulling_p(sw)					// call at power on
{
	bool buf[100];
	bool res = TRUE;
	u1 i;

	for(i=0; i<100; i++) buf[i] = 1;	// init

	for(i=0; i<100; i++)
		buf[i] = GPIOGetPinValue(0, sw);

	for(i=0; i<100; i++) {
		if(buf[i] == 1) {				// 0:pulled, 1:released
			res = FALSE;
			break;
		}
	}
	return res;
}

bool pulled_p(sw)
{
	u1 *pks;
	u1 k = OFF;

	if(sw == SW_FWD) {
		pks = &keystate[SW_FWD];
		k = (GPIOGetPinValue(0, SW_FWD))? OFF:ON;		// HI:OFF	 LO:ON
	} else if(sw == SW_BACK) {
		if(sw_type_back == SW_TYPE_A)
			k = (GPIOGetPinValue(0, SW_BACK))? OFF:ON;	// HI:OFF	 LO:ON
		else
			k = (GPIOGetPinValue(0, SW_BACK))? ON:OFF;	// LO:OFF	 HI:ON
	}
	if((k == ON) && (*pks == 0))
		*pks = 1;
	else if((k == OFF) && (*pks == 1))
		*pks = 2;

	return (*pks == 2);
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

void send_report(u1 scan_code)
{
	//                0     1     2     3     4     5     6     7     8     9     10
	//				 start len	 desc  mod	  00   scan1 scan2 scan3 scan4 scan5 scan6
	u1 report[11] = {0xFD, 0x09, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	report[5] = scan_code;
	UARTSend(LPC_USART0, (uint8_t *)report, 11);
}

void init_hw(void)
{
	u4 regVal;

	SystemCoreClockUpdate();

	// systick wait
	SysTick_Config(SystemCoreClock/1000);		// interrupt every 1ms

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

	// vars
	sw_type_back = (keep_pulling_p(SW_BACK))? SW_TYPE_B:SW_TYPE_A;
}


// main function
int main (void)
{
	bool reported_p = FALSE;
	u1 i;
	init_hw();

	// decide switch type of slide back
	sw_type_back = (keep_pulling_p(SW_BACK))? SW_TYPE_B:SW_TYPE_A;

#if 0
	UARTSend(LPC_USART0, (uint8_t *)"C", 1); // 0x43
	delay_ms(10);
	UARTSend(LPC_USART0, (uint8_t *)"a", 1); // 0x61
	delay_ms(10);
#endif

	// do setting bluetooth module(RN42) if the trigger keep pulling at power on
	if(keep_pulling_p(SW_FWD))
		do_setting_bt_module();

	while (1)
	{
		if(reported_p == FALSE) {
			if(pulled_p(SW_FWD)) {
				send_report(USID_KBD_PAGE_DOWN);
				reported_p = TRUE;
			} else if(pulled_p(SW_BACK)) {
				send_report(USID_KBD_PAGE_UP);
				reported_p = TRUE;
			}
		} else {
			send_report(0x00);		// released
			reported_p = FALSE;

			for(i=0;i<sizeof(keystate); i++)
				keystate[i] = 0;
		}
	}
	return EXIT_FAILURE;
}
