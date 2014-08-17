/*
 * main.c
 *
 *	Created on: Jun 30, 2014
 *		Author: g
 */
#include "LPC8xx.h"
#include "lpc8xx_clkconfig.h"
#include "lpc8xx_gpio.h"
#include "lpc8xx_uart.h"
#include "mystd.h"
#include "timer.h"
#include "keycode.h"

#define SW_FWD		0
#define SW_BACK		1

#define SW_TYPE_A	0		// normal open
#define SW_TYPE_B	1		// normal close
u1 sw_type_back;			// SW_TYPE_A: BERETTA
							// SW_TYPE_B: SIG, GLOCK

enum {KS_INIT, KS_PRESSED1, KS_PRESSED2, KS_PRESSED3};
u1 keystate[2] = {KS_INIT, KS_INIT};	// [0]:sw_fwd,    [1]:sw_back

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

bool keep_pulling_p(sw)					// call at powered on
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
	const u4 DELAY_UNTIL_REPEAT	= 1000;			// ms
	const u4 REPEAT_INTERVAL	= 200;			// ms
	bool out = OFF;
	bool k = OFF;
	u1 *pks;

	pks = &keystate[sw];

	if(sw == SW_FWD) {
		k = (GPIOGetPinValue(0, SW_FWD))? OFF:ON;		// HI:OFF	 LO:ON
	} else if(sw == SW_BACK) {
		if(sw_type_back == SW_TYPE_A)
			k = (GPIOGetPinValue(0, SW_BACK))? OFF:ON;	// HI:OFF	 LO:ON
		else
			k = (GPIOGetPinValue(0, SW_BACK))? ON:OFF;	// LO:OFF	 HI:ON
	}

	if((k == ON) && (*pks == KS_INIT)) {				// pulled first time?
		*pks = KS_PRESSED1;
	}
	else if((k == ON) && (*pks == KS_PRESSED1)) {			// pulled second time?
		out = ON;
		start_timer(sw);
		*pks = KS_PRESSED2;
	}
	else if((k == ON) && (*pks == KS_PRESSED2)) {		// keep pulling?
		if(read_timer(sw) > DELAY_UNTIL_REPEAT) {
			out = ON;
			stop_timer(sw);
			start_timer(sw);
			*pks = KS_PRESSED3;
		}
	} else if((k == ON) && (*pks == KS_PRESSED3)) {
		if(read_timer(sw) > REPEAT_INTERVAL) {
			out = ON;
			stop_timer(sw);
			start_timer(sw);
		}
	}
	else if(k == OFF) {		// released?
		stop_timer(sw);
		*pks = KS_INIT;
	}
	return out;
}

void do_setting_bt_module(void)
{
	const u4 WAIT = 100;		// ms

	UARTSend(LPC_USART0, (uint8_t *)"$$$", 3);
	wait_ms(WAIT);
	UARTSend(LPC_USART0, (uint8_t *)"s~,6\r\n", 6);				// HID
	wait_ms(WAIT);
	UARTSend(LPC_USART0, (uint8_t *)"sm,6\r\n", 6);				// pairing mode
	wait_ms(WAIT);
	UARTSend(LPC_USART0, (uint8_t *)"s-,SLIDEARM201\r\n", 16);	// BT Name
	wait_ms(WAIT);
	UARTSend(LPC_USART0, (uint8_t *)"r,1\r\n", 5);				// reset
	wait_ms(WAIT);
}

void send_report(u1 scan_code)
{
	//                     0     1     2     3     4     5     6     7     8     9     10
	//                    start len   desc  mod	   00   scan1 scan2 scan3 scan4 scan5 scan6
	uint8_t report[11] = {0xFD, 0x09, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	report[5] = scan_code;
	UARTSend(LPC_USART0, report, 11);
	wait_ms(100);
}

void init_hw(void)
{
	u4 regVal;

	SystemCoreClockUpdate();

#if 0
	// Config CLKOUT, mostly used for debugging
	regVal = LPC_SWM->PINASSIGN8;
	regVal &= ~( 0xFF << 16 );
	regVal |= ( 17 << 16 );
	LPC_SWM->PINASSIGN8 = regVal;							/* P0.17 is CLKOUT, ASSIGN(23:16). */
	CLKOUT_Setup( CLKOUTCLK_SRC_MAIN_CLK );
//	CLKOUT_Setup( CLKOUTCLK_SRC_IRC_OSC );
#endif

	// systick wait
	SysTick_Config(SystemCoreClock/1000);		// interrupt every 1ms

	// timer
	LPC_SYSCON->SYSAHBCLKCTRL |= (1<<6);		// Enable AHB clock to the GPIO domain.
	init_timer();

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
	bool reported_p = FALSE;
	init_hw();

	// decide switch type of slide back
	sw_type_back = (keep_pulling_p(SW_BACK))? SW_TYPE_B:SW_TYPE_A;

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
		}
	}

	return EXIT_FAILURE;			// should not be reach here
}
