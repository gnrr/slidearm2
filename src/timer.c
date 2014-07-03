/*
 * timer.c
 *
 *	Created on: Jun 29, 2014
 *		Author: g
 */
#include "timer.h"

static u4 counter[2] = {0, 0};		// [0]:sw_fwd,    [1]:sw_back

void init_timer(void)
{
	const u4 INTERVAL = SystemCoreClock/1000;		// 1ms
//	const u4 INTERVAL = SystemCoreClock/100;		// 10ms
//	const u4 INTERVAL = SystemCoreClock/10;			// 100ms
//	const u4 INTERVAL = SystemCoreClock;			// 1s

	/* Enable clock to MRT and reset the MRT peripheral */
	LPC_SYSCON->SYSAHBCLKCTRL |= (0x1<<10);		// supply clock to MRT
	LPC_SYSCON->PRESETCTRL &= ~(0x1<<7);		// reset MRT (write 0-->1)
	LPC_SYSCON->PRESETCTRL |= (0x1<<7);

	// fwd
	LPC_MRT->Channel[0].INTVAL = INTERVAL;
	LPC_MRT->Channel[0].CTRL = MRT_INT_ENA;

	// back
	LPC_MRT->Channel[1].INTVAL = INTERVAL;
	LPC_MRT->Channel[1].CTRL = MRT_INT_ENA;

	/* Enable the MRT Interrupt */
	NVIC_EnableIRQ(MRT_IRQn);
}

void start_timer(u1 timer_num)
{
	counter[timer_num]= 0;
	LPC_MRT->Channel[timer_num].INTVAL |= 0x1UL<<31;	// start timer
}

void stop_timer(u1 timer_num)
{
	LPC_MRT->Channel[timer_num].INTVAL &= ~(0x1UL<<31);	// stop timer
}

u4 read_timer(u1 timer_num)
{
	return counter[timer_num];
}

// ISR
void MRT_IRQHandler(void)
{
	// fwd
	if (LPC_MRT->Channel[0].STAT & MRT_STAT_IRQ_FLAG) {
		LPC_MRT->Channel[0].STAT = MRT_STAT_IRQ_FLAG;  // clear interrupt flag
		if(counter[0] < U4MAX) counter[0]++;
	}

	// back
	if (LPC_MRT->Channel[1].STAT & MRT_STAT_IRQ_FLAG) {
		LPC_MRT->Channel[1].STAT = MRT_STAT_IRQ_FLAG;  // clear interrupt flag
		if(counter[1] < U4MAX) counter[1]++;
	}
}
