/*
 * timer.h
 *
 *	Created on: Jun 29, 2014
 *		Author: g
 */

#ifndef __TIMER_H__
#define __TIMER_H__

#include "LPC8xx.h"
#include "mystd.h"

/* Control register bit definition. */
#define MRT_INT_ENA					(0x1<<0)
#define MRT_REPEATED_MODE			(0x00<<1)
#define MRT_ONE_SHOT_INT			(0x01<<1)
#define MRT_ONE_SHOT_STALL			(0x02<<1)

/* Status register bit definition */
#define MRT_STAT_IRQ_FLAG			(0x1<<0)
#define MRT_STAT_RUN				(0x1<<1)

void init_timer(void);
void start_timer(u1 timer_num);
void stop_timer(u1 timer_num);
u4 read_timer(u1 timer_num);

#endif /* __TIMER_H__ */
