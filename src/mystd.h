/* mystd.h */

#ifndef __MYSTD_H__
#define __MYSTD_H__

/* types and values */
typedef unsigned char	u1;
typedef unsigned short	u2;
typedef unsigned long	u4;

typedef signed char		s1;
typedef signed short	s2;
typedef signed long		s4;

typedef unsigned char	bool;

#define FALSE	0
#define TRUE	1

#define LO		0
#define HI		1

#define OFF		0
#define ON		1

#define NG		0
#define OK		1

#define U1MAX	0xFF
#define U2MAX	0xFFFF
#define U4MAX	0xFFFFFFFF

#define EXIT_SUCCESS	0
#define EXIT_FAILURE	1

#define BIT_POS_7		7
#define BIT_POS_6		6
#define BIT_POS_5		5
#define BIT_POS_4		4
#define BIT_POS_3		3
#define BIT_POS_2		2
#define BIT_POS_1		1
#define BIT_POS_0		0

#define BIT_MASK_7		0x80
#define BIT_MASK_6		0x40
#define BIT_MASK_5		0x20
#define BIT_MASK_4		0x10
#define BIT_MASK_3		0x08
#define BIT_MASK_2		0x04
#define BIT_MASK_1		0x02
#define BIT_MASK_0		0x01


/* pseudo functions */
// #define __ei()	sei()
// #define __di()	cli()

#endif	/* __MYSTD_H__ */
