/*---------------------------------------------------------------------*
*	Timer.h
*----------------------------------------------------------------------*/

#ifndef _TIMER_H_
#define _TIMER_H_

#include "general.h"


/* ----- types for timers ----- */
#define TIMER_STATUS_READY    	0
#define TIMER_STATUS_RUNNING	1
#define TIMER_STATUS_STOPPED	2


typedef struct timerStruct {
	unsigned int WaitTime;
	unsigned char Status;
} TIMERDEF, *pTIMERDEF;


/* ----- prototypes ----- */
bit IsTimerRunning( uchar timer_num );
void TimerStart( uchar timer_num, uint num_ticks );
void TimerReset( uchar timer_num );
unsigned int TimerQuery( uchar timer_num );
bit IsTimerReady( uchar timer_num );


uint timer0_count( void );
void TIMER0_Init( uint clock );
uint timer0_elapsed_count( uint count );
void WaitTicks( uint count);
void delay_ms( uint count );
void delay_2sec( void );

#endif

