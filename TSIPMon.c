//-----------------------------------------------------------------------------
// TSIPMon.c
//-----------------------------------------------------------------------------
//
// Program Description:
//
// This program uses a Silabs C8051F330 microcontroller and LCD display to 
// receive, decode and display time, position and status information from a 
// Trimble Thunderbolt GPS Disciplined Reference Oscillator.
//
// The data is transmitted serially from the Thunderbolt at 9600,8,N,1.
// The default Thunderbolt configuration sends 2 packet types every second:
// 	- Packet 0x8F-AB is the Primary Timing Packet
// 	- Packet 0x8F-AC is the Supplemental Timing Packet.
//
//
// Target:         C8051F330 + HD447870 compatible LCD display (2 lines x 16 char)
// Tool chain:     SDCC 2.6.0
//
// Release 1.0
//    -Initial Revision (DJ)
//    -12 Jul 2009
//
// Copyright 2008-2010 Didier Juges 
// http://www.ko4bb.com
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the 
// Free Software Foundation, Inc., 
// 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
//
//-----------------------------------------------------------------------------

/*-----------------------------------------------------------------------------*
*	Version
*
* Vers.  Date       Who        What
* ====== ========== ========== ==========================
* v0.0.1  7July08	D. Juges   Initial
* v0.0.9 18July08   D. Juges   Changed messages
* v0.1.1 25July08   D. Juges   Displays Temperature, 
* v0.1.2 29July08   D. Juges   Also displays DAC voltage,
*                              Disciplining Mode and Rx Mode successively
*                              by pressing SW1.
* v0.1.3 2Aug08     D. Juges   Try a fix for randon turn off of Noritake?
* v0.2.0 12Dec08    Dan Karg   Many changes/fixes additions
* v0.2.1 28Dec08    D. Juges   Made LCD_PORT push-pull to fix problems with VFD
* v0.2.2 1Jan09     D. Juges   Display DAC voltage with more decimals,
*                              fix problem with temp and dac voltage
* v0.2.3 10Dec10    D. Juges   Added choice of time zones
* v0.3 0 11Dec10    D. Juges   Time zone and GPS offset works, P0.6 can be used
*                              (by grounding) to revert to no time zone
*                              P0.7 is used to select daylight savings time (when grounded)
* v0.3.3 12Feb11    D. Juges   Fixed bug with time zone offset
* v0.3.4 08Apr12    D. Juges   Added support for 2x20 LCD
*------------------------------------------------------------------------------*/

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------

#include <C8051F330.h>                 // SFR declarations

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "general.h"
#include "lcd_if.h"
#include "timer.h"
#include "TSIP.h"
#include "iconv.h"
#include "extdebug.h"

/*-----------------------------------------------------------------------------
* Customizable constants
*-----------------------------------------------------------------------------*/
#define TIME_ZONE	-6	// -6 = Central time
//#define TIME_ZONE 	-8	// -8 = Pacific time
//#define GPS_OFFSET 	15	// 15 seconds until the next leap second in July

/*-----------------------------------------------------------------------------
* version:
*-----------------------------------------------------------------------------*/
code uchar VersionMsg[]     = " TSIPMon v0.3.4     ";  // make 20 char long
code uchar LCDInitMsg[] 	= " www.ko4bb.com      ";  // make 20 char long


//-----------------------------------------------------------------------------
// Global Constants
//-----------------------------------------------------------------------------
#define INTERRUPT_UART0         4  // Serial Port 0

#define SYSCLK             24500000/8  // SYSCLK in Hz (24.5 MHz internal
                                       // oscillator / 8)
                                       // the internal oscillator has a
                                       // tolerance of +/- 2%

#define TIMER_PRESCALER            12  // Based on Timer2 CKCON and TMR2CN
                                       // settings

#define TIMER_RATE                2  // LED toggle rate in milliseconds
                                       // if LED_TOGGLE_RATE = 1, the LED will
                                       // be on for 1 millisecond and off for
                                       // 1 millisecond

// There are SYSCLK/TIMER_PRESCALER timer ticks per second, so
// SYSCLK/TIMER_PRESCALER/1000 timer ticks per millisecond.
#define TIMER_TICKS_PER_MS  SYSCLK/TIMER_PRESCALER/1000

// Note: LED_TOGGLE_RATE*TIMER_TICKS_PER_MS should not exceed 65535 (0xFFFF)
// for the 16-bit timer

#define AUX1     TIMER_TICKS_PER_MS*TIMER_RATE
#define AUX2     -AUX1

#define TIMER2_RELOAD            AUX2  // Reload value for Timer2

#define LED  	P1_3                   // LED='1' means ON

#define SW2 	P0_1	// /INT0	// right switch (pin 1 of F330D)
#define SW1 	P0_2	// /INT1	// left switch (pin 0 of F330D)
#define LED1	P0_3	// green LED
//#define LED2	P0_7	// red LED

#define OFFSET_SELECT	P0_6	// pin 18 on the 20 pin DIP package
								// ground this pin to display GPS time
							// leave open to display time in selected time zone
#define DST_SELECT		P0_7

/* ===== GPS Stuff ===== */
#define ETX								0x03
#define DLE								0x10
#define IO_BUF_ID_INDEX					1
#define IO_BUF_ID2_INDEX				2

#define SUPERPACKET						0x8F
#define PRIMARY_TIMING_PCKT				0xAB
#define PRIMARY_TIMING_PCKT_LEN			17
#define SUPPLEMENTAL_TIMING_PCKT		0xAC
#define SUPPLEMENTAL_TIMING_PCKT_LEN	60

code uchar prompt[2][9]={
	"TEMP:  ",
	"DAC V: "
};



/* ===== Global Variables ===== */

/* ----- Timers ----- */
TIMERDEF Timer[NUM_TIMERS];

static uchar TxRx_State;
static uchar TxRx_Count;

bit Rx_Pending;
bit Tx_In_Progress;

INTType xdata Alarms;

// static uchar TXRX_STORAGE_CLASS *p_TxRx_Buf;
// uchar TXRX_STORAGE_CLASS TxRxBuf[TXRX_BUF_LEN];

bit switch2, switch1, func2, func1, err1;

uchar LCD_STORAGE_CLASS lcdbuf[LCD_SIZE+1];

char tz = TIME_ZONE;	
uchar gpsoffset;// = GPS_OFFSET;

/* ===== External variables ===== */
// extdebug.c
extern code uchar ALARM_MSG[NUM_ALARMS][];	

// iconv.c
//extern uchar lzb;	

//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------

void Port_Init( void );                 // Port initialization routine
void Timer2_Init( void );               // Timer2 initialization routine
void UART0_Init( void );
void WaitTicks( uint );
void ProcessRxMsg( void );
void PrimaryTiming( uchar TXRX_STORAGE_CLASS * );
void SupplementalTiming( uchar TXRX_STORAGE_CLASS *RxBuf );
void UnsignedToAscii( uint, uchar TXRX_STORAGE_CLASS *, uchar );
// void IntToAscii( int value, uchar TXRX_STORAGE_CLASS *pbuf, uchar nbdigits );
void TimerStart( uchar timer_num, uint num_ticks );
bit TimerReady( uchar timer_num );
bit TimerRunning( uchar timer_num );
void clr_LCDBuf( void );

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

//sfr16 TMR2RL = 0xCA;                   // Timer2 Reload Register
//sfr16 TMR2 = 0xCC;                     // Timer2 Register

uint ticks = 0;

static uchar TXRX_STORAGE_CLASS *p_TxRx_Buf;
uchar TXRX_STORAGE_CLASS TxRxBuf[TXRX_BUF_LEN];

/*-----------------------------------------------------------------------------
* extern variables
*-----------------------------------------------------------------------------*/

//-----------------------------------------------------------------------------
// main() Routine
//-----------------------------------------------------------------------------

void main( void ){

	PCA0MD &= ~0x40;					// Clear watchdog timer enable

	Timer2_Init();						// Initialize the Timer2
	Port_Init();						// Init Ports
	UART0_Init();
	EA = TRUE;							// Enable global interrupts
	LED = 0;							// turn ON LED
	//LED2 = 1;							// turn ON LED2
	LED1 = 1;							// turn ON LED1
	TimerStart( SYS_TIMER, 500 );		// wait for display to wake up
	while( !TimerReady( SYS_TIMER ))
		;
	InitLCD();							// Init LCD Controller
	ClearLCD( 3 );						// Clear display
	printCode2LCD( 1, VersionMsg, 0 );
	printCode2LCD( 2, LCDInitMsg, 0 );
	
	TimerStart( SYS_TIMER, 1500 );
	while( !TimerReady( SYS_TIMER ))
		;
	ClearLCD( 3 );

	if( OFFSET_SELECT == 1 ){
		strcpy( lcdbuf, "Time Zone : " );
		CharToAscii( tz, lcdbuf+12, 3 );
		printLCD( 1, lcdbuf, 0 );
		strcpy( lcdbuf, "GPS offset: Auto" );
		//CharToAscii( gpsoffset, lcdbuf+12, 3 );
		printLCD( 2, lcdbuf, 0 );
		TimerStart( SYS_TIMER, 1500 );
		while( !TimerReady( SYS_TIMER ))
			;
		ClearLCD( 3 );
		//TimerStart( SYS_TIMER, 500 );		// wait for display to wake up
		//while( !TimerReady( SYS_TIMER ))
		//	;
		clr_LCDBuf();
	}


	//LED2 = 0;							// turn OFF LED2
	LED1 = 0;							// turn OFF LED1
	func2 = FALSE;
	func1 = FALSE;
	err1 = FALSE;

	printCode2LCD( 1, "Waiting for GPS", 0 );
	TimerStart( SYS_TIMER, 1500 );
	while( !TimerReady( SYS_TIMER ))
		;	
	LED = 1;							// turn OFF LED
	TimerStart( SWITCH_TIMER, 50 );		// switch timer

	while( 1 ){							// Loop forever
		if( Rx_Pending ){
			TimerReset( SYS_TIMER );
			ProcessRxMsg();
			TimerStart( SYS_TIMER, 1200 );
			err1 = FALSE;
		}
		if( TimerReady( SYS_TIMER )){
			if( !err1 ){
				TimerStart( SYS_TIMER, 1200 );
				printCode2LCD( 1, "   No Message   ", 0 );
//				printCode2LCD( 2, "  Check GPS RX  ", 0 );
				err1 = TRUE;
			}else{
				TimerStart( SYS_TIMER, 500 );
				ClearLCD( 3 );
				err1 = FALSE;
			}
		}

		if( TimerReady( SWITCH_TIMER )){
			switch1 = 1;
			switch2 = 1;
			TimerReset( SWITCH_TIMER );
		}
   	}

}  // main()

//-----------------------------------------------------------------------------
// Initialization Subroutines
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Port_Init
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// This function configures the crossbar and GPIO ports.
//
// Pinout:
//
//	P0.0 -> VRef (not used)
//	P0.1 -> /INT0 -> switch 1 (active low)
//	P0.2 -> /INT1 -> switch 2 (active low)
//	P0.3 -> LED 1 (active high)
//	P0.4 -> Tx (not used)
//	P0.5 -> Rx
//  P0.6 -> input (GPS offset select)
//	//P0.7 -> LED 2 (active high)
//  P0.7 -> input (DST select)
//
//	P1.x -> LCD Display
//
//-----------------------------------------------------------------------------
void Port_Init( void ){

	P0SKIP    = 0x01;					// P0.0 -> Vref
//    P0MDOUT = 0x98;						// Tx, LED2, LED1 are push-pull
    P0MDOUT = 0x18;						// Tx, LED1 are push-pull
	XBR0 = 0x01;						// enable UART
	XBR1 = 0x40;                        // Enable crossbar
	//P1MDOUT = 0x08;                     // Set LED2 to push-pull
	P1MDOUT = 0x7F;						// make LCD_PORT push-pull (except busy flag) v021

	// set PO.1 to be /INT0 and P0.2 to be /INT1
	IE        |= 0x05;		// enable /INT0 and /INT1
	IT01CF    = 0x21;		// /INT0 and /INT1 active low, P0.1 and P0.2 respectively
	// for /INT0 Interrupt Active low, edge sensitive,
	// set IT0 = 1	and IN0PL = 0		
	// for /INT1 Interrupt Active low, edge sensitive,
	// set IT1 = 1	and IN1PL = 0	
	// IN0PL and IN1PL are in IT01CF
	// IT0 and IT1 are in TCON
	IT0 = 1;	// /INT0 edge triggered
	IT1 = 1;	// /INT1 edge triggered

	

}  // Port_Init()

//-----------------------------------------------------------------------------
// Timer2_Init
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// This function configures Timer2 as a 16-bit reload timer, interrupt enabled.
// Using the SYSCLK at 16MHz/8 with a 1:12 prescaler.
//
// Note: The Timer2 uses a 1:12 prescaler.  If this setting changes, the
// TIMER_PRESCALER constant must also be changed.
//-----------------------------------------------------------------------------
void Timer2_Init( void ){

	uchar i;
	// I think this should be 0x30 DAK
	CKCON &= ~0x60;                     // Timer2 uses SYSCLK/12
	TMR2CN &= ~0x01;					// Set bit 0 low, select SYSCLK/12

	TMR2RL = TIMER2_RELOAD;             // Reload value to be used in Timer2
	TMR2 = TMR2RL;                      // Init the Timer2 register

	TMR2CN = 0x04;                      // Enable Timer2 in auto-reload mode
	ET2 = 1;                            // Timer2 interrupt enabled

	for( i = 0; i < NUM_TIMERS; i++ ){	// Initialize software timers.
		Timer[i].Status = 0;
		Timer[i].WaitTime = 0;
	}

}  // Timer2_Init()


//-----------------------------------------------------------------------------
// Interrupt Service Routines
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Timer2_ISR
//-----------------------------------------------------------------------------
//
// Here we process the Timer2 interrupt and toggle the LED
//
//-----------------------------------------------------------------------------
void Timer2_ISR( void ) interrupt 5{

	uchar i;

//	LED1 = !LED1;                         	// Toggle the LED
	TF2H = 0;                           	// Reset Interrupt
	if( ticks > 0 )
		ticks--;
	for( i = 0; i < NUM_TIMERS; i++ ){		// Cycle through all timers, to update.
		if( Timer[i].WaitTime > 0 ){		// Is it Expired?
			Timer[i].WaitTime--;			// No: Count it down.
			Timer[i].Status = TIMER_STATUS_RUNNING;
		}else{								// Yes: Flag it as 'ready'.
			Timer[i].Status = TIMER_STATUS_READY;
		}
	}										// No: Skip it.

}  // Timer2_ISR()

/*---------------------------------------------------------------------*
* FUNCTION: void WaitTicks( uint delay );
*
*This function waits for 'delay' mS to pass.
*----------------------------------------------------------------------*/
void WaitTicks( uint delay){

	EA = FALSE;
	ticks = delay;
	EA = TRUE;
	while( ticks > 0 )
		;

}  // WaitTicks()

/*---------------------------------------------------------------------*
* FUNCTION: TimerStart()
*
*	Arguments: 	timer_num = Timer Number
*		num_ticks = number of 1mS clock ticks
*
*	Initiates a timer function.  The time is in system time base units
*	( 1 ms ).  The timer is enabled and will start counting down
*	at the next time base interrupt.
*---------------------------------------------------------------------*/
void TimerStart( uchar timer_num, uint num_ticks ){

	INT_DISABLE;

	Timer[timer_num].WaitTime	= num_ticks;
	Timer[timer_num].Status	= TIMER_STATUS_RUNNING;

	INT_ENABLE;

}   // end TimerStart()

/*---------------------------------------------------------------------*
*  FUNCTION: TimerRunning()
*
*---------------------------------------------------------------------*/
bit TimerRunning( uchar timer_num ){

	return( (Timer[timer_num].Status == TIMER_STATUS_RUNNING)?TRUE:FALSE );

}  // TimerRunning()

/*---------------------------------------------------------------------*
*  FUNCTION: TimerReset()
*
*---------------------------------------------------------------------*/
void TimerReset( uchar timer_num ){

	INT_DISABLE;
    Timer[timer_num].WaitTime	= 0;
    Timer[timer_num].Status	= TIMER_STATUS_STOPPED;
	INT_ENABLE;

} // end TimerReset()

/*---------------------------------------------------------------------*
*  FUNCTION:  TimerReady()
*---------------------------------------------------------------------*/
bit TimerReady( uchar timer_num ){

    if( Timer[timer_num].Status == TIMER_STATUS_READY )
		return( TRUE );
    else
		return( FALSE );

} // end TimerReady()

// DAK disable unreachable code warning for this section
#pragma save
#pragma disable_warning 126
//-----------------------------------------------------------------------------
// UART0_Init
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// Configure the UART1 using Timer1, for <baudrate> and 8-N-1.
//
//-----------------------------------------------------------------------------
void UART0_Init( void ){

   SCON0 = 0x10;                       // SCON0: 8-bit variable bit rate
                                       //        level of STOP bit is ignored
                                       //        RX enabled

                                       //        ninth bits are zeros
                                       //        clear RI0 and TI0 bits



   if( SYSCLK/BAUDRATE/2/256 < 1 ){
      TH1 =  (unsigned char) -(SYSCLK/BAUDRATE/2);
      CKCON &= ~0x0B;                  // T1M = 1; SCA1:0 = xx
      CKCON |=  0x08; 
   }else if( SYSCLK/BAUDRATE/2/256 < 4 ){
      TH1 = (unsigned char) -(SYSCLK/BAUDRATE/2/4);
      CKCON &= ~0x0B;                  // T1M = 0; SCA1:0 = 01
      CKCON |=  0x09;
   }else if( SYSCLK/BAUDRATE/2/256 < 12 ){
      TH1 = (unsigned char) -(SYSCLK/BAUDRATE/2/12);
      CKCON &= ~0x0B;                  // T1M = 0; SCA1:0 = 00
   }else{
      TH1 = (unsigned char) -(SYSCLK/BAUDRATE/2/48);
      CKCON &= ~0x0B;                  // T1M = 0; SCA1:0 = 10
      CKCON |=  0x02; 
   }


	TL1 = TH1;                          // Init Timer1
	TMOD &= ~0xf0;                      // TMOD: timer 1 in 8-bit autoreload
	TMOD |=  0x20;
	TR1 = 1;                            // START Timer1
	TI0 = 1;                            // Indicate TX0 ready

	/* from Configurer
    TMOD      = 0x20;
    CKCON     = 0x08;
    TH1       = 0x60;
	*/

   	RI0 = FALSE;		// turn off any pending receive interrupt
	REN0 = TRUE;		// UART0 receive enable
	ES0 = TRUE;         // UART0 interrupt enable

}  // UART0_Init()

#pragma restore
/*---------------------------------------------------------------------*
*  Serial Port Interrupt Service
*----------------------------------------------------------------------*/
void SioIntService( void ) interrupt INTERRUPT_UART0 using SIO_REG_BANK {

	uchar tchar;
	static bit bEvenDLE;
	

	// Receiver Section
	if( RI0 ){
		tchar = SBUF0;	// & PARITY_MASK;
		RI0 = FALSE;

		// Prohibit Buffer Overrun
		if( TxRx_Count > TXRX_BUF_LEN - 1 ) {
			TxRx_State = WAIT_FOR_START;
			TxRx_Count = 0;
			return;
		}

		switch( TxRx_State ){
			case WAIT_FOR_START:
				// actually this waits for an end-of-message sequence
				// Phase Receiver with char-DLE-ETX
				if( tchar == DLE ){
					TxRx_Count = 0;
					TxRxBuf[TxRx_Count++] = tchar;
					TxRx_State = WAIT_FOR_ID;
				}else if( tchar != ETX ){
					TxRx_State = WAIT_FOR_DLE_ETX;
					Rx_Pending = FALSE;
					TxRx_Count = 0;
				}
				bEvenDLE = FALSE;
				break;

			case WAIT_FOR_DLE_ETX:
				if( tchar == DLE )
					TxRx_State = WAIT_FOR_ETX;
				else
					TxRx_State = WAIT_FOR_START;
				break;

			case WAIT_FOR_ETX:  // rarely happens
				if( tchar == ETX )	
					// found end of a message
					TxRx_State = WAIT_FOR_DLE;
				else
					TxRx_State = WAIT_FOR_START;
				break;

			case WAIT_FOR_DLE:
				if( tchar == DLE ){
					TxRx_Count = 0;
					TxRxBuf[TxRx_Count++] = tchar;
					TxRx_State = WAIT_FOR_ID;
				}else
					TxRx_State = WAIT_FOR_START;
				bEvenDLE = FALSE;
				break;

			case WAIT_FOR_ID: // never happens
				if( tchar == DLE || tchar == ETX ){
					TxRx_State = WAIT_FOR_START;
					TxRx_Count = 0;
				}else{
					TxRx_State = WAIT_FOR_END_MSG;
					TxRxBuf[TxRx_Count++] = tchar;
				}
				break;

			case WAIT_FOR_END_MSG:
				if( tchar == DLE && TxRxBuf[TxRx_Count-1] == DLE && !bEvenDLE ){
					// byte stuffing
					bEvenDLE = TRUE;
				}else if( tchar == ETX && TxRxBuf[TxRx_Count-1] == DLE && !bEvenDLE ){
					// complete message received, in buffer
					//P0_6 = 0;	// 7
					Rx_Pending = TRUE;
					TxRxBuf[TxRx_Count] = '\0';
					TxRx_State = WAIT_FOR_DLE;

					//P0_6 = 1;
				}else{
					TxRxBuf[TxRx_Count++] = tchar;
					bEvenDLE = FALSE;
				}
				break;

			default:
				TxRx_State = WAIT_FOR_START;
				break;
		}

	}else{
        // Transmitter Section
        if( TI0 ){
			TI0 = FALSE;

            if( !Tx_In_Progress ){
                RI0 = FALSE;
                Tx_In_Progress = FALSE;
                Rx_Pending = FALSE;
				return;
			}
			switch( TxRx_State ){

				case SEND_HDR:
					SBUF0 = DLE;
					TxRx_State = SEND_MSG;
					TxRx_Count--;
					p_TxRx_Buf = TxRxBuf+1;
					bEvenDLE = FALSE;
					break;

				case SEND_MSG:
		            tchar = *p_TxRx_Buf;
					SBUF0 = tchar;
					if( tchar == DLE && !bEvenDLE && TxRx_Count > 1 ){
						// do DLE byte stuffing
						bEvenDLE = TRUE;
						return;
					}
					p_TxRx_Buf++;
					bEvenDLE = FALSE;
					if( TxRx_Count-- == 0 ){
		                Tx_In_Progress = FALSE;
		                TxRx_Count = 0;
		            }
					break;

				default:
					break;
            }
        }	// end if( TI )
    }	// end if( ! RI )

    return;


} // end SioIntService()

/*-----------------------------------------------------------------------------*
*	Function: ProcessRxMsg()
*------------------------------------------------------------------------------*/
void ProcessRxMsg( void ){

	uchar TXRX_STORAGE_CLASS pBuf[TXRX_BUF_LEN];
	uchar i;
	uchar id, id2;

	id = TxRxBuf[IO_BUF_ID_INDEX];
	id2 = TxRxBuf[IO_BUF_ID2_INDEX];
	for( i=2; i<TxRx_Count; i++)
		pBuf[i-2] = TxRxBuf[i];
	Rx_Pending = FALSE;

	switch( id ){
		case SUPERPACKET:
			switch( id2 ){
				case PRIMARY_TIMING_PCKT:
					PrimaryTiming( pBuf );
					break;

				case SUPPLEMENTAL_TIMING_PCKT:
					SupplementalTiming( pBuf );
					break;

				default:
					break;
			}
		default:
			break;
	}

}  // ProcessRxMsg()

/*-----------------------------------------------------------------------------*
* Function: PrimaryTiming()
*
*	Display time and date on top line of the display
*------------------------------------------------------------------------------*/
void PrimaryTiming( uchar TXRX_STORAGE_CLASS *RxBuf ){

	char sec, min, hr, dom, mo, y;
	UINTType yr;
	//int utcoffset;
	char ctz;

	ctz = tz;

	yr.b.hi = *(RxBuf+7);
	yr.b.lo = *(RxBuf+8);
	gpsoffset = yr.i;

	// decode date/time from packet
	sec = *(RxBuf+10);

	min = *(RxBuf+11);

	hr = *(RxBuf+12);

	dom = *(RxBuf+13);

	mo = (*(RxBuf+14)-1)*3;	// note: mo starts at 0 (January)

	yr.b.hi = *(RxBuf+15);
	yr.b.lo = *(RxBuf+16);

	y = yr.u - 2000;

	// if apply GPS offset...
	if( OFFSET_SELECT == 0 ){
		// compute effect of GPS offset
		sec -= gpsoffset;
		if( sec < 0 ){
			sec += 60;
			min--;
			if( min < 0 ){
				min += 60;
				hr--;
				if( hr < 0 ){
					hr += 24;
					ctz--;
				}
			}
		}

		// compute for time zone
		if( ctz > 0 ){
			hr += ctz;
			if( DST_SELECT == 0 )
				hr++;
			if( hr > 23 ){
				hr -= 24;
				dom++;
				if( (uchar)dom > DIM[mo] ){
					// check for leap year ***TODO
					dom = 0;
					mo++;
					if( mo > 11 ){
						mo = 0;
						y++;
					}
				}
			}
		}
		if( ctz < 0 ){
			hr += ctz;
			if( DST_SELECT == 0 )
				hr++;
			if( hr < 0 ){
				hr += 24;
				dom--;
				if( dom < 0 ){
					mo--;
					if( mo < 0 ){
						mo = 12;
						y--;
					}
					dom = DIM[mo];
					// check for leap year ***TODO
				}
			}
		}
	}

	// display on LCD
	UnsignedToAscii( (uint)sec, lcdbuf+6, 2 );
	if( lcdbuf[6] == ' ' ) lcdbuf[6] = '0';
	lcdbuf[8] = ' ';

	UnsignedToAscii( (uint)min, lcdbuf+3, 2 );
	if( lcdbuf[3] == ' ' ) lcdbuf[3] = '0';
	lcdbuf[5] = ':';

	UnsignedToAscii( (uint)hr, lcdbuf, 2 );
	if( lcdbuf[0] == ' ' ) lcdbuf[0] = '0';
	lcdbuf[2] = ':';

	// Display day-of-month
	UnsignedToAscii( (uint)dom, lcdbuf+9, 2 );
	// DAK add leading zero to date 
	if( lcdbuf[9] == ' ' ) lcdbuf[9] = '0';

	// Display month
	lcdbuf[11] = Month[mo];
	lcdbuf[12] = Month[mo+1];
	lcdbuf[13] = Month[mo+2];

	// Display year
	UnsignedToAscii( (uint)y, lcdbuf+14, 2 );
	if( lcdbuf[14] == ' ' ) lcdbuf[14] = '0';
	lcdbuf[16] = '\0';

	if( OFFSET_SELECT == 1 ){ 
		lcdbuf[16] = ' ';
		lcdbuf[17] = 'G';
		lcdbuf[18] = 'P';
		lcdbuf[19] = 'S';
		lcdbuf[20] = '\0';
	}else{
		lcdbuf[16] = ' ';
		lcdbuf[17] = 'C';
		if( DST_SELECT == 0 )
			lcdbuf[18] = 'D';
		else
			lcdbuf[18] = 'S';
		lcdbuf[19] = 'T';
		lcdbuf[20] = '\0';
	}

//	printLCD( 1, lcdbuf, 0 );

	//offset.b.hi = *(RxBuf+7);
	//offset.b.lo = *(RxBuf+8);
	//UnsignedToAscii( (uint)RxBuf[8], lcdbuf, 2 );
	//printLCD( 2, lcdbuf, 0 );


}  // PrimaryTiming()

/*-----------------------------------------------------------------------------*
* Function: SupplementalTiming()
*
*	Diay various GPS receiver status info on second line of display
*-----------------------------------------------------------------------------*/
void SupplementalTiming( uchar TXRX_STORAGE_CLASS *RxBuf ){
//#define TEST_EXTDEBUG

#define NUM_DISPLAY_MODES		6

	static bit b = FALSE;
	static bit c = FALSE;

	FLOATType temp;
	uchar val;
	uint fval;
	//char dac;
	//char tempbuf[17];
	static char mode = -1;

#ifdef TEST_EXTDEBUG
	if( Alarms.u == 0 )
		Alarms.u = 1;
	else
		Alarms.u = Alarms.u<<1;
#else

	printLCD( 1, lcdbuf, 0 );	// print 1st line

	// Critical alarms are lower 5 bits of byte 8-9 (so they are in Byte 9)
	// Minor alarms are lower 9 bits of byte 10-11 (lower bits are in byte 11)
	// Load Minor Alarms
	Alarms.b.hi = RxBuf[10];
 	Alarms.b.lo = RxBuf[11];
	// shift them up by 5 to make room for Critical Alarms
	Alarms.u = Alarms.u<<5;
	// OR with Critical Alarms
	Alarms.b.lo |= RxBuf[9];
#endif

	c = !c;  // only change display every 2nd time thru ( about every 2 seconds)
	if( c )
		return;

	ClearLCD( 2 );
	if( Alarms.u != 0 && mode == NUM_DISPLAY_MODES - 1 ){
		b = Fault_Msg_Query( Alarms.u, lcdbuf, ALARM_MSG[0] );
		if( b )
			printLCD( 2, lcdbuf, 0 ); 
		if( RxBuf[9] != 0 ) // if critical alarm, keep showing it
			return;        
	}

	if( !b ){
		if( ++mode >= NUM_DISPLAY_MODES )
			mode = 0;

		val = 28;
		fval = 10000;
		switch( mode ){
			case 0:	// disciplining mode
				strcpy( lcdbuf , DiscMode[RxBuf[2]]);
				break;

			case 1:	// Discipling Activity
				strcpy( lcdbuf , DiscActivity[RxBuf[13]]);
				break;

			case 2:	// Receiver mode
				strcpy( lcdbuf , RxMode[RxBuf[1]]);
				break;

			case 3:	// GPS Decode Status
				strcpy( lcdbuf , GPSDecodeStatus[RxBuf[12]]);
				break;

			case 4:	// RxBuf[32-35] is temperature (float)
				val = 32;
				fval = 100;
				// intentionally falls through (no break)

			case 5:	// RxBuf[28-31] is DAC Voltage (float)
				temp.b.hhi = RxBuf[val];
				temp.b.hi = RxBuf[val+1];
				temp.b.lo = RxBuf[val+2];
				temp.b.llo = RxBuf[val+3];

				val = temp.f;
				fval = ((temp.f - val) * fval);

				// ***TEST
				//val = 249;
				//fval = 209;

				strcpy( lcdbuf, prompt[mode-4] );
				UnsignedToAscii( val, lcdbuf+6, 3 );
				lcdbuf[9] = '.';
				UnsignedToAscii( fval, lcdbuf+10, 2 + 2*(mode-4) );
				// unblank leading zeros on fractional part
				for( val=0; val<3; val++ ){
					if( lcdbuf[10+val] == ' ' ) 
						lcdbuf[10+val] = '0';
					else
						break;
				}
				break;

		} // switch( mode )
		printLCD( 2, lcdbuf, 0 );
	}

}  //  SupplementalTiming()

/*----------------------------------------------------------------------*
* FUNCTION: clr_LCDBuf()
*----------------------------------------------------------------------*/
void clr_LCDBuf( void ){

	uchar i;

	for( i=0; i<LCD_SIZE; i++ )
		lcdbuf[i] = ' ';
	lcdbuf[i] = '\0';

} // clr_LCDBuf()


/*-----------------------------------------------------------------------------*
* Function: External_Interrupt_0()
*
*-----------------------------------------------------------------------------*/
void External_Interrupt_0( void ) interrupt 0{

	// SW2 is active low
	if( switch2 != 0 ){
		// switch just pressed
		switch2 = 0;
		//func2 = !LED2;
		//LED2 = func2;
		TimerStart( SWITCH_TIMER, 50 );	// to reset switch2 to 1
	}

}  //  External_Interrupt_0()

/*-----------------------------------------------------------------------------*
* Function: External_Interrupt_1()
*
*-----------------------------------------------------------------------------*/
void External_Interrupt_1( void ) interrupt 2{

	// SW1 is active low
	if( switch1 != 0 && !TimerRunning( SWITCH_TIMER )){
		// switch just pressed
		switch1 = 0;
		func1 = TRUE;

		TimerStart( SWITCH_TIMER, 150 );	// to reset switch1 to 1
	}

}  //  External_Interrupt_1()

/* ----- unused F330 interrupts ----- */

//Reset 0x0000 Top 
//void External_Interrupt_0( void ) interrupt 0{}
//void Timer0_ISR( void ) interrupt 1{}	// Timer0 used for baud rate
//void External_Interrupt_1( void ) interrupt 2{}
void Timer_1_Overflow( void ) interrupt 3{}
//void SioIntService( void ) interrupt 4{}
//void Timer_2_Overflow( void ) interrupt 5{}
void SPI0( void ) interrupt 6{}
void SMB0( void ) interrupt 7{}
// reserved 8
void ADC0_Window_Comparator( void ) interrupt 9{}
void ADC0_End_of_Conversion( void ) interrupt 10{}
void Programmable_Counter_Array( void ) interrupt 11{}
void Comparator( void ) interrupt 12{}
// reserved 13
void Timer_3_Overflow( void ) interrupt 14{}

//-----------------------------------------------------------------------------
// End Of File
//-----------------------------------------------------------------------------
