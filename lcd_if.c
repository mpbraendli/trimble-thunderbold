/*---------------------------------------------------------------------*
*  LCD_IF.C		LCD interface routines
*
*	Designed for a Noritake CU16025ECBP-U5J Vacuum Fluorescent Display (VFD), 
*	but generally compatible with the Hitachi H44780 LCD display controller 
*	interface (some timing may need to be adjusted).
*
*	A compile option is available to accomodate using either lower nibble
*	of LCD_PORT to drive the data bus (#define USE_LOWER_BITS), or the upper 
*	nibble (#define USE_UPPER_BITS)
*
*	If USE_UPPER_BITS is defined, the display is wired as follows:
*	- DB4-7 -> P0.4-7 (DB7 = Busy Flag)
*	- E     -> P0.3 (can be changed)
*	- R/W   -> P0.2 (can be changed)
*	- RS    -> P0.1 (can be changed)
*	(P0.0 available)
*
*	If USE_LOWER_BITS is defined, the display is wired as follows:
*	- DB4-7 -> P0.0-3 (DB3 = Busy Flag)
*	- E     -> P0.4 (can be changed)
*	- R/W   -> P0.5 (can be changed)
*	- RS    -> P0.6 (can be changed)
*	(P0.7 available)
*
*	Make sure LCD_PORT is configured as pull-up (not push-pull) for normal 
*	8051 chips (SiLabs for instance).
*
*	For Phillips and TI chips which have data direction PxDDRL and PxDDRH
*	registers, use appropriate compile option (#define PHILLIPS) and read 
*	comments in InitLCD()
*
*	For chips with normal open collector and internal pull-up on their IO port, 
*	make sure the internal pull-ups are sufficient to guarantee clean pulses, 
*	particularly on the LCD_EN. An external pull-up resistor may be required
*	(~3.3kohm @ 3.3V, ~4.7kohm @ 5V), particularly with the Noritake VFDs.
*	If you use the BUSY_FLAG, the LCD_RW and BUSY_FLAG pins should also have 
*	external pull-ups 
*
*	For Noritake Vacuum Fluorescent Displays, the following timing applies:
*		* wait 260mS after power up (Vcc > 4.75) before addressing the display.
*		* Display Clear instruction takes 2.3mS max.
*		* The last character written is briefly displayed again in the new cursor 
*		  position during the high period of the E signal. 
*		  This is not visible on an LCD, but the brightness of the VFD makes it 
*		  visible sometimes. 
*		  To limit problems, the high period of the E signal should be kept as 
*		  short as possible (5uS or less).
*
*	These functions take advantage of the BUSY_FLAG, which makes then pretty
*	fast, but require to actively control the state of the LCD_RW pin and also
*	require to be able to read the BUSY_FLAG, meaning the output port must be
*	periodically changed from output to input. This is easier with some chips 
*	than others.
*
*	On a Silabs C8051F530 running at 25 MHz, it takes ~140uS to print 5 characters.
*
*	Notes found on the web:
*
*		Reading data back is best used in applications which require data to be 
*		moved back and forth on the LCD (such as in applications which scroll data 
*		between lines). The "Busy Flag" can be polled to determine when the last 
*		instruction that has been sent has completed processing. 
*		In most applications, it's enough to just tie the "R/W" line to ground 
*		because there is no need to read anything back. 
*		This simplifies the application because when data is read back, the 
*		microcontroller I/O pins have to be alternated between input 
*		and output modes.
*
*		For most applications when there really is no reason to read from the LCD, 
*		tie "R/W" to ground and just wait the maximum amount of time 
*		for each instruction (4.1 msecs for clearing the display or moving the 
*		cursor/display to the "home position", 160 usecs for all other commands). 
*		As well as making application software simpler, it also frees up a 
*		microcontroller pin for other uses. Different LCDs execute instructions at 
*		different rates and to avoid problems later on (such as if the LCD is 
*		changed to a slower unit), simply use the maximum delays given above. 
*
*  Copyright 2008 Didier Juges 
*  http://www.ko4bb.com
*
*  This program is free software; you can redistribute it and/or
*  modify it under the terms of the GNU General Public License
*  as published by the Free Software Foundation; either version 2
*  of the License, or (at your option) any later version.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with this program; if not, write to the 
*  Free Software Foundation, Inc., 
*  51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*
*----------------------------------------------------------------------*/

#include "general.h"
#include "timer.h"
#include "lcd_if.h"

/* ----- User Changable constants ----- */
#define LCD_RS				P1_0
#define LCD_RW				P1_1
#define LCD_EN				P1_2	// normal state is low
#define LCD_PORT			P1
#define BUSY_FLAG			P1_7 // this is DB7 of the LCD controller bus
// #define USE_UPPER_BITS or USE_LOWER_BITS of LCD_PORT for 4 bits mode
#define USE_UPPER_BITS
//#define USE_LOWER_BITS

/* end of User Changable constants ----- */


#if defined USE_UPPER_BITS
#define LCD_MASK			0x0f	// set bits used for LCD data bus 
									// to 0, others to 1 (to set as input)
#define LCD_INIT_1			0x30
#define LCD_INIT_2			0x20
#else
#define LCD_MASK			0xf0
#define LCD_INIT_1			0x03
#define LCD_INIT_2			0x02
#endif

#define INSTR_REG			FALSE
#define DATA_REG			TRUE

/* ----- global variables ----- */



/* ----- local variables ----- */


/*---------------------------------------------------------------------*
* Local function prototypes (public prototypes are in lcd_if.h)
*----------------------------------------------------------------------*/
bit WriteLCD( bit reg, uchar out );
void IRWriteLCD( uchar);
void DRWriteLCD( uchar);
void IRWriteLCD8( uchar);
bit WaitLCD( void );
void SetLCDIn( void );
void SetLCDOut( void );
void ClockEn( void );
void SetLCDRs( void );
void ClrLCDRs( void );
void SetLCDRw( void );
void ClrLCDRw( void );



/*=====================================================================*
* Public Functions
*======================================================================*/

/*---------------------------------------------------------------------*
* FUNCTION: InitLCD()
*
*  initialize LCD module per specs
*
* NOTE: the Noritake display needs at least 260mS after Vcc rise > 4.75V
*----------------------------------------------------------------------*/
void InitLCD( void ){

	// 1st part of initialization is in 8 bit mode, but only 
	// the upper bits matter.
	
	// For PHILLIPS chips (with PxDDRx registers) these should be
	// configured as outputs prior to calling this function, or
	// add the Port init code here

    WaitTicks( 15 );        // Wait more than 15ms
    LCD_PORT = LCD_INIT_1;  // Startup Sequence
    ClockEn();				// raise E, wait >100uS, lower E
    WaitTicks( 5 );         // Wait more than 4.1ms
    ClockEn();  
    WaitTicks( 1 );         // Wait more than 0.1ms
    ClockEn();    
    LCD_PORT = LCD_INIT_2;
    ClockEn();
    WaitTicks( 1 );         // Wait more than 0.1ms

	// LCD is now initialized, rest of setup is in 4 bits mode

    IRWriteLCD( 0x28 );   	// Function Set
                            // DL=0 4bit, N=1 2Line, F=0 5x7
    IRWriteLCD( 0x0C );   	// Display on/off control
                            // D=1 Disp on, C=0 Curs off, B=0 Blink off    
	IRWriteLCD( 0x06 );   	// Entry Mode Set
                            // I/D=1 Increment, S=0 No shift
    IRWriteLCD( 0x01 );   	// Clear Display
    WaitTicks( 2 );         // Wait more than 1.64ms

} // end InitLCD()

/*---------------------------------------------------------------------*
* FUNCTION: ClearLCD()
*
* 	to clear line 1: ClearLCD( 1 );
*	to clear line 2: ClearLCD( 2 );
*	to clear both lines: ClearLCD( 3 );
*---------------------------------------------------------------------*/
void ClearLCD( uchar line ){

	uchar i=0;

	if( line == 3 ){
		// clear by instruction (fast)
		IRWriteLCD( 0x01 );
		// wait 4.1 mS
		WaitTicks( 10 );
	}else{	// 1 line at a time
		// Position the cursor
		if( line & 1 ){
//			WaitLCD();
			IRWriteLCD( 0x80 );
			while( i < LCD_SIZE ){
				WaitLCD();
				DRWriteLCD( ' ' );
				i++;
			}
		}
		if( line & 2 ){
//			WaitLCD();
			IRWriteLCD( 0xC0 );
			i = 0;
			while( i < LCD_SIZE ){
				WaitLCD();
				DRWriteLCD( ' ' );
				i++;
			}
		}
	}
} // end ClearLCD()

/*---------------------------------------------------------------------*
* FUNCTION: printLCD()
*
*	line: 1 or 2 (for 2 line display)
*	*chr_ptr: pointer to string to print in data space, NULL terminated
*	offset: offset from left of display where to print string
*			(characters skipped are not blanked, allowing to refresh
*			a portion of a line without affecting the rest)
*---------------------------------------------------------------------*/
void printLCD( uchar line, uchar LCD_STORAGE_CLASS *chr_ptr, uchar offset ){

	// position the cursor
	if( line == 1 )
		IRWriteLCD( 0x80 + offset );	// address of left-most character to print
	else
		IRWriteLCD( 0xC0 + offset );

	// Write out the data
	chr_ptr--;
	while( *(++chr_ptr) != '\0' ){
		WaitLCD();
		DRWriteLCD( *chr_ptr );
	}
//	IRWriteLCD( 0x0C );					// turn display on ***TEST 2-Aug-08

} // end printLCD()

/*---------------------------------------------------------------------*
* FUNCTION: printCode2LCD()
*
*	Same as printLCD except that pointer to code space
*---------------------------------------------------------------------*/
void printCode2LCD( uchar line, uchar code *chr_ptr, uchar offset ){

	// position the cursor
	if( line == 1 )
		IRWriteLCD( 0x80 + offset );
	else
		IRWriteLCD( 0xC0 + offset );

	// Write out the data
	//chr_ptr--;
	do{
		WaitLCD();
		DRWriteLCD( *chr_ptr++ );
	}while( *chr_ptr != '\0' );

} // end printCode2LCD()



/*======================================================================*
* Private Functions
*======================================================================*/



/*---------------------------------------------------------------------*
* FUNCTION: IRWriteLCD()
*
*  Writes LCD Instruction ( 4-bits mode, MSB first )
*---------------------------------------------------------------------*/
void IRWriteLCD( uchar outbyte){

	uchar i;

	i = outbyte;                // save for later

	SetLCDOut();                // Configs Port bits to output
	i = i & 0xf0;               // Strip low nibble
#if defined USE_LOWER_BITS
	i = i/16;					// Shift high to low
#endif
	LCD_PORT = i;               // Write high nibble
	ClrLCDRs();                 // Instruction
	ClrLCDRw();                 // Write
	ClockEn();                	// Trigger

	i = outbyte;                // Later is now!

	i = (i & 0x0f);             // Strip high nibble
#if defined USE_UPPER_BITS
	i = i*16;					// Shift low nibble to high
#endif
	LCD_PORT = i;               // Write low nibble
	ClrLCDRs();                 // Instruction
	ClrLCDRw();                 // Write
	ClockEn();

} // end IRWriteLCD()

/*---------------------------------------------------------------------*
* FUNCTION: DRWriteLCD()
*
*  Writes LCD Data ( 4-bits mode, MSB first )
*---------------------------------------------------------------------*/
void DRWriteLCD( uchar outbyte){

	uchar i;

	i = outbyte;                // preserve outbyte for later

	SetLCDOut();                // Configs Port bits to output
	i = i & 0xf0;               // Strip low nibble
#if defined USE_LOWER_BITS
	i = i/16;					// Shift high to low
#endif
	LCD_PORT = i;               // Write high nibble
	SetLCDRs();                 // Data
	ClrLCDRw();                 // Write
	ClockEn();

	i = outbyte;                // Later is now!

	i = (i & 0x0f);             // Strip high nibble
#if defined USE_UPPER_BITS
	i = i*16;					// Shift low to high
#endif
	LCD_PORT = i;               // Write high nibble
	SetLCDRs();                 // Data
	ClrLCDRw();                 // Write
	ClockEn();

} // end DRWriteLCD()


/*---------------------------------------------------------------------*
* FUNCTION: WaitLCD()
*
*	Wait for Display to be 'not-busy'
*
*	return value: 	1: BUSY_FLAG = 1 (TRUE) => was busy (problem with display!!!)
*					0: BUSY_FLAG = 0 (FALSE) => not busy
*---------------------------------------------------------------------*/
bit WaitLCD( void ){

	bit bi, bt;
	uchar i;

	//P0_7 = 0;	// ***TEST

	//TimerStart( LCD_TIMER, 10 );	// start a 10 mS timer
//	WaitTicks( 5 );

	bt = FALSE;

	SetLCDIn();		// set busy flag line as input
	ClrLCDRs();		// instruction
	SetLCDRw();		// set display to *talk* mode

	do{
		for( i=0; i<5; i++ )	// wait 1uS
			;
		LCD_EN = 1;
		for( i=0; i<5; i++ )	// wait 1uS
			;
		bi = BUSY_FLAG;
		//bt = IsTimerReady( LCD_TIMER );
		LCD_EN = 0;
	}while( bi ); //&& !bt );     // We wait here until BF goes low

	//P0_7 = 1;	// ***TEST

	ClrLCDRw();		// set display back to *listen* mode (default)

	return( bi );

} // end WaitLCD()

/*---------------------------------------------------------------------*
* FUNCTION: SetLCDIn()
*
*  Configures P0.0 - P0.2 as outputs, P0.3 as Input,
*  P0.4 - 6 as outputs, P0.7 as input
*
*	Note: this function only works with USE_LOWER_BITS
*	(easy to fix, but got lazy...)
*---------------------------------------------------------------------*/
void SetLCDIn( void ){


#if defined PHILLIPS
	BUSY_FLAG = 0;    // We'll drive this fellow low before we switch him to input.
	P0DDRL = 0x11010101;	/* P0.0-2 as CMOS outputs, P0.3 as input */
	P0DDRH = 0x11010101;	/* P0.4-6 as CMOS outputs, P0.7 as input */
#else
	BUSY_FLAG = 1;			// write 1 to make pin an input
#endif

} // end SetLCDIn()

/*---------------------------------------------------------------------*
* FUNCTION: SetLCDOut()
*
*  Configures P0.0 - P0.3 as Outputs
*  P0.4 - P0.6 as outputs, P0.7 as input
*
*	Note: see SetLCDIn()
*---------------------------------------------------------------------*/
void SetLCDOut( void ){

#if defined PHILLIPS
	P0DDRL = 0x01010101;
	P0DDRH = 0x11010101;
#else
	// do nothing
#endif

} // end SetLCDOut()

/*---------------------------------------------------------------------*
* FUNCTION: ClockEn()
*---------------------------------------------------------------------*/
void ClockEn( void ){

	uchar i;

	LCD_EN = 1;
	// delay 100uS minimum for LCD displays, 
	// 5uS maximum for Noritake VFDs to limit character flashing
	for( i=0; i<5; i++ )
		;
	LCD_EN = 0;

} // end ClockEn()

/*---------------------------------------------------------------------*
* FUNCTION: SetLCDRs()
*
*  Sets register select (instruction)
*---------------------------------------------------------------------*/
void SetLCDRs( void ){

	// Reference the instruction register (commands)
	LCD_RS = 1;

} // end SetLCDRs()

/*---------------------------------------------------------------------*
* FUNCTION: ClrLCDRs()
*
*  Clears register select (data)
*---------------------------------------------------------------------*/
void ClrLCDRs( void ){

   // Reference the data register (data)
   LCD_RS = 0;

} // end ClrLCDRs()

/*---------------------------------------------------------------------*
* FUNCTION: SetLCDRw()
*
*  Sets data read
*---------------------------------------------------------------------*/
void SetLCDRw( void ){

   // Data register read (data)
   LCD_RW = 1;

} // end SetLCDRw()

/*---------------------------------------------------------------------*
* FUNCTION: ClrLCDRw()
*
*  Sets data write
*---------------------------------------------------------------------*/
void ClrLCDRw( void ){

   // Data register write (data)
   LCD_RW = 0;

} // end ClrLCDRw()

