/*---------------------------------------------------------------------*
* GENERAL.H
*
* General system definitions for F530 LCD_Demo project
*
*	Timer0 is used as system timer
*	Timer1 is used for baud rate generation for serial port
*	Timer2 is available
*---------------------------------------------------------------------*/

#ifndef _GENERAL_
#define _GENERAL_

#include <C8051F330.h>

/*---------------------------------------------------------------------*
*	Misc definitions
*----------------------------------------------------------------------*/
#ifndef TRUE
#define TRUE 0x01
#endif

#ifndef FALSE
#define FALSE 0x00
#endif

#define ON  0x01
#define OFF 0x00

#define HIGH(x)     		( ( x >> 8 ) & 0x00ff )
#define LOW(x)      		( x & 0x00ff )


#ifndef NULL
#define NULL 0x00
#endif

/*----------------------------------------------------------------------*
* Global system definitions
*----------------------------------------------------------------------*/
//#define SYSCLK      24500000           // SYSCLK frequency in Hz

/* ----- user adjustable parameters ----- */
#define BAUDRATE        9600           // Baud rate of UART in bps

#define NUM_STATUS_BYTE			4

#define TXRX_STORAGE_CLASS	xdata

#define LCD_STORAGE_CLASS	xdata

//#define NUM_RPM_BYTES		6

/* ----- end of user adjustable parameters ----- */

/* ----- do not change those ----- */
#define uint 				unsigned int
#define uchar 				unsigned char
#define cchar 				const char
#define ByteType			unsigned char

#define INT_DISABLE		(ET0 = FALSE)	// (EA = FALSE)
#define INT_ENABLE		(ET0 = TRUE)	// (EA = TRUE)


#define RPM_ENABLE    1    					   // when true, unit sends RPM messages

#define NUM_SCALES	8	// used in pca_if.c


// defines for UART BAUDRATE
//#define BAUDRATE      19200                    // communication baudrate in bps

/* serial comm paramsters */
#define R_ACK				1
#define R_NAK				0
#define R_ACK_NO_DATA		2

#define NEW_LINE_CFG_BIT	0x0400
#define EVEN_PARITY			0x00
#define PARITY_BIT			0x80
#define BS_CHAR				8
#define NULL_CHAR	 		'\0'

#define IO_BUF_CMD_INDEX 	2
#define IO_BUF_DATA_INDEX 	3
#define IO_BUF_RESP_INDEX	3


#define TXRX_BUF_LEN		85 // largest of all TSIP packets including overhead


/*---------------------------------------------------------------------*
* Interrupts and register banks
*---------------------------------------------------------------------*/
#define SYS_REG_BANK  	0

/* Timer 0 */
#define TIMER0_REG_BANK 1
#define TIMER0_INT_NUM  1

/* Serial Port I/O */
#define SIO_REG_BANK 	2
#define SIO_INT_NUM  	INTERRUPT_UART0

/* AUX Interrupts (ADC) */
#define ADC_REG_BANK 	3
#define ADC_INT_NUM  	INTERRUPT_ADC0_EOC

/* External Interrupts */
#define EXT_INT0_NUM  	0
#define EXT_INT1_NUM  	2


/*---------------------------------------------------------------------*
* I2C devices definitions
*----------------------------------------------------------------------*/
// MAX 1609 I2C to parallel converter
#define MAX1609_I2C_ADDR	0x4C	// 76

// on-board DAC parameters
#define DAC_I2C_ADDR 		0x5A	// 90

// off board DAC address
#define EXT_DAC_I2C_ADDR	0x58	// 88

// on-board EEPROM parameters (X24645 add 1 to I2C address to enable X24645 format)
#define EEPROM_I2C_ADDR		0x81	// 129
#define SIGNATURE			12345

// off board EEPROM address
#define EXT_EEPROM_I2C_ADDR	0xA0	// 160


/*======================================================================*
* soft timers 
*=======================================================================*/
#define NUM_TIMERS      3

#define SYS_TIMER		0
#define LCD_TIMER		1
#define SWITCH_TIMER	2

/*======================================================================*
* on-board EEPROM Memory Map
*=======================================================================*/
#define NUM_BINS			12
#define ADC_TBL_NDX			0
#define DAC_TBL_NDX			NUM_BINS
/* end of ADC and DAC tables at address 23 */

#define REFRESH_PERIOD_NDX	30	// word
#define ADDR_NDX			32
#define TMR0_INIT_INDEX		34  // v0.0.C
#define CAL_FACTOR_INDEX    36  // v0.0.E
#define SIGNATURE_INDEX		38
#define DECIMATION_INDEX	40	//v1.0.6


/*---------------------------------------------------------------------*
*	POB Fault Definitions
*----------------------------------------------------------------------*/
#define I2C_STUCK_FAULT			1
#define EEPROM_I2C_FAULT		2
#define EEPROM_DATA_FAULT		4
#define DAC_I2C_FAULT			8
#define LCD_BUSY_FAULT			16


#define I2C_STUCK_FLT_TXT		"I2C STUCK"
#define EEPROM_I2C_FLT_TXT		"EEPROM I2C"
#define EEPROM_DATA_FLT_TXT		"EEPROM DATA"
#define DAC_I2C_FLT_TXT			"DAC I2C"
#define LCD_BUSY_FLT_TXT		"LCD BUSY"

#define NUM_MSG					16
#define MSG_LEN					17



#endif
