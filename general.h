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

/* ----- user adjustable parameters ----- */
#define BAUDRATE        9600           // Baud rate of UART in bps

#define NUM_STATUS_BYTE			4

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


/* serial comm paramsters */
#define R_ACK				1
#define R_NAK				0
#define R_ACK_NO_DATA		2

#define NEW_LINE_CFG_BIT	0x0400
#define EVEN_PARITY			0x00
#define PARITY_BIT			0x80
#define BS_CHAR				8
#define NULL_CHAR			'\0'

#define IO_BUF_CMD_INDEX 	2
#define IO_BUF_DATA_INDEX 	3
#define IO_BUF_RESP_INDEX	3


#define TXRX_BUF_LEN		85 // largest of all TSIP packets including overhead


#define NUM_MSG					16
#define MSG_LEN					17



#endif
