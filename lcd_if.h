/********************************************************************
*  FUNCTION HEADER MODULE:      rpm_lcd_if.h
*
*   R80-2000K Remote Power Monitor LCD interface headers
*
*********************************************************************/

#ifndef RPM_LCD_IF_H

#include "general.h"

/*
*  REVISION LOG:
*
*  Date     Name          Version  Reason
*  -------  ------------  -------  -------------------------------
*  17Dec01  J.F. Bush     1.0      Created
*  24Nov03  D. Juges               modified for MSC1210
*
*********************************************************************/

/* ----- constants ----- */
#define LCD_SIZE			20
#define LCD_BUF_LEN			LCD_SIZE+1

/********************************************************************
*  FUNCTION PROTOTYPE
*********************************************************************/
void InitLCD( void );
void ClearLCD( uchar line);
void printLCD( uchar line, uchar LCD_STORAGE_CLASS *chr_ptr, uchar offset );
void printCode2LCD( uchar line, uchar code *chr_ptr, uchar offset );


#define RPM_LCD_IF_H
#endif
