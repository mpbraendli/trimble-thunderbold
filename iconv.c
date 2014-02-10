/*==================================================================================*
* MODULE: ICONV.C
*
*   Integer conversion functions
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
*===================================================================================*/

/*----------------------------------------------------------------------------------*
*	Version
*
* Vers.  Date       Who             What
* ====== ========== =============== ==========================
* v1.0.1 29Sept07	D. Juges        Fixed overflow (would put one too many '?')
* v1.1.0 13July08   D. Juges        Fixed overflow function
*-----------------------------------------------------------------------------------*/

/* ----- definitions ----- */
#include <stdio.h>

#include "general.h"
#include "iconv.h"

#define MAX_A2B_DIGITS		5
#define ASCII_MASK			0x0F

#define CHAR_TO_ASCII


/*---------------------------------------------------------------------*
*  FUNCTION: AsciiToUnsigned()
*
*  ASCII To Unsigned Integer (Conversion)
*
*  DESCRIPTION:
*
*  Converts an ASCII string terminated with an EOS_CHAR into an
*  unsigned integer.
*
*  Returns FALSE when:
*     1) There are no digits.
*     2) There are more than MAX_A2B_DIGITS digits.
*     3) There are more than MAX_A2B_DIGITS +1 characters.
*     4) There are illegal characters.
*
*----------------------------------------------------------------------*/
uchar AsciiToUnsigned( uchar TXRX_STORAGE_CLASS *p_ascii, 
		uint *p_int ){

	uchar i = 0, n = 0;
	uint value = 0;

	while( (i < (MAX_A2B_DIGITS+1) ) && (n < MAX_A2B_DIGITS) ){
		if( *p_ascii == '\0' )
			break;

		if( (*p_ascii < '0') || (*p_ascii > '9') ){
			if( *p_ascii != ' ' )
				n = MAX_A2B_DIGITS;
		}else{
			n++;
			value *= 10;
			value += ( *p_ascii & ASCII_MASK );
		}

		i++;
		p_ascii++;
	}

	if( (*p_ascii == '\0') && (n > 0) ){
		*p_int = value;
		return( TRUE );
	}else
		return( FALSE );

} /* end AsciiToUnsigned() */

/*---------------------------------------------------------------------*
* FUNCTION: UnsignedToAscii()
*
*	Converts an unsigned integer to a 'nbdigits' character ASCII string
*	terminated with a NULL char.
*	Puts result directly into pbuf
*----------------------------------------------------------------------*/
void UnsignedToAscii( uint value, 
						uchar TXRX_STORAGE_CLASS *pbuf, 
						uchar nbdigits ){

	char ndx;	// must be signed
	uint remainder, result;

	ndx = nbdigits;

	*(pbuf+ndx--) = '\0';
	for( ; ndx>=0; ndx-- ){
		result = value / 10;
		remainder = value - ( 10 * result );
		value = result;
		*(pbuf+ndx) = ( remainder | '0' );
		if( value == 0 )
			break;
	}
	if( value > 0 ){
		// overflow
	    for( ndx=nbdigits-1; ndx>=0; --ndx )
	       *(pbuf+ndx) = '?';
		return;
	}

	// blank leading zeroes
	ndx--;
	while( ndx >= 0 )
		*(pbuf+ndx--) = ' ';
	
} // UnsignedToAscii()

/*---------------------------------------------------------------------*
* FUNCTION: LongToAscii()
*
*	Converts an unsigned long integer to a 'nbdigits' character ASCII string
*	terminated with a NULL char.
*	Puts result directly into pbuf
*----------------------------------------------------------------------*/
/*void LongToAscii( unsigned long value, 
						uchar TXRX_STORAGE_CLASS *pbuf, 
						uchar nbdigits ){

	uchar i;
	char ndx;	// must be signed
	unsigned long remainder, result;

	ndx = nbdigits;

	*(pbuf+ndx--) = NULL;
	for( ; ndx>=0; ndx-- ){
		result = value / 10;
		remainder = value - ( 10 * result );
		value = result;
		*(pbuf+ndx) = ( remainder | '0' );
		if( value == 0 )
			break;
	}
	if( value > 0 ){
		// overflow
	    for( ndx=nbdigits-1; ndx>=0; --ndx )
	       *(pbuf+ndx) = '?';
		return;
	}

	// blank leading zeroes
	ndx--;
	while( ndx >= 0 )
		*(pbuf+ndx--) = ' ';
	
} // LongToAscii()
*/

/*---------------------------------------------------------------------*
* FUNCTION: CharToAscii()
*
*   Converts a signed char to an ASCII string terminated with a NULL_CHAR.
*   Puts the result at *(pbuf)
*   (this is a lightweight IntToAscii() )
*---------------------------------------------------------------------*/
#if defined CHAR_TO_ASCII
void CharToAscii( char value, 
		uchar TXRX_STORAGE_CLASS *pbuf, 
		uchar nbdigits ){

   uchar remainder, result;
   char ndx, sign;	// must be signed

   ndx = nbdigits;

   if( value < 0 ){
      value = -value;
      sign = -1;
   }else
      sign = 1;

   *(pbuf+ndx--) = '\0';
   for( ; ndx>0; ndx-- ){
      result = value / 10;
      remainder = value - ( 10 * result );
      value = result;
      *(pbuf+ndx) = ( remainder + '0' );
      if( value == 0 )
         break;
   }

   // check for overflow
   if( value > 0 ){
      for( ndx=nbdigits-1; ndx>=0; --ndx )
         *(pbuf+ndx) = '?';
      return;
   }

   ndx--;

   if( sign == -1 ){
      if( ndx > 0 )
	      *(pbuf+ndx--) = '-';
      else{
         for( ndx=nbdigits-1; ndx>=0; --ndx )
            *(pbuf+ndx) = '?';
		 return;
	  }
   }
	// blank leading zeroes
	while( ndx >= 0 )
		*(pbuf+ndx--) = ' ';

} // end CharToAscii()
#endif

/*---------------------------------------------------------------------*
* FUNCTION: IntToAscii()
*
*	Converts a signed integer to a 'nbdigits' character ASCII string
*	terminated with a NULL char.
*	Puts result directly into pbuf
*----------------------------------------------------------------------*/
/*
void IntToAscii( int value, 
		uchar TXRX_STORAGE_CLASS *pbuf, 
		uchar nbdigits ){

	char ndx;
	char sign;	// must be signed
	uint remainder, result;

	ndx = nbdigits;

	if( value < 0 ){
		value = -value;
		sign = -1;
	}else
		sign = 1;

	*(pbuf+ndx--) = '\0';

	for( ; ndx>=0; ndx-- ){
		result = value / 10;
		remainder = value - ( 10 * result );
		value = result;
		*(pbuf+ndx) = ( remainder | '0' );
		if( value == 0 )
			break;
	}
	if( value > 0 ){
		// overflow
	    for( ndx=nbdigits-1; ndx>=0; --ndx )
	       *(pbuf+ndx) = '?';
		return;
	}

	ndx--;

	if( sign == -1 ){
		if( ndx > 0 )
			*(pbuf+ndx--) = '-';
		else{
			for( ndx=nbdigits-1; ndx>=0; --ndx )
				*(pbuf+ndx) = '?';
			return;
		}
   }
	// blank leading zeroes
	while( ndx >= 0 )
		*(pbuf+ndx--) = ' ';

} // IntToAscii()
*/
/*---------------------------------------------------------------------*
*  FUNCTION:
*  ASCII To Integer (Conversion)
*
*  Converts an ASCII string terminated with an EOS_CHAR into an
*  integer.
*
*  Returns FALSE when:
*     1) There are no digits.
*     2) There are more than MAX_A2B_DIGITS digits.
*     3) There are more than MAX_A2B_DIGITS +1 characters.
*     4) There are illegal characters.
*
*---------------------------------------------------------------------*/
#if defined ASCII_TO_INT
uchar AsciiToInt(
		uchar TXRX_STORAGE_CLASS *p_ascii,
		int *p_int ){

	uchar idata i = 0, n = 0, sign = 0, space_ok = TRUE;
	int idata value = 0;

	while( ( i < (MAX_A2B_DIGITS+1) ) && ( n < MAX_A2B_DIGITS ) ){
		if( *p_ascii == '\0' )
			break;

		if( ( *p_ascii < '0' ) || ( *p_ascii > '9' ) ){
			switch( *p_ascii ) {
				case ' ':
					break;

				case '+':
					if ( ( sign == 0 ) && space_ok )
						sign = 1;
					else
						n = MAX_A2B_DIGITS;
					break;

				case '-':
					if ( ( sign == 0 ) && space_ok )
						sign = -1;
					else
						n = MAX_A2B_DIGITS;
					break;

				default:
					n = MAX_A2B_DIGITS;
					break;
			}
		}else{
			space_ok = FALSE;
			n++;
			value *= 10;
			value += ( *p_ascii & ASCII_MASK );
		}

		i++;
		p_ascii++;
	}

	if( ( *p_ascii == '\0' ) && ( n > 0 ) ){
		if( sign < 0 )
			*p_int = -value;
		else
			*p_int = value;

		return( TRUE );
	}else
		return( FALSE );

} // end AsciiToInt
#endif



























