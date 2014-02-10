/*=====================================================================*
* HEADER FILE: iconv.h
*======================================================================*/

#include "general.h"

void ByteToAscii( uchar value, 
		uchar TXRX_STORAGE_CLASS *pbuf, 
		uchar nbdigits, 
		uchar offset );
void IntToAscii( int value, 
		uchar TXRX_STORAGE_CLASS *pbuf, 
		uchar nbdigits );
void CharToAscii( char value, 
		uchar TXRX_STORAGE_CLASS *pbuf, 
		uchar nbdigits );
uchar AsciiToInt(
		uchar TXRX_STORAGE_CLASS *p_ascii,
		int *p_int );
uchar AsciiToUnsigned( uchar TXRX_STORAGE_CLASS *p_ascii, 
		uint *p_int );
void UnsignedToAscii( uint value, 
		uchar TXRX_STORAGE_CLASS *pbuf, 
		uchar nbdigits );
void LongToAscii( unsigned long value, 
						uchar TXRX_STORAGE_CLASS *pbuf, 
						uchar nbdigits );

