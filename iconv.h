/*=====================================================================*
 * HEADER FILE: iconv.h
 *======================================================================*/

#include "general.h"

void UnsignedToAscii(uint value, char *pbuf, char nbdigits);

void ByteToAscii( uchar value, 
        uchar *pbuf, 
        uchar nbdigits, 
        uchar offset );
void IntToAscii( int value, 
        uchar *pbuf, 
        uchar nbdigits );
void CharToAscii( char value, 
        uchar *pbuf, 
        uchar nbdigits );
uchar AsciiToInt(
        uchar *p_ascii,
        int *p_int );
uchar AsciiToUnsigned( uchar *p_ascii, 
        uint *p_int );
void LongToAscii( unsigned long value, 
        uchar *pbuf, 
        uchar nbdigits );

