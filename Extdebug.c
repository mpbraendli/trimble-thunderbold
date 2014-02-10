/**********************************************************************
* FUNCTION MODULE:      extdebug.c
*
*   Extended Debug Functions Module
*
* NOTES:
*
*   To test this module by itself, see gcc.h
*
*  REVISION LOG:
*
*   Date    Name        Version Reason
*   ------- ---------   ------- ------------------------------
*   23Jan03 D. Juges    1.2.1   Created from stuff previously in remote.c
*   22Jul04 D. Juges    1.3.8   LINT comments
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
***********************************************************************/

/*-------------------------------------------------------------------
*  INCLUDE FILES
*--------------------------------------------------------------------*/

//#define GCC

#ifdef GCC
#include "gcc.h"    /* gcc declarations */
#endif

#include "general.h"

#include "extdebug.h"

/*---------------------------------------------------------------------*
*   CONSTANT DEFINITIONS
*---------------------------------------------------------------------*/

//#define NUM_ALARMS		16		// alarm word is uint

code uchar ALARM_MSG[NUM_ALARMS][MSG_LEN] = {
	// Critical Alarms UINT16  	bit
	"ROM Checksum Err",			// 0
	"RAM Check Failed",			// 1
	"PowerSupply Fail",			// 2
	"FPGA Check Fail ",			// 3
	"VCO at rail",				// 4
	// Minor Alarms    UINT16  	bit
	"VCO Near Rail",			// 0
	"Antenna Open",				// 1
	"Antenna Shorted",			// 2
	"Not Tracking Sat",			// 3
	"Not Disciplining",			// 4
	"Survey In Progrs",			// 5
	"No Stored Pos",			// 6
	"Leap Sec Pending",			// 7
	"Test Mode"					// 8
	"",							// 9 undef
	""							// 10 undef
};



/*---------------------------------------------------------------------*
* external variables
*---------------------------------------------------------------------*/


/*----------------------------------------------------------------------*
* Private Variables
*-----------------------------------------------------------------------*/
static uchar xdata fault_query_ndx = 0;

/*---------------------------------------------------------------------*
* FUNCTION: Fault_Msg_Query()
*
*   Test_Result: a bit mapped 16 bit value, where 1's represent set bits
*   *buf:           output buffer where error message is placed
*   *msg:           pointer to the list of possible error messages
*
*   fault_query_index:  a static variable that points to the specific 
*	bit being processed
*----------------------------------------------------------------------*/
bit Fault_Msg_Query( uint Test_Result, uchar TXRX_STORAGE_CLASS *buf,
                uchar code *msg ){

    uchar i;
	uint j=1;

    if( Test_Result == 0 )
        return( FALSE );
    i = 0;
	if( fault_query_ndx > 15 )
		fault_query_ndx = 0;
    while( !(Test_Result & j<<fault_query_ndx) ){
        fault_query_ndx++;
        if( fault_query_ndx >= NUM_MSG || i++ >= NUM_MSG ){
            fault_query_ndx = 0;
			return( FALSE );
		}
        //if( i++ >= NUM_MSG )
        //    return( FALSE );
    }
    if( *(msg+MSG_LEN*fault_query_ndx) == NULL_CHAR ){
        fault_query_ndx++;
        buf[0] = NULL_CHAR;
        return( FALSE );
    }

    for( i=0; i<MSG_LEN; i++ ){
        buf[i] = *(msg+MSG_LEN*fault_query_ndx+i);
        if( buf[i] == NULL_CHAR )
            break;
    }
    fault_query_ndx++;

    return( TRUE );
}

/*---------------------------------------------------------------------*
* FUNCTION: main()
*
*   use this to compile this module under a generic MS-DOS comiler
*   and test the functions within.
*---------------------------------------------------------------------*/
#ifdef GCC
void main( void ){

    int i, j, start, finish, fault;
    uchar buf1[256], buf2[256], *buf;

    start = 0;
    finish = 15;
    fault = 0xFF;

    buf = TxRxBuf;
    for( i=start; i<finish; i++ ){
        if( Fault_Msg_Query( fault, buf, &SELFTEST_FLT[0][0] ))
            printf( "%s\n", buf );
    }

}


#endif


