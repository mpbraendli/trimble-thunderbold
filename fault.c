#include "general.h"
#include "fault.h"

static uchar fault_query_ndx = 0;

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
int Fault_Msg_Query(int Test_Result, char *buf, char *msg) {

    uchar i;
    uint j=1;

    if (Test_Result == 0)
        return FALSE;
    i = 0;
    if (fault_query_ndx > 15)
        fault_query_ndx = 0;
    while (!(Test_Result & j<<fault_query_ndx)) {
        fault_query_ndx++;
        if (fault_query_ndx >= NUM_MSG || i++ >= NUM_MSG) {
            fault_query_ndx = 0;
            return FALSE;
        }
        //if( i++ >= NUM_MSG )
        //    return( FALSE );
    }
    if (*(msg+MSG_LEN*fault_query_ndx) == NULL_CHAR) {
        fault_query_ndx++;
        buf[0] = NULL_CHAR;
        return( FALSE );
    }

    for (i=0; i<MSG_LEN; i++) {
        buf[i] = *(msg+MSG_LEN*fault_query_ndx+i);
        if (buf[i] == NULL_CHAR)
            break;
    }
    fault_query_ndx++;

    return TRUE;
}

