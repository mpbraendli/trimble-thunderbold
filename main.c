#include "general.h"
#include "extdebug.h"

#include "TSIP.h"

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

extern unsigned char ALARM_MSG[NUM_ALARMS][MSG_LEN];

int set_interface_attribs(int fd, int speed, int parity)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        fprintf(stderr, "error %d from tcgetattr", errno);
        return -1;
    }

    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // ignore break signal
    tty.c_lflag = 0;                // no signaling chars, no echo,
    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    //tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
    {
        fprintf(stderr, "error %d from tcsetattr", errno);
        return -1;
    }
    return 0;
}

void set_blocking(int fd, int should_block)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        fprintf(stderr, "error %d from tggetattr", errno);
        return;
    }

    tty.c_cc[VMIN]  = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
        fprintf(stderr, "error %d setting term attributes", errno);
}

const unsigned int tz = 0;
unsigned char gpsoffset;

#define OFFSET_SELECT 0
#define DST_SELECT 0

void parse_primary_timing(uchar *RxBuf) {

    char sec, min, hr, dom, mo, y;
    UINTType yr;
    //int utcoffset;
    char ctz;

    ctz = tz;

    yr.b.hi = RxBuf[7];
    yr.b.lo = RxBuf[8];
    gpsoffset = yr.i;

    // decode date/time from packet
    sec = RxBuf[10];

    min = RxBuf[11];

    hr = RxBuf[12];

    dom = RxBuf[13];

    mo = (RxBuf[14]-1)*3;	// note: mo starts at 0 (January)

    yr.b.hi = RxBuf[15];
    yr.b.lo = RxBuf[16];

    y = yr.u - 2000;

    // if apply GPS offset...
    if (OFFSET_SELECT == 0) {
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
        if (ctz > 0) {
            hr += ctz;
            if (DST_SELECT == 0)
                hr++;
            if (hr > 23) {
                hr -= 24;
                dom++;
                if ((uchar)dom > DIM[mo]) {
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
        if (ctz < 0) {
            hr += ctz;
            if (DST_SELECT == 0)
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

    char lcdbuf[256];
    memset(lcdbuf, 0, 256);

    // display on LCD
    UnsignedToAscii( (uint)sec, lcdbuf+6, 2 );
    if (lcdbuf[6] == ' ') lcdbuf[6] = '0';
    lcdbuf[8] = ' ';

    UnsignedToAscii( (uint)min, lcdbuf+3, 2 );
    if (lcdbuf[3] == ' ') lcdbuf[3] = '0';
    lcdbuf[5] = ':';

    UnsignedToAscii( (uint)hr, lcdbuf, 2 );
    if (lcdbuf[0] == ' ') lcdbuf[0] = '0';
    lcdbuf[2] = ':';

    // Display day-of-month
    UnsignedToAscii( (uint)dom, lcdbuf+9, 2 );
    // DAK add leading zero to date 
    if (lcdbuf[9] == ' ') lcdbuf[9] = '0';

    // Display month
    lcdbuf[11] = Month[mo];
    lcdbuf[12] = Month[mo+1];
    lcdbuf[13] = Month[mo+2];

    // Display year
    UnsignedToAscii( (uint)y, lcdbuf+14, 2 );
    if( lcdbuf[14] == ' ' ) lcdbuf[14] = '0';
    lcdbuf[16] = '\0';

    if (OFFSET_SELECT == 1) {
        lcdbuf[16] = ' ';
        lcdbuf[17] = 'G';
        lcdbuf[18] = 'P';
        lcdbuf[19] = 'S';
        lcdbuf[20] = '\0';
    }
    else {
        lcdbuf[16] = ' ';
        lcdbuf[17] = 'C';
        if( DST_SELECT == 0 )
            lcdbuf[18] = 'D';
        else
            lcdbuf[18] = 'S';
        lcdbuf[19] = 'T';
        lcdbuf[20] = '\0';
    }

    printf("%s\n", lcdbuf);

}

INTType Alarms;

void parse_suppl_timing(unsigned char *RxBuf) {
    //#define TEST_EXTDEBUG

#define NUM_DISPLAY_MODES		6

    static int b = FALSE;
    static int c = FALSE;

    FLOATType temp;
    uchar val;
    uint fval;
    //char dac;
    //char tempbuf[17];
    static char mode = -1;

    unsigned char lcdbuf[256];


#ifdef TEST_EXTDEBUG
    if( Alarms.u == 0 )
        Alarms.u = 1;
    else
        Alarms.u = Alarms.u<<1;
#else

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

    if( Alarms.u != 0 && mode == NUM_DISPLAY_MODES - 1 ){
        b = Fault_Msg_Query( Alarms.u, lcdbuf, ALARM_MSG[0] );
        if( b )
            printf("%s\n", lcdbuf);
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
                //
                const unsigned char prompt[2][9]={
                    "TEMP:  ",
                    "DAC V: "
                };

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
        printf("%s\n", lcdbuf);
    }

}

void message_received(unsigned char* rx_buffer, int rx_count)
{
    /*
    for (int i = 0; i <= rx_count; i++) {
        printf("%x ", rx_buffer[i]);
    }*/

    int id = rx_buffer[IO_BUF_ID_INDEX];
    int id2 = rx_buffer[IO_BUF_ID2_INDEX];

    switch (id) {
        case SUPERPACKET:
            switch (id2) {
                case PRIMARY_TIMING_PCKT:
                    parse_primary_timing(rx_buffer + 2);
                    break;

                case SUPPLEMENTAL_TIMING_PCKT:
                    parse_suppl_timing(rx_buffer + 2);
                    break;

                default:
                    break;
            }
        default:
            break;
    }
}

static unsigned char rx_state;
static int bEvenDLE;
static unsigned char rx_buffer[TXRX_BUF_LEN];
static unsigned char rx_count;

void receiveloop(char tchar)
{
    switch( rx_state ){
        case WAIT_FOR_START:
            // actually this waits for an end-of-message sequence
            // Phase Receiver with char-DLE-ETX
            if (tchar == DLE) {
                rx_count = 0;
                rx_buffer[rx_count++] = tchar;
                rx_state = WAIT_FOR_ID;
            }
            else if (tchar != ETX) {
                rx_state = WAIT_FOR_DLE_ETX;
                rx_count = 0;
            }
            bEvenDLE = FALSE;
            break;

        case WAIT_FOR_DLE_ETX:
            if (tchar == DLE)
                rx_state = WAIT_FOR_ETX;
            else
                rx_state = WAIT_FOR_START;
            break;

        case WAIT_FOR_ETX:  // rarely happens
            if (tchar == ETX) {
                // found end of a message
                rx_state = WAIT_FOR_DLE;
            }
            else {
                rx_state = WAIT_FOR_START;
            }
            break;

        case WAIT_FOR_DLE:
            if (tchar == DLE) {
                rx_count = 0;
                rx_buffer[rx_count++] = tchar;
                rx_state = WAIT_FOR_ID;
            }
            else {
                rx_state = WAIT_FOR_START;
            }
            bEvenDLE = FALSE;
            break;

        case WAIT_FOR_ID: // never happens
            if (tchar == DLE || tchar == ETX) {
                rx_state = WAIT_FOR_START;
                rx_count = 0;
            }
            else {
                rx_state = WAIT_FOR_END_MSG;
                rx_buffer[rx_count++] = tchar;
            }
            break;

        case WAIT_FOR_END_MSG:
            if (tchar == DLE && rx_buffer[rx_count-1] == DLE && !bEvenDLE) {
                // byte stuffing
                bEvenDLE = TRUE;
            }
            else if (tchar == ETX && rx_buffer[rx_count-1] == DLE && !bEvenDLE) {
                // complete message received, in buffer
                //P0_6 = 0;	// 7
                rx_buffer[rx_count] = '\0';
                rx_state = WAIT_FOR_DLE;

                message_received(rx_buffer, rx_count);

                //P0_6 = 1;
            }
            else {
                rx_buffer[rx_count++] = tchar;
                bEvenDLE = FALSE;
            }
            break;

        default:
            rx_state = WAIT_FOR_START;
            break;
    }

}

int main(int argc, char **argv)
{
    printf("Hello\n");

    char *portname = "/dev/ttyUSB0";

    int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);

    if (fd < 0) {
        fprintf(stderr, "error %d opening %s: %s", errno, portname, strerror (errno));
        return errno;
    }

    set_interface_attribs(fd, B9600, 0);  // set speed to 115,200 bps, 8n1 (no parity)
    //set_blocking (fd, 0);                // set no blocking

    while (1) {
        char buf[1];
        int n = read(fd, buf, sizeof(buf));  // read up to 100 characters if ready to read

        if (n == 1) {
            receiveloop(buf[0]);
        }
        else if (n > 1) {
            fprintf(stderr, "got %d\n", n);
        }

    }

    return 0;
}

