#include "general.h"
#include "extdebug.h"

#include "TSIP.h"

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdint.h>

extern char ALARM_MSG[NUM_ALARMS][MSG_LEN];

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

const int tz = 0;
char gpsoffset;

void parse_primary_timing(uint8_t *RxBuf) {

    int sec, min, hr, dom, mo, y;
    UINTType yr;
    int ctz;

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

    char mon[4];
    memcpy(mon, Month+mo, 3);
    mon[3] = 0;

    printf("%02d:%02d:%02d %02d %s %02d GPS\n",
            hr, min, sec,
            dom, mon, yr.u);

}

INTType Alarms;

void parse_suppl_timing(uint8_t *RxBuf) {
    //#define TEST_EXTDEBUG

#define NUM_DISPLAY_MODES		6

    static int b = FALSE;

    FLOATType temp;
    FLOATType dac;
    LONGType holdover_duration;

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

    if (Alarms.u != 0) {
        char buf[256];
        b = Fault_Msg_Query(Alarms.u, buf, ALARM_MSG[0]);
        if (b) {
            printf("%s\n", buf);
        }
        if (RxBuf[9] != 0) { // if critical alarm, keep showing it
            return;
        }
    }

    if (!b) {

        printf("%s\n", DiscMode[(unsigned)RxBuf[2]]);
        printf("%s\n", DiscActivity[(unsigned)RxBuf[13]]);
        printf("%s\n", RxMode[(unsigned)RxBuf[1]]);
        printf("%s\n", GPSDecodeStatus[(unsigned)RxBuf[12]]);

        printf("SelfSurvey: %d%%\n", RxBuf[3]);

        holdover_duration.b.hhi = RxBuf[4];
        holdover_duration.b.hi  = RxBuf[5];
        holdover_duration.b.lo  = RxBuf[6];
        holdover_duration.b.llo = RxBuf[7];
        printf("Holdover: %ds\n", holdover_duration.l);

        // RxBuf[32-35] is temperature (float)
        temp.b.hhi = RxBuf[32];
        temp.b.hi  = RxBuf[33];
        temp.b.lo  = RxBuf[34];
        temp.b.llo = RxBuf[35];

        printf("Temp: %f\n", temp.f);

        // RxBuf[28-31] is DAC Voltage (float)
        dac.b.hhi = RxBuf[28];
        dac.b.hi  = RxBuf[29];
        dac.b.lo  = RxBuf[30];
        dac.b.llo = RxBuf[31];

        printf("DAC V: %f\n\n", dac.f);
    }

}

void message_received(uint8_t* rx_buffer, int rx_count)
{
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

static int rx_state;
static int bEvenDLE;
static uint8_t rx_buffer[TXRX_BUF_LEN];
static int rx_count;

void receiveloop(uint8_t tchar)
{
    if (rx_count >= TXRX_BUF_LEN) {
        fprintf(stderr, "resetting state machine\n");
        rx_state = WAIT_FOR_START;
        rx_count = 0;
    }

    switch (rx_state) {
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
        uint8_t buf;
        int n = read(fd, &buf, 1);  // read up to 100 characters if ready to read

        if (n == 1) {
            receiveloop(buf);
        }
        else if (n > 1) {
            fprintf(stderr, "got %d\n", n);
        }

    }

    return 0;
}

