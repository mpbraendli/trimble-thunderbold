/*=============================================================================*
*	TSIP.h
*
*	TSIP is Trimble's interface software command set
*
*	TSIP packet structure is the same for both commands and reports. 
*	The packet format is: 
*	<DLE> <id> <data string bytes> <DLE> <ETX> 
*	Where:
*	<DLE> is the byte 0x10
*	<ETX> is the byte 0x03
*	<id> is a packet identifier byte, which can have any value excepting 
*		<ETX> and <DLE>.
*	The bytes in the data string can have any value. 
*	To prevent confusion with the frame sequences <DLE> <id> and <DLE> <ETX>, 
*	every <DLE> byte in the data string is preceded by an extra <DLE> byte 
*	('stuffing'). These extra <DLE> bytes must be added ('stuffed') before 
*	sending a packet and removed after receiving the packet. Notice that a 
*	simple <DLE> <ETX> sequence does not necessarily signify the end of the 
*	packet, as these can be bytes in the middle of a data string. 
*	The end of a packet is <ETX> preceded by an odd number of <DLE> bytes.    
*
*	Multiple-byte numbers (integer, float, and double) follow the 
*	ANSI / IEEE Std. 754 IEEE Standard for binary Floating-Point Arithmetic. 
*	They are sent most-significant byte first. 
*	Note that switching the byte order will be required in Intel-based machines. 
*
*	The data types used in the ThunderBolt TSIP are defined below. 
*		UINT8 - An 8 bit unsigned integer (0 to 255).
*		SINT8 - An 8 bit signed integer (-128 to 127).
*		INT16 - A 16 bit unsigned integer (0 to 65,535).
*		SINT16 - A 16 bit signed integer (-32,768 to 32,767).
*		UINT32 - A 32 bit unsigned integer (0 to 4,294,967,295)
*		SINT32 - A 32 bit signed integer (-2,147,483,648 to 2,147,483,647).
*		Single Float (4 bytes) (3.4x10-38 to 1.7x10+38) (24 bit precision)
*		Double Float (8 bytes) (1.7x10-308 to 3.4x10+308) (53 bit precision)
*
*	Note: Default serial port settings are: 9600, 8-none-1
*
*==============================================================================*/

/*=============================================================================*
*   Report Packet 0x8F-AB Primary Timing Packet
*
*   Byte Bit    Item            Type    Value   Description
*   ==== ====== =============== ======= ======= ===============================
*   0           Subcode         UINT8   0xAB
*   1-4         Time of Week    UINT32          GPS seconds of week
*   5-6         Week Number     UINT16          GPS Week Number (see above)
*   7-8         UTC Offset      SINT16          UTC Offset (seconds) GPS Time
*   9           Timing Flag     Bits    
*        0                              0       GPS Time
*                                       1       UTC Time
*        1                              0       GPS PPS
*                                       1       UTC PPS
*        2                              0       time is set
*                                       1       time is not set
*        3                              0       have UTC info
*                                       1       does not have UTC info
*        4                              0       time from GPS
*                                       1       time from user
*   10          Seconds         UINT8   0-59    (60 for UTC leap second event)
*   11          Minutes         UINT8   0-59    Minutes of Hour
*   12          Hours           UINT8   0-23    Hour of Day
*   13          Day of Month    UINT8   1-31    Day of Month
*   14          Month           UINT8   1-12    Month of Year
*   15-16       Year            UINT16          Four digits of Year (e.g. 1998)
*
* Notes: 
*	1) multi-byte data is sent most significant byte first
*	2) byte position refers to the data packet (<DLE> and <ID> removed)
*==============================================================================*/


/*=============================================================================*
*   Report Packet 0x8F-AC Supplemental Timing Packet
*
*   Byte Bit    Item            Type    Value   Description
*   ==== ====== =============== ======= ======= ===============================
*   0           Subcode         UINT8   0xAC
*   1           Receiver Mode   UINT8   0       Automatic (2D/3D)
*                                        1       Single Satellite (Time)
*                                        3       Horizontal (2D)
*                                        4       Full Position (3D)
*                                        5       DGPS Reference
*                                        6       Clock Hold (2D)
*                                        7       Overdetermined Clock
*    2          Disciplining Mode UINT8  0       Normal
*                                        1       Power-Up
*                                        2       Auto Holdover
*                                        3       Manual Holdover
*                                        4       Recovery
*                                        5       Not Used
*                                        6       Disciplining disabled
*    3           Self-Survey     UINT 8          0-100% Progress
*    4-7      Holdover Duration  UINT 32         seconds
*    8-9         Critical Alarms UINT16  bit    
*		0 										 ROM checksum error
*       1										 RAM check has failed
*       2										 Power supply failure
*       3										 FPGA check has failed
*       4										 Oscillator control voltage at rail
*    10-11       Minor Alarms    UINT16  bit     
*		0										 Control voltage is near rail
*       1										 Antenna open
*       2										 Antenna shorted
*       3										 Not tracking satellites
*       4										 Not disciplining oscillator
*       5										 Survey-in progress
*       6										 No stored position
*       7										 Leap second pending
*       8										 In test mode
*    12          GPS Decoding    UINT8   0       Doing fixes
*                                        1       Don’t have GPS time
*                                        3       PDOP is too high
*                                        8       No usable sats
*                                        9       Only 1 usable sat
*                                        10      Only 2 usable sats
*                                        11      Only 3 usable sats
*                                        12      The chosen sat is unusable
*                                        16      TRAIM rejected the fix
*    13    Disciplining Status   UINT8   0       Phase locking
*                                        1       Oscillator warming up
*                                        2       Frequency locking
*                                        3       Placing PPS
*                                        4       Initializing loop filter
*                                        5       Compensating OCXO
*                                        6       Inactive
*                                        7       Not used
*                                        8       Recovery mode
*    14      Spare Status 1      UINT8   0
*    15      Spare Status 2      UINT8   0
*    16-19   PPS Offset          Single          Estimate of UTC/GPS offset (ns)
*    20-23   10 MHz Offset       Single          Estimate of UTC/GPS offset(ppb)
*    24-27   DAC Value           UINT32          Offset binary (0x00 - 0xFFFFF)
*    28-31   DAC Voltage         Single          Volts
*    32-35   Temperature         Single          degrees C
*    36-43   Latitude            Double          radians
*    44-51   Longitude           Double          radians
*    52-59   Altitude            Double          meters
*    60-67   Spare                               Future expansion

* Receive COM 8 Packet ID: 8F-AC  Data Length: 68
* AC
* 07
* 00
* 00
* 00 00 00 00
* 00 00 
* 00 80 
* 00 
* 00
* 00 
* 00
* BF A6 07 4B 
* 3C 9F 82 FA 
* 00 08 9E B9 Dac Value 89eb9
* 3E C6 67 40 dac voltage .387516
* 42 21 22 43
* 3F E9 16 02 98 F4 95 06 
* BF F9 F5 46 8F 0A D0 12 
* 40 72 2C 86 63 97 C0 00 
* 00 00 00 00 00 00 00 01 
*==============================================================================*/

#define NUM_MODES		17
//#define MSG_LEN	18

/* ===== GPS Stuff ===== */
enum {
	WAIT_FOR_START,
	WAIT_FOR_DLE_ETX,	// we have received a char that was not DLE or ETX
	WAIT_FOR_ETX,		// we have received char+DLE, waiting for ETX
	WAIT_FOR_DLE,	// we have received char+DLE+ETX, waiting for DLE or else exit
	WAIT_FOR_ID,
	WAIT_FOR_END_MSG,
	SEND_HDR,
	SEND_MSG,
	SEND_ETX
};

#define ETX								0x03
#define DLE								0x10
#define IO_BUF_ID_INDEX					1
#define IO_BUF_ID2_INDEX				2

#define SUPERPACKET						0x8F
#define PRIMARY_TIMING_PCKT				0xAB
#define PRIMARY_TIMING_PCKT_LEN			17
#define SUPPLEMENTAL_TIMING_PCKT		0xAC
#define SUPPLEMENTAL_TIMING_PCKT_LEN	60


/* ===== TSIP stuff ===== */
code uchar Month[]          = "JanFebMarAprMayJunJulAugSepOctNovDec";
// NUM_MODES is defined in TSIP.h
// MSG_LEN  is defined in general.h

code uchar DIM[]  = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

code uchar RxMode[8][MSG_LEN] = {
	"RX: Auto 2D/3D",
	"RX: Single Sat",
	"RX: Unknown!!!",
	"RX: Horiz. (2D)",
	"RX: Full Pos(3D)",
	"RX: DGPS Refer",
	"RX: Clk Hold(2D)",
	"RX: Overdet Clk"	// Normal mode
};

code uchar DiscMode[7][MSG_LEN] = {
	"MODE: Normal",	// Normal mode
	"MODE: Power-Up",
	"MODE: Auto Hldvr",
	"MODE: Man. Hldvr",
	"MODE: Recovery",
	"MODE: Unknown!!!",
	"MODE: Disabled"
};

code uchar GPSDecodeStatus[17][MSG_LEN] = {
	"GPS: Doing Fixes", // Normal Mode
	"GPS: No GPS Time",
	"GPS: Unknown!!!",
	"GPS: PDOP to HI",
	"GPS: Unknown!!!",
	"GPS: Unknown!!!",
	"GPS: Unknown!!!",
	"GPS: Unknown!!!",
	"GPS: No Use SATS",
	"GPS: Only 1 SAT",
	"GPS: Only 2 SATS",
	"GPS: Only 3 SATS",
	"GPS: SAT Unusabl",
	"GPS: Unknown!!!",
	"GPS: Unknown!!!",
	"GPS: Unknown!!!",
	"GPS: TRIAM Rejec" 
};

code uchar DiscActivity[9][MSG_LEN] = {
	"ACT: Phase Lock",
	"ACT: Osc Warmup",
	"ACT: Freq Lock",
	"ACT: Placing PPS",
	"ACT: Init. loop",
	"ACT: Compensate",
	"ACT: Inactive",
	"ACT: Unknown!!!",
	"ACT: Recovery LP"
};

/* ----- typedef and union used to convert byte to uint or int ----- */
typedef union int_t INTType;

union int_t { 
	int i;
	unsigned int u;
	struct {
		uchar lo;
		uchar hi;
	}b;
};

typedef union uint_t UINTType;

union uint_t { 
	unsigned int u;
	int i;
	struct {
		uchar lo;
		uchar hi;
	}b;
};

typedef union float_t FLOATType;

union float_t {
	float f;
	struct {
		uchar llo;
		uchar lo;
		uchar hi;
		uchar hhi;
	}b;
};

