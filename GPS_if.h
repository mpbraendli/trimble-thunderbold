/*============================================================================*
* GPS_if.h
*
*	This file contains standard NMEA sentence information
*
*=============================================================================*/

/*===== NMEA syntax ===== */

/* ----- RMC datasentence ----- */

//	The RMC-Datasentence (RMC=recommended minimum sentence C) is a 
//	recommendation for the minimum, that a GPS-Receiver should give back.
//	It looks like this: 
//	"$GPRMC,191410,A,4735.5634,N,00739.3538,E,0.0,0.0,181102,0.4,E,A*19"

typedef struct RMC_type {
	uchar UtcTime[7],
    uchar ReceiverWarning,
    float Latitude,
    uchar LatitudeDir,	// 'N', 'S'
    float Longitude,
    uchar LongitudeDir,	// 'W', 'E'
    int SpeedKMH,
    int Course,
    uchar DateStamp[7],
    float MagneticDeclination
} RMCDEF, *pRMCDEF;


/* ----- GGA datasentence ----- */

//	The GGA-Datasentence contains the most important information about 
//	GPS-position and accuracy.
//	It looks like this:
//	"$GPGGA,191410,4124.8963,N,08151.6838,E,1,05,1.5,280.2,M,-34.0,M,,*75"
//
//	Name------------------- Field---------- Description-----------------------
//	Sentence Identifier  	$GPGGA  		Global Positioning System Fix Data
//	Time 					191410 			19:14:10 Z
//	Latitude 				4124.8963, N 	41d 24.8963' N or 41d 24' 54" N
//	Longitude 				08151.6838, W 	81d 51.6838' W or 81d 51' 41" W
//	Fix Quality: 0 = Invalid, 1 = GPS fix, 2 = DGPS fix
//					 		1 				Data is from a GPS fix
//	Number of Satellites 	05 				5 Satellites are in view
//	Horizontal Dilution of Precision (HDOP) 	
//							1.5 			Relative accuracy of horizontal position
//	Altitude 				280.2, M 		280.2 meters above mean sea level
//	Height of geoid above WGS84 ellipsoid 	
//							-34.0, M 		-34.0 meters
//	Time since last DGPS update 	
//							blank 			No last update
//	DGPS reference station id 	
//							blank 			No station id
//	Checksum 				*75 			Used by program to check for transmission errors

typedef struct GGA_type {
    uchar UtcTime[7],
    float Latitude,
    uchar LatitudeDir,
    float Longitude,
    uchar LongitudeDir,
    uchar Quality,
    uchar SatellitesIV,
    float HDOP,
    float AltitudeSea,
    uchar AltitudeSeaUnit,
    float AltitudeEllipsoid,
    uchar AltitudeEllipsoidUnit
} GGADEF, *pGGADEF;


/* ----- GSA datasentence ----- */

//	The GSA-Datasentence contains information about the PRN-Numbers of the satellites that are used
//	for calculating the actual position and some more detailed info about the accuracy.
//	It looks like this: 
//	"$GPGSA,A,3,,,,15,17,18,23,,,,,,4.7,4.4,1.5*3F"

//	1    = Mode:
//       M=Manual, forced to operate in 2D or 3D
//       A=Automatic, 3D/2D
//	2    = Mode:
//       1=Fix not available
//       2=2D
//       3=3D
//	3-14 = IDs of SVs used in position fix (null for unused fields)
//	15   = PDOP
//	16   = HDOP
//	17   = VDOP

typedef struct GSA_type {
    uchar AutoSel,
    uchar mode[7],
    uchar PRN,
    float PDOP,
    float HDOP,
    float VDOP
} GSADEF, *pGSADEF;


/* ----- GSV datasentence ----- */

// $GPGSV,3,1,12,22,89,000,00,14,59,000,00,15,53,000,00,18,51,000,00*7F

// $GPGSV,3,2,12,09,47,000,00,19,15,000,00,21,13,000,,31,13,000,*7C
'              ------------ ---------- ------------ ------------
// $GPGSV,3,3,12,03,06,000,00,11,04,000,,28,02,000,00,05,01,000,00*77

// $GPGSV,3,1,12,22,89,000,00,14,59,000,00,15,53,000,00,18,51,000,00*7F
// $GPGSV,3,2,12,09,47,000,00,19,15,000,00,21,13,000,,31,13,000,*7C
// $GPGSV,3,3,12,03,06,000,00,11,04,000,,28,02,000,00,05,01,000,00*77
// GSV - Satellites in view
//         1 2 3 4 5 6 7     n
//         | | | | | | |     |
//  $--GSV,x,x,x,x,x,x,x,...*hh<CR><LF>
//  1) total number of messages (satellites in this record)
//  2) message number
//  3) satellites in view
//  4) satellite number (PRN)
//  5) elevation in degrees
//  6) azimuth in degrees to true (000 to 359)
//  7) SNR in dB (00-99)
//  more satellite infos like 4)-7)
//  n) checksum

typedef struct GSV_type {
    uchar tnm,
    uchar mn,
    uchar SatsInView,
    uchar SatNr,
    float Elevation,
    float Azimuth,
    float SNRdB,
} GSVDEF, *pGSVDEF;

