
//User defined part
// config file to defines callsign, ssid, symbols and home location
#define MYCALL "RA4NHY"
#define DEST_ADDR "APZ058"
#define RELAY "WIDE2 2"
#define CALL_SSID '9'

// Define your home lat & lon below
const float HOME_LAT = 58.606806;
const float HOME_LON = 49.570434;
//
//
//On/Off temp transmit
//#define TEMP_TX

//On/Off voltage transmit 
//#define VOLT_TX

//On/Off debug information
//#define DEBUG_INFO


// Custom Comments here
#define COMMENT "QAPRS SmartTrack"

//End of user defined part
/////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////
#define SECONDS(X) X*1000
#define MINUTES(X) X*60000

//Heading delta threshold degrees
#define NEG_HEAD_THR -20
#define POS_HEAD_THR 20

//lastTxdistance threshold in meters
#define LAST_TX_DIST_THR 600

//gps.location.age threshold in seconds
#define LOCATION_AGE_THR SEC(3)


//low and high speed threshold for smart beacon code km/h
#define HIGH_SPEED_THR 60
#define LOW_SPEED_THR 20

