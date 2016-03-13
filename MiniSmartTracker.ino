

#include <TinyGPS++.h>
#include <string.h>
#include <Arduino.h>
#include <ArduinoQAPRS.h>
#include <SPI.h>
#include <OneWire.h>

#include "config.h"

// The TinyGPS++ object
TinyGPSPlus gps;

OneWire  ds(6);
char * packet_buffer  = "                                                                                    \n ";
char gradbuf[4];

const int analogInPin = A0;
int ledPin = 13;
const byte highSpeed = 60;       // High speed
const byte lowSpeed = 20;        // Low speed
unsigned long previousMillis = 0;
float latitude = 0.0;
float longitude = 0.0;

// Initial lat/lng pos, change to your base station coordnates
float lastTxLat = HOME_LAT;
float lastTxLng = HOME_LON;
float lastTxdistance;

int previousHeading, currentHeading = 0;

unsigned int mCounter = 0;
unsigned int txCounter = 0;
unsigned long txTimer = 0;
unsigned long lastTx = 0;
unsigned long lastRx = 0;
unsigned long txInterval = 80000L;  // Initial 80 secs internal


void setup()
{
  Serial.begin(9600);            
  Serial.println("WAIT!");
  delay(100);
  pinMode(ledPin, OUTPUT);
  QAPRS.init(3, 2);
  delay(2000);
}

void loop()
{
 int headingDelta = 0;
 
     while (Serial.available())
      {
        gps.encode(Serial.read());
      } 
 
 ///////////////// Triggered by location updates /////////////////////// 
   if ( gps.location.isUpdated() ) { 
          
    lastTxdistance = TinyGPSPlus::distanceBetween(
          gps.location.lat(),
          gps.location.lng(),
          lastTxLat,
          lastTxLng);
          
     latitude = gps.location.lat();
     longitude = gps.location.lng();   
          
      // Get headings and heading delta
      currentHeading = (int) gps.course.deg();
      if ( currentHeading >= 180 ) { 
      currentHeading = currentHeading-180; 
      }
      headingDelta = (int) ( previousHeading - currentHeading ) % 360;   
     
    } // endof gps.location.isUpdated()
  
  ///////////////// Triggered by time updates /////////////////////// 
// Update LCD every second

   if ( gps.time.isUpdated() ) {   
    
// Change the Tx internal based on the current speed
// This change will not affect the countdown timer
// Based on HamHUB Smart Beaconing(tm) algorithm

      if ( gps.speed.kmph() < 5 ) {
            txInterval = 300000;         // Change Tx internal to 5 mins
       } else if ( gps.speed.kmph() < lowSpeed ) {
            txInterval = 60000;          // Change Tx interval to 60
       } else if ( gps.speed.kmph() > highSpeed ) {
            txInterval = 20000;          // Change Tx interval to 20 secs
       } else {
        // Interval inbetween low and high speed 
            txInterval = (highSpeed / gps.speed.kmph()) * 20000;       
       } // endif
      
   }  // endof gps.time.isUpdated()
               
 ////////////////////////////////////////////////////////////////////////////////////
 // Check for when to Tx packet
 ////////////////////////////////////////////////////////////////////////////////////
 
  lastTx = millis() - txTimer;

  // Only check the below if locked satellites < 3

   if ( (gps.satellites.value() > 3) and (gps.location.age() < 3000) ) {
    if ( lastTx > 5000 ) {
        // Check for heading more than 25 degrees
        if ( (headingDelta < -25 || headingDelta >  25) && lastTxdistance > 5 ) {
            if (TxtoRadio()) {
                lastTxdistance = 0;   // Ensure this value is zero before the next Tx
                previousHeading = currentHeading;
            }
        } // endif headingDelta
    } // endif lastTx > 5000
    
    if ( lastTx > 10000 ) {
         // check of the last Tx distance is more than 600m
         if ( lastTxdistance > 600 ) {  
            if ( TxtoRadio() ) {
                lastTxdistance = 0;   // Ensure this value is zero before the next Tx
            }
       } // endif lastTxdistance
    } // endif lastTx > 10000
    
    if ( lastTx >= txInterval ) {
        // Trigger Tx Tracker when Tx interval is reach 
        // Will not Tx if stationary bcos speed < 5 and lastTxDistance < 20
        if ( lastTxdistance > 20 ) {
              TxtoRadio(); 
        } // endif lastTxdistance > 20 
        
        //Debug info
        Serial.print("sentencesWithFix=");Serial.println(gps.sentencesWithFix());
        Serial.print("Sentences that failed checksum=");
        Serial.println(gps.failedChecksum());
        //end DEBUG info
        
    } // endif of check for lastTx > txInterval
    
   } // Endif check for satellites
    

  }


///////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////
  


///////////////////////////////////////////////////////////////////////////////////////////////////////
//формирование пакета APRS и его отправка
boolean TxtoRadio(void) {
  
     char tmp[10];
     float latDegMin, lngDegMin = 0.0;
     String latOut, lngOut, cmtOut = "";
     unsigned int Mem = freeRam();
     float Volt = (float) readVcc();
     
     digitalWrite(ledPin, HIGH);
     
     lastTxLat = gps.location.lat();
     lastTxLng = gps.location.lng();
     
             
       latDegMin = convertDegMin(lastTxLat);
       lngDegMin = convertDegMin(lastTxLng);

       // Convert Lat float to string
       dtostrf(fabs(latDegMin), 2, 2, tmp );
 //      latOut.concat("lla");      // set latitute command 
       
       // Append 0 if Lat less than 10 
       if ( fabs(lastTxLat) < 10 ) {
       latOut.concat("0");              
       }  
       
       latOut.concat(tmp);      // Actual Lat in DDMM.MM

       // Determine E or W       
       if (latDegMin >= 0) {
           latOut.concat("N");
       } else if (latDegMin < 0) {
           latOut.concat("S");
       }

       cmtOut.concat("!");
       cmtOut.concat(latOut);
       cmtOut.concat("/");

       // Convert Lng float to string
       dtostrf(fabs(lngDegMin), 2, 2, tmp );
 //      lngOut.concat("llo");       // set longtitute command
       
       // Append 0 if Lng less than 100
       if ( ( fabs(lastTxLng) < 100) ) {
             lngOut.concat("0");       // set longtitute command
       }     

       // Append 0 if Lng less than 10
       if ( fabs(lastTxLng) < 10 ) {
       latOut.concat("0");      // Append 0 if Lng less than 10         
       }  

       lngOut.concat(tmp);      // Actual Lng in DDDMM.MM

       // Determine E or W
       if (lngDegMin >= 0) {
           lngOut.concat("E");
       } else if (latDegMin < 0) {
           lngOut.concat("W");
       }

       cmtOut.concat(lngOut);
       
     //aprs = "!" + lat + ns + "/" + lon + ew + ">" + "/A=000" + alti + " Bat=" + outputValue + "V" + " Sat=" + sat; 
       
       
       cmtOut.concat(">");
       cmtOut.concat(padding((int) gps.course.deg(),3));
       cmtOut.concat("/");
       cmtOut.concat(padding((int)gps.speed.mph(),3));
       cmtOut.concat("/A=");
       cmtOut.concat(padding((int)gps.altitude.feet(),6));
       cmtOut.concat(" ");
       cmtOut.concat("V=");
       cmtOut.concat(Volt);
       cmtOut.concat(" ");
       cmtOut.concat("Sat=");
       cmtOut.concat(gps.satellites.value());
       cmtOut.concat(" ");

    #ifdef TEMP_TX
      cmtOut.concat("T=");
      cmtOut.concat(getTemp());
      cmtOut.concat(" ");
    #endif
       
       cmtOut.concat(COMMENT); 
        

        // convert string to char_array
         int str_len = cmtOut.length() + 1;
         char char_array[str_len];
         cmtOut.toCharArray(char_array, str_len);

         // send via packet_buffer
         packet_buffer = char_array;
         Serial.print("Send:    "),Serial.println(packet_buffer);
         QAPRS.send(MYCALL, CALL_SSID, DEST_ADDR, '0', RELAY, packet_buffer);
         
         digitalWrite(ledPin, LOW);
        
} // endof TxtoRadio()
///////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////////

float convertDegMin(float decDeg) {
  
  float DegMin;
  
  int intDeg = decDeg;
  decDeg -= intDeg;
  decDeg *= 60;
  DegMin = ( intDeg*100 ) + decDeg;
 
 return DegMin; 
}

///////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////

float readVcc() {  
                 
 float sensorValue = 0;
 float outputValue = 0;
//измерение напряжения
  delay(10);
  sensorValue = analogRead(analogInPin);
  outputValue = float(analogRead(analogInPin)) / 204.6;
  outputValue = (outputValue) * 10;
  return outputValue;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////

String padding( int number, byte width ) {
  String result;
  
  // Prevent a log10(0) = infinity
  int temp = number;
  if (!temp) { temp++; }
    
  for ( int i=0;i<width-(log10(temp))-1;i++) {
       result.concat('0');
  }
  result.concat(number);
  return result;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////
int freeRam() {
#if defined(__arm__) && defined(TEENSYDUINO)
  char top;
        return &top - reinterpret_cast<char*>(sbrk(0));
#else  // non ARM, this is AVR
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
#endif  
}
///////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////
//измерение температуры
int getTemp(void)
  {
          byte i;
          byte present = 0;
          byte data[12];
          byte addr[8];  
          if ( !ds.search(addr)) {
            ds.reset_search();
            return 0;
          }  

          if ( OneWire::crc8( addr, 7) != addr[7]) {
            return 0;
          }

          if ( addr[0] != 0x10) {
            return 0;
          }

          ds.reset();
          ds.select(addr);
          ds.write(0x44,1); 
          delay(1000);
          present = ds.reset();
          ds.select(addr);    
          ds.write(0xBE);

          for ( i = 0; i < 9; i++) {
            data[i] = ds.read();
          }

          int HighByte, LowByte, TReading, Tce;
          LowByte = data[0];
          HighByte = data[1];
          TReading = (HighByte << 8) + LowByte; 
          Tce = TReading/2;
          return (Tce);
        }
 ///////////////////////////////////////////////////////////////////////////////////////////////////////

