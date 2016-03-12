/*
  Copyright (C) 2013 Lukasz Nidecki SQ5RWU
  Edit by RA4FHE xDD 2015
  1.Добавленно кол-принимаемых спутников
  2.Вольтметр        (A0)
  3.Термометр DS1820 (D6)
  Edit by RA4NHY 03.2016
  1.Для работы с GPS использована библиотека TinyGPS++
  2.Для формирования строки APRS использован код из проекта SVTrackR
  3.Добавлен алгоритм SmartBeacon на основе изменения скорости
  4.Переработан основной алгоритм
*/

#include <TinyGPS++.h>
#include <string.h>
#include <Arduino.h>
#include <ArduinoQAPRS.h>
#include <SPI.h>
#include <OneWire.h>


#define COMMENT "QAPRS"
//#define TEMP_TX

// The TinyGPS++ object
TinyGPSPlus gps;

OneWire  ds(6);
char * packet_buffer  = "                                                                                    \n ";
char gradbuf[4];
char from_addr[] = "RA4NHY";      // Позывной
char dest_addr[] = "APZ058";     // Адрес
char relays[] = "WIDE2 2";       // Путь
const int analogInPin = A0;
float lastTxLat;
float lastTxLng;
int ledPin = 13;
const byte highSpeed = 60;       // High speed
const byte lowSpeed = 20;        // Low speed
unsigned long txInterval = 80000L;  // Initial 80 secs internal
unsigned long previousMillis = 0;

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
 do
  {
        do
         {
          while (Serial.available())
            {
            gps.encode(Serial.read());
            }   
          }
          
          while (!gps.location.isValid() and !gps.location.isUpdated() );
     
  }
  while (SmartBeaconPause());
  
  if (gps.location.age() < 3000 )
      {

        Serial.print("sentencesWithFix=");Serial.println(gps.sentencesWithFix());
        Serial.print("Sentences that failed checksum=");
        Serial.println(gps.failedChecksum());

        TxtoRadio();
        
      }
      
  }


///////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////
   //Smart Beaconing
   //возвращает 0 при необходимости отправки пакета, в остальных случаях возвращает 1
   int SmartBeaconPause(void)
         {
         // Change the Tx internal based on the current speed
         // This change will not affect the countdown timer
         // Based on HamHUB Smart Beaconing(tm) algorithm

         if ( gps.speed.kmph() < 5 ) {
         txInterval = 300000 ;//         // Change Tx internal to 5 mins
       } else if ( gps.speed.kmph() < lowSpeed ) {
         txInterval = 600000;          // Change Tx interval to 60 secs
       } else if ( gps.speed.kmph() > highSpeed ) {
         txInterval = 20000;          // Change Tx interval to 20 secs
       } else {
         // Interval inbetween low and high speed
         txInterval = (highSpeed / gps.speed.kmph() ) * 20000;
       } // endif

        
        unsigned long currentMillis = millis();
        
        if (currentMillis - previousMillis >= txInterval)
          {
          previousMillis = currentMillis;
          return 0;
          }
         else
         {
         return 1;
          }
       }
///////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////////
//формирование пакета APRS и его отправка
boolean TxtoRadio(void) {
  
     char tmp[10];
     float latDegMin, lngDegMin = 0.0;
     String latOut, lngOut, cmtOut = "";
     unsigned int Mem = freeRam();
     float Volt = (float) readVcc();
  
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
         QAPRS.send(from_addr, '9', dest_addr, '0', relays, packet_buffer);    // SSID-9

        
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

