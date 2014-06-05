
/*Macomb Suncycle by K.Cole, R.Shirley, A.Distel.   ####Rich- your solartracking subroutine is stuck in a loop. Race condition?###
/*Modified and mutated from the following:          ####  I made this a separate tab -KC                                 ###
/ Theres some weird stuff being added to this git. Delete or comment out anything with a <<<<<<<<   

##Xively WiFi Sensor Tutorial## 
By Calum Barnes 3-4-2013
BSD 3-Clause License - [http://opensource.org/licenses/BSD-3-Clause]
Copyright (c) 2013 Calum Barnes

Adafruit GPS parsing example, DHT tester - ladyada

MPL3115A2 Barometric Pressure Sensor Library Example Code
By: Nathan Seidle
SparkFun Electronics
Date: September 24th, 2013

library example for the HMC5883 magnentometer/compass
Written by Kevin Townsend for Adafruit Industries with some heading example from
  Love Electronics (loveelectronics.co.uk)
*/

#include <Adafruit_GPS.h>
#include <Wire.h>
#include "MPL3115A2.h"
#include <SoftwareSerial.h>
#include <SD.h>

#include <SPI.h>
#include <WiFi.h>
#include <HttpClient.h>
#include "DHT.h"
#include <Xively.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
    Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
        void displaySensorDetails(void)
{
  sensor_t sensor;                                            //displays compass sensor information
  mag.getSensor(&sensor);    
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

MPL3115A2 myPressure;

Adafruit_GPS GPS(&Serial1);
boolean usingInterrupt = false;  // ADafruit GPS likes to use interrupts, which are not working well with our setup
//void useInterrupt(boolean);

char ssid[] = "Mconnect"; //  your network SSID (name) 
char pass[] = "";    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;            // your network key Index number (needed only for WEP)
 
int status = WL_IDLE_STATUS;
//WiFiServer server(80); 
// Your Xively key to let you upload data
char xivelyKey[] = "bKeZHx0198IJEpc0akjolePFZJUCFz8uuCouo1B9Oiq2vt4t"; //"pUty0uR%1v3lyK3/h3r3";

//your xively feed ID
#define xivelyFeed  1927926610 //8675309 
//datastreams
char sensorID[] = "AmbientLight";
char homeID[] = "Panel_Homed";
char dtID[] = "Temperature";
char dhID[] = "Humidity";
char baromID[] = "Barometer";
char headingID[] = "Heading";
char speedID[] = "Speed";
char satID[] = "Satellites";
char latID[] = "Lat";
char lonID[] = "Lon";

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences.
#define GPSECHO  false 


#define ldtop     A10    //Red wire from Relay for solar tracking motors  --- panel Breakout Cable #1
#define ldbot     A11    //Brown
#define ldlef     A12    //Green
#define ldrig     A13    //Blue
#define AmblightPin A14   //Ambient Light pin on A14 for temt6000
// digital pin declarations!
#define ledPin 9
#define relayup            24 //Purple wire   these are outputs            --- panel Breakout Cable # 2
#define relaydown          25 //Grey 
#define relayright         26 //Brown
#define relayleft          27 //Blue from photosensor cable

#define DHTPin             22
#define DHTTYPE DHT11

// Touch sensors
#define LimitUp          28  // Is the panel facing forward
#define LimitDown        29  // Is the panel down
#define limitLeft        30  // Did we go max left
#define limitRight       31  // Did we go max right
#define limitHome        32  // is the solar panel at center
 
#define trikeLights        50

 
DHT dht(DHTPin, DHTTYPE);
// Define the strings for our datastream IDs
XivelyDatastream datastreams[] = {
  XivelyDatastream(sensorID, strlen(sensorID), DATASTREAM_FLOAT),   //0
  XivelyDatastream(homeID, strlen(homeID), DATASTREAM_FLOAT),         //1
  XivelyDatastream(dtID, strlen(dtID), DATASTREAM_FLOAT),           //2
  XivelyDatastream(dhID, strlen(dhID), DATASTREAM_FLOAT),           //3
  XivelyDatastream(baromID, strlen(baromID), DATASTREAM_FLOAT),     //4
  XivelyDatastream(headingID, strlen(headingID), DATASTREAM_FLOAT), //5
  XivelyDatastream(speedID, strlen(speedID), DATASTREAM_FLOAT),     //6
  XivelyDatastream(satID, strlen(satID), DATASTREAM_FLOAT),         //7 
  XivelyDatastream(latID, strlen(latID), DATASTREAM_FLOAT),         //8
  XivelyDatastream(lonID, strlen(lonID), DATASTREAM_FLOAT),         //9 
};

XivelyFeed feed(xivelyFeed, datastreams, 10 /* number of datastreams */);

WiFiClient client;
XivelyClient xivelyclient(client);
 
void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
 
  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
 
  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm \n");
 
}

unsigned long start, finish, duration;
int counter;
void setup() {
Setup();
}
 
void loop() {
   
  //adjust LED level. set from Xively
  int getReturn = xivelyclient.get(feed, xivelyKey);    //get data from xively
  if(getReturn > 0){
    
  }else Serial.println("HTTP Error");
  
  //write value to LED - change brightness
/*  int level = feed[1].getFloat();
  if(level < 0){
    level = 0;
  }else if(level > 255){
    level = 255;
  }*/

  // If ambient light is too low, enable the trikes lights.
  int level = constrain(datastreams[0].getFloat(), 0, 255);
 // level = constrain(level, 0, 255);
  if (level < 30) {
    digitalWrite(trikeLights, HIGH);
   digitalWrite(ledPin, HIGH); // Turn on the lights
  } else {
    digitalWrite(trikeLights, LOW);
    digitalWrite(ledPin, LOW);  // Turn the lights off
  }
  // Write the value
  digitalWrite(ledPin, level);
 
////////////////////////////////////////////////////////////////////////////////////////////////
  //read sensor values
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  float pressure = myPressure.readPressure();
  float temperature = myPressure.readTempF();  // needed for adjustment of Barometer
  const int station_elevation_m = 188; // MCC's elevation in meters on placekeeper.com
  //1 pascal = 0.01 millibars
  pressure /= 100; //pressure is now in millibars
  float part1 = pressure - 0.3; //Part 1 of formula
  const float part2 = 8.42288 / 100000.0;
  float part3 = pow((pressure - 0.3), 0.190284);
  float part4 = (float)station_elevation_m / part3;
  float part5 = (1.0 + (part2 * part4));
  float part6 = pow(part5, (1.0/0.190284));
  float altimeter_setting_pressure_mb = part1 * part6; //Output is now in adjusted millibars
  float baroin = altimeter_setting_pressure_mb * 0.02953;
  int LightValue = analogRead(AmblightPin);
   
 
  
  datastreams[0].setFloat(LightValue);
   
   datastreams[2].setFloat(Fahrenheit(t));
    datastreams[3].setFloat(h);
      datastreams[4].setFloat(baroin);  
   
        compass();  //datastream 5 is in compass routine. Doing calculations in function instead of up top.
        Serial.println("going to gps");
        gps(); // datastream 6 - 9 (sat count and speed) are in gps function
 
  //print the sensor valye
  
  Serial.println("sensor values ");
  Serial.println(datastreams[0].getFloat());
  Serial.println(datastreams[1].getFloat());
  Serial.println(datastreams[2].getFloat());
  Serial.println(datastreams[3].getFloat());
  Serial.println(datastreams[4].getFloat());
  Serial.println(datastreams[5].getFloat());
  Serial.println(datastreams[6].getFloat());
  Serial.println(datastreams[7].getFloat());
  //send value to xively
    
  Serial.println("Uploading it to Xively");
  int ret = xivelyclient.put(feed, xivelyKey);
  //return message
  Serial.print("xivelyclient.put returned ");
  Serial.println(ret);
  Serial.println("");
  
  
 
   delay(50);
  SolarTracker(counter);
  Serial.print("\t");
  Serial.println("out of tracker function");
  Serial.print("counter =    ");
  Serial.print(SolarTracker(counter) );  // I made SolarTracker into a variable returning function to help with timing. This way,
  if (SolarTracker(counter) < 4999){     //  a counter within solartracker will be able to spit out its info to the main loop,
 for (int x = 0; x < 15000; x++){      //   and we use either the 18sec delay of counter in the solar function,
                                      //    or a 15 sec delay (x) between calls here -- KC.
    
   delay(1);
  // Serial.print("     x =    ");
  // Serial.println(x);
  
  } Serial.println("     delayed 15 seconds   ");
    } else { 
   return; }
}
//-------------------------------------------------------------------------------------------------------

double Fahrenheit(double t)
{
       return (t*9 +2)/5 + 32;
	//return 1.8 * t + 32;
}


//--------------------------------------------------------------------------------------------------------


/* These are the timer interrput functions for the gps to function 
while everything else is going. This allows us to read from the gps while
running the solar panel function and upload data to xively.
This has to stay down here to compile correctly. -KC
*/
  

SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}
//_____________________END_OF_TIMER_FUNCTION____________________________________
