
/*Macomb Suncycle by K.Cole, R.Shirley, A.Distel.   ####Rich- your solartracking subroutine is stuck in a loop. Race condition?###
/*Modified and mutated from the following:          ####  I commented it out for now - KC                                      ###

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
boolean usingInterrupt = false;  // 
void useInterrupt(boolean);     

char ssid[] = "Mconnect"; //  your network SSID (name) 
char pass[] = "";    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;            // your network key Index number (needed only for WEP)
 
int status = WL_IDLE_STATUS;
//WiFiServer server(80); 
// Your Xively key to let you upload data
char xivelyKey[] = "pUty0uR%1v3lyK3/h3r3";

//your xively feed ID
#define xivelyFeed   8675309
//datastreams
char sensorID[] = "AmbientLight";
char ledID[] = "LED";
char dtID[] = "Temperature";
char dhID[] = "Humidity";
char baromID[] = "Barometer";
char headingID[] = "Heading";
char speedID[] = "Speed";
char satID[] = "Satellites";

#define GPSECHO  true 

<<<<<<< HEAD
//Analog pin definitions!
#define ldtop     A10
#define ldbot     A11
#define ldlef     A12
#define ldrig     A13
#define sensorPin A14   //Ambient Light pin on A14 for temt6000

// digital pin definitions!
#define ledPin 		    9	//status led on wifi shield
#define relayup            22	//These are
#define relaydown          23	//for Rich's
#define relayright         24	//Relay Control board
#define relayleft          25	//for the solar panel tracking motors

#define DHTPin 		   27
#define trikeLights        28	//White undercarriage leds, controlled by mosfet
		// Touch sensors
#define homeBase           30	// Is the panel facing forward
#define homeTop            31	// Is the panel down
#define extremeLeft        32	// Did we go max left
#define extremeRight       33	// Did we go max right

#define DHTTYPE DHT11 // We are using the cheap blue DHT11 variety of Digital Temperature Humidity sensors
=======

#define ldtop     A10    //Red wire from Relay for solar tracking motors
#define ldbot     A11    //Brown
#define ldlef     A12    //Green
#define ldrig     A13    //Blue
#define sensorPin A14   //Ambient Light pin on A14 for temt6000
// digital pin declarations!
#define relayup            22 //Purple wire from photosensor cable
#define relaydown          23 //Grey 
#define relayright         24 //Brown
#define relayleft          25 //Blue from photosensor cable
#define extraTrackerPIN    26
#define DHTPin             27
#define trikeLights        28

// Touch sensors
#define homeBase           30  // Is the panel facing forward
#define homeTop            31  // Is the panel down
#define extremeLeft        32  // Did we go max left
#define extremeRight       33  // Did we go max right

#define DHTTYPE DHT11 

#define ledPin 9
 
>>>>>>> Commented Out Race Condition- Solar Tracking function
DHT dht(DHTPin, DHTTYPE);

// Define the strings for our datastream IDs
XivelyDatastream datastreams[] = {
  XivelyDatastream(sensorID, strlen(sensorID), DATASTREAM_FLOAT),   //0
  XivelyDatastream(ledID, strlen(ledID), DATASTREAM_FLOAT),         //1
  XivelyDatastream(dtID, strlen(dtID), DATASTREAM_FLOAT),           //2
  XivelyDatastream(dhID, strlen(dhID), DATASTREAM_FLOAT),           //3
  XivelyDatastream(baromID, strlen(baromID), DATASTREAM_FLOAT),     //4
  XivelyDatastream(headingID, strlen(headingID), DATASTREAM_FLOAT), //5
  XivelyDatastream(speedID, strlen(speedID), DATASTREAM_FLOAT),     //6
  XivelyDatastream(satID, strlen(satID), DATASTREAM_FLOAT),         //7
};

XivelyFeed feed(xivelyFeed, datastreams, 8 /* number of datastreams */);

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



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin(); 
 // server.begin();                           // start the web server on port 80

  myPressure.begin();
      myPressure.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa
      myPressure.setOversampleRate(7); // Set Oversample to the recommended 128
      myPressure.enableEventFlags();
  GPSsetup();
    
  dht.begin();
  //pin setup
  for(int pinNumber =22; pinNumber<27; pinNumber++){  
      pinMode(pinNumber,OUTPUT);
      digitalWrite(pinNumber, LOW);
  }  
  pinMode(sensorPin, INPUT);
  pinMode(DHTPin, INPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(trikeLights, OUTPUT);
  
  for (int pinNumber=30; pinNumber < 34; pinNumber++) {
     pinMode(pinNumber, INPUT); 
  }
  
  Serial.println("Starting single datastream upload to Xively...");
  Serial.println();
 
  // attempt to connect to Wifi network:
  while ( status != WL_CONNECTED) { 
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid);
    // wait 10 seconds for connection:
    delay(10000);
  } 
  Serial.println("Connected to wifi");

  printWifiStatus();
   Serial.println("HMC5883 Magnetometer Test"); Serial.println("");
  
    /* Initialise the sensor */
    if(!mag.begin())
    {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }
  
  /* Display some basic information on this sensor */
  displaySensorDetails();
}
 
void loop() {
 
  //adjust LED level. set from Xively
  int getReturn = xivelyclient.get(feed, xivelyKey);    //get data from xively
  if(getReturn > 0){
    Serial.println("LED Datastream");
    Serial.println(feed[1]);
  }else Serial.println("HTTP Error");
  
  //write value to LED - change brightness
/*  int level = feed[1].getFloat();
  if(level < 0){
    level = 0;
  }else if(level > 255){
    level = 255;
  }*/

  // If ambient light is too low, enable the trikes lights.
  int level = constrain(feed[1].getFloat(), 0, 255);
  //level = constrain(level, 0, 255);
  if (level < 30) {
    digitalWrite(trikeLights, HIGH); // Turn on the lights
  } else {
    digitalWrite(trikeLights, LOW);  // Turn the lights off
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
  int sensorValue = analogRead(sensorPin);
   
 
  
  datastreams[0].setFloat(sensorValue);
   datastreams[2].setFloat(Fahrenheit(t));
    datastreams[3].setFloat(h);
      datastreams[4].setFloat(baroin);  
        compass();  //datastream 5 is in compass routine. Doing calculations in function instead of up top.
        gps(); // datastream 6 & 7 (sat count and speed) are in gps function
 
  //print the sensor valye
  Serial.print("Read sensor value ");
  Serial.println(datastreams[0].getFloat());
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
  
  
  //delay between calls
  //solartracker();
  delay(15000);
}
//-------------------------------------------------------------------------------------------------------

double Fahrenheit(double t)
{
       return (t*9 +2)/5 + 32;
	//return 1.8 * t + 32;
}


//--------------------------------------------------------------------------------------------------------
void solartracker()
{
  String side = "none";
int lastRun = 0;
  // Check every 15 minutes for change. 
  if ( (lastRun - millis() ) < 900000 ) {
    return;  // Cannot run yet
  } else {
   
  lastRun = millis();  
  int HB, HT, EL, ER;  // Home base, home top, extreme left, extreme right
  int notDone = 1;
  // If in motion ensure the system is down or put it down
  if (GPS.speed > 1 && ( ( HB = digitalRead(homeBase) == LOW) || (HT = digitalRead(homeTop) == LOW ) ) ) {
    while (notDone) {
     if (HB == LOW) {
      if (side == "left") {
       digitalWrite(relayleft, LOW);
       digitalWrite(relayright, HIGH);
      } 
      else if (side == "right")
      {
       digitalWrite(relayleft, HIGH);
       digitalWrite(relayright, LOW);
      }
    }
     if (HT == LOW) {
      digitalWrite(relayup, LOW);
      digitalWrite(relaydown, HIGH); 
     }
     HB = digitalRead(homeBase);
     HT = digitalRead(homeTop);
     if ( ( HT == HIGH ) && (HB == HIGH) ) { notDone = 0;}
    }
    return; // Do not continue on if speed is higher than 0
  }
  
  
  notDone = 1;  // Recycle the variable
  int top = analogRead(ldtop); // Top of panel
  int bot = analogRead(ldbot); // Bottom of Panel
  int lef = analogRead(ldlef); // Left side of Panel
  int rig = analogRead(ldrig); //Right side of Panel
  
  
  Serial.println("TOP  ");
  Serial.print(top);
  Serial.print("\t");
  Serial.print("BOTTOM  ");
  Serial.print(bot);
  Serial.print("\t");
  Serial.print("LEFT  ");
  Serial.print(lef);
  Serial.print("\t");
  Serial.print("RIGHT  ");
  Serial.println(rig);
  
  int sidesDone = 0;
  int topDone = 0;
  while (notDone) {
    if (top > bot)
    {
      digitalWrite(relayup, LOW);
      digitalWrite(relaydown, HIGH);
    }
    else if (top < bot) 
    {
     digitalWrite(relayup, HIGH);
     digitalWrite(relaydown, LOW); 
    }
    else if (top == bot) // Equalized!
    {
      digitalWrite(relayup, LOW);
      digitalWrite(relaydown, LOW);
    }
    
    HB = digitalRead(homeBase);
    if (lef > rig && !sidesDone)
    {
      if (EL = digitalRead(extremeLeft) == HIGH) { sidesDone = 1; continue; }  // Cannot move left any more, side movement done
      if (HB == HIGH) { side = "left"; }
      digitalWrite(relayright, LOW);
      digitalWrite(relayleft, HIGH);
    }
    else if (lef < rig && !sidesDone)
    {
      if (ER = digitalRead(extremeRight) == HIGH) {sidesDone = 1; continue; } // Cannot move right any more, side movement done
      if (HB == HIGH) { side = "right"; }
      digitalWrite(relayleft, LOW);
      digitalWrite(relayright, HIGH);
    }
    else if ( rig == lef)
    {
      sidesDone = 1;
      side = "none";
      digitalWrite(relayleft, LOW);
      digitalWrite(relayright, LOW);
    }
       
    if ( ( top == bot ) && (lef == rig) ) { notDone = 0;}
    }
  }
 }

//____________________________________________________________________________________________________

void compass() 
{
  /* Get a new sensor event */ 
  sensors_event_t event; 
  mag.getEvent(&event);
 
  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = 0.22;
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI; 
  
  Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
   datastreams[5].setFloat(headingDegrees);
  //delay(500);
}
//_____________________________________________________________________________________________________
void gps(){
 
delay(10);
uint32_t timer = millis();
                     // run over and over again

  // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    //char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    //if (GPSECHO)
      //if (c) Serial.print(c);
  }
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
 // if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
 // if (millis() - timer > 2000) { 
   // timer = millis(); // reset the timer
    
    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", "); 
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
       float sat = GPS.satellites;
      float speedmph = GPS.speed*1.15078;
      datastreams[6].setFloat(speedmph);
      datastreams[7].setFloat(sat);
      datastreams[8].setFloat(GPS.lat);
    
    }
  }
 
void GPSsetup()  
{
  boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy  
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  Serial.println("Adafruit GPS library basic test!");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  useInterrupt(true);

  delay(5000);
  
}

//________________________________________________________________________________
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
