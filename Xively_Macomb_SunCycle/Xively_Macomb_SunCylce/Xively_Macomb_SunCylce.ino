
/*Macomb Suncycle by K.Cole, R.Shirley, A.Distel
/*Modified from the following:

##Xively WiFi Sensor Tutorial## 
By Calum Barnes 3-4-2013
BSD 3-Clause License - [http://opensource.org/licenses/BSD-3-Clause]
Copyright (c) 2013 Calum Barnes


*/
#include <Adafruit_GPS.h>
#include <Wire.h>
#include "MPL3115A2.h"
#include <SoftwareSerial.h>
#include <SD.h>
#include <TinyGPS.h>
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
WiFiServer server(80); 
// Your Xively key to let you upload data
char xivelyKey[] = "pUty0ur%1v37yK3yh3r3";
//your xively feed ID
#define xivelyFeed   8679305
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


#define ldtop     A10
#define ldbot     A11
#define ldlef     A12
#define ldrig     A13
#define sensorPin A14   //Ambient Light pin on A14 for temt6000
// digital pin declarations!
#define relayup            22
#define relaydown          23
#define relayright         24
#define relayleft          25
#define  extraTrackerPIN   26


#define DHTTYPE DHT11 
#define DHTPin 27
#define ledPin 9
 
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
  server.begin();                           // start the web server on port 80
  printWifiStatus();  
  myPressure.begin();
      myPressure.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa
      myPressure.setOversampleRate(7); // Set Oversample to the recommended 128
      myPressure.enableEventFlags();
  GPSsetup();
    
  dht.begin();
  //pin setup
  for(int pinNumber =22; pinNumber<26; pinNumber++){  
      pinMode(pinNumber,OUTPUT);
      digitalWrite(pinNumber, LOW);
  }  
  pinMode(sensorPin, INPUT);
  pinMode(DHTPin, INPUT);
  pinMode(ledPin, OUTPUT);
  
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
  int level = feed[1].getFloat();
  if(level < 0){
    level = 0;
  }else if(level > 255){
    level = 255;
  }
  //actually write the value
  digitalWrite(ledPin, level);
 
///////////////////////////////////////////////////////
  //read sensor values
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  float pressure = myPressure.readPressure();
  int sensorValue = analogRead(sensorPin);
   
 
  
  datastreams[0].setFloat(sensorValue);
   datastreams[2].setFloat(Fahrenheit(t));
    datastreams[3].setFloat(h);
      datastreams[4].setFloat(pressure);   //datastream 5 is in compass routine
        compass();
        gps();
  //print the sensor valye
  client.print("Read sensor value ");
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
  solartracker();
  delay(15000);
}


double Fahrenheit(double t)
{
       return (t*9 +2)/5 + 32;
	//return 1.8 * t + 32;
}


void solartracker()
{
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
 
  if(top > bot)
{  
  digitalWrite(relayup, LOW);
  digitalWrite(relaydown, HIGH);
  delay (10);
}
  else if(top < bot)
{
  digitalWrite(relayup, HIGH);
  digitalWrite(relaydown, LOW);
  delay (10); 
}
  else if (top == bot)
{
  digitalWrite(relayup, LOW);
  digitalWrite(relaydown, LOW);
  delay (10);  
  
  }
    if(lef > rig)
{  
  digitalWrite(relayright, LOW);
  digitalWrite(relayleft, HIGH);
  delay (10);
}
  else if(lef < rig)
{
  digitalWrite(relayright, HIGH);
  digitalWrite(relayleft, LOW);
  delay (10); 
}
  else if (lef == rig)
{
  digitalWrite(relayright, LOW);
  digitalWrite(relayleft, LOW);
  delay (10);
  
  
  
  }

}


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

void gps(){
 
delay(10);
uint32_t timer = millis();
                     // run over and over again

  // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
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

  delay(1000);
  
}
  // Ask for firmware version
//  mySerial.println(PMTK_Q_RELEASE);
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



