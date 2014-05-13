/*Modified from the following:

##Xively WiFi Sensor Tutorial## 
By Calum Barnes 3-4-2013
BSD 3-Clause License - [http://opensource.org/licenses/BSD-3-Clause]
Copyright (c) 2013 Calum Barnes


*/
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

MPL3115A2 myPressure;

TinyGPS gps;
SoftwareSerial ss(2,3);
File rawdata;

char ssid[] = "Mconnect"; //  your network SSID (name) 
char pass[] = " ";    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;            // your network key Index number (needed only for WEP)
 
int status = WL_IDLE_STATUS;
 
// Your Xively key to let you upload data
char xivelyKey[] = "puty0ur%iv37yk3/h3r3";
//your xively feed ID
#define xivelyFeed 867-5309
//datastreams
char sensorID[] = "LIGHT_SENSOR_CHANNEL";
char ledID[] = "LED_CHANNEL";
char dtID[] = "Temperature";
char dhID[] = "Humidity";
char baromID[] = "Barometer";


 
// Analog pin which we're monitoring (0 and 1 are used by the Ethernet shield)

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
  XivelyDatastream(sensorID, strlen(sensorID), DATASTREAM_FLOAT),
  XivelyDatastream(ledID, strlen(ledID), DATASTREAM_FLOAT),
  XivelyDatastream(dtID, strlen(dtID), DATASTREAM_FLOAT),
  XivelyDatastream(dhID, strlen(dhID), DATASTREAM_FLOAT),
  XivelyDatastream(baromID, strlen(baromID), DATASTREAM_FLOAT),
};
// Finally, wrap the datastreams into a feed
XivelyFeed feed(xivelyFeed, datastreams, 4 /* number of datastreams */);
 
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
  Serial.begin(9600);
  Wire.begin(); 
  myPressure.begin();
  myPressure.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa
  myPressure.setOversampleRate(7); // Set Oversample to the recommended 128
  myPressure.enableEventFlags();
  dht.begin();
  //pin setup
  for(int pinNumber =22; pinNumber<26; pinNumber++){  
      pinMode(pinNumber,OUTPUT);
      digitalWrite(pinNumber, LOW);
  }  
  pinMode(sensorPin, INPUT);
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
      datastreams[4].setFloat(pressure);
  //print the sensor valye
  Serial.print("Read sensor value ");
  Serial.println(datastreams[0].getFloat());
  Serial.println(datastreams[2].getFloat());
  Serial.println(datastreams[3].getFloat());
  
 
  //send value to xively
  Serial.println("Uploading it to Xively");
  int ret = xivelyclient.put(feed, xivelyKey);
  //return message
  Serial.print("xivelyclient.put returned ");
  Serial.println(ret);
  Serial.println("");
  
  
  //delay between calls
  solartracker();
  delay(10000);
}

double Fahrenheit(double t)
{
       return (t*9 +2)/5 + 32;
	//return 1.8 * t + 32;
}

void Location(){
  
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  // Open data file
  File rawdata = SD.open("rawdata.txt", FILE_WRITE);
  
  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (ss.available())
    {
      char c = ss.read();
       // Send raw data to the terminal window
       Serial.write(c); 
       
       // Send raw data to the SD card
       if(rawdata)
         rawdata.write(c);
         
      if (gps.encode(c)) 
        newData = true;
    }
  }
  // Send manipulate data to the terminal window.
  /* 
  if (newData)
  {
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    Serial.print("LAT=");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(" LON=");
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    Serial.print(" SAT=");
    Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    Serial.print(" PREC=");
    Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
  }*/
  
  // Send a carriage return and close the file
  Serial.println("");
  rawdata.write("\r");
  rawdata.close();
  
}

void solartracker()
{
  int top = analogRead(ldtop); // Top of panel
  int bot = analogRead(ldbot); // Bottom of Panel
  int lef = analogRead(ldlef); // Left side of Panel
  int rig = analogRead(ldrig); //Right side of Panel
  
  
  Serial.print("TOP  ");
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
