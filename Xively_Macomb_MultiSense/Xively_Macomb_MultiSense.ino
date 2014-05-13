/*Modified from the following:

##Xively WiFi Sensor Tutorial## 
By Calum Barnes 3-4-2013
BSD 3-Clause License - [http://opensource.org/licenses/BSD-3-Clause]
Copyright (c) 2013 Calum Barnes


*/
#include <SPI.h>
#include <WiFi.h>
#include <HttpClient.h>
#include "DHT.h"
#include <Xively.h>
 
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
 
// Analog pin which we're monitoring (0 and 1 are used by the Ethernet shield)
#define sensorPin A2 //Ambient Light pin on A2
#define DHTTYPE DHT11
#define DHTPin 24
#define ledPin 9
 
DHT dht(DHTPin, DHTTYPE);
// Define the strings for our datastream IDs
XivelyDatastream datastreams[] = {
  XivelyDatastream(sensorID, strlen(sensorID), DATASTREAM_FLOAT),
  XivelyDatastream(ledID, strlen(ledID), DATASTREAM_FLOAT),
  XivelyDatastream(dtID, strlen(dtID), DATASTREAM_FLOAT),
  XivelyDatastream(dhID, strlen(dhID), DATASTREAM_FLOAT),
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
  //pin setup
  dht.begin();
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
  int sensorValue = analogRead(sensorPin);
  datastreams[0].setFloat(sensorValue);
   datastreams[2].setFloat(Fahrenheit(t));
    datastreams[3].setFloat(h);
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
  delay(15000);
}

double Fahrenheit(double t)
{
       return (t*9 +2)/5 + 32;
	//return 1.8 * t + 32;
}

