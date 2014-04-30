/*
  WiFi Web Server

 A simple web server that shows the value of the analog input pins.
 using a WiFi shield.

 This example is written for a network using WPA encryption. For
 WEP or WPA, change the Wifi.begin() call accordingly.

 Circuit:
 * WiFi shield attached
 * Analog inputs attached to pins A0 through A5 (optional)

 created 13 July 2010
 by dlf (Metodo2 srl)
 modified 31 May 2012
 by Tom Igoe

 */

#include <SPI.h>
#include <WiFi.h>
#define echoPin  5
#define trigPin  6
#define StatLED  9

char ssid[] = "Mconnect";      // your network SSID (name)
char pass[] = "secretPassword";   // your network password
int keyIndex = 0;                 // your network key Index number (needed only for WEP)

int maximumRange = 200; // Maximum range needed for ultrasonic
int minimumRange = 30; // Minimum range needed for ultrasonic
long duration, distance; // Duration used to calculate distance -- ultrasonic
long brake;

int status = WL_IDLE_STATUS;

WiFiServer server(80);

void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);

  pinMode(echoPin, INPUT);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }

  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue:
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if ( fv != "1.1.0" )
    Serial.println("Please upgrade the firmware");

  // attempt to connect to Wifi network:
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid);

    // wait 10 seconds for connection:
    delay(10000);
  }
  server.begin();
  // you're connected now, so print out the status:
  printWifiStatus();
}


void loop() {
  // listen for incoming clients
  WiFiClient client = server.available();
  if (client) {
    Serial.println("new client");
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        Serial.write(c);
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {
          // send a standard http response header
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/html");
          client.println("Connection: close");  // the connection will be closed after completion of the response
          client.println("Refresh: 5");  // refresh the page automatically every 5 sec
          client.println();
          client.println("<!DOCTYPE HTML>");
          client.println("<html>");
          // output the value of each analog input pin
          for (int analogChannel = 0; analogChannel < 6; analogChannel++) {
            //int sensorReading = analogRead(analogChannel);
            ultrasonic();
            client.print("distance is     ");
            client.print(distance);
            client.print("   cm  ");
            //client.print("analog input ");
            //client.print(analogChannel);
            //client.print(" is ");
            //client.print(sensorReading);
            client.println("<br />");
          }
          client.println("</html>");
          break;
        } //bracket for FOR analog statement
        if (c == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
        }
        else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
    // give the web browser time to receive the data
    delay(1);

    // close the connection:
    client.stop();
    Serial.println("client disonnected");
  }
}


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
  Serial.println(" dBm");
}

void ultrasonic() {

/* The following trigPin/echoPin cycle is used to determine the

 distance of the nearest object by bouncing soundwaves off of it. */ 

 digitalWrite(trigPin, LOW); 

 delayMicroseconds(2); 



 digitalWrite(trigPin, HIGH);

 delayMicroseconds(10); 

 

 digitalWrite(trigPin, LOW);

 duration = pulseIn(echoPin, HIGH);

 

 //Calculate the distance (in cm) based on the speed of sound.

 distance = duration/58.2;
 brake = map(distance, maximumRange, minimumRange, 0, 255);

 if (distance >= maximumRange){

 /* Send a negative number to computer and Turn LED ON 

 to indicate "out of range" */

 Serial.println("out of range");

 digitalWrite(StatLED, HIGH); 

 }

 else {

 /* Send the distance to the computer using Serial protocol, and

 turn LED OFF to indicate successful reading. */

 Serial.println(distance);
//Serial.println(brake);
analogWrite(StatLED, distance);

 }

 

 //Delay 50ms before next reading.

 delay(75);

} 



