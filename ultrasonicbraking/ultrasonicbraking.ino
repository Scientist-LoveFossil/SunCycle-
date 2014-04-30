/*

 HC-SR04 Ping distance sensor:

 VCC to arduino 5v 

 GND to arduino GND

 Echo to Arduino pin 7 

 Trig to Arduino pin 8

 

 This sketch originates from Virtualmix: http://goo.gl/kJ8Gl

 Has been modified by Winkle ink here: http://winkleink.blogspot.com.au/2012/05/arduino-hc-sr04-ultrasonic-distance.html

 And modified further by ScottC here: http://arduinobasics.blogspot.com/

 on 10 Nov 2012.

 */


#include <Servo.h> 
Servo brake;


#define echoPin 7 // Echo Pin

#define trigPin 8 // Trigger Pin

#define LEDPin 3 // Pwm LED brake
#define STATpin 13 //onboard status led




int maximumRange = 200; // Maximum range needed
int braking;
int minimumRange = 30; // Minimum range needed

long duration, distance; // Duration used to calculate distance


void setup() {
 brake.attach(2);
 
 Serial.begin (9600);

 pinMode(trigPin, OUTPUT);

 pinMode(echoPin, INPUT);

 pinMode(LEDPin, OUTPUT); // Use LED indicator (if required)
 pinMode(STATpin, OUTPUT);
}



void loop() {

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




Serial.println(distance);

brake.write(distance);
delay(15);
digitalWrite(STATpin, LOW);


delay(75);
 }

 

 //Delay 50ms before next reading.








