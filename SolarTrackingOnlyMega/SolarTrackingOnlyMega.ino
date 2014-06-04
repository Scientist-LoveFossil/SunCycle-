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
void setup(){
  Serial.begin(115200);
   for(int pinNumber =22; pinNumber<27; pinNumber++){  
      pinMode(pinNumber,OUTPUT);
      digitalWrite(pinNumber, LOW);
      }
}
void loop(){
  String side = "none";
int lastRun = 0;

  // Check every 15 minutes for change. 
  if ( (lastRun - millis() ) < 900000 ) {
    return;  // Cannot run yet
  } else {
   
  //lastRun = millis();  
  int HB, HT, EL, ER;  // Home base, home top, extreme left, extreme right
  int notDone = 1;
  // If in motion ensure the system is down or put it down
 // if (GPS.speed > 0 && ( ( HB = digitalRead(homeBase) == LOW) || (HT = digitalRead(homeTop) == LOW ) ) ) {
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
   // return; // Do not continue on if speed is higher than 0
//  }
  
  
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
