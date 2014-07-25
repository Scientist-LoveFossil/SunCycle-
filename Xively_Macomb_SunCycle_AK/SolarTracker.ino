void SolarTracker(){
 digitalWrite(relayup, LOW);
 digitalWrite(relaydown, LOW);
 digitalWrite(relayleft, LOW);
 digitalWrite(relayright, LOW);
 Serial.println("in solar tracker function- relays are low");
 // for (int counter = 0; counter < 5000; counter++){
       delay(1);
  
  String side = "none";
  
  int top = analogRead(ldtop); // Top light dependant sensor on Panel
  int bot = analogRead(ldbot); // Bottom of Panel
  int lef = analogRead(ldlef); // Left side of Panel
  int rig = analogRead(ldrig); //Right side of Panel
  int tol = 50;  // tolerance for levels between panels
  int HB = digitalRead(limitUp);
  int HT = digitalRead(limitDown);
  int EL = digitalRead(limitLeft); 
  int ER = digitalRead(limitRight);  // Home base, home top, extreme left, extreme right
  int notDone = true;
  int homed = digitalRead(limitHome);
  
  // If in motion ensure the system is down or put it down
  if (GPS.speed > 0.30 && ( ( HB = digitalRead(limitUp) == LOW) || (HT = digitalRead(limitDown) == LOW ) ) ) {
    while (notDone == true) {
     if (HB == LOW) {
      if (side == "left") {
       digitalWrite(relayleft, LOW);
       digitalWrite(relayright, HIGH);
       delay(500);
       digitalWrite(relayright, LOW);
       Serial.println (" limit Right ");
       Serial.println (ER);
       readvals();
      } 
      else if (side == "right") {
        digitalWrite(relayleft, HIGH);
        digitalWrite(relayright, LOW);
        delay (500);
        digitalWrite(relayleft, LOW);
        
        Serial.println (" limit Left ");
        Serial.println (EL);
        readvals();
      }
    }
     if (HT == LOW) {
      digitalWrite(relayup, LOW);
      digitalWrite(relaydown, HIGH); 
      delay (500);
      digitalWrite(relaydown, LOW);
       Serial.println (" limit Down ");
      Serial.println (HT);
     }
     digitalRead(limitUp);
     digitalRead(limitDown);
     if ( ( HT == HIGH ) && (HB == HIGH) ) { notDone = false;}
     
    } // end While
   
   datastreams[1].setFloat(homed);
    return; // Do not continue on if speed is higher than 0
  } //end If GPS.speed
  
  
   // Recycle the variable
  int rup = digitalRead(relayup);  // for debugging if relay outputs are high or low
  int rdown = digitalRead(relaydown);
  int rleft = digitalRead(relayleft);
  int rright = digitalRead(relayright);
  readvals();
 

//  int dvert = top - bot; // check the diffirence of up and down
//  int dhoriz = lef - rig;// check the diffirence of left and right
   notDone = true;
  
  //Serial.println("\t");
  Serial.println ("Panel Sensor Values");
  Serial.println ("TOP ");      Serial.print(ldtop);  Serial.print("\t");
  Serial.print("BOTTOM  ");  Serial.print(ldbot);  Serial.print("\t");
  Serial.print("LEFT  ");    Serial.print(ldlef);  Serial.print("\t");
  Serial.print("RIGHT  ");   Serial.print(ldrig);  Serial.println("\t");
  Serial.print (" limitRight ");  Serial.print (ER);  Serial.print("\t");
  Serial.print (" limitLeft ");   Serial.print (EL);  Serial.println("\t");
  Serial.print (" limitUp ");     Serial.print (HB);  Serial.print("\t");
  Serial.print (" limitDown ");   Serial.print (HT);  Serial.println("\t");
  digitalRead(homed);
  Serial.print (" limit Home ");  Serial.print (homed);
 
  int sidesDone = false;
  int topDone = false;
  //break;
  while (notDone == true) {
 //   Serial.println("  not done  ");
 
  /*  Serial.print("Speed =   ");
    Serial.print(GPS.speed);
    Serial.print("\t");
    Serial.print("dvert =   ");
    Serial.print(dvert);
    Serial.print("\t");
    Serial.print("dhoriz =   ");
    Serial.print(dhoriz);
    Serial.println("\t"); */
      
    //if (-1*tol > dvert || dvert > tol)
    if (top > bot  && !topDone)
    {
      digitalWrite(relayup, LOW);
      digitalWrite(relaydown, HIGH);
      delay (1000);
      digitalWrite(relaydown, LOW);
      Serial.println("Top is Greater");
     
    }
    else if (top < bot && !topDone) 
    {
     digitalWrite(relaydown, LOW);
     digitalWrite(relayup, HIGH);
     delay (1000);
     digitalWrite(relayup, LOW);
    Serial.println("Bottom is Greater");
   // Serial.println(rdown);
    }
    else if ( abs(top - bot) < tol )  // Equalized!
    {
      topDone = true;
      Serial.print ("equal!");
      digitalWrite(relayup, LOW);
      digitalWrite(relaydown, LOW);
    }  // End up and downs
    
    HB = digitalRead(limitUp);
    //if (-1*tol > dhoriz || dhoriz > tol){
    if (lef > rig && !sidesDone)
    {
      if (EL = digitalRead(limitLeft) == HIGH) { sidesDone = true; continue; }  // Cannot move left any more, side movement done
      if (HB == HIGH) { side = "left"; }
      Serial.print ("Left limit reached- moving right");
      digitalWrite(relayright, HIGH);
      digitalWrite(relayleft, LOW);
      delay (1000);
      digitalWrite(relayright, LOW);limit
    }
    else if (lef < rig && !sidesDone)
    {
      if (ER = digitalRead(limitRight) == HIGH) {sidesDone = true; continue; } // Cannot move right any more, side movement done
      if (HB == HIGH) { side = "right"; }
      Serial.println ("Right limit reached-moving to left");
      digitalWrite(relayleft, HIGH);
      digitalWrite(relayright, LOW);
      delay (1000);
      digitalWrite(relayleft, LOW);
    }
    else if ( abs(rig - lef) < tol )
    {
      sidesDone = true;
      side = "none";
      Serial.println ("sides done    ");
      digitalWrite(relayleft, LOW);
      digitalWrite(relayright, LOW);
      
    }
       
    if ( ( topDone == true ) && ( sidesDone == true )) {notDone = false;}
  }
}  //while end 
 //  break; delay (2000);
 
  /* Serial.print("    Tracker counter = ");     decommment to view values, otherwise it screws up my serial port from too many iterations
   Serial.println( counter );
   Serial.println("\t");
   Serial.print("Relay up =   ");
   Serial.print(rup, DEC);
   Serial.print("\t");
   Serial.print("Relay down =   ");
   Serial.print(rdown, DEC);
   Serial.print("\t");
   Serial.print("Relay left =   ");
   Serial.print(rleft, DEC);
   Serial.print("\t");
   Serial.print("Relay right =   ");
   Serial.println(rright, DEC);    */
 

 /*   
    Serial.print("Speed =   ");
    Serial.print(GPS.speed);
    Serial.print("\t");
    Serial.print("dvert =   ");
    Serial.print(dvert);
    Serial.print("\t");
    Serial.print("dhoriz =   ");
    Serial.print(dhoriz);
    Serial.println("\t");
    Serial.println("\t");
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
  Serial.print("    Tracker counter = ");  
   Serial.println( counter );
   Serial.println("\t");
   Serial.print("Relay up =   ");
   Serial.print(rup, DEC);
   Serial.print("\t");
   Serial.print("Relay down =   ");
   Serial.print(rdown, DEC);
   Serial.print("\t");
   Serial.print("Relay left =   ");
   Serial.print(rleft, DEC);
   Serial.print("\t");
   Serial.print("Relay right =   ");
   Serial.println(rright, DEC); */
   

     void readvals() {
  int top = analogRead(ldtop); // Top of panel
  int bot = analogRead(ldbot); // Bottom of Panel
  int lef = analogRead(ldlef); // Left side of Panel
  int rig = analogRead(ldrig); //Right side of Panel
   }            

 


