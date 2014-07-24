double SolarTracker(double counter){
 Serial.print("  counter = "); 
 Serial.println( counter );
 Serial.println("in solar tracker function");
 // for (int counter = 0; counter < 5000; counter++){
       delay(1);
  
  String side = "none";
  


  int HB, HT, EL, ER;  // Home base, home top, extreme left, extreme right
  int notDone = 1;
  int homed = digitalRead(limitHome);
  Serial.println ("Home Limit");
  Serial.println (homed);
  // If in motion ensure the system is down or put it down
  if (GPS.speed > 0.30 && ( ( HB = digitalRead(LimitUp) == LOW) || (HT = digitalRead(LimitDown) == LOW ) ) ) {
    while (notDone == true) {
     if (HB == LOW) {
      if (side == "left") {
       digitalWrite(relayleft, LOW);
       digitalWrite(relayright, HIGH);
       Serial.println (" Limit Right ");
       Serial.println (ER);
      } 
      else if (side == "right") {
        digitalWrite(relayleft, HIGH);
        digitalWrite(relayright, LOW);
        Serial.println (" Limit Left ");
        Serial.println (EL);
      }
    }
     if (HT == LOW) {
      digitalWrite(relayup, LOW);
      digitalWrite(relaydown, HIGH); 
       Serial.println (" Limit Down ");
      Serial.println (HT);
     }
     digitalRead(LimitUp);
     digitalRead(LimitDown);
     if ( ( HT == HIGH ) && (HB == HIGH) ) { notDone = 0;}
     
    } // end While
   
   datastreams[1].setFloat(homed);
    return 0; // Do not continue on if speed is higher than 0
  } //end If GPS.speed
  
  
   // Recycle the variable
  int rup = digitalRead(relayup);  // for debugging if relay outputs are high or low
  int rdown = digitalRead(relaydown);
  int rleft = digitalRead(relayleft);
  int rright = digitalRead(relayright);
  int top = analogRead(ldtop); // Top of panel
  int bot = analogRead(ldbot); // Bottom of Panel
  int lef = analogRead(ldlef); // Left side of Panel
  int rig = analogRead(ldrig); //Right side of Panel
  int tol = 75;                // Added a tolerance to photosensor values so we won't continually chase the tiny variations.
  
//  int dvert = top - bot; // check the diffirence of up and down
//  int dhoriz = lef - rig;// check the diffirence of left and right
   notDone = 1;
  
  //Serial.println("\t");
  Serial.print("TOP ");      Serial.print(top);  Serial.print("\t");
  Serial.print("BOTTOM  ");  Serial.print(bot);  Serial.print("\t");
  Serial.print("LEFT  ");    Serial.print(lef);  Serial.print("\t");
  Serial.print("RIGHT  ");   Serial.print(rig);  Serial.println("\t");
  Serial.print (" LimitRight ");  Serial.print (ER);  Serial.print("\t");
  Serial.print (" LimitLeft ");   Serial.print (EL);  Serial.println("\t");
  Serial.print (" LimitUp ");     Serial.print (HB);  Serial.print("\t");
  Serial.print (" LimitDown ");   Serial.print (HT);  Serial.println("\t");
  Serial.print (" Limit Home ");  Serial.print (homed);
 
  int sidesDone = 0;
  int topDone = 0;
  //break;
  while (notDone) {
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
      Serial.println(rup);
      Serial.println(rdown);
    }
    else if (top < bot && !topDone) 
    {
     digitalWrite(relaydown, LOW);
     digitalWrite(relayup, HIGH);
    Serial.println(rup);
    Serial.println(rdown);
    }
    else if ( abs(top - bot) < tol )  // Equalized!
    {
      topDone = 1;
      digitalWrite(relayup, LOW);
      digitalWrite(relaydown, LOW);
    }  // End up and downs
    
    HB = digitalRead(LimitUp);
    //if (-1*tol > dhoriz || dhoriz > tol){
    if (lef > rig && !sidesDone)
    {
      if (EL = digitalRead(limitLeft) == HIGH) { sidesDone = 1; continue; }  // Cannot move left any more, side movement done
      if (HB == HIGH) { side = "left"; }
      digitalWrite(relayright, LOW);
      digitalWrite(relayleft, HIGH);
    }
    else if (lef < rig && !sidesDone)
    {
      if (ER = digitalRead(limitRight) == HIGH) {sidesDone = 1; continue; } // Cannot move right any more, side movement done
      if (HB == HIGH) { side = "right"; }
      digitalWrite(relayleft, LOW);
      digitalWrite(relayright, HIGH);
    }
    else if ( abs(rig - lef) < tol )
    {
      sidesDone = 1;
      side = "none";
      digitalWrite(relayleft, LOW);
      digitalWrite(relayright, LOW);
      
    }
       
    if ( ( topDone ) && ( sidesDone )) {notDone = 0;}
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
   

    

 


