// stand horizontal servo

// stand vertical servo
// relays 2 & 3 are Top and Bottom
// relays 4 & 5 are Left and Right
// LDR pin connections
// name = analogpin;
int ldtop = 0; // top panel
int ldbot = 1; // bottom panel
int ldlef = 2; // left panel
int ldrig = 3; // right panel
 int top = analogRead(ldtop); // Top of panel
  int bot = analogRead(ldbot); // Bottom of Panel
  int lef = analogRead(ldlef); // Left side of Panel
  int rig = analogRead(ldrig); //Right side of Panel  
  
void setup(){

  Serial.begin(9600);
  
  for(int pinNumber =2; pinNumber<6; pinNumber++){
    pinMode(pinNumber,OUTPUT);
    digitalWrite(pinNumber, LOW);
}   
} 
  
void loop()    //every 6 min, look at solar sensors; store 4 sensor values
{
 

  
  
  //Serial.print("TOP  ");
  //Serial.println(top);
  //delay(10);
  //Serial.print("BOTTOM  ");
  //Serial.println(bot);
  //delay(10);
  //Serial.print("LEFT  ");
  //Serial.println(lef);
  //delay(10);
  //Serial.print("RIGHT  ");
  //Serial.println(rig);
  //delay(10);
  
}
  
  


void motorforwardTB() {
  
  digitalWrite(2, LOW);
  digitalWrite(3, HIGH);
  
}
  


void motorbackTB() {
  
  digitalWrite(2, HIGH);
  digitalWrite(3, LOW);
 
}


  
void compareTB(){     //compare Top, Bottom values. which is higher 
  
}  

void compareLR(){     //compare L, R values , which is higher 
  
} 
