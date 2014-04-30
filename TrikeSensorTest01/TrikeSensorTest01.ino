int hydroValue =0;
int amb =0; //ambient light
const int thresh = 15;
void setup()
{
  Serial.begin(9600);      // sets the serial port to 9600
}

void loop()
{
  hydroValue = analogRead(4); 
  amb = analogRead(5);  // read analog input pin 0
    if (hydroValue > thresh){
    digitalWrite(13, HIGH); 
    Serial.println("Who cut the cheese?");
    delay(5000);
    
     }else{
       
  Serial.print( "Methane -     "); 
  Serial.print(hydroValue, DEC);
  Serial.print( "     Ambient Light -      "); 
  Serial.print(amb, DEC); // prints the value read
  Serial.println ("    ");
  delay(100);    
   // wait 100ms for next reading
        }
}
