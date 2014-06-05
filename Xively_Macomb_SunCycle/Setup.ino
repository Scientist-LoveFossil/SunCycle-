void Setup(){  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin(); 
 // server.begin();                           // start the web server on port 80

  myPressure.begin();
      myPressure.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa
      myPressure.setOversampleRate(7); // Set Oversample to the recommended 128
      myPressure.enableEventFlags();
  GPSsetup();
    
  dht.begin();
  //pin setup
  for(int pinNumber =24; pinNumber<28; pinNumber++){  
      pinMode(pinNumber,OUTPUT);
      digitalWrite(pinNumber, LOW);
  }  
  pinMode(AmblightPin, INPUT);
  pinMode(DHTPin, INPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(trikeLights, OUTPUT);
  
  for (int pinNumber=28; pinNumber < 32; pinNumber++) {
     pinMode(pinNumber, INPUT); 
  }
  
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
