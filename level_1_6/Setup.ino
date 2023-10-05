void CreateTasks(){
  
   xTaskCreatePinnedToCore(
                Task1code,   /* Task function. */
                "Task1",     /* name of task. */
                10000,       /* Stack size of task */
                NULL,        /* parameter of the task */
                1,           /* priority of the task */
                &Task1,      /* Task handle to keep track of created task */
                1);          /* pin task to core 0 */

                
    xTaskCreatePinnedToCore(
                Task2code,   /* Task function. */
                "Task2",     /* name of task. */
                10000,       /* Stack size of task */
                NULL,        /* parameter of the task */
                1,           /* priority of the task */
                &Task2,      /* Task handle to keep track of created task */
                0);          /* pin task to core 0 */    
  Serial.println("Tasks created!\n");
}

void Initport(){
  //pinMode(PPMpin, INPUT); //Patita 4 como entrada / Pin 4 as input

  servo_roll_d.attach(32);
  servo_roll_g.attach(33);
  servo_pitch.attach(25);
  servo_yaw.attach(26);
  motor.attach(27);
  motor.write(45); // pour init le moteur
}

void InitSerial(){
  Serial.begin(115200);
  Serial.println("------------------------------------");
  Serial.println("             FailPlane v13          ");
  Serial.println("Level rev1");
  Serial.println("------------------------------------\n");

}
void InitIBUS(){
  IBusServo.begin(Serial1, 1, PPMpin, NULL);
}

void InitMPU(){
  Serial.println("InitMPU()");
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  Serial.println("Wire");
  // initialize device
  mpu.initialize();
  Serial.println("test");
  pinMode(MPUinterrupt_Pin, INPUT);

  // verify connection
  
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful!") : F("MPU6050 connection failed!"));
  Serial.println("MPU CON");
  // load and configure the DMP
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
      // Calibration Time: generate offsets and calibrate our MPU6050
      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);
      mpu.PrintActiveOffsets();
      // turn on the DMP, now that it's ready
      mpu.setDMPEnabled(true);

      // enable Arduino interrupt detection
      Serial.print(digitalPinToInterrupt(MPUinterrupt_Pin));
      Serial.println(F(")..."));
      attachInterrupt(digitalPinToInterrupt(MPUinterrupt_Pin), dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      Serial.println(F("MPU all good!\n"));
      dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
  }
}

void InitOLED(){
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed\n"));
  }
  else{
    Serial.println(F("OLED all good!\n"));
    
    display.clearDisplay();
    display.setTextSize(2);      // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.setCursor(0, 0);     // Start at top-left corner
    display.println("FailPlane");
    display.setCursor(20, 45);
    display.println("V13");
    display.display();
  }
}

void InitBMP(){
  if (!bmp.begin(0x76)) {
    Serial.println("ERREUR BMP\n");
    BMPState = 0;
  }
  else{
    BMPState = 1;
    Serial.println("BMP all good!\n");
  }

    /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  sensors_event_t temp_event, pressure_event;
  bmp_temp->getEvent(&temp_event);
  bmp_pressure->getEvent(&pressure_event);
  
  StartPressure = 0;
  for(int i = 0; i<30; i++){
    StartPressure += pressure_event.pressure;
  }
  StartPressure = StartPressure/30;
  BMPdata = BMPdata + String(StartPressure);
}

void InitSD(){
  SD.begin(SDpin);  
  if(!SD.begin(SDpin)) {
      SDState = false;
      Serial.println("Error SDcard\n"); 
      file_exists = false;
      return;
  }
  uint8_t cardType = SD.cardType();
  if(cardType == CARD_NONE) {
      SDState = false;
      Serial.println("Error SDcard\n"); 
      file_exists = false;
      return;
  }
  if (!SD.begin(SDpin)) {
      SDState = false;
      Serial.println("Error SDcard\n"); 
      file_exists = false;
      return;   
  }
  else{
      SDState = true;
  }
  
  //boucle de création de fichier csv
  Serial.println("Creating SD file...");
  do{
      name_file = String("/data_log_") + int(count_file) + String(".csv");
      Serial.println("initalisation du fichier : " + name_file);
      File file = SD.open(name_file);
      if(!file) {
          Serial.println("File doens't exist");
          Serial.println("Creating file...");
        
          //display.println("-fichier CSV OK");display.display();
          writeFile(SD, name_file.c_str(), BMPdata.c_str());
          appendFile(SD, name_file.c_str(), ScanData.c_str());
          appendFile(SD, name_file.c_str(), "Temps(ms);GPS Time;GPS Date;GPS sat;GPS speed;GPSalt;latitude;longitude;direction;GX;GY;GZ;AX;AY;AZ;BMPtemp;BMPpression;BMPalt;ch1;ch2;ch3;ch4;ch5;ch6;mpu;bmp;rtc;oled;ppm;GPSState;SDState;mpuerror;ppm error;RCfrec1;MPUfrec1;SDfrec1;BPMfrec1;RTCfrec1;GPSfrec1;task1frec1;task2frec1;ErrorLVL\r\n");
          file_exists = true;
          }
      else {
          Serial.println("fichier existant : " + name_file );
          count_file ++;
          file.close();
          } 
  }while (file_exists == false);
  Serial.println("SD all good!\n");  
  file.close();  
}

void InitLED(){
  FastLED.addLeds<WS2812, 4, RGB>(leds, 1);
  Serial.println("LED all good!\n"); 
  SetLED(255,255,255);
  tone(BUZZERpin, 1500,400);
  Serial.println("Buzz!\n");
}

void InitGPS(){
  ss.begin(GPSBaud);
  Serial.println("GPS all good!\n"); 
}

void InitPartyTime(){
  SetLED(255,68,221);
  tone(BUZZERpin, 1000,200);
  delay(200); 
  SetLED(0,0,0);
  delay(200);  
  SetLED(255,255,255);
  tone(BUZZERpin, 1000,200);     
  delay(200); 
  SetLED(0,0,0);
  delay(200); 
  SetLED(255,255,255); 
  tone(BUZZERpin, 1500,400);
  delay(200); 
  SetLED(0,0,0);

  Serial.println("------------------------------------");
  Serial.println("Setup donne!");
  Serial.println("------------------------------------\n");
    
}

void ScanI2C(){
  int nDevices;
  byte error, address; //variable for error and I2C address
  int fonctionalite = 0;
  
  Wire.begin(); // Wire communication begin
  Serial.println("Scanning...");
  nDevices = 0;
  for (address = 1; address < 127; address++ ){ 
    Wire.beginTransmission(address);
      error = Wire.endTransmission();
      if (error == 0){
        Serial.print("I2C device found at address 0x");
        ScanData = ScanData + "I2C device found at address 0x";
        if (address < 16){
          Serial.print("0");
          ScanData = ScanData + "0";
        }
        Serial.print(address,HEX);
        Serial.println("  !");
        ScanData = ScanData + String(address,HEX)+ ", ";
        
        if(address == 104 || address == 105){
          fonctionalite=+1;
          Serial.println("MPU OK!");
        }
          
        if(address == 119 || address == 118){
          fonctionalite=+1;
          Serial.println("BMP OK!");
        }
        if(address == 81)
          Serial.println("RTC OK!");
        nDevices++;
      }
      else if (error == 4){
        Serial.print("Unknown error at address 0x");
        ScanData = ScanData + "Unknown error at address 0x";
        if (address < 16){
          Serial.print("0");
          ScanData = ScanData + "0";
        }
        Serial.println(address, HEX);
        ScanData = ScanData + String(address,HEX)+ ", ";
      }
  }
  if (nDevices == 0){
    Serial.println("No I2C devices found\n");
    ScanData =ScanData+ "No I2C devices found\n";
  }
  else{
    Serial.println("done\n");
    ScanData =ScanData+ "done\n";
  }
}
