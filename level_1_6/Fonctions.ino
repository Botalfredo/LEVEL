void ReadMPU(){
  if (!dmpReady) return;
   if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    MPUfrec++;
    mpu.dmpGetQuaternion(&qua, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &qua);
    mpu.dmpGetYawPitchRoll(ypr, &qua, &gravity);
    
    GY = ypr[1] * 180/M_PI;
    GX = ypr[2] * 180/M_PI;
    GZ = ypr[0] * 180/M_PI;

/*
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180/M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180/M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180/M_PI);

    */
    mpu.dmpGetQuaternion(&qua, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &qua);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &qua);
    
    AX =(float(aaWorld.x)/ 8192); 
    AY =(float(aaWorld.y)/ 8192);  
    AZ =(float(aaWorld.z)/ 8192);

    MPUState = 1;
    mpuInterrupt = 0;
    MPUError = 0;
   }
   else{
    MPUError++;
    if (MPUError >= 10){
      MPUState = 0;
      MPUError = 0;
    }
  }
}


void ReadPPM(){
  long timer_ppm = millis();
  
  while((millis()-timer_ppm) < PPMreadtime ) {
    if(digitalRead(PPMpin)==HIGH){
    if(pulseIn1(PPMpin, HIGH, 100000) > 3000){ //If pulse > 3000 useconds, continues
      for(int i = 0; i <= RCchannumber-1; i++){ //Read the pulses of the channels  
        channel[i]=pulseIn1(PPMpin, HIGH,100000);
      }
      for(int i = 0; i <= RCchannumber-1; i++){ //Average the pulses
        if((channel[i] > 2000) || (channel[i] <100)){ //If channel > max range, chage the value to the last pulse
          channel[i]= lastReadChannel[i];
      }
      else{
        channel[i]=(lastReadChannel[i]+channel[i])/2; //Average the last pulse eith the current pulse
        RC_conter++; // increment counter
        }
      }
      break; // if ppm is read exit while
    }
  }
}

  if(RC_conter > PPMfilter){// If counter is > than PPMfilter, then prints values
    for(int i = 0; i <= RCchannumber-1; i++){ //Cycle to print values     
      //Serial.print(map(channel[i],600,1600,0,100));Serial.print(", "); 
      lastReadChannel[i]=channel[i];
      RCfrec++;
    }
  //Serial.println(" "); 
  RC_conter=0; //Restart couter.
  RCError = 0;
  RCState = 1;
  }
  else{
    RCError++;
  }
  if(RCError == 15)RCState=0;
}

void ReadBMP(){
  BMPtemp = bmp.readTemperature();
  BMPpres = bmp.readPressure()/100;
  BMPalt  = bmp.readAltitude(StartPressure);
  BPMfrec++;
}
/*
static void GPSreadtime(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      tinyGPS.encode(ss.read());
  } while (millis() - start < ms);
}

void ReadGPS(){

 while (ss.available()>0)
      tinyGPS.encode(ss.read());
  
  Serial.print("Lat: "); Serial.println(tinyGPS.location.lat(), 6);
  Serial.print("Long: "); Serial.println(tinyGPS.location.lng(), 6);
  Serial.print("Alt: "); Serial.println(tinyGPS.altitude.meters());
  Serial.print("Course: "); Serial.println(tinyGPS.course.deg());
  Serial.print("Speed: "); Serial.println(tinyGPS.speed.kmph());
  Serial.print("Date: "); printDate();
  Serial.print("Time: "); printTime();
  Serial.print("Sats: "); Serial.println(tinyGPS.satellites.value());
  Serial.println();

  
  GPSspd = tinyGPS.speed.kmph();
  GPSlat = tinyGPS.location.lat();
  GPSlng = tinyGPS.location.lng();
  GPSalt = tinyGPS.altitude.meters();
  GPScrs = tinyGPS.course.deg();
  GPSsta = tinyGPS.satellites.value();

  GPShour  = tinyGPS.time.hour();
  GPSmin   = tinyGPS.time.minute();
  GPSsec   = tinyGPS.time.second();
  GPSday   = tinyGPS.date.day();
  GPSmonth = tinyGPS.date.month();
  GPSyear  = tinyGPS.date.year();

  
  if(GPSsta != 0){
    GPSState = 1;
    GPSfrec++;
  }
  else GPSState = 0;
}

// printDate() formats the date into dd/mm/yy.
void printDate()
{
  Serial.print(tinyGPS.date.day());
  Serial.print("/");
  Serial.print(tinyGPS.date.month());
  Serial.print("/");
  Serial.println(tinyGPS.date.year());
}

// printTime() formats the time into "hh:mm:ss", and prints leading 0's
// where they're called for.
void printTime()
{
  Serial.print(tinyGPS.time.hour());
  Serial.print(":");
  if (tinyGPS.time.minute() < 10) Serial.print('0');
  Serial.print(tinyGPS.time.minute());
  Serial.print(":");
  if (tinyGPS.time.second() < 10) Serial.print('0');
  Serial.println(tinyGPS.time.second());
}
*/
void writeFile(fs::FS &fs, const char * path, const char * message) {
       // Serial.printf("Writing file: %s\n", path);
    File file = fs.open(path, FILE_WRITE);
    if(!file) {
        SDState = 0;
        SDError++;
        return;
    }
    if(file.print(message)) {      
       SDState = 1; 
       SDError = 0;      
    } else {
        SDState = 0; 
        SDError++;     
    }
    file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message) {
    //Serial.printf("Ecriture sur : %s\n", path);
    File file = fs.open(path, FILE_APPEND);
    if(!file) {
        SDState = false;
        SDError++;  
        return;
    }
    if(file.print(message)) {
        SDState = true;
        SDError=0; 
        SDfrec++; 
    } else {
        SDState = false;
        SDError++;    
    }
    file.close();
}

void RightMessage(){
  //long start = millis();
  dataMessage =
      String(millis())+";"+String(GPShour)+":"+String(GPSmin)+":"+String(GPSsec)+";"
    + String(GPSday)+"/"+String(GPSmonth)+"/"+String(GPSyear)+";"
    + String(GPSsta)+";"+String(GPSspd)+";"+String(GPSalt)+";"+String(GPSlat,6)+";"+String(GPSlng,6)+";"+String(GPScrs)+";"
    + String(GX)+";"+String(GY)+";"+ String(GZ)+";"+ String(AX) + ";" + String(AY)+ ";"+ String(AZ)+ ";"
    + String(BMPtemp)+ ";" + String(BMPpres)+ ";" + String(BMPalt)+";"
    + String(channel[0]) + ";" + String(channel[1]) + ";" + String(channel[2]) + ";" + String(channel[3]) + ";" + String(channel[4]) + ";" + String(channel[5])+";"   
    + String(MPUState)+";"+String(BMPState)+";"+String(RTCState)+";"+String(OLEDState)+";"+String(RCState)+";"+String(GPSState)+";"+String(SDState)+";"+String(MPUError)+";"+String(RCError)+";"
    + String(RCfrec1)+";"+String(MPUfrec1)+";"+String(SDfrec1)+";"+String(BPMfrec1)+";"+String(RTCfrec1)+";"+String(GPSfrec1)+";"+String(task1frec1)+";"+String(task2frec1)+";"+String(errorLVL)+";"
    "\n";   
    //Serial.print("data ");   
   //Serial.println(millis()-start);   

  appendFile(SD, name_file.c_str(), dataMessage.c_str());
  if (SDError >= 100 && file_exists == 0){
    InitSD(); 
    SDError = 0;
  }
  //Serial.print("right ");  
  //Serial.println(millis()-start);
  
}

unsigned long pulseIn1(uint8_t pin, uint8_t state, unsigned long timeout)
{
    // cache the port and bit of the pin in order to speed up the
    // pulse width measuring loop and achieve finer resolution.  calling
    // digitalRead() instead yields much coarser resolution.
    uint8_t bit = digitalPinToBitMask(pin);
    uint8_t port = digitalPinToPort(pin);
    uint8_t stateMask = (state ? bit : 0);
    unsigned long width = 0; // keep initialization out of time critical area

    // convert the timeout from microseconds to a number of times through
    // the initial loop; it takes 16 clock cycles per iteration.
    unsigned long numloops = 0;
    unsigned long maxloops = microsecondsToClockCycles(timeout) / 16;

    // wait for any previous pulse to end
    while ((*portInputRegister(port) & bit) == stateMask)
        if (numloops++ == maxloops)
            return 0;

    // wait for the pulse to start
    while ((*portInputRegister(port) & bit) != stateMask)
        if (numloops++ == maxloops)
            return 0;

    // wait for the pulse to stop
    while ((*portInputRegister(port) & bit) == stateMask) {
        if (numloops++ == maxloops)
            return 0;
        width++;
    }
    return clockCyclesToMicroseconds(width * 21 + 16); 
}

void SetLED(int r,int g,int b){
  red = r;
  green = g;
  blue = b;
  leds[0].setRGB(green,red,blue);
  FastLED.show();
}

void AddLED(){
  leds[0].setRGB(green,red,blue);
  FastLED.show();
}

void RightSerial(){
  Serial.print("MPU : "); Serial.print(MPUState); Serial.print("   ");
  Serial.print("RC  : "); Serial.print(RCState); Serial.print("   ");
  Serial.print("SD  : "); Serial.print(SDState); Serial.print("   ");
  Serial.print("BMP : "); Serial.print(BMPState); Serial.print("   ");
  Serial.print("GPS : "); Serial.print(GPSState); Serial.print("   ");
  Serial.print("RTC : "); Serial.print(RTCState); Serial.print("   ");
  Serial.print("MPUe: "); Serial.print(MPUError); Serial.print("   ");
  Serial.print("RCe: "); Serial.print(RCError); Serial.print("   ");
  Serial.print("SDe: "); Serial.print(SDError); Serial.println("   ");
}

void CalculFrec(){
  static long Startfrec = millis();
  if( millis()-Startfrec >= 1000){

        
    RightSerial();
    Startfrec = millis();
/*
    Serial.print("RC  "); 
    Serial.print(RCfrec); 
    Serial.print("hz   ");
    
    Serial.print("MPU "); 
    Serial.print(MPUfrec); 
    Serial.print("hz   ");
    
    Serial.print("SD "); 
    Serial.print(SDfrec); 
    Serial.print("hz   ");
    
    Serial.print("BMP "); 
    Serial.print(BPMfrec); 
    Serial.print("hz   "); 
    
    Serial.print("GPS "); 
    Serial.print(GPSfrec); 
    Serial.print("hz   ");

    Serial.print("t1 "); 
    Serial.print(task1frec); 
    Serial.print("hz   "); 
    
    Serial.print("t2 "); 
    Serial.print(task2frec); 
    Serial.println("hz   ");
*/
    RCfrec1 = RCfrec;
    MPUfrec1 = MPUfrec; 
    SDfrec1 = SDfrec;  
    BPMfrec1 = BPMfrec; 
    GPSfrec1 = GPSfrec;
    task1frec1 = task1frec;
    task2frec1 = task2frec;
    
    RCfrec = 0;
    MPUfrec = 0; 
    SDfrec = 0;  
    BPMfrec = 0; 
    GPSfrec = 0;
    task1frec = 0;
    task2frec = 0;
  }
}

void AfficherOLED(){
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.setCursor(0, 0);     // Start at top-left corner
    display.setTextSize(1);
    display.print("MPU "); display.print(MPUState); display.print("  ");
    display.print("RC  "); display.print(RCState); display.print("  ");
    display.print("SD  "); display.print(SDState); display.println("  ");
    display.setCursor(0, 12);     // Start at top-left corner
    display.print("BMP "); display.print(BMPState); display.print("  ");
    display.print("GPS "); display.print(GPSState); display.print("  ");
    display.print("ERR "); display.print(int(errorLVL)); display.println(" ");
    display.setCursor(0, 24);     // Start at top-left corner
    display.print("T1  "); display.print(task1frec1); display.print("  ");
    display.print("T2  "); display.print(task2frec1); display.print("  ");
    display.setCursor(0, 36);     // Start at top-left corner
    display.print("file  "); display.print(count_file); display.print(" ");
    display.print("RC  "); display.print(RCfrec); display.println(" ");
    display.display();
}

void DefaultDetection(){

  if (MPUStateOLD != MPUState)error=true;
  if (RCStateOLD != RCState)error=true;
  if (SDStateOLD  != SDState)error=true;
  if (BMPStateOLD != BMPState)error=true;
  if (GPSStateOLD != GPSState)error=true;
  if (RTCStateOLD != RTCState)error=true;
  

  
  static bool Ledflash = 0;
  Ledflash = !Ledflash;
  
  if(MPUState==1 && RCState==1 && SDState==1 && BMPState==1 && GPSState==1){
    errorLVL = 0;
    SetLED(0,255,255);
  }
  if(BMPState==0){
    errorLVL = 1;
    SetLED(0,255,0);
  }
  if(GPSState==0){
    errorLVL = 1;
    SetLED(0,255,0);
  }
  if(SDState==0){
    errorLVL = 2;
    SetLED(255,213,0);
  }
  if(RCState==0){
    errorLVL = 3;
    SetLED(243,114,32);
  }
  if(MPUState==0){
    errorLVL = 4;
    if (Ledflash == true)SetLED(255,0,0);
    else SetLED(0,0,0);
  }
  if(MPUState==0 && RCState==0 && SDState==0 && BMPState==0 && GPSState==0){
    errorLVL = 5;
    if (Ledflash == true)SetLED(100,100,0);
    else SetLED(0,0,0);
  }

  MPUStateOLD = MPUState;
  RCStateOLD = RCState;
  SDStateOLD = SDState;
  BMPStateOLD = BMPState;
  GPSStateOLD = GPSState;
  RTCStateOLD = RTCState;
}

void RightServo(){
  servo_roll_d.write(map(channel[0], 600, 1595, 170, 10));
  servo_roll_g.write(map(channel[0], 600, 1595, 170, 10));
  servo_pitch .write(map(channel[1], 600, 1595, 10, 170));
  servo_yaw   .write(map(channel[3], 600, 1595, 10, 170));
  motor       .write(map(channel[2], 600, 1595, 40, 180));
  
}

void ReadIBUS(){
  RCState = IBusServo.Ibus_state;
  for (int i=0; i<RCchannumber ; i++) {
    channel[i] = IBusServo.readChannel(i);
  }
  RCfrec++;
}

void ReadRadio(){
  
}

void displayInfo()
{
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}
  
