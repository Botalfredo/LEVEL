/*====================================================================== 
  _      ________      ________ _        
 | |    |  ____\ \    / /  ____| |      
 | |    | |__   \ \  / /| |__  | |       
 | |    |  __|   \ \/ / |  __| | |      
 | |____| |____   \  /  | |____| |____  
 |______|______|   \/   |______|______| 
 
====================================================================== 
First code for Level flight computer, including MPU6050, BMP280, 
6 channels, ppm signal, up to 9 servos, micro sd, rgb led, and more  
1.5 news
-update for Level pcb rev2
-add Ibus support
-add GPS
======================================================================  
Version 1.6 made by @fredlebricolo
create 13/09/2022
last update 13/09/2022
====================================================================*/  
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BMP280.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "FS.h"
#include "SD.h"
#include <ESP32Servo.h>
#include <FastLED.h> 
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <IBusBM.h>

#define MPUinterrupt_Pin 34 
#define RCchannumber    6 
#define PPMreadtime      20
#define PPMfilter        10 // Glitch Filter
#define PPMpin           35
#define SDpin            5 
#define BUZZERpin        2
#define GPSrxpin         17 
#define GPStxpin         16

//---------------------------------VARIABLES---------------------------------
//               \/ SD \/ 
String dataMessage, name_file = "null.csv";
int count_file = 0;
bool file_exists = false;
int SDError = 0;
String ScanData = "Scan I2C : ", BMPdata = "Start pressure :";

//               \/ PPM \/ 
int channel[RCchannumber]; // readed Channel values
int lastReadChannel[RCchannumber]; // Last  values readed
int RC_conter=0; //couter
unsigned int IbusError = 0;

//               \/ State \/ 
bool MPUState = 0, BMPState = 0, RTCState = 0, OLEDState = 0, RCState = 0, GPSState = 0, SDState = 0;
int MPUError = 0, RCError = 0, RCfrec, MPUfrec, SDfrec, BPMfrec, RTCfrec = 0, GPSfrec,task1frec,task2frec;
int RCfrec1, MPUfrec1, SDfrec1, BPMfrec1, RTCfrec1 = 0, GPSfrec1, task1frec1, task2frec1;
bool MPUStateOLD, RCStateOLD, SDStateOLD, BMPStateOLD, GPSStateOLD, RTCStateOLD;
char red, green, blue;
bool error = false;
int errorLVL = 0;
long task1timer,task2timer;
int SX,SY,SZ;

//                \/ BMP \/ 
float StartPressure,BMPtemp, BMPpres, BMPalt;

//                \/ GPS \/ 
double GPSspd, GPSlat, GPSlng, GPSalt, GPScrs;
int GPSsta, GPShour,GPSmin,GPSsec,GPSday,GPSmonth,GPSyear; 

//                \/ MPU \/ 
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
float AX, AY, AZ, GX, GY, GZ;
Quaternion qua;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//---------------------------------Proto---------------------------------
void CreateTasks();
void Initport();
void InitSerial();
void InitIBUS();
void InitMPU();
void InitOLED();
void InitBMP();
void InitSD();
void InitLED();
void InitGPS();
void InitPartyTime();
void ReInitSD();
void ScanI2C();
static void GPSreadtime(unsigned long ms);
void ReadPPM();
void ReadIBUS();
void ReadRadio();
void ReadMPU();
void ReadBMP();
//void ReadGPS();
void RightServo();
void printDate();
void printTime();
void RightMessage();
void RightSerial();
void CalculFrec();
void SetLED(int r,int g,int b);
void AddLED();
void writeFile(fs::FS &fs, const char * path, const char * message);
void appendFile(fs::FS &fs, const char * path, const char * message);
unsigned long pulseIn1(uint8_t pin, uint8_t state, unsigned long timeout);
void Task1code(void * pvParameters);
void Task2code(void * pvParameters);
void displayInfo();
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

void dmpDataReady() {
    mpuInterrupt = true;
}

//Déclarations des thread
TaskHandle_t Task1;
TaskHandle_t Task2;

IBusBM IBusServo;

File file;

MPU6050 mpu;

Adafruit_SSD1306 display(128, 64, &Wire, -1);

Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

Servo servo_roll_d, servo_roll_g,servo_pitch, servo_yaw, motor;

CRGB leds[1];


static const int RXPin = 17, TXPin = 16;
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

void setup() {
  
  InitSerial();

  //ScanI2C();
  
  InitLED();
  
  Initport();
  
  InitIBUS();
  
  InitMPU();

  InitBMP();
  
  InitGPS();
   
  InitOLED();
  
  InitSD();
  
  CreateTasks();
  
  InitPartyTime();
  
}

void loop() {
vTaskDelay(1000);
}
