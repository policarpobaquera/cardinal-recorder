/*///////////////////////////////////////////////////////////////////////////////
/////////////////////////////// CARDINAL RECORDER ///////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

Created 9th Dec 2020
by Policarpo Baquera
Carnegie Mellon University                                                     */

/*////////////////////////// MICRO SD CARD ADAPTER ////////////////////////////// 
The Micro SD Adapter is connected to the Arduino board as follows:
Nano: CS:   D10 | SCK:  D13 | MOSI: D11 | MISO: D12 | VCC:  5V | GND:  GND 
Mega: CS:   D10 | SCK:  D13 | MOSI: D11 | MISO: D12 | VCC:  5V | GND:  GND     */

#include <SD.h>
#include <SPI.h>
//const byte pinSD = 10;  // Arduino Nano
const byte pinSD = 53;    // Arduino Mega 2560 Pro

/*////////////////////////////// AUDIO LIBRARY //////////////////////////////////
TMRpcm Library Usage and Information on
https://github.com/TMRh20/TMRpcm/wiki/Advanced-Features#wiki-recording-audio
In order to run this code, edit the file pcmConfig.h
(usually in C:\...\Documents\Arduino\libraries\TMRpcm)
    1. On Uno or non-mega boards, #define buffSize 128. May need to increase.
    2. Uncomment #define SD_FULLSPEED
    3. Uncomment #define ENABLE_RECORDING and #define BLOCK_COUNT 10000UL
Once edited, assign the microphone and speaker connections as follows:
Nano: MIC OUT: A0 | SPEAKER IN: D9  
Mega: MIC OUT: A0 | SPEAKER 1: D11 | SPEAKER 2: D12                            */

#include <TMRpcm.h>
TMRpcm audio;

//const byte pinSpk = 9;  // Arduino Nano
const byte pinSpk1 = 11;  // Arduino Mega 2560 Pro
const byte pinSpk2 = 12;  // We also defined Speaker 2 although not in use
const byte pinMic = A0; 

/*////////////////////////////// RGB LED SETUP //////////////////////////////////
LED Light fade and blink Library
https://robotsbigdata.com/docs-arduino-light.html                              */
#include <RBD_Timer.h>
#include <RBD_Light.h>

//Arduino Nano
//const byte pinR = 2;    // Red    LED pin
//const byte pinG = 3;    // Green  LED pin (PWM)
//const byte pinB = 4;    // Blue   LED pin

//Arduino Mega 2560 Pro
const byte pinR = 40;     // Red    LED pin
const byte pinG = 42;     // Green  LED pin (PWM)
const byte pinB = 44;     // Blue   LED pin

RBD::Light light(pinG);

/* /////////////////////////////// BUTTON SETUP /////////////////////////////////
Button Event managing Library
https://github.com/mickey9801/ButtonFever                                      */
#include <BfButton.h>
//const byte pinButton = 5;   // Arduino Nano
const byte pinButton   = 34;  // Arduino Mega 2560 Pro
const int  longPress   = 1000;
BfButton btn(BfButton::STANDALONE_DIGITAL, pinButton, false, HIGH);

////////////////////////////////////// MODES ////////////////////////////////////
enum modes{magnetoMode, recordMode, playMode, eraseMode};
enum modes mode;

////////////////////////////// CARDINAL ORIENTATION /////////////////////////////
String cardinalNames[] = {"north", "northeast", "east", "southeast", 
                          "south", "southwest", "west", "northwest"};

byte recordings[8] = {0, 0, 0, 0, 0, 0, 0, 0};
char* files[8] = {"0.wav", "1.wav", "2.wav", "3.wav", 
                  "4.wav", "5.wav", "6.wav", "7.wav"};
byte orientation = 0;  // Current cardinal slot
bool state = false;    // State of the current slot: False -> empty | True -> full

/* /////////////////////////////// 9DoF SENSOR //////////////////////////////////
BNO080 IMU Library
https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library                    */
#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"
BNO080 myIMU;

////////////////////////////////////// SETUP ////////////////////////////////////
void setup() {
  // RGB LED Setup
  pinMode(pinR, OUTPUT);
  pinMode(pinG, OUTPUT);
  pinMode(pinB, OUTPUT);
  digitalWrite(pinR, HIGH); delay(500); digitalWrite(pinR, LOW);
  digitalWrite(pinG, HIGH); delay(500); digitalWrite(pinG, LOW);
  digitalWrite(pinB, HIGH); delay(500); digitalWrite(pinB, LOW);
  
  // Initializes the serial for debugging purposes
  Serial.begin(115200);
  while (!Serial);
  LEDBlink(1);
  
  // Audio Setup
  audio.speakerPin = pinSpk1;
  pinMode(pinSpk2,OUTPUT);
  audio.CSPin = pinSD;
  
  // SD Card Setup
  initSDCard();
  LEDBlink(2);
  
  // Button Setup
  btn.onPress(pressHandler)
     .onDoublePress(pressHandler)
     .onPressFor(pressHandler, longPress);
  
  // Magnetometer Setup
  initMagnetometer();
  LEDBlink(3);
 
  // Mode Setup
  mode = magnetoMode;
}

void initSDCard() {
  // Initializes the SD Card
  SD.begin(pinSD);
  Serial.println("Initializing SD Card...");
  // Checks the card is functioning correctly
  if (!SD.begin(pinSD)) {
    digitalWrite(pinR, HIGH);
    Serial.println("Initialization failed. Check:");
    Serial.println("1. Is a card inserted?");
    Serial.println("2. Is your wiring correct?");
    Serial.println("3. Did you change the 'pinSD' to match your module?");
    while (true);
  }
  // Reads the existing audionotes and updates the recording log.
  //Serial.println("Reading files...");
  for(int i = 0; i < 8; i++) {
    if(SD.exists(files[i])) {
      recordings[i] = 1;
      Serial.print("Found ");
      Serial.print(recordings[i]);
      Serial.print(" audionote  to the ");
      Serial.println(cardinalNames[i]);  
    }
    else {
      Serial.print("Found ");
      Serial.print(recordings[i]);
      Serial.print(" audionotes to the ");
      Serial.println(cardinalNames[i]); 
    }
  }
    Serial.println("Done.");
    Serial.println("___________________________________");
}

void initMagnetometer() {
  Wire.begin();
  while(myIMU.begin() == false) {
    Serial.println(F("BNO080 not detected at default I2C address."));
  }
  Wire.setClock(400000);          //Increase I2C data rate to 400kHz
  myIMU.enableRotationVector(50); //Send data update every 50ms
}


void lightW(bool s) {
  //White LED light
  if(s) {
    digitalWrite(pinR, HIGH); digitalWrite(pinG, HIGH); digitalWrite(pinB, HIGH);
  } else {
    digitalWrite(pinR, LOW);  digitalWrite(pinG, LOW);  digitalWrite(pinB, LOW);
  }
}

void LEDBlink(byte times) {
  // White blink used for initial checking of sensors and debugging
  for(byte i = 0; i < times; i++) {
    lightW(true);  delay(100); 
    lightW(false); delay(50);
  }
  delay(300);
}

void pressHandler (BfButton *btn, BfButton::press_pattern_t pattern) {
  Serial.print("Button ");
  switch (pattern) {
    case BfButton::SINGLE_PRESS:
      Serial.println("single pressed.");
      //A single press plays the audio note in magnetoMode
      if(state && mode == magnetoMode) {
        mode = playMode;
      }
      //A single press stops the recording in recordMode
      if(mode == recordMode) {
        mode = magnetoMode;
      }
      //A single press stops the playback in playMode
      if(mode == playMode) {
        audio.stopPlayback();
      }
      break;
    case BfButton::LONG_PRESS:
      Serial.println("long pressed.");
      //A long press writes or overwrites on an empty or full orientation slot
      if(mode == magnetoMode) {
        mode = recordMode;
        }
      break;
    case BfButton::DOUBLE_PRESS:
      Serial.println("double pressed.");
      //A double press erases a full orientation slot
      if(state && mode == magnetoMode) {
        mode = eraseMode; 
      }
      break;
  }
}

/////////////////////////////////////// LOOP ////////////////////////////////////
void loop() {
  if (mode == magnetoMode){
    searching();
  }
  if (mode == recordMode){
    recording(orientation);
  }
  if (mode == playMode){
    playing(orientation);
  } 
  if (mode == eraseMode){
    erasing(orientation);
  }
}

void searching() {
  // Reads the orientation of the sensor and returns its state
  state = recordings[readOrientation()];
  
  if(state && mode == magnetoMode) {
    lightW(false);
    light.fade(500,50,1000,50,1000);
    byte initial = readOrientation();
    while(initial == readOrientation() && mode == magnetoMode) {
      light.update();
      btn.read();
    }
    light.off(); delay(200);
  }
  if(!state && mode == magnetoMode) {
    light.off();
    lightW(true);
    byte initial = readOrientation();
    while(initial == readOrientation() && mode == magnetoMode) {
      btn.read();
    } 
    lightW(false); delay(200);
  }
}

void recording(byte i) {
  //LED Animation before recording
  delay(100); digitalWrite(pinR, HIGH); delay(200); digitalWrite(pinR, LOW); 
  delay(100); digitalWrite(pinR, HIGH); delay(200); digitalWrite(pinR, LOW); 
  delay(300);
  
  //Red LED light while recording
  digitalWrite(pinR, HIGH);
  audio.startRecording(files[i], 16000, pinMic);
  Serial.print("Recording...");
  
  //The audio is recorded until the user press the button or its length is 1min
  unsigned long startTime = millis();
  while(mode == recordMode && ((millis() - startTime) < 60000)) btn.read();
  
  digitalWrite(pinR, LOW);
  audio.stopRecording(files[i]);
  Serial.println("end");

  recordings[i] = 1;
  mode = magnetoMode;
  delay(300);
}

void playing(byte i) {
  //Plays a recording
  digitalWrite(pinG, HIGH);
  audio.play(files[i]);
  //If single button the playback stops and returns to magneto mode
  while(audio.isPlaying()){
    btn.read();
  }
  digitalWrite(pinG, LOW);
 
  mode = magnetoMode;
  delay(300);
}

void erasing(byte i) {
  //Erase recording in the specified cardinal slot
  if(SD.remove(files[i])) {
    recordings[i] = 0;
    Serial.print("Erase 1 audionote to the "); 
    Serial.println(cardinalNames[i]);
  }
  else {
    Serial.println("Error: Audionote does not exist");
  }
  mode = magnetoMode;
  delay(300);
}

byte readOrientation() {
  //Reads the current cardinal orientation
  byte reading = 0;
  float angle = 0;
  if (myIMU.dataAvailable() == true) {    
    //Convert yaw / heading to degrees
    angle = (myIMU.getYaw()) * 180.0 / PI; 
    if(angle < 0) angle = 360 + angle;
    angle = angle - 250;
    if(angle < 0) angle = 360 + angle;
    Serial.println(angle);
    if (0   < angle && angle <=  45) reading = 4;
    if (45  < angle && angle <=  90) reading = 5;
    if (90  < angle && angle <= 135) reading = 6;
    if (135 < angle && angle <= 180) reading = 7;
    if (180 < angle && angle <= 225) reading = 0;
    if (225 < angle && angle <= 270) reading = 1;
    if (270 < angle && angle <= 315) reading = 2;
    if (315 < angle)                 reading = 3;
    orientation = reading;
    Serial.print("Heading to ");
    Serial.println(cardinalNames[orientation]); 
  }
  return orientation;
}
