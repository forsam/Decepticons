#include <Servo.h>

// For the SD-card things!
#include <SD.h>
#include <Wire.h>
#include "RTClib.h"
#include <SPI.h>

// Create real time clock object
RTC_DS1307 RTC; 

// Define all the pins!
#define steeringPin 6 
#define motorPin 3
#define SDPin 10    

// Things for the SD-card
#define SYNC_INTERVAL 1000 // mills between calls to flush() - to write data to the card
uint32_t syncTime = 0;     // time of last sync()

// create servo object to control the steering
Servo Steering;  

// create servo object to control the motor
Servo Motor;  

// Variables
long startLoop = 0;
long endLoop = 0;
long loopTime = 50;
// For the randomSpeed
int interval = 1000;
long lastTime = 0;
long currentTime = 0;
int randSpeed = 90;

// For the SD-card
char filename[] = "LOGGER00.CSV";

//for the speed monitoring
int rpm = 0;
unsigned long timeold =0;


// the logging file
File logfile;

void setAngle(float angle){
  Steering.write(angle);
  }

void setSpeed(float mps){
  Motor.write(mps);
  }

void magnetDetect() {
  updateRpm();
  }

void writeToFile(){
  logfile.print(millis());
  logfile.print(",");
  logfile.print(rpm);
  logfile.print(",");
  logfile.println(randSpeed);

  Serial.print(millis());
  Serial.print(",");
  Serial.print(rpm);
  Serial.print(",");
  Serial.println(randSpeed);
  }

int getRandomSpeed(){
  unsigned long currentMillis = millis();
  if ((currentMillis - lastTime) > interval){
    lastTime = currentMillis;
    interval = random(5000, 10000);
    randSpeed = random(100,130);
    } 
    return randSpeed; 
  }

void initFile(){
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i/10 + '0';
    filename[7] = i%10 + '0';
    if (!SD.exists(filename)) {
      // only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE);
      break;  // leave the loop!
    }
  }
  }

int updateRpm(){
    rpm = 60*0.25*1000 / ((millis() - timeold));
    timeold = millis();
  }

void setup(void)
{
  Serial.begin(9600);
  
  
  attachInterrupt(0, magnetDetect, RISING);

  SD.begin(SDPin);
  pinMode(SDPin, OUTPUT);
  
  //Attach the steering
  Steering.attach(steeringPin);
  pinMode(steeringPin,OUTPUT);

  // Attach the motor
  Motor.attach(motorPin);
  pinMode(motorPin,OUTPUT);

  // creates a new file named LOGGER01, LOGGER02...LOGGER## etc...
  initFile();
  
  // connect to real time clock(RTC)
  Wire.begin();

}

void loop(void){
  startLoop = millis();
  setSpeed(getRandomSpeed());
  setAngle(40);
  writeToFile();

  // Now we write data to disk! Don't sync too often - requires 2048 bytes of I/O to SD card
  // which uses a bunch of power and takes time
  if ((millis() - syncTime) < SYNC_INTERVAL) return;
  syncTime = millis();
  logfile.flush();  

  endLoop = millis();

  if(loopTime - (startLoop-endLoop) > 0)
  {
    delay(loopTime - (startLoop-endLoop));  
  }
}
