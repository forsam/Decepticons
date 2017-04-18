#include <SPI.h>
#include <SD.h>
#include <Servo.h>
#include <Wire.h>
#include "RTClib.h"
#include "Timer.h"

#define anglePin 5
#define servoPin 9
#define motorPin 6

#define chipSelect 10    // for the data logging shield, we use digital pin 10 for the SD cs line
#define SYNC_INTERVAL 1000 // mills between calls to flush() - to write data to the card
uint32_t syncTime = 0;     // time of last sync()

Servo myservo;  // create servo object to control a servo
Servo mymotor;  // create servo object to control a motor

int potRead;
double randAngle = 90;
int interval = 1000;
int lastTime = 0;
float angleVal=0;
int currentTime = 0;
// for the data logging shield, we use digital pin 10 for the SD cs line
volatile byte half_revolutions;
unsigned int rpm;
unsigned long timeold;

RTC_DS1307 RTC; // define the Real Time Clock object
#define chipSelect 10
// the logging file
File logfile;

void setAngle(float angle){
  myservo.write(angle);
  }

// This looks okay!
void SetSpeed(float mps){
  mymotor.write(mps);
  }

void magnetDetect() {
  half_revolutions++;
  Serial.println("detected");
  }

void setup(void)
{
  Serial.begin(9600);
  attachInterrupt(0, magnetDetect, RISING);
  half_revolutions = 0;
  rpm = 0;
  timeold =0;
  myservo.attach(9);
  mymotor.attach(10);

  pinMode(chipSelect, OUTPUT);
  pinMode(anglePin, OUTPUT);


  // creates a new file named LOGGER01, LOGGER02...LOGGER## etc...
  char filename[] = "LOGGER00.CSV";
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i/10 + '0';
    filename[7] = i%10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE);
      break;  // leave the loop!
    }
  }

  // connect to real time clock(RTC)
  Wire.begin();

}

void loop(void){

  angleVal = analogRead(anglePin);

  unsigned long currentMillis = millis();
  if ((currentMillis - lastTime) > interval){
    lastTime = currentMillis;
    interval = random(200, 600);
    randAngle = random(80,110);
    }

  if (half_revolutions >= 20) {
    rpm = 30 * 1000 / (millis() - timeold) * half_revolutions;
    timeold = millis();
    half_revolutions = 0;
    }
  setAngle(randAngle);
  SetSpeed(120);

  //logfile.print(randAngle);
  //Serial.println(angleVal);

  // Now we write data to disk! Don't sync too often - requires 2048 bytes of I/O to SD card
  // which uses a bunch of power and takes time
  if ((millis() - syncTime) < SYNC_INTERVAL) return;
  syncTime = millis();
  logfile.flush();

}
