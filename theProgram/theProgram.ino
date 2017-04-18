#include <Servo.h>

// Define all the pins!
#define steeringPin 2 
#define motorPin 3  
int[] lineSensorPins =  {5,6,7,8,9,10,11,12};

// create servo object to control the steering
Servo Steering;  

// create servo object to control the motor
Servo Motor;  

// Variables
// For the randomSpeed
int interval = 1000;
int lastTime = 0;
int currentTime = 0;
int randSpeed = 90;

//for the speed monitoring
int rpm = 0;
unsigned long timeold =0;

void setAngle(float angle){
  Steering.write(angle);
  }

void setSpeed(float mps){
  Motor.write(mps);
  }

void magnetDetect() {
  updateRpm();
  }

int updateRpm()
{
    rpm = 60*0.25*1000 / ((millis() - timeold));
    timeold = millis();
}

void update()
{
  checkLineSensors();
}

void checkLineSensors()
{
  for(int pin = 0; pin < lineSensorPins.length(); pin++)
  {
    if(digitalRead(lineSensorPins[pin]) == HIGH)
    {
      Serial.println(lineSensorPin[pin]);
    }
  }
}

void setup(void)
{
  Serial.begin(9600);
  
  attachInterrupt(2, magnetDetect, RISING);
  
  //Attach the steering
  Steering.attach(steeringPin);
  pinMode(steeringPin,OUTPUT);

  //Attach the motor
  Motor.attach(motorPin);
  pinMode(motorPin,OUTPUT);

  //Attach the linesensors
  for(pin = 0; pin < lineSensorPins.length(); pin++)
  {
    pinMode(lineSensorPins[pin],INPUT);
  }
}

void loop(void){
  
  setSpeed(getRandomSpeed());
  setAngle(40);
  //update
    //check speed
    //check steering

  checkLineSensors();
  //execute

  // Now we write data to disk! Don't sync too often - requires 2048 bytes of I/O to SD card
  // which uses a bunch of power and takes time

}
