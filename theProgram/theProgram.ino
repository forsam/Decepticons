#include <Servo.h>

// DEFINE ALL THE PINS //
////////////////////////////////////////////////
  #define steeringPin 2
  #define motorPin 3
  int lineSensorPins[] =  {5,6,7,8,9,10,11,12};


// CREATE OBJECTS!! //
////////////////////////////////////////////////
  Servo Steering;
  Servo Motor;


// CREATE GLOBAL VARIABLES!! //
///////////////////////////////////////////////
  /*for the speed monitoring*/
  int RPM = 0;
  unsigned long RPMTimeLastUpdate =0;


// CREATE FUNCTIONS //
///////////////////////////////////////////
  void setSteerAngle(float angle)
  {
    Steering.write(angle);
  }

  void setSpeed(float mps)
  {
    Motor.write(mps);
  }

  /*this runs when the magnet on the wheel is detected*/
  void magnetDetect()
  {
    updateRpm();
  }

  /*Updates the global variable rpm*/
  int updateRpm()
  {
      RPM = 60*0.25*1000 / ((millis() - RPMTimeLastUpdate));
      RPMTimeLastUpdate = millis();
  }

  /*a function to check which linesensors that are high!*/
  void checkLineSensors()
  {
    for(int pin = 0; pin < 8; pin++)
    {
      if(digitalRead(lineSensorPins[pin]) == HIGH)
      {
        Serial.println(lineSensorPin[pin]);
      }
    }
  }

  /*This is the update function!*/
  void update()
  {
    checkLineSensors();
  }

// RUN THE FIRST SETUP LOOP //
///////////////////////////////////////////////////////
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
    for(pin = 0; pin < 8; pin++)
    {
      pinMode(lineSensorPins[pin],INPUT);
    }
  }


// THIS IS THE LOOP!! //
///////////////////////////////////////////////////////
  void loop(void){

    setSpeed(100);
    setSteerAngle(40);
    checkLineSensors();

  }
