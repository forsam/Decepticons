#include <Servo.h>

// DEFINE ALL THE PINS //
////////////////////////////////////////////////
  #define steeringPin 3
  #define motorPin 4
  int lineSensorPins[] =  {5,6,7,8,9,10,11,12};
  int lineSensorAmount = 7;
  #define echoPin 13
  #define trigPin A0
  

// CREATE OBJECTS!! //
////////////////////////////////////////////////
  Servo Steering;
  Servo Motor;


// CREATE GLOBAL VARIABLES!! //
///////////////////////////////////////////////
  /*for the speed monitoring*/
  int RPM = 0;
  unsigned long RPMTimeLastUpdate =0;
  
  /*Distance monitoring*/
  long echoDuration, distanceToBrick;

  /*Linesensor monitoring*/
  int lineSensorBool[] = {0,0,0,0,0,0,0}

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
    for(int pin = 0; pin < lineSensorAmount; pin++)
    {
      if(digitalRead(lineSensorPins[pin]) == HIGH)
      {
        lineSensorBool[pin] = 1;
      }
      else
      {
        lineSensorBool[pin] = 0;
      }
    }
  }

  /**/
  void checkDistance()
  {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2); // Added this line
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10); // Added this line
    digitalWrite(trigPin, LOW);
    echoDuration = pulseIn(echoPin, HIGH);
    distanceToBrick = (echoDuration/2) / 29.1;
  }

  void checkVelocity()
  {
    
  }

  /*This is the update function!*/
  void checkSensors()
  {
    checkLineSensors();
    checkDistance();
  }

  void execute()
  {
    
  }

// RUN THE FIRST SETUP LOOP //
///////////////////////////////////////////////////////
  void setup(void)
  {

    attachInterrupt(0, magnetDetect, RISING);

    //Attach the steering
    Steering.attach(steeringPin);
    pinMode(steeringPin,OUTPUT);

    //Attach the motor
    Motor.attach(motorPin);
    pinMode(motorPin,OUTPUT);

    //Attach the linesensors
    for(int pin = 0; pin < lineSensorAmount; pin++)
    {
      pinMode(lineSensorPins[pin],INPUT);
    }
    //Attach the echosensor!
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
  }


// THIS IS THE LOOP!! //
///////////////////////////////////////////////////////
  void loop(void)
  {
    checkSensors();
    execute();
    
  }
