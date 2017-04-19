#include <Servo.h>

// DEFINE ALL THE PINS //
//-----------------------------------------------//
  #define velocityPin 2
  #define steeringPin 3
  #define motorPin 4
  int lineSensorPins[] =  {5,6,7,8,9,10,11,12};
  int lineSensorAmount = 8;
  #define echoPin 13
  #define trigPin A0


// CREATE OBJECTS!! //
//-----------------------------------------------//
  Servo Steering;
  Servo Motor;


// CREATE GLOBAL VARIABLES!! //
//-----------------------------------------------//
  /*for the speed updates*/
  float RPM[] = {0,0};
  float RPMTimeLastUpdate = 0;
  float RPMdt;
  
  /*Distance monitoring*/
  long echoDuration, distanceToBrick;

  /*Linesensor monitoring*/
  int lineSensorBool[] = {0,0,0,0,0,0,0,0};

  /*Essential controller stuff*/
  float distanceTravelled = 0;
  float acceleration = 0;
  float velocity = 0;
  float wantedVelocity = 5;
  int inputSpeed = 102;
  int Kp = 0.1;
  
  /*Basic variables*/
  int mm = 0.001;
  int wheelDiameter = 65*mm;
  float RPMtoMS = wheelDiameter*3.1415/60;

// CREATE FUNCTIONS //
//-------------------------------------------------//
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
  void updateRpm()
  {
      Serial.print("detected");
      RPMdt = (millis() - RPMTimeLastUpdate)/1000;
      RPM[1] = RPM[0];
      RPM[0] = 60*0.25*1000 / (RPMdt);
      RPMTimeLastUpdate = millis();
  }
  void updateAcceleration()
  {
    acceleration = (RPM[1] - RPM[0])*RPMtoMS/RPMdt;
  } 

  void updateDistance()
  {
    distanceTravelled = distanceTravelled + RPM[0]*RPMtoMS*RPMdt;
  }

  void updateVelocity()
  {
    velocity = RPM[0]*RPMtoMS;
  }

  void updateInputSpeed()
  {
    double error = wantedVelocity - velocity;
    inputSpeed = inputSpeed + error*Kp;
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

  /*This is the update function!*/
  void checkSensors()
  {
    checkLineSensors();
    //checkDistance();
  }

  void updateValues()
  {
    updateAcceleration();
    updateDistance();
    updateVelocity();
    updateInputSpeed();
  }

  void execute()
  {
    setSpeed(inputSpeed);
  }

// RUN THE FIRST SETUP LOOP //
//--------------------------------------------------------//
  void setup(void)
  {
    Serial.begin(9600);

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
//----------------------------------------------------------//
  void loop(void)
  {
    checkSensors();
    updateValues();
    execute();
  }
