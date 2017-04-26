#include <Servo.h>

// DEFINE ALL THE PINS //
//-----------------------------------------------//
  #define steeringPin 3
  #define motorPin 2
  #define velocityPin 4
  int lineSensorPins[] =  {5,6,7,8,9,10,11,12};
  int lineSensorAmount = 8;
  #define echoPin 13
  #define trigPin A1


// CREATE OBJECTS!! //
//-----------------------------------------------//
  Servo Steering;
  Servo Motor;


// CREATE GLOBAL VARIABLES!! //
//-----------------------------------------------//
 /*Basic variables*/
  float mm = 0.001;
  float wheelDiameter = 65*mm;
  float RPMtoMS = wheelDiameter*3.1415/60;
  float RADtoDEGREE = 180/3.1415;

  /*for the speed updates*/
  float RPM[] = {0,0};
  float RPMTimeLastUpdate = 0;
  float RPMdt;

  /*Distance monitoring*/
  long echoDuration, distanceToBrick;

  /*Linesensor monitoring*/
  int lineSensorBool[] = {0,0,0,0,0,0,0,0};
  float lineSensorWeights[] = {-67*mm,-51*mm,-30*mm,-10*mm,10*mm,30*mm,47*mm,68*mm};
  float ds = 0;

  /*Essential controller stuff*/
  float distanceTravelled = 0;
  float acceleration = 0;
  float velocity = 0;
  float wantedVelocity = 0.5;
  float inputSpeed = 102;
  //P parameter for velocity
  float Kp = 0.01;
  //I controller for velocity
  float Ki = 0;
  //P controller for angle
  float KpA = 170;
  float sumError = 0;
  float ERRORTimeLastUpdate = 0;
  float ERRORdt;
  float ts = 5;
  //controlangle to servo [degrees]
  float alpha = 90;
  //length of car [m]
  float Lc = 262*mm;
  //length to sensors from front wheel axle [m]
  float Ls = 90*mm;

// CREATE FUNCTIONS //
//-------------------------------------------------//
  void setSteerAngle(float angle)
  {
    if(angle > 180)
    {
      angle = 180;  
    }
    else if(angle < 0)
    {
      angle = 0;
    }
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
      RPMdt = (millis() - RPMTimeLastUpdate)/1000;
      RPM[1] = RPM[0];
      RPM[0] = 60*0.25 / (RPMdt);
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
  //calculates the wanted angle based on lineposition
  void updateAngle()
  {
      float feedforwardAlpha = 90 + atan(2*ds*Lc/(ds*ds + (Lc + Ls)*(Lc + Ls)))*RADtoDEGREE;
      //Angle alpha is set using a P controller and feedforward of the calculated angle
      alpha = ds*KpA + feedforwardAlpha;
      
  }

  void updateInputSpeed()
  {
    ERRORdt = (millis() - ERRORTimeLastUpdate)/1000;

    double error = wantedVelocity - velocity;
    sumError = sumError + error*ERRORdt;

    //inputspeed is determined by a PI regulator
    inputSpeed = inputSpeed + error*Kp + sumError*Ki;
    ERRORTimeLastUpdate = millis();
  }

  /*a function to check which linesensors that are high and calculate the lineposition based on this!*/
  void checkLineSensors()
  {
    // saves lasat lineposition
    float lastds = ds;
    //resets linepostion to 0;
    ds = 0;
    //keeping a counter of how many of the sensor are true
    int nrOfTrueSensors = 0;
    //checking alla sensors and summing the position of the true sensors
    for(int pin = 1; pin < lineSensorAmount; pin++)
    {
      if(digitalRead(lineSensorPins[pin]) == HIGH)
      {
        lineSensorBool[pin] = 1;
        ds = lineSensorWeights[pin] + ds;
        nrOfTrueSensors++;
      }
      else
      {
        lineSensorBool[pin] = 0;
      }
    }
    if (nrOfTrueSensors == 0)
    {
      // if no sensors are true, the last calculated position is kept.
      ds = lastds;
    }
    else
    {
    //ds is the mean of the positions of the true sensors
    ds = ds / nrOfTrueSensors;
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
    updateAngle();
    updateInputSpeed();
  }

  void execute()
  {
    setSpeed(inputSpeed);
    setSteerAngle(alpha);
  }

// RUN THE FIRST SETUP LOOP //
//--------------------------------------------------------//
  void setup(void)
  {
    Serial.begin(9600);

    //Attach the hallsensor!
    pinMode(velocityPin,INPUT_PULLUP);
    attachInterrupt(velocityPin, magnetDetect, RISING);
    
    //Attach the steering
    Steering.attach(steeringPin);
    pinMode(steeringPin,OUTPUT);

    //Attach the motor
    Motor.attach(motorPin);
    pinMode(motorPin,OUTPUT);

    //Attach the linesensors
    for(int pin = 1; pin < lineSensorAmount; pin++)
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
    //Serial.print(lineSensorBool[0]);
    //Serial.print(", ");
    //Serial.print(lineSensorBool[1]);
    //Serial.print(", ");
    //Serial.print(lineSensorBool[2]);
    //Serial.print(", ");
    //Serial.print(lineSensorBool[3]);
    //Serial.print(", ");
    //Serial.print(lineSensorBool[4]);
    //Serial.print(", ");
    //Serial.print(lineSensorBool[5]);
    //Serial.print(", ");
    //Serial.print(lineSensorBool[6]);
    //Serial.print(", ");
    //Serial.print(lineSensorBool[7]);
    //Serial.print(", ");
    //Serial.print(alpha);
    //Serial.print(", ");
    //Serial.print(ds);
    //Serial.println(", ");
    //Serial.print(velocity);
    //Serial.print(", ");
    //Serial.print(wantedVelocity);
    //Serial.print(", ");
    //Serial.print(inputSpeed);
    //Serial.print(", ");
    //Serial.println(distanceToBrick);
    delay(ts*2);
  }
