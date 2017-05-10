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
  float straightAngle = 90;
  float testingVar = 1000;

  /*for the speed updates*/
  float RPM[] = {0,0};
  float RPMTimeLastUpdate = 0;
  float RPMdt;
  boolean switched = true;

  /*Distance monitoring*/
  long echoDuration, distanceToBrick;
  float avoidModeDistance = 0;
  boolean avoidMode = 0; // 0 = linjeföljning, 1 = undvik tegelsten  

  /*Linesensor monitoring*/
  int lineSensorBool[] = {0,0,0,0,0,0,0,0};
  float lineSensorWeights[] = {-99*mm,-72*mm,-44*mm,-15*mm,15*mm,44*mm,72*mm,99*mm};
  float lastAngle = straightAngle;
  float initiateBrickAvoidDistance = 50; //cm
  float ds = 0;
  float dsDot = 0;
  boolean offline = 0;

  /*Essential controller stuff*/
  float distanceTravelled = 0;
  float acceleration = 0;
  float velocity = 0;
  float wantedVelocity = 0;
  float maxVelocity = 1.3;
  float minVelocity = 0.8;
  float inputSpeed = 105;
  //P parameter for velocity
  float Kp = 10;
  //I controller for velocity
  float Ki = 1;
  //Steering limits
  float maxAngle = 110;
  float minAngle = 70;
  float minInput = 74;
  float maxInput = 106;
  //PD controller for angle
  float KpA = 120;
  float KdA = 1;
 //D controller for velocity
  float KdV = 2;
  float sumError = 0;
  float ERRORTimeLastUpdate = 0;
  float ERRORdt;
  //samplingstiden för ultraljud är 10ms så lite mer än de bör samplingstiden ts vara
  float ts = 15;
  //controlangle to servo [degrees]
  float alpha = 90;
  //length of car [m]
  float Lc = 262*mm;
  //length to sensors from front wheel axle [m]
  float Ls = 85*mm;

  String readString = "";

// CREATE FUNCTIONS //
//-------------------------------------------------//
  void setSteerAngle(float angle)
  {
    //value is converted so that the angle sent to the servo results in the requested angle by linear approximation
    if(angle > maxAngle)
    {
      angle = maxAngle;  
    }
    else if(angle < minAngle)
    {
      angle = minAngle;
    }
    float m = (minInput - (minAngle / maxAngle) * maxInput) /  ((minAngle / maxAngle) + 1);
    float k = (maxInput - m) / maxAngle;
    angle = k * angle + m;
    Steering.write(angle);
    
  }

  void setSpeed(float mps)
  {
    if(mps > 150)
    {
      mps = 150;  
    }
    else if(mps < 0)
    {
      mps = 0;
    }
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
      updateDistance();
      updateVelocity();
      updateAcceleration();
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
      float feedforwardAlpha = straightAngle + atan(2*ds*Lc/(ds*ds + (Lc + Ls)*(Lc + Ls)))*RADtoDEGREE;
      //Angle alpha is set using a P controller and feedforward of the calculated angle
      alpha = ds*KpA + feedforwardAlpha + dsDot*KdA;
      
  }
  void updateWantedVelocity()
  {
    if (avoidMode == 0)
    {
      float x = (maxVelocity - minVelocity)/(90-minAngle);
      wantedVelocity = maxVelocity - abs(90-alpha) * x - dsDot*KdV;
    }
    else
    {
      wantedVelocity = 0.5;
    }
  }
  void updateInputSpeed()
  {
    ERRORdt = (millis() - ERRORTimeLastUpdate)/1000.0;
    double error = wantedVelocity - velocity;
    sumError = sumError + error*ERRORdt;
    if (sumError < 0)
    {
      sumError = 0;
    }
    else if (sumError > 50)
    {
      sumError = 50;
    }

    //inputspeed is determined by a PI regulator
    //inputSpeed = inputSpeed + error*Kp + sumError*Ki;
    inputSpeed = 101 + error*Kp + sumError*Ki;
    if(inputSpeed < 101)
    {
      inputSpeed = 101;  
    }
    ERRORTimeLastUpdate = millis();
  }

  /*a function to check which linesensors that are high and calculate the lineposition based on this!*/
  void checkLineSensors()
  {
    // saves last lineposition
    float lastds = ds;
    //resets linepostion to 0;
    ds = 0;
    //keeping a counter of how many of the sensor are true
    int nrOfTrueSensors = 0;
    //checking alla sensors and summing the position of the true sensors
    for(int pin = 0; pin < lineSensorAmount; pin++)
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
      offline = 1;
    }
    else
    {
    //ds is the mean of the positions of the true sensors
    ds = ds / nrOfTrueSensors;
    //dsDot is the derivative of the line´position over time
    dsDot = 1000*abs(ds - lastds) / ts;
    }
    if (nrOfTrueSensors > 0 && offline == 1)
    {
      if (lastds >= 0 && ds >= 0)
      {
        offline = 0;
      }
      else if (lastds<0 && ds < 0)
      {
        offline = 0;
      }
      else
      {
        ds = lastds;
      }
    }
  }

  /**/
  void checkDistance()
  {
    float startTime = millis();
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2); // Added this line
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10); // Added this line
    digitalWrite(trigPin, LOW);
    echoDuration = pulseIn(echoPin, HIGH,9000);
    distanceToBrick = (echoDuration/2) / 29.1; //Centimeter
   
    if (10-(millis()-startTime)<0)
    {
      delay(10-(millis()-startTime));
    }
    if(( (distanceToBrick < initiateBrickAvoidDistance) && distanceToBrick >= 10 ) && avoidMode == 0)
    {
      avoidMode = 1;
      avoidModeDistance = distanceTravelled;
    }
  }

  void avoidBrick()
  {
    float distance = distanceTravelled - avoidModeDistance; //Distans sedan avoidMode initierades, dvs. hur långt roboten åkt sedan den märkt av roboten.
    float angle;
    if(distance < 0.4)
    { //Första steget, när roboten ändrar riktning och åker tills den är bredvid tegelstenen.
      angle = minAngle;
    } 
    else
    {
      angle = maxAngle;
      if (lineSensorBool[4] == 1)
      {
        avoidMode = 0;
      }
    }
    setSteerAngle(angle);
    //Serial.println(distance);
  }

  /*This is the update function!*/
  void checkSensors()
  {
    checkLineSensors();
    //checkDistance();
  }

  void updateValues()
  {
    updateAngle();
    updateWantedVelocity();
    updateInputSpeed();
  }

  void execute()
  {
    setSpeed(inputSpeed);
    if(avoidMode == 0)
    {
      setSteerAngle(alpha);
    }
    else
    {
      avoidBrick();
    }
      
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
    pinMode(steeringPin,OUTPUT);
    Steering.attach(steeringPin);
    
    //Attach the motor
    pinMode(motorPin,OUTPUT);
    Motor.attach(motorPin,800,2200);

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
    float startTime = millis();
    checkSensors();
    updateValues();
    execute();
   /* 
    if(startTime > 10000 && switched) {
      setSteerAngle(76);
      switched = false;
      } else if(startTime > 20000 && !switched) {
        setSteerAngle(90);
        }
    if (startTime > 30000)
    {
      setSteerAngle(110);
    }
    if (startTime > 40000)
    {
      setSteerAngle(90);
    }
    */
    /*
    Serial.print(lineSensorBool[0]);
    Serial.print(", ");
    Serial.print(lineSensorBool[1]);
    Serial.print(", ");
    Serial.print(lineSensorBool[2]);
    Serial.print(", ");
    Serial.print(lineSensorBool[3]);
    Serial.print(", ");
    Serial.print(lineSensorBool[4]);
    Serial.print(", ");
    Serial.print(lineSensorBool[5]);
    Serial.print(", ");
    Serial.print(lineSensorBool[6]);
    Serial.print(", ");
    Serial.println(lineSensorBool[7]);
    */
    //Serial.println(ds);
    //Serial.print(alpha);
    //Serial.print(", ");
    //Serial.println(", ");
    //Serial.print(velocity);
    //Serial.print(", ");
    //Serial.println(wantedVelocity);
    //Serial.print(", ");
    //Serial.println(inputSpeed);
    //Serial.print(", ");
    //Serial.println(distanceToBrick);
    float endTime = millis();
    if(ts-(endTime-startTime) > 0) 
    {
      delay(ts-(endTime-startTime));
    }
  }
