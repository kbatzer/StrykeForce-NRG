/*****************************************************************************
 * Tank control main code file
 *
 * This file contains constants and functions for the Arduino Robot Car,
 * allowing players to complete the NRG Y1 and Y2 challenges
 *****************************************************************************/

/******************************************************************************
 * Example of a list of drive states, or actions the car will take
 * automatically:
 *
 * DriveState_E driveStateSequence[] = {TRACK_LINE_ULTRASONIC,
 *                                      LEFT_90,
 *                                      FORWARD_ULTRASONIC,
 *                                      LEFT_90,
 *                                      TRACK_LINE_ULTRASONIC,
 *                                      RIGHT_45,
 *                                      FWD_6IN,
 *                                      FWD_6IN,
 *                                      FWD_6IN,
 *                                      RIGHT_45,
 *                                      FWD_TILL_LINE,
 *                                      LINE_TRACK_RESYNC,
 *                                      TRACK_LINE_ULTRASONIC,
 *                                      CONTROLLER};
 *
 * You can add commands inside the {} symbols to the right of the
 * driveStateSequence initialization on line 39.
 *****************************************************************************/

#include "String.h"
#include <Math.h>
#include "nrg.h"
#include <Servo.h>  //servo library

//****************************************************************************************/
// global variables
//****************************************************************************************/
DriveState_E driveStateSequence[]     = {CONTROLLER};

String inputString = "";    // a string to hold incoming data
String outputString = "";   // a string for debug

const float LEFT_SCALE                = 1.0; //Pick a number from 0 to 1 to balance your motors if one side is faster than the other
const float RIGHT_SCALE               = 1.0; //Pick a number from 0 to 1 to balance your motors if one side is faster than the other
const int LINE_TRACKED_RESET          = 100;

ControllerInputs_T controllerInputs   = { .LY=0, .RY=0,.buttons=0, .isCommTimeoutDisabled=1};

CarCommand_T carCommand               = {.direction=STOP,
                                         .leftMotorSpeed =0,
                                         .rightMotorSpeed=0,
                                         .leftScale = LEFT_SCALE, //Pick a number from 0 to 1 to balance your motors if one side is faster than the other
                                         .rightScale= RIGHT_SCALE //Pick a number from 0 to 1 to balance your motors if one side is faster than the other
                                         };


SensorFeedback_T sensors              = {.distance = 0};

///////////////////////////////////////////////////////////////////////////////
// Encoder Variables and Library

#include <Encoder.h>

Encoder myEncL(A0, A1); //Left Encoder
Encoder myEncR(A2, A3); //Right Encoder
long oldPositionL  = -999; //Initialize position
long newPositionL;
long oldPositionR  = -999; //Initialize position
long newPositionR;
int distance; //Distance for car to drive in inches
double encoderDistance; //Distance for car to drive in encoder counts
double encoderScale = 1.3; //For converting from inches to encoder counts
int leftSpeed;
int rightSpeed;
bool leftStopped = false;
bool rightStopped = false;

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
/*** Bucket Servos ***/
Servo liftServo;
Servo bucketServo;

const int liftServoDumpAngle          = 0;
const int liftServoLiftAngle          = 0;
const int liftServoLoadAngle          = 0;

const int bucketServoDumpAngle        = 0;
const int bucketServoLiftAngle        = 0;
const int bucketServoLoadAngle        = 0;
///////////////////////////////////////////////////////////////////////////////

/*** Ultrasonic Servo ***/
Servo myservo;
int servoAngle                        = 45;
/************************/

bool searching                        = true;
int SPEED                             = 50; //SPEED is in percent

int driveStateIndex                   = 0;
DriveState_E driveState               = driveStateSequence[driveStateIndex]; //Initialize which State the car starts in
long startOfStateTime                 = 0; //keeps track of how much time it is in a start

int lineTrackedCounter                = LINE_TRACKED_RESET;
int servoOffset                       = -4;


//****************************************************************************************/
// Standard Arduino Functions
//****************************************************************************************/

  //****************************************************************************************/
  // setup()
  // Initialization function.  Runs once on startup.
  //****************************************************************************************/
  void setup() {
    myservo.attach(3);
    inputString.reserve(255); // make inputString large enough to hold 255 chars
    Serial.begin(9600);       // Turn on bluetooth and initialize to 9600 bod
    InitNrgPins();
  }


  //****************************************************************************************/
  // loop()
  //
  // Main loop.  Executes repeatedly after setup() is complete.
  //****************************************************************************************/
  void loop() {

    // ***** Inputs ************
    ReadSensors();
    //ControllerButtonChecks();  // disable controller buttons mode changes for base drive code

    // ***** Determine Actions ************
    UpdateState();
    CheckCommTimeout();

    // ***** Drive Outputs ************
    MoveCar(&carCommand);
    SetLeds();
  }


  //****************************************************************************************/
  // serialEvent
  //****************************************************************************************/
  void serialEvent()
  {
    while (Serial.available())
    {
      char c = Serial.read();  // Read character
      inputString += c;

      if (c == ';')
        // If end-of-line, parse and reset inputStringfer and send back the data
      {
        //Serial.println(inputString);
        ParseInputString(&controllerInputs, inputString);  // Parse received data
        inputString.remove(0, inputString.length());
      }
    }
  }


//****************************************************************************************/
// State Handling
//****************************************************************************************/

  //****************************************************************************************/
  // ControllerButtonChecks()
  //
  // Check for controller button presses and update state.  Note that controller updates are
  // received at a slower rate than the main loop executes, meaning a single button press
  // will be detected multiple times.
  //****************************************************************************************/
  void ControllerButtonChecks()
  {
    //if button Y is pressed the car jumps into Controller State
    if (isButtonPressed(controllerInputs.buttons, BUTTON_Y_MASK))
    {
      startOfStateTime = micros();
      driveState = CONTROLLER;
    }
    else if (isButtonPressed(controllerInputs.buttons, BUTTON_A_MASK))
    {
      startOfStateTime = micros();
      driveState = AUTO;
    }
    else if (isButtonPressed(controllerInputs.buttons, BUTTON_X_MASK))
    {
      startOfStateTime = micros();
      driveState = TRACK_LINE;
    }
    else if (isButtonPressed(controllerInputs.buttons, BUTTON_R1_MASK))
    {
      SetBucketLift();
    }
    else if (isButtonPressed(controllerInputs.buttons, BUTTON_R3_MASK))
    {
      SetBucketLoad();
    }
    else if (isButtonPressed(controllerInputs.buttons, BUTTON_B_MASK))
    {
      SetBucketDump();
    }

    // TODO: Add cases for forklift/claw operation.
  }


  //****************************************************************************************/
  // StateTransition()
  // If condition is met, capture system timer and move to next state
  //****************************************************************************************/
  void StateTransitionCheck(bool stateTransitionRequired)
  {
    if(stateTransitionRequired==true)
    {
      startOfStateTime = micros();
      driveStateIndex++;
      driveState = driveStateSequence[driveStateIndex];
    }
  }


  //****************************************************************************************/
  // UpdateState()
  //
  // Contains the implementation for each defined behavior.
  //****************************************************************************************/
  void UpdateState()
  {
    switch(driveState)
      {
        //****************************************************************************************/
        // Controller inputs are used to drive car (tank control mode).
        // If A is pressed, the state index is set to 0
        //****************************************************************************************/
        case CONTROLLER:
          SetDirection(&controllerInputs, &carCommand);
          SetMotorSpeeds(controllerInputs.LY, controllerInputs.RY, &carCommand);

//          if isButtonPressed(controllerInputs.buttons, BUTTON_A_MASK)
//          {
//            startOfStateTime = micros();
//            driveStateIndex = 0;
//            driveState = driveStateSequence[driveStateIndex];
//          }
        break;



        //****************************************************************************************/
        // Basic line tracking without state sequencing.  Update to include StateTransitionCheck()
        //****************************************************************************************/
        case TRACK_LINE:
          TrackLine_BackExit();
          break;


        //****************************************************************************************/
        // Line tracking with exit condition based on ultrasonic sensor
        //****************************************************************************************/
        case TRACK_LINE_ULTRASONIC:
          TrackLine_ForwardExit();
          StateTransitionCheck(sensors.distance != 0 && sensors.distance < 10);
          break;


        //****************************************************************************************/
        // Line tracking with exit condition based on being aligned for LINE_TRACKED_RESET
        // iterations
        //****************************************************************************************/
        case LINE_TRACK_RESYNC:
          TrackLine_BackExit();

          // if either left or right sensor don't see the line, decrement counter
          if(sensors.LineTracker_L == 0 && sensors.LineTracker_R == 0)
          {
            lineTrackedCounter--;
          }
          // else reset counter
          else
          {
            lineTrackedCounter = LINE_TRACKED_RESET;
          }

          // if the counter reaches zero, move to next state
          if(lineTrackedCounter == 0)
          {
            lineTrackedCounter = LINE_TRACKED_RESET;
            StateTransitionCheck(true);
          }
          break;


        //****************************************************************************************/
        // Move forward until ultrasonic sesnor detects object within specified distance
        //****************************************************************************************/
        case FORWARD_ULTRASONIC:
          myservo.write(90+servoOffset); // use servo to point ultrasonic sensor forward
          Forward(20);
          StateTransitionCheck(sensors.distance != 0 && sensors.distance < 10);
          break;


        //****************************************************************************************/
        // Move forward until any line tracker module detects a line
        //****************************************************************************************/
        case FWD_TILL_LINE:
          Forward(10);
          StateTransitionCheck(sensors.LineTracker_L == 1 || sensors.LineTracker_M == 1 || sensors.LineTracker_R == 1);
          break;


        //****************************************************************************************/
        // Detect car angle using ultrasonic sensor and servo
        // NOT WORKING - EARLY DEVELOPMENT
        //****************************************************************************************/
        case FIND_CAR_ANGLE:
          carCommand.direction = STOP;
          ServoSweep2();
          if(isTimeElapsed(startOfStateTime, 20000))
          //if(GetElapsedTime(startOfStateTime) > 20000)
          {
            startOfStateTime = micros();
            driveStateIndex++;
            driveState = driveStateSequence[driveStateIndex];
          }
          break;


        //****************************************************************************************/
        // Move forward until limit switch is closed
        //****************************************************************************************/
        case LIM_SW:
          Forward(20);
          StateTransitionCheck(sensors.SwitchClosed);
          break;


        //****************************************************************************************/
        // Stop car for 3 seconds
        //****************************************************************************************/
        case DELAY_3S:
          Stop(0);
          StateTransitionCheck(isTimeElapsed(startOfStateTime, 3000));
          break;


        //****************************************************************************************/
        // Left 90 degree turn based on time. Note that the time required will vary with battery
        // charge level.
        //****************************************************************************************/
        case LEFT_90:
          Left(30);
          StateTransitionCheck(isTimeElapsed(startOfStateTime, 400));
          break;


        //****************************************************************************************/
        // Right 90 degree turn based on time. Note that the time required will vary with battery
        // charge level.
        //****************************************************************************************/
        case RIGHT_90:
          Right(30);
          StateTransitionCheck(isTimeElapsed(startOfStateTime, 400));
          break;


        //****************************************************************************************/
        // Right 45 degree turn based on time. Note that the time required will vary with battery
        // charge level.
        //****************************************************************************************/
        case RIGHT_45:
          Right(30);
          StateTransitionCheck(isTimeElapsed(startOfStateTime, 300));
          break;


        //****************************************************************************************/
        // Move forward 6 inches based on time. Note that the time required will vary with battery
        // charge level.
        //****************************************************************************************/
        case FWD_6IN:
          Forward(50);
          StateTransitionCheck(isTimeElapsed(startOfStateTime, 300));
          break;


        //****************************************************************************************/
        // Move backward 6 inches based on time. Note that the time required will vary with battery
        // charge level.
        //****************************************************************************************/
        case BACK_6IN:
          Back(50);
          StateTransitionCheck(isTimeElapsed(startOfStateTime, 300));
          break;


        //****************************************************************************************/
        // Move forward, sweeping ultrasonic sensor. If an object is detected, turn in the
        // opposite direction
        //****************************************************************************************/
        case AUTO:
          Forward(50);

          if(sensors.distance != 0 && sensors.distance < 20)
          {
            if(servoAngle < 90){driveState = AUTO_LEFT;}
            else {              driveState = AUTO_RIGHT;}
            startOfStateTime = micros();
          }
          ServoSweep();

          break;


        //****************************************************************************************/
        // Turn right for 500 ms and return to AUTO
        //****************************************************************************************/
        case AUTO_RIGHT:
          Right(50);

          if(GetElapsedTime(startOfStateTime) > 500)
          {
            startOfStateTime = micros();
            driveState = AUTO;
          }
          break;

        //****************************************************************************************/
        // Turn left for 500 ms and return to AUTO
        //****************************************************************************************/
        case AUTO_LEFT:
          Left(50);

          if(GetElapsedTime(startOfStateTime) > 500)
          {
            startOfStateTime = micros();
            driveState = AUTO;
          }
          break;

        //****************************************************************************************/
        // Car will drive until encoder conditions are satisfied
        //****************************************************************************************/
        case ENCODER_DRIVE:
          encoderDrive(distance, speed);
          StateTransitionCheck(true);
          break;

        //****************************************************************************************/
        // If undefined state, move to CONTROLLER
        //****************************************************************************************/
        default:
          startOfStateTime = micros();
          driveState = CONTROLLER;
        break;
      }
  }


//****************************************************************************************/
// State (behavior) Helper functions
//****************************************************************************************/
  //****************************************************************************************/
  // Simplified direction function overrides
  //****************************************************************************************/
  void Forward(int speed) { Forward(speed, &carCommand, false); }
  void Back(int speed)    { Back(speed, &carCommand, false); }
  void Left(int speed)    { Left(speed, &carCommand, false); }
  void Right(int speed)   { Right(speed, &carCommand, false); }
  void Stop(int speed)    { Stop(speed, &carCommand, false); }

  //****************************************************************************************/
  // Forward
  //****************************************************************************************/
  void Forward(int speed, CarCommand_T * carCommand, bool moveCarNow)
  {
    SetMotorSpeeds(int(float(speed) * carCommand->leftScale) , int(float(speed)* carCommand->rightScale), carCommand);
    carCommand->direction = FORWARD;
    if(moveCarNow){ MoveCar(carCommand); }
  }

  //****************************************************************************************/
  // Back
  //****************************************************************************************/
  void Back(int speed, CarCommand_T * carCommand, bool moveCarNow)
  {
    SetMotorSpeeds(int(float(speed) * carCommand->leftScale) , int(float(speed)* carCommand->rightScale), carCommand);
    carCommand->direction = BACK;
    if(moveCarNow){ MoveCar(carCommand); }
  }

  //****************************************************************************************/
  // Left
  //****************************************************************************************/
  void Left(int speed, CarCommand_T * carCommand, bool moveCarNow)
  {
    SetMotorSpeeds(int(float(speed) * carCommand->leftScale) , int(float(speed)* carCommand->rightScale), carCommand);
    carCommand->direction = LEFT;
    if(moveCarNow){ MoveCar(carCommand); }
  }

  //****************************************************************************************/
  // Right
  //****************************************************************************************/
  void Right(int speed, CarCommand_T * carCommand, bool moveCarNow)
  {
    SetMotorSpeeds(int(float(speed) * carCommand->leftScale) , int(float(speed)* carCommand->rightScale), carCommand);
    carCommand->direction = RIGHT;
    if(moveCarNow){ MoveCar(carCommand); }
  }

  //****************************************************************************************/
  // Stop
  //****************************************************************************************/
  void Stop(int speed, CarCommand_T * carCommand, bool moveCarNow)
  {
    SetMotorSpeeds(int(float(speed) * carCommand->leftScale) , int(float(speed)* carCommand->rightScale), carCommand);
    carCommand->direction = STOP;
    if(moveCarNow){ MoveCar(carCommand); }
  }

  //****************************************************************************************/
  // TrackLine_ForwardExit
  //****************************************************************************************/
  void TrackLine_ForwardExit()
  {
    if(sensors.LineTracker_M == 1)      { Forward(20);  } //the middle sensor is on a line
    else if(sensors.LineTracker_L== 1)  { Left(20);     } //left sensor is on line
    else if(sensors.LineTracker_R == 1) { Right(20);    } //right motor is on the line
    else                                { Forward(20);  }
  }

  //****************************************************************************************/
  // TrackLine_BackExit
  //****************************************************************************************/
  void TrackLine_BackExit()
  {
    if(sensors.LineTracker_M == 1)      { Forward(20);  } //the middle sensor is on a line
    else if(sensors.LineTracker_L== 1)  { Left(20);     } //left sensor is on line
    else if(sensors.LineTracker_R == 1) { Right(20);    } //right motor is on the line
    else                                { Back(20);     }
  }


  //****************************************************************************************/
  //
  //****************************************************************************************/
  void ServoSweep()
  {
    const int sweepStepTime = 250;
    const int sweepStepSize = 15;
    const int maxAngle = 135;
    const int minAngle = 45;

    static long lastMoveTime = micros();

    if(GetElapsedTime(lastMoveTime) > sweepStepTime)
    {
      lastMoveTime = micros();
      myservo.write(servoAngle);
      servoAngle += sweepStepSize;
      if(servoAngle > maxAngle)
      {
        servoAngle = minAngle;
      }
    }
  }

  //****************************************************************************************/
  //
  //****************************************************************************************/
  void ServoSweep2()
  {
    const int sweepStepTime = 125;
    const int sweepStepSize = 7;
    const int maxAngle = 135;
    const int minAngle = 45;
    static int dir = 1;

    static long lastMoveTime = micros();

    if(GetElapsedTime(lastMoveTime) > sweepStepTime)
    {
      lastMoveTime = micros();
      myservo.write(servoAngle);
      //servoAngle += sweepStepSize;
      if(servoAngle >= maxAngle)
      {
        dir = -1;
      }
      else if( servoAngle <= minAngle)
      {
        dir = 1;
      }

      if(dir == 1)
      {
        servoAngle += sweepStepSize;
      }
      else
      {
        servoAngle -= sweepStepSize;
      }
    }
  }


//****************************************************************************************/
// CheckCommTimeout()
//
// Safety function.  Stops car is comm timeout is elapsed.
//****************************************************************************************/
void CheckCommTimeout()
{
  // stop car if timeout is enabled and more than 2 seconds has elapsed since last comm
  if(controllerInputs.isCommTimeoutDisabled != 1)
  {
    if((GetElapsedTime(controllerInputs.lastCommTime) > 2000))
    {
      carCommand.direction = STOP;
    }
  }
  else
  {
    //debugPrint();
  }
}


//****************************************************************************************/
// SetLeds()
//
// Drives Leds based on car direction
//****************************************************************************************/
void SetLeds()
{
  switch(carCommand.direction){
    case FORWARD:
       //GREEN
       digitalWrite(RGBLED_G,HIGH);
       digitalWrite(RGBLED_R, LOW);
       digitalWrite(RGBLED_B, LOW);
       //
    break;

    case BACK:
       //RED
       digitalWrite(RGBLED_G,LOW);
       digitalWrite(RGBLED_R, HIGH);
       digitalWrite(RGBLED_B, LOW);
       //
    break;

    case LEFT:
       //MAGENTA
       digitalWrite(RGBLED_G,LOW);
       analogWrite(RGBLED_R, 150);
       analogWrite(RGBLED_B, 200);
       //
    break;

    case RIGHT:
       //BLUE
       digitalWrite(RGBLED_G,LOW);
       digitalWrite(RGBLED_R, LOW);
       digitalWrite(RGBLED_B, HIGH);
       //
    break;
  }
}


//****************************************************************************************/
// ReadSensors
//****************************************************************************************/
void ReadSensors()
{
  int lastDistance      = sensors.distance;
  sensors.distance      = GetDistance();
  sensors.SwitchClosed  = digitalRead(LimSwitch);
  sensors.LineTracker_L = digitalRead(LineTrack_L);
  sensors.LineTracker_M = digitalRead(LineTrack_M);
  sensors.LineTracker_R = digitalRead(LineTrack_R);
}


//****************************************************************************************/
// GetElapsedTime
// param   startTime - time in micro-seconds
// returns elapsed time in milli-seconds.
//****************************************************************************************/
float GetElapsedTime(long startTime)
{
  return ((float)(micros()-startTime))/1000.0;
}


//****************************************************************************************/
// isTimeElapsed
//****************************************************************************************/
bool isTimeElapsed(long startTime, float totalTimeMilliSeconds)
{
  bool status = false;
  if(GetElapsedTime(startOfStateTime) > totalTimeMilliSeconds)
  {
    status = true;
  }
  return status;
}


//****************************************************************************************/
//
//****************************************************************************************/
int GetDistance() {
  digitalWrite(Trig, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);

  float Fdistance = pulseIn(Echo, HIGH, 16000);
  Fdistance = Fdistance / 148;
  return (int)Fdistance;
}


//****************************************************************************************/
//
//****************************************************************************************/
void encoderDrive(double distance, long speed)
{
  encoderDistance = distance*encoderScale;
  leftSpeed = int(float(speed)*LEFT_SCALE);
  rightSpeed = int(float(speed)*RIGHT_SCALE);
  Serial.println(encoderDistance);
  while(!leftStopped || !rightStopped){ //while encoder position has not been reached for either motor

    SetMotorSpeeds(leftSpeed, rightSpeed, &carCommand); //power left and right motors
    carCommand.direction = FORWARD;
    MoveCar(&carCommand);

    if (oldPositionL >  encoderDistance) { //if left motor surpasses encoder position stop left motor
      SetMotorSpeeds (0,rightSpeed,&carCommand) ;
      carCommand.direction = FORWARD;
      MoveCar(&carCommand);
      leftStopped = true;
      leftSpeed = 0;
    }

    if (oldPositionR >  encoderDistance) { //if left motor surpasses encoder position stop right motor
      SetMotorSpeeds (leftSpeed,0,&carCommand);
      carCommand.direction = FORWARD;
      MoveCar(&carCommand);
      rightStopped = true;
      rightSpeed = 0;
    }

    long newPositionL = myEncL.read(); //read encoder
    long newPositionR = myEncR.read();

    if (newPositionL != oldPositionL) { //count encoder position
      oldPositionL = newPositionL;
    }

    if (newPositionR != oldPositionR) {
      oldPositionR = newPositionR;
    }

  }
  Stop(0,1,1); //Stop car
}


//****************************************************************************************/
// serialEvent
//****************************************************************************************/
void debugPrint()
{
  outputString = "LY: ";
  outputString.concat(controllerInputs.LY);
  outputString.concat(" RY: ");
  outputString.concat(controllerInputs.RY);
  outputString.concat(" isCommTimeoutDisabled: ");
  outputString.concat(controllerInputs.isCommTimeoutDisabled);

  outputString.concat(" DIR: ");
  outputString.concat(carCommand.direction);
  outputString.concat(" leftSpeed: ");
  outputString.concat(carCommand.leftMotorSpeed);
  outputString.concat(" rightSpeed: ");
  outputString.concat(carCommand.rightMotorSpeed);
  outputString.concat(" dist: ");
  outputString.concat(sensors.distance);
  outputString.concat(" servo: ");
  outputString.concat(servoAngle);
  Serial.println(outputString);
}

//****************************************************************************************/
//
//****************************************************************************************/
void SetBucketLoad()
{
  liftServo.write(liftServoLoadAngle);
  bucketServo.write(bucketServoLoadAngle);
}

//****************************************************************************************/
//
//****************************************************************************************/
void SetBucketLift()
{
  liftServo.write(liftServoLiftAngle);
  bucketServo.write(bucketServoLiftAngle);
}

//****************************************************************************************/
//
//****************************************************************************************/
void SetBucketDump()
{
  liftServo.write(liftServoDumpAngle);
  bucketServo.write(bucketServoDumpAngle);
}
