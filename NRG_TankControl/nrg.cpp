#include "nrg.h"


static void UpdateMotorSpeed(CarCommand_T* carCommand);
static void Stop(CarCommand_T* carCommand);
static void Forward(CarCommand_T* carCommand);
static void Back(CarCommand_T* carCommand);
static void Left(CarCommand_T* carCommand);
static void Right(CarCommand_T* carCommand);
static void RightMotorForward();
static void RightMotorBack();
static void RightMotorStop();
static void LeftMotorForward();
static void LeftMotorBack();
static void LeftMotorStop();
static void VerifyMotorSpeeds(CarCommand_T* carCommand);

//****************************************************************************************/
//
//****************************************************************************************/
void InitNrgPins()
{
  //servo
  pinMode(L928N_N1,   OUTPUT); // send motor direction to motor controller //
  pinMode(L928N_N2,   OUTPUT); // send motor direction to motor controller //
  pinMode(L928N_N3,   OUTPUT); // send motor direction to motor controller //
  pinMode(L928N_N4,   OUTPUT); // send motor direction to motor controller //
  pinMode(L928N_ENA,  OUTPUT); // send left motor speed to motor controller //
  pinMode(L928N_ENB,  OUTPUT); // send right motor speed to motor controller //  
  
  pinMode(Echo,       INPUT);
  pinMode(Trig,       OUTPUT);

  pinMode(LineTrack_L, INPUT);
  pinMode(LineTrack_M, INPUT);
  pinMode(LineTrack_R, INPUT);

  pinMode(LED_L,       OUTPUT);
  pinMode(LED_R,       OUTPUT);
  pinMode(LimSwitch,    INPUT);

  pinMode(RGBLED_R,     OUTPUT);
  pinMode(RGBLED_G,     OUTPUT);
  pinMode(RGBLED_B,     OUTPUT);
}

//****************************************************************************************/
//
//****************************************************************************************/
void ParseInputString(ControllerInputs_T* controllerInputs, String inputString)
{  
  Serial.println(inputString);
  if (inputString.length() > 3)
  {
    int i = inputString.indexOf(' ');
    controllerInputs->isCommTimeoutDisabled = inputString.substring(0, i).toInt(); 
    inputString.remove(0, i + 1);

    i = inputString.indexOf(' ');
    controllerInputs->LY = inputString.substring(0, i).toInt();
    inputString.remove(0, i + 1);

    i = inputString.indexOf(' ');
    inputString.remove(0, i + 1);

    i = inputString.indexOf(' ');
    inputString.remove(0, i + 1);

    i = inputString.indexOf(' ');
    inputString.remove(0, i + 1);

    i = inputString.indexOf(' ');
    controllerInputs->RY = inputString.substring(0, i).toInt();
    inputString.remove(0, i + 1);

    i = inputString.indexOf(';');
    controllerInputs->buttons = inputString.substring(0, i).toInt();
    inputString.remove(0, inputString.length());

    controllerInputs->lastCommTime = micros();
  }  
  else
  {
    inputString.remove(0, inputString.length());

    controllerInputs->RY = 0;
    controllerInputs->LY = 0;
    //zeroVariables();
    
  }
  
} 

//****************************************************************************************/
//
//****************************************************************************************/
void SetDirection(ControllerInputs_T* controllerInputs, CarCommand_T* carCommand)
{
  carCommand->direction = STOP;

  if (abs(controllerInputs->LY) > deadSpace && abs(controllerInputs->RY) > deadSpace)  // make sure that there is enough to move motors //
  {
    if (controllerInputs->LY > 0 && controllerInputs->RY > 0)
    {
      carCommand->direction = FORWARD;
    }
    if (controllerInputs->LY < 0 && controllerInputs->RY < 0)
    {
      carCommand->direction = BACK;
    }
    if (controllerInputs->LY > 0 && controllerInputs->RY < 0)
    {
      carCommand->direction = RIGHT;
    }
    if (controllerInputs->LY < 0 && controllerInputs->RY > 0)
    {
      carCommand->direction = LEFT;
    }
  } else
  {
    if (abs(controllerInputs->LY) > deadSpace)  // is there enough from the left stick to move motors //
    {
      if (controllerInputs->LY > 0)
      {
        carCommand->direction = RIGHT;
      }
      else
      {
        carCommand->direction = LEFT;
      }
    }
    if (abs(controllerInputs->RY) > deadSpace)  // is there enough from the right stick to move motors //
    {
      if (controllerInputs->RY > 0)
      {
        carCommand->direction = LEFT;
      }
      else
      {
        carCommand->direction = RIGHT;
      }
    }
  }
}


//****************************************************************************************/
//
//****************************************************************************************/
void MoveCar(CarCommand_T* carCommand)
{
  switch(carCommand->direction)
  {
    case STOP:
      Stop(carCommand);
      break;
      
    case FORWARD:
      Forward(carCommand);
      break;
      
    case LEFT:
      Left(carCommand);
      break;
      
    case RIGHT:
      Right(carCommand);
      break;
      
    case BACK:
      Back(carCommand);
      break;
  };
 
}


//****************************************************************************************/
//
//****************************************************************************************/
static void UpdateMotorSpeed(CarCommand_T* carCommand)
{
  analogWrite(L928N_ENB, carCommand->leftMotorSpeed);  
  analogWrite(L928N_ENA, carCommand->rightMotorSpeed);
}

//****************************************************************************************/
static void Stop(CarCommand_T* carCommand)
{
  UpdateMotorSpeed(carCommand);
  RightMotorStop();
  LeftMotorStop();
}

//****************************************************************************************/
static void Forward(CarCommand_T* carCommand)
{
  UpdateMotorSpeed(carCommand);
  RightMotorForward();
  LeftMotorForward();
}

//****************************************************************************************/
static void Back(CarCommand_T* carCommand)
{
  UpdateMotorSpeed(carCommand);
  RightMotorBack();
  LeftMotorBack();
}

//****************************************************************************************/
static void Left(CarCommand_T* carCommand)
{
  UpdateMotorSpeed(carCommand);
  LeftMotorBack();
  RightMotorForward();
}

//****************************************************************************************/
static void Right(CarCommand_T* carCommand) //car will turn right until Stopped
{  
  UpdateMotorSpeed(carCommand);
  LeftMotorForward();
  RightMotorBack();
}

//****************************************************************************************/
static void RightMotorForward()
{  
  digitalWrite(L928N_N1, HIGH); 
  digitalWrite(L928N_N2, LOW);
}

//****************************************************************************************/
static void RightMotorBack()
{   
  digitalWrite(L928N_N1, LOW);
  digitalWrite(L928N_N2, HIGH);
}

//****************************************************************************************/
static void RightMotorStop()
{ 
  digitalWrite(L928N_N1, LOW); 
  digitalWrite(L928N_N2, LOW); 
}

//****************************************************************************************/
static void LeftMotorForward()
{ 
  digitalWrite(L928N_N3, LOW);
  digitalWrite(L928N_N4, HIGH);
}

//****************************************************************************************/
static void LeftMotorBack()
{ 
  digitalWrite(L928N_N3, HIGH);
  digitalWrite(L928N_N4, LOW);
}

//****************************************************************************************/
static void LeftMotorStop()
{ 
  digitalWrite(L928N_N3, LOW);
  digitalWrite(L928N_N4, LOW);
}



//****************************************************************************************/
//
//****************************************************************************************/
void SetMotorSpeeds(int leftSpeed, int rightSpeed, CarCommand_T* carCommand)
{  
  float tempSpeed = 0.0; // used to set motor speeds
  tempSpeed       = (float)abs(leftSpeed) * StickRatio;   // this way a max stick value of 100 results in 255 //
  carCommand->leftMotorSpeed  = (int)tempSpeed;                                 // motor speed in as integer but using a float in calculation so need to convert back to an integer //
  tempSpeed       = (float)abs(rightSpeed) * StickRatio;   // this way a max stick value of 100 results in 255 //
  carCommand->rightMotorSpeed = (int)tempSpeed;                                 // motor speed in as integer but using a float in calculation so need to convert back to an integer //

  // make sure that motor speed is not under min or over max speeds //
  VerifyMotorSpeeds(carCommand);
}

//****************************************************************************************/
//
//****************************************************************************************/
static void VerifyMotorSpeeds(CarCommand_T* carCommand)
{
  // make sure that leftMotorSpeed is not under min or over max to protect the motors //
  if (carCommand->leftMotorSpeed > 0 && carCommand->leftMotorSpeed < minSpeed)
  {
    carCommand->leftMotorSpeed = minSpeed;
  }
  if (carCommand->leftMotorSpeed > maxSpeed)
  {
    carCommand->leftMotorSpeed = maxSpeed;
  }
  // make sure that rightMotorSpeed is not under min or over max to protect the motors //
  if (carCommand->rightMotorSpeed > 0 && carCommand->rightMotorSpeed < minSpeed)
  {
    carCommand->rightMotorSpeed = minSpeed;
  }
  if (carCommand->rightMotorSpeed > maxSpeed)
  {
    carCommand->rightMotorSpeed = maxSpeed;
  }
}
