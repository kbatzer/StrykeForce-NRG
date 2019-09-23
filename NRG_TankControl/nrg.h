#include "String.h"
#include "Arduino.h"

// Line Tracker Variable Definition
#define LT_R !digitalRead(2)
#define LT_M !digitalRead(4)
#define LT_L !digitalRead(11)

// Controller Button Definitions
#define BUTTON_UP_MASK      1
#define BUTTON_DOWN_MASK    2
#define BUTTON_LEFT_MASK    4
#define BUTTON_RIGHT_MASK   8
#define BUTTON_START_MASK   16
#define BUTTON_SELECT_MASK  32
#define BUTTON_L3_MASK      64
#define BUTTON_R3_MASK      128
#define BUTTON_L1_MASK      256
#define BUTTON_R1_MASK      512
#define BUTTON_A_MASK       4096
#define BUTTON_B_MASK       8192
#define BUTTON_X_MASK       16384
#define BUTTON_Y_MASK       32768

#define isButtonPressed(reg, mask)  ((reg & mask) == mask)


//************** Confguration **************************************/
const int minSpeed    = 75;   // the smallest value we want to set as a motor speed
const int maxSpeed    = 255;  // the largest value we want to set as a motor speed 
const int deadSpace   = 5;    // used to make the stick a little less sensative for moving //
const int StickRatio  = 2;    // multiplier for stick input


//************** Pin Mapping ***************************************/
/* const int L928N_ENA = 5;  // left
const int L928N_ENB = 6;  // right
const int L928N_N1  = 7; // right
const int L928N_N2  = 8;  
const int L928N_N3  = 9;  // left
const int L928N_N4  = 11;  */

const int L928N_ENA = 6;  // right speed
const int L928N_ENB = 5;  // left speed
const int L928N_N1  = 11; // right direction 
const int L928N_N2  = 9;  // right direction
const int L928N_N3  = 8;  // left direction
const int L928N_N4  = 7;  // left direction

const int LineTrack_L  = 2; //left
const int LineTrack_M  = 4; //left
const int LineTrack_R  = 10; //left

const int LED_L        = 12;  //
const int LED_R        = A0; 
const int LimSwitch    = 13;  //

const int Echo      = A4; // ultrasonic feedback
const int Trig      = A5; // ultrasonic trigger

const int RGBLED_R  = A1; //Red
const int RGBLED_G  = A2; //Green
const int RGBLED_B  = A3; //Red


//************** Structures ***************************************/
typedef struct ControllerInputs_S
{
  int LY;       // Left Stick Foward/Back
  int RY;       // Right Stick Foward/Back
  int buttons;  //   
  int isCommTimeoutDisabled;
  long lastCommTime;
} ControllerInputs_T;

typedef struct SensorFeedback_S
{
  int distance;
  int SwitchClosed; // 1 true, 0 false
  int LineTracker_L; // 1 true, 0 false
  int LineTracker_M; // 1 true, 0 false
  int LineTracker_R; // 1 true, 0 false
 
 
} SensorFeedback_T;

typedef enum DriveState_E
{
  CONTROLLER,
  AUTO,
  AUTO_BACK,
  AUTO_RIGHT,
  AUTO_LEFT,
  TRACK_LINE,
  TRACK_LINE_ULTRASONIC,
  FORWARD_ULTRASONIC,
  FWD_TILL_LINE,
  LINE_TRACK_RESYNC,
  FIND_CAR_ANGLE,
  LIM_SW,
  LEFT_90,
  RIGHT_90,
  RIGHT_45,
  FWD_6IN,
  BACK_6IN,
  SEARCH,
  DELAY_3S,
  ENCODER_DRIVE
} DriveState_E;


typedef enum CarDirection_E
{
  STOP,
  FORWARD,
  LEFT,
  RIGHT,
  BACK
} CarDirection_E;


typedef struct CarCommand_S
{
  CarDirection_E  direction;                
  int             leftMotorSpeed;           
  int             rightMotorSpeed;  
  float           leftScale;
  float           rightScale;
} CarCommand_T;


//************** Prototypes ***************************************/
void InitNrgPins();
void ParseInputString(ControllerInputs_T* controllerInputs, String inputString);
void SetDirection(ControllerInputs_T* controllerInputs, CarCommand_T* carCommand);
void MoveCar(CarCommand_T* carCommand);
void SetMotorSpeeds(int leftSpeed, int rightSpeed, CarCommand_T* carCommand);
