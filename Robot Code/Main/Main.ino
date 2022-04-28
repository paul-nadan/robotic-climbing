#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>
using namespace BLA;

#include <actuator.h>
#include <Dynamixel2Arduino.h>

#define DXL_BUS_SERIAL 3
#define GOAL_POSITION_XM 116
#define TORQUE_ENABLE_XM 64
#define TORQUE_LIMIT_AX 34
#define GOAL_CURRENT_XM 102
#define OPERATING_MODE_XM 11
#define PRESENT_LOAD_AX 40
#define PRESENT_CURRENT_XM 126
#define PRESENT_POSITION_AX 36
#define PRESENT_POSITION_XM 132

#define N_MOTORS 14

#define WIDTH 154 // mm
#define LENGTH 283.4
#define HEIGHT 54
#define LINK1 23.5
#define LINK2A 71
#define LINK2B 22.5
#define LINK3 98.7 //53.7 + 45
#define TAIL1 330.0
//#define LINK3 53.7 //53.7 + 45

#define YAW_LIMIT_MIN -70 // deg
#define YAW_LIMIT_MAX 70
#define SHOULDER_LIMIT_MIN -60
#define SHOULDER_LIMIT_MAX 90
#define KNEE_LIMIT_MIN -90
#define KNEE_LIMIT_MAX 60
#define BODY_LIMIT_MIN -60
#define BODY_LIMIT_MAX 60
#define TAIL_LIMIT_MIN -80
#define TAIL_LIMIT_MAX 80

#define POS_TOL 1 // mm

/* MOTOR ORIENTATION:
   Yaw motors (1-4) are positive when rotated forward
   Shoulder motors (5-8) are positive when rotated down
   Knee motors (9-12) are positive when rotated inward
   Body motor (13) is positive when rotated up
   Tail motor (14) is positive when rotated up
   Coordinates are [Right, Front, Up]
   Legs are ordered front left = 1, front right = 2, rear left = 3, rear right = 4
*/

#define DXL_SERIAL Serial3                        // OpenCM9.04 EXP Board's DXL port Serial. (Serial for the DXL port on the OpenCM 9.04 board)
const uint8_t DXL_DIR_PIN = 22;                   // OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
Dynamixel2Arduino Dxl(DXL_SERIAL, DXL_DIR_PIN);

enum behavior {IDLE, WALK, CLIMB, SILLYWALK};                         // possible behaviors
enum controller {OFF, DECENTRALIZED, FULLRANK, LEASTSQUARES, TAIL};   // possible control strategies
controller DEFAULT_CONTROL = TAIL;                                    // default control strategy
enum display {NONE, POSITION, ANGLE, FORCE, TORQUE};                  // possible display variables

behavior state = IDLE;        // current behavior being executed
controller control = OFF;     // current control strategy for holding position
display output = NONE;        // current variable to print to Serial

long t0 = 0;                  // time that current behavior started
long tb = 0;                  // time of last behavior update (change in substep)
long t = 0;                   // time of last loop iteration
int dt = 0;                   // time since last loop iteration
int substep = 0;              // progress of current behavior (reset for each behavior)
int step = 0;                 // progress of overall gait (reset only when gait is restarted)

int speed = 200;              // motor movement speed
int period = 500;             // gait step duration
int goalSteps = 0;            // desired number of steps to take (0 indicates no limit)

float angles[5][3];           // current motor angles (legs x4, followed by [body, tail, -])
float setpoints[5][3];        // desired motor angles
float torques[5][3];          // desired motor torque limits
BLA::Matrix<12, 1> integral;  // accumulated error integral from controller

void setup() {
  Dxl.begin(1000000);
  Dxl.setPortProtocolVersion(2.0);
  Serial.begin(9600);
  delay(1000);

  sprawl();
  setBehavior(IDLE);
  setController(OFF);
  setDisplay(NONE);
}

void loop() {
  updateMotorPos(); // 15ms
  dt = millis() - t;
  t = millis();
  checkSerial();
  switch (state) {
    case WALK: walk(); break;
    case CLIMB: climb(); break;
    case SILLYWALK: sillywalk(); break;
  }
  switch (control) {
    case DECENTRALIZED: impedanceControlDecentralized(); break;
    case FULLRANK: impedanceControl(); break;
    case LEASTSQUARES: impedanceControlPseudoInverse(); break;
    case TAIL: impedanceControlTail(); break;
  }
  switch (output) {
    case POSITION: printPositions(); break;
    case ANGLE: printAngles(); break;
    case FORCE: printForces(); break;
    case TORQUE: printTorques(); break;
  }
  if (state == IDLE && control == OFF) {
//    delay(100);
  }
  moveMotors();
}
