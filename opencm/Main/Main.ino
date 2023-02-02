/* MOTOR ORIENTATION:
   Yaw motors (1-4) are positive when rotated forward
   Shoulder motors (5-8) are positive when rotated down
   Knee motors (9-12) are positive when rotated inward
   Body motor (13) is positive when rotated up
   Tail motor (14) is positive when rotated up
   Coordinates are [Right, Front, Up]
   Legs are ordered front left = 1, front right = 2, rear left = 3, rear right = 4
*/

#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>
using namespace BLA;

#include <actuator.h>
#include <Dynamixel2Arduino.h>

#define N_MOTORS 14

#define WIDTH 154 // mm
#define LENGTH 283.4
#define HEIGHT 54
#define LINK1 23.5
#define LINK2A 111
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

#define DXL_BUS_SERIAL 3
#define DXL_SERIAL Serial3                        // OpenCM9.04 EXP Board's DXL port Serial. (Serial for the DXL port on the OpenCM 9.04 board)
const uint8_t DXL_DIR_PIN = 22;                   // OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)

Dynamixel2Arduino Dxl(DXL_SERIAL, DXL_DIR_PIN);

enum behavior {IDLE, WALK, CLIMB, SILLYWALK, RECENTER};               // possible behaviors
enum controller {OFF, DECENTRALIZED, FULLRANK, LEASTSQUARES, TAIL};   // possible control strategies
controller DEFAULT_CONTROL = TAIL;                                    // default control strategy
enum display {NONE, POSITION, ANGLE, FORCE, TORQUE, SPEED, FOOTVEL};  // possible display variables

behavior state = IDLE;        // current behavior being executed
controller control = OFF;     // current control strategy for holding position
display output = NONE;        // current variable to print to Serial

long t0 = 0;                  // time that current behavior started
long tb = 0;                  // time of last behavior update (change in substep)
long t = 0;                   // time of last loop iteration
int dt = 0;                   // time since last loop iteration
int substep = 0;              // progress of current behavior (reset for each behavior)
int step = 0;                 // progress of overall gait (reset only when gait is restarted)

float motor_speed = 40;       // motor default movement speed (deg/s)
float motor_torque = 1000;    // motor default torque (Nmm)
int gait_period = 500;        // gait step duration (ms)
int goalSteps = 0;            // desired number of steps to take (0 indicates no limit)
float kP = 50;//3;            // controller proportional gain
float kI = 0;                 // controller integral gain
float kD = 1;                 // controller derivative gain
float kF = 0;                 // force controller gain
float overshoot = 0.1;//.05;  // amount to overshoot angle setpoints during torque control (deg)
float max_overshoot = 1;     // maximum amount to overshoot angle setpoints during torque control (deg)
bool retry = false;           // true if this is the second attempt at a step

float setpoints[5][3];        // controller angle setpoints
float angles[5][3];           // current motor angles
float goal_angles[5][3];      // desired motor angles
float torques[5][3];          // current motor torques (signed)
float goal_torques[5][3];     // motor torque limits (unsigned)
float speeds[5][3];           // current motor velocities (signed)
float goal_speeds[5][3];      // motor speed limits (unsigned)
float foot_weights[5];        // relative weighting of each foot and tail for force distribution (0 to 1)

void setup() {
  Dxl.begin(1000000);
  Dxl.setPortProtocolVersion(2.0);
  Serial.begin(1000000);

  setBehavior(IDLE);
  setController(OFF);
  setDisplay(NONE);

  enable();
  delay(1000);
  enable();
  stick();
  sprawl();
}

int loops = 0;
int time = 0;
void loop() {
  loops += 1;
  if (millis() - time > 1000) {
    Serial << "Status: " << readVoltage() << " V | " << int(loops*1000.0/(millis()-time)) << " Hz | " << int((millis()-time)*1.0/loops) << " ms\n";
    time = millis();
    loops = 0;
  }
  readAngles();
  // readSpeeds();
  // readTorques();
  dt = millis() - t;
  t = millis();
  checkSerial();
  autoclimb();
  switch (state) {
    case WALK: walk(); break;
    case CLIMB: climb(); break;
    case SILLYWALK: sillywalk(); break;
    case RECENTER: recenter(step); break;
  }
  switch (control) {
    case DECENTRALIZED: impedanceControlDecentralized(); break;
    case FULLRANK: impedanceControl(); break;
    case LEASTSQUARES: impedanceControlPseudoInverse(); break;
    case TAIL: impedanceControlTail(); break;
    case OFF: noControl(); break;
  }
  switch (output) {
    case POSITION: printPositions(); break;
    case ANGLE: printAngles(); break;
    case FORCE: printForces(); break;
    case TORQUE: printTorques(); break;
    case SPEED: printSpeeds(); break;
    case FOOTVEL: printFootVels(); break;
  }
  // writeSpeeds();
  writeTorques();
  writeAngles();
}
