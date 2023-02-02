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

/* MOTOR ORIENTATION:
 * Yaw motors (1-4) are positive when rotated forward
 * Shoulder motors (5-8) are positive when rotated down
 * Knee motors (9-12) are positive when rotated inward
 * Body motor (13) is positive when rotated up
 * Tail motor (14) is positive when rotated up
 * Coordinates are [Right, Front, Up]
 * Legs are ordered front left = 1, front right = 2, rear left = 3, rear right = 4
 */

#define DXL_SERIAL Serial3 //OpenCM9.04 EXP Board's DXL port Serial. (Serial for the DXL port on the OpenCM 9.04 board)
const uint8_t DXL_DIR_PIN = 22; //OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
Dynamixel2Arduino Dxl(DXL_SERIAL, DXL_DIR_PIN);

int speed = 200;    // global motor movement speed
int state = 0;      // 0 = wait, 1 = grasp
float angles[4][3]; // current motor angles

// Convert a vector to a skew-symmetric matrix
BLA::Matrix<3,3> getSkew(float x, float y, float z) {
  BLA::Matrix<3,3> skew = {0, -z, y, z, 0, -x, -y, x, 0};
  return(skew);
}

// Compute the pseudo-inverse of a matrix
template <int rows, int cols, class MemT>
BLA::Matrix<cols, rows, MemT> pinv(BLA::Matrix<rows, cols, MemT> A) {
  if (rows > cols) return(BLA::Inverse((~A)*A)*(~A));
  else return((~A)*BLA::Inverse(A*(~A)));
//  else return(BLA::Inverse(A));
}

// Convert vector to diagonal matrix
template <int rows, class MemT>
BLA::Matrix<rows, rows, MemT> diag(BLA::Matrix<rows, 1, MemT> V) {
  BLA::Matrix<rows, rows, MemT> A = BLA::Matrix<rows,rows>();
  for(int i=0; i<rows; i++) {
    A(i,i) = V(i);
  }
  return(A);
}

// Multiply matrix by a scalar
template <int rows, int cols, class MemT>
BLA::Matrix<rows, cols, MemT> scale(BLA::Matrix<rows, cols, MemT> A, float s) {
  for(int i=0; i<rows; i++) {
    for(int j=0; j<cols; j++) {
      A(i,j) *= s;
    }
  }
  return(A);
}

// Multiply matrix by a vector elementwise
template <int rows, int cols, class MemT>
BLA::Matrix<rows, cols, MemT> hscale(BLA::Matrix<rows, cols, MemT> A, BLA::Matrix<cols, 1> S) {
  for(int i=0; i<rows; i++) {
    for(int j=0; j<cols; j++) {
      A(i,j) *= S(j);
    }
  }
  return(A);
}

// Multiply matrix by a vector elementwise
template <int rows, int cols, class MemT>
BLA::Matrix<rows, cols, MemT> vscale(BLA::Matrix<rows, 1> S, BLA::Matrix<rows, cols, MemT> A) {
  for(int i=0; i<rows; i++) {
    for(int j=0; j<cols; j++) {
      A(i,j) *= S(i);
    }
  }
  return(A);
}

// Compute the grasp matrix
BLA::Matrix<12,12> getG() {
  BLA::Matrix<12,12> G;
  G.Fill(0);
  BLA::Matrix<1,3> X;
  int row = 6;
  for(int i=0; i<4; i++) {
    G.Submatrix<3,3>(0,3*i) = Identity<3,3>();
    float *x = getFootPos(i+1);
    X = {x[0], x[1], x[2]};
    G.Submatrix<3,3>(3,3*i) = getSkew(x[0], x[1], x[2]);
    for(int j=i+1; j<4; j++) {
      G.Submatrix<1,3>(row, 3*i) += X;
      G.Submatrix<1,3>(row, 3*j) -= X;
      row++;
    }
  }
  return(G);
}

// Compute the grasp matrix
BLA::Matrix<6,12> getGpsuedo() {
  BLA::Matrix<6,12> G;
  for(int i=0; i<4; i++) {
    G.Submatrix<3,3>(0,3*i) = Identity<3,3>();
    float *x = getFootPos(i+1);
    G.Submatrix<3,3>(3,3*i) = getSkew(x[0], x[1], x[2]);
  }
  return(G);
}

// Determine current foot position vector
BLA::Matrix<12,1> getXf() {
  BLA::Matrix<12,1> Xf;
  for(int i=0; i<4; i++) {
    float *x = getFootPos(i+1);
    for(int j=0; j<3; j++) {
      Xf(i*3+j) = x[j];
    }
  }
  return(Xf);
}

// Maintain current position using pseudo-inverse impedance control strategy
void impedanceControlPseudoInverse(long duration) {
//  moveMotor(5, 20);
//  moveMotor(9, -20);
  long t0 = millis(), ti = millis(), dt = 0;
  BLA::Matrix<6,12> G;
  BLA::Matrix<6,1> Xb, Fb, wrenchIntegral;
  BLA::Matrix<6,1> k = {1,1,1,1,1,1};
  updateMotorPos();
  BLA::Matrix<12,1> Xf, Ff, Xf0 = getXf();
  BLA::Matrix<12,1> w = {1.0,1.0,1.0, 1.0,1.0,1.0, 1.0,1.0,1.0, 1.0,1.0,1.0};
  BLA::Matrix<12,6> temp;
  wrenchIntegral.Fill(0);
  float kP = 1, kI = 0;
  for(int t=0; t<10000; t++) { // 40ms loop (25 Hz)
    dt = millis() - ti;
    ti += dt;
//    if(ti-t0 > 5000) {
//      Xf0(5) = 60;
//      moveMotor(5, 60);
//      Xf0(9) = -60;
//      moveMotor(9, -60);
//    }
    if(duration > 0 && ti-t0 > duration) break;    
    updateMotorPos(); // 15ms
    G = getGpsuedo();
    Xf = getXf();
    Xb = ~pinv(G)*(Xf-Xf0); // 2 ms
    Fb = -vscale(k, Xb);
    wrenchIntegral += scale(Fb, dt)/1000.0;
    temp = pinv(hscale(G,w));
    Ff = vscale(w, temp)*(scale(Fb, kP) + scale(wrenchIntegral, kI));
    for(int i=0; i<4; i++) {
      limitFootForce(i+1, Ff(3*i), Ff(3*i+1), Ff(3*i+2)); // 16 ms (total)
    }
    Serial << int(ti-t0) << "  |  " << Fb << "  |   " << wrenchIntegral << "  |   " << Ff << "\n";
//    printForces(); // 17 ms
  }
}

// Maintain current position using impedance control strategy
void impedanceControl(long duration) {
//  moveMotor(5, 20);
//  moveMotor(9, -20);
  long t0 = millis(), ti = millis(), dt = 0;
  BLA::Matrix<12,12> G, Ginv;
  BLA::Matrix<12,1> Xb, Fb, wrenchIntegral;
  BLA::Matrix<12,1> k = {1,1,1,1,1,1,0,0,0,0,0,0};
  updateMotorPos();
  BLA::Matrix<12,1> Xf, Ff, Xf0 = getXf();
  wrenchIntegral.Fill(0);
  float kP = 10, kI = 0;
  for(int t=0; t<10000; t++) { // 40ms loop (25 Hz)
    dt = millis() - ti;
    ti += dt;
//    if(ti-t0 > 5000) { // reference trajectory for testing
//      Xf0(5) = 60;
//      moveMotor(5, 60);
//      Xf0(9) = -60;
//      moveMotor(9, -60);
//    }
    if(duration > 0 && ti-t0 > duration) break;    
    updateMotorPos(); // 15ms
    G = getG();
    Ginv = BLA::Inverse(G); // 5 ms
    Xf = getXf();
    Xb = ~Ginv*(Xf-Xf0);
    Fb = -vscale(k, Xb);
    wrenchIntegral += scale(Fb, dt)/1000.0;
    Ff = Ginv*(scale(Fb, kP) + scale(wrenchIntegral, kI));
    for(int i=0; i<4; i++) {
      limitFootForce(i+1, Ff(3*i), Ff(3*i+1), Ff(3*i+2)); // 16 ms (total)
    }
    Serial << int(ti-t0) << "  |  " << Fb << "  |   " << wrenchIntegral << "  |   " << Ff << "\n";
//    printForces(); // 17 ms
  }
}

// Maintain current position using decentralized impedance control strategy
void impedanceControlDecentralized(long duration) {
//  moveMotor(5, 20);
//  moveMotor(9, -20);
  long t0 = millis(), ti = millis(), dt = 0;
  BLA::Matrix<12,1> k = {1,1,1,1,1,1,1,1,1,1,1,1};
  updateMotorPos();
  BLA::Matrix<12,1> Xf, Ff, integral, Xf0 = getXf();
  integral.Fill(0);
  float kP = 0.1, kI = 0;
  for(int t=0; t<10000; t++) { // 40ms loop (25 Hz)
    dt = millis() - ti;
    ti += dt;
//    if(ti-t0 > 5000) { // reference trajectory for testing
//      Xf0(5) = 60;
//      moveMotor(5, 60);
//      Xf0(9) = -60;
//      moveMotor(9, -60);
//    }
    if(duration > 0 && ti-t0 > duration) break;    
    updateMotorPos(); // 15ms
    Xf = getXf();
    Xf = vscale(k, Xf-Xf0);
    integral += scale(Xf, dt)/1000.0;
    Ff = -(scale(Xf, kP) + scale(integral, kI));
    for(int i=0; i<4; i++) {
      limitFootForce(i+1, Ff(3*i), Ff(3*i+1), Ff(3*i+2)); // 16 ms (total)
    }
    Serial << int(ti-t0) << "  |  " << Xf << "  |   " << integral << "  |   " << Ff << "\n";
//    printForces(); // 17 ms
  }
}

void setup() {
  // BLA::Matrix<6,12> G = getG();
//  Serial.attachInterrupt(teleop);
//  Serial3.setDxlMode(true);
//  Dxl.begin(3);
  Dxl.begin(1000000);
  Dxl.setPortProtocolVersion(2.0);
  Serial.begin(9600);
  enableTorque();
//  stand();
  sprawl();
  delay(3000);
  speed = 200;
//  impedanceControlPseudoInverse(-1);
  impedanceControl(-1);
//  impedanceControlDecentralized(-1);
//  enableTorque();
//  stand();
//  balanceForces(0);
  
//  delay(1000);
//  wag(3, 500);
//  
//  delay(1000);
//  backbend(3, 500);
//  
//  delay(1000);
  // walk(0, 500);
//
//  delay(1000);
//  sillywalk(0, 500);
//  delay(1000);
//  grasp();
}

void loop() {
  delay(100);
  updateMotorPos();
  printForces();
//  Serial << "C: " << C << '\n';
//  for (int i=0; i<4; i++) {
//    Serial.println(getMotorPos(i+1));
//    Serial.println(getMotorPos(i+5));
//    Serial.println(getMotorPos(i+9));
//  }
}

// Parse commands from Serial
void teleop(byte* buffer, byte nCount) {
  char message [nCount];
  for (int i=0; i<20; i++) {
    message[i] = 0;
  }
  for (int i=0; i<nCount; i++) {
    message[i] = (char)buffer[i];
  }
  if (message[0] == 'g') {
    delay(1000);
    graspForce();
    return;
  }
  if (message[0] == 'b') {
    int n = atoi(message + 1);
    delay(1000);
    balanceForces(n);
    return;
  }
  if (message[0] == 's') {
    enableTorque();
    stand();
    return;
  }
  if (message[0] == 't') {
    int id = atoi(message + 1);
    float torque = atof(message + 1 + (id >= 10 ? 3:2));
    limitTorque(id, torque);
    Serial.print("Setting motor ");
    Serial.print(id);
    Serial.print(" to torque ");
    Serial.println(torque);
    return;
  }
  int id = atoi(message);
  float angle = atof(message + (id >= 10 ? 3:2));
  moveMotor(id, angle);
  Serial.print("Moving motor ");
  Serial.print(id);
  Serial.print(" to angle ");
  Serial.println(angle);
}

/* Utility functions */

// Determine if motor is XM rather than AX
bool isXM(int id) {
  return(id <= 4 || id == 13);
}

// Determine if motor direction is reversed
bool isMotorMirrored(int id) {
  // Note: XM motors (2 & 3) are instead mirrored in firmware
  return(id==6 || id==10 || id==7 || id==11  || id==14);
}

// Determine if leg orientation is reversed
bool isLegMirrored(int foot) {
  return(foot == 1 || foot == 3);
}

// Enable torque on all motors (and reset operating mode)
void enableTorque() {
  for(int i=0; i<N_MOTORS; i++) {
    if(isXM(i+1)) {
      Dxl.setPortProtocolVersion(2.0);
      Dxl.torqueOff(i+1);
//      Dxl.setOperatingMode(i+1,OP_POSITION);    
      Dxl.setOperatingMode(i+1,OP_CURRENT_BASED_POSITION);
      Dxl.torqueOn(i+1);
    } else {
      Dxl.setPortProtocolVersion(1.0);
      Dxl.writeControlTableItem(ControlTableItem::TORQUE_LIMIT,i+1,1023);      
    }
  }
}

// Set motor position (deg)
void moveMotor(int id, float angle) {
  if (id > 0 && id <= 4) {
    if (angle > YAW_LIMIT_MAX) angle = YAW_LIMIT_MAX;
    if (angle < YAW_LIMIT_MIN) angle = YAW_LIMIT_MIN;
  } else if (id <= 8) {
    if (angle > SHOULDER_LIMIT_MAX) angle = SHOULDER_LIMIT_MAX;
    if (angle < SHOULDER_LIMIT_MIN) angle = SHOULDER_LIMIT_MIN;
  } else if (id <= 12) {
    if (angle > KNEE_LIMIT_MAX) angle = KNEE_LIMIT_MAX;
    if (angle < KNEE_LIMIT_MIN) angle = KNEE_LIMIT_MIN;
  } else if (id == 13) {
    if (angle > BODY_LIMIT_MAX) angle = BODY_LIMIT_MAX;
    if (angle < BODY_LIMIT_MIN) angle = BODY_LIMIT_MIN;
  } else if (id == 14) {
    if (angle > TAIL_LIMIT_MAX) angle = TAIL_LIMIT_MAX;
    if (angle < TAIL_LIMIT_MIN) angle = TAIL_LIMIT_MIN;
  } else {
    return;
  }
  if (isMotorMirrored(id)) angle *= -1;
  if (isXM(id)) {
    Dxl.setPortProtocolVersion(2.0);
    Dxl.setGoalPosition(id, (angle+180)*4095.0/360.0);
    Dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, id, speed);
  } else {
    Dxl.setPortProtocolVersion(1.0);
    Dxl.setGoalPosition(id, (angle+150)*1023.0/300.0);
    Dxl.writeControlTableItem(ControlTableItem::MOVING_SPEED, id, speed);
  }
}

// Set motor max torque (Nmm)
void limitTorque(int id, float torque) {
  if (isXM(id)) { // XM
    Dxl.setPortProtocolVersion(2.0);
//    Dxl.torqueOff(id);
//    Dxl.setOperatingMode(id,OP_CURRENT_BASED_POSITION);
//    Dxl.torqueOn(id);
    Dxl.setGoalCurrent(id,torque*2.3/4.1/2.69);
  } else { // AX
    Dxl.setPortProtocolVersion(1.0);
    Dxl.writeControlTableItem(ControlTableItem::TORQUE_LIMIT,id,torque/1800*1023);
  }
}

void limitFootForce(int foot, float fx, float fy, float fz) {
  float *t = getJointTorque(fx, fy, fz, angles[foot-1][0], angles[foot-1][1], angles[foot-1][2], foot);
  limitTorque(foot, abs(t[0]));
  limitTorque(foot+4, abs(t[1]));
  limitTorque(foot+8, abs(t[2]));
}

// Get current motor position (deg)
float getMotorPos(int id) {
  float angle;
  if (isXM(id)) { // XM
    Dxl.setPortProtocolVersion(2.0);
    int pos = Dxl.readControlTableItem(ControlTableItem::PRESENT_POSITION, id);
    angle = pos/4095.0*360 - 180;
  } else { // AX
    Dxl.setPortProtocolVersion(1.0);
    int pos = Dxl.readControlTableItem(ControlTableItem::PRESENT_POSITION, id);
    angle = pos/1023.0*300 - 150;
  }
  if (isMotorMirrored(id)) angle *= -1;
  return angle;
}

// updates the global array of current motor angles
void updateMotorPos() {
  for (int i=0; i<4; i++) {
    angles[i][0] = getMotorPos(i+1);
    angles[i][1] = getMotorPos(i+5);
    angles[i][2] = getMotorPos(i+9);
  }
}

// Get current motor output torque (Nmm)
float getMotorTorque(int id) {
  if (isXM(id)) { // XM
    Dxl.setPortProtocolVersion(2.0);
    int current = Dxl.readControlTableItem(ControlTableItem::PRESENT_CURRENT, id);
    if (current >= 32768) current -= 32768*2;
    if (isMotorMirrored(id)) current *= -1;
    return(current*2.69*4.1/2.3);
  } else { // AX
    Dxl.setPortProtocolVersion(1.0);
    int load = Dxl.readControlTableItem(ControlTableItem::PRESENT_LOAD, id);
    if (load >= 1024) load = -(load - 1024);
    if (isMotorMirrored(id)) load *= -1;
    return(load/1023.0*1800);
  }
}

// Determine current end effector forces (N)
float* getFootForce(int foot) {
  float t1 = getMotorTorque(foot);
  float t2 = getMotorTorque(foot+4);
  float t3 = getMotorTorque(foot+8);
  return(getEndForce(-t1, -t2, -t3, angles[foot-1][0], angles[foot-1][1], angles[foot-1][2], foot));
}

// Determine foot position (mm) relative to centroid
float* getFootPos(int foot) {
  return(getEndPosCentroidal(angles[foot-1][0], angles[foot-1][1], angles[foot-1][2], foot));
}

// Determine joint angles position (deg) from end effector position (mm) relative to shoulder
float* getJointPos(float x, float y, float z, int foot) {
  if (isLegMirrored(foot)) x *= -1;
  static float a[3];
  a[0] = (1)*180/PI;
  a[1] = (1)*180/PI;
  a[2] = (1)*180/PI;
  // TODO
  return(a);
}

// Determine joint angles position (deg) from end effector position (mm) relative to centroid
float* getJointPosCentroidal(float x, float y, float z, int foot) {
  x -= 0;
  y -= 0;
  z -= 0;
  // TODO
  float *a = getJointPos(x, y, z, foot);
  return(a);
}

// Determine end effector position (mm) relative to shoulder from joint angles (deg)
float* getEndPos(float a1, float a2, float a3, int foot) {
  a1 = a1*PI/180;
  a2 = a2*PI/180;
  a3 = a3*PI/180;
  static float x[3];
  x[0] = cos(a1)*(LINK1 - LINK3*sin(a2 + a3) + LINK2A*cos(a2) - LINK2B*sin(a2));
  x[1] = sin(a1)*(LINK1 - LINK3*sin(a2 + a3) + LINK2A*cos(a2) - LINK2B*sin(a2));
  x[2] = - LINK3*cos(a2 + a3) - LINK2B*cos(a2) - LINK2A*sin(a2);
  if (isLegMirrored(foot)) x[0] *= -1;
  return(x);
}

// Determine end effector position (mm) relative to centroid from joint angles (deg)
float* getEndPosCentroidal(float a1, float a2, float a3, int foot) {
  float *x = getEndPos(a1, a2, a3, foot);
  x[0] += isLegMirrored(foot) ? -WIDTH/2 : WIDTH/2;
  x[1] += foot <= 2 ? LENGTH/2 : -LENGTH/2;;
  x[2] += HEIGHT/2;
  return(x);
}

// Determine joint velocities (deg/s) from end effector velocity (mm/s) and joint angles (deg)
float* getJointVel(float vx, float vy, float vz, float a1, float a2, float a3, int foot) {
  a1 = a1*PI/180;
  a2 = a2*PI/180;
  a3 = a3*PI/180;
  if (isLegMirrored(foot)) vx *= -1;
  static float w[3];
  w[0] = ((vy*cos(a1) - vx*sin(a1))/(LINK1 - LINK3*sin(a2 + a3) + LINK2A*cos(a2) - LINK2B*sin(a2)))*180/PI;
  w[1] = (-(vz*cos(a2)*cos(a3) - vz*sin(a2)*sin(a3) + vx*cos(a1)*cos(a2)*sin(a3) + vx*cos(a1)*cos(a3)*sin(a2) + vy*cos(a2)*sin(a1)*sin(a3) + vy*cos(a3)*sin(a1)*sin(a2))/(LINK2A*cos(a3) + LINK2B*sin(a3)))*180/PI;
  w[2] = ((LINK3*vz*cos(a2 + a3) + LINK2B*vz*cos(a2) + LINK2A*vz*sin(a2) + LINK3*vx*sin(a2 + a3)*cos(a1) + LINK3*vy*sin(a2 + a3)*sin(a1) - LINK2A*vx*cos(a1)*cos(a2) + LINK2B*vx*cos(a1)*sin(a2) - LINK2A*vy*cos(a2)*sin(a1) + LINK2B*vy*sin(a1)*sin(a2))/(LINK3*(LINK2A*cos(a3) + LINK2B*sin(a3))))*180/PI;
  return(w);
}

// Determine end effector velocity (mm/s) from joint velocities (deg/s) and joint angles (deg)
float* getEndVel(float w1, float w2, float w3, float a1, float a2, float a3, int foot) {
  a1 = a1*PI/180;
  a2 = a2*PI/180;
  a3 = a3*PI/180;
  w1 = w1*PI/180;
  w2 = w2*PI/180;
  w3 = w3*PI/180;
  static float v[3];
  v[0] = - w2*cos(a1)*(LINK3*cos(a2 + a3) + LINK2B*cos(a2) + LINK2A*sin(a2)) - w1*sin(a1)*(LINK1 - LINK3*sin(a2 + a3) + LINK2A*cos(a2) - LINK2B*sin(a2)) - LINK3*w3*cos(a2 + a3)*cos(a1);
  v[1] = w1*cos(a1)*(LINK1 - LINK3*sin(a2 + a3) + LINK2A*cos(a2) - LINK2B*sin(a2)) - w2*sin(a1)*(LINK3*cos(a2 + a3) + LINK2B*cos(a2) + LINK2A*sin(a2)) - LINK3*w3*cos(a2 + a3)*sin(a1);
  v[2] = w2*(LINK3*sin(a2 + a3) - LINK2A*cos(a2) + LINK2B*sin(a2)) + LINK3*w3*sin(a2 + a3);
  if (isLegMirrored(foot)) v[0] *= -1;
  return(v);
}

// Determine joint torques (N*mm) from end effector forces (N) and joint angles (deg)
float* getJointTorque(float fx, float fy, float fz, float a1, float a2, float a3, int foot) {
  a1 = a1*PI/180;
  a2 = a2*PI/180;
  a3 = a3*PI/180;
  if (isLegMirrored(foot)) fx *= -1;
  static float t[3];
  t[0] = (fy*cos(a1) - fx*sin(a1))*(LINK1 - LINK3*sin(a2 + a3) + LINK2A*cos(a2) - LINK2B*sin(a2));
  t[1] = fz*(LINK3*sin(a2 + a3) - LINK2A*cos(a2) + LINK2B*sin(a2)) - fx*cos(a1)*(LINK3*cos(a2 + a3) + LINK2B*cos(a2) + LINK2A*sin(a2)) - fy*sin(a1)*(LINK3*cos(a2 + a3) + LINK2B*cos(a2) + LINK2A*sin(a2));
  t[2] = LINK3*fz*sin(a2 + a3) - LINK3*fx*cos(a2 + a3)*cos(a1) - LINK3*fy*cos(a2 + a3)*sin(a1);
  return(t);
}

// Determine end effector forces (N) from joint torques (N*mm) and joint angles (deg)
float* getEndForce(float t1, float t2, float t3, float a1, float a2, float a3, int foot) {
  a1 = a1*PI/180;
  a2 = a2*PI/180;
  a3 = a3*PI/180;
  static float f[3];
  f[0] = (t3*cos(a1)*(LINK3*sin(a2 + a3) - LINK2A*cos(a2) + LINK2B*sin(a2)))/(LINK3*(LINK2A*cos(a3) + LINK2B*sin(a3))) - (t2*sin(a2 + a3)*cos(a1))/(LINK2A*cos(a3) + LINK2B*sin(a3)) - (t1*sin(a1))/(LINK1 - LINK3*sin(a2 + a3) + LINK2A*cos(a2) - LINK2B*sin(a2));
  f[1] = (t1*cos(a1))/(LINK1 - LINK3*sin(a2 + a3) + LINK2A*cos(a2) - LINK2B*sin(a2)) - (t2*sin(a2 + a3)*sin(a1))/(LINK2A*cos(a3) + LINK2B*sin(a3)) + (t3*sin(a1)*(LINK3*sin(a2 + a3) - LINK2A*cos(a2) + LINK2B*sin(a2)))/(LINK3*(LINK2A*cos(a3) + LINK2B*sin(a3)));
  f[2] = (LINK3*t3*cos(a2 + a3) - LINK3*t2*cos(a2 + a3) + LINK2B*t3*cos(a2) + LINK2A*t3*sin(a2))/(LINK3*(LINK2A*cos(a3) + LINK2B*sin(a3)));
  if (isLegMirrored(foot)) f[0] *= -1;
  return(f);
}

/* Demo behaviors */

void walk(int steps, int period) {
  float yaw[] = {60, 60, 0, 0, -60, -60, -60, 60};
  float shoulder[] = {30, 30, 30, 30, 30, 30, 0, 0};
  float knee[] = {-30, -30, 0, 0, -30, -30, 0, 0};
  float body[] = {0, 0, 0, 0, 0, 0, 0, 0};
  float tail[] = {-20, -20, -20, -20, -20, -20, -20, -20};
  float order[] = {3, 1, 4, 2};
  int count = 0;
  while (true) {
    updateMotorPos();
    for (int i = 0; i < 4; i++) {
      moveMotor(order[i], yaw[(count+8-i*2)%8]);
      moveMotor(order[i]+4, shoulder[(count+8-i*2)%8]);
      moveMotor(order[i]+8, knee[(count+8-i*2)%8]);
    }
    moveMotor(13, body[count%8]);
    moveMotor(14, tail[count%8]);
    count++;
    if (count >= steps && steps > 0) return;
    delay(period);
  }
}

void stand() {
  for (int i = 0; i < 4; i++) {
      moveMotor(i+1, 0);
      moveMotor(i+5, 30);
      moveMotor(i+9, -30);
  }
  moveMotor(13, 0);
  moveMotor(14, -20);
}

void sprawl() {
  for (int i = 0; i < 4; i++) {
      moveMotor(i+1, 0);
      moveMotor(i+5, -50);
      moveMotor(i+9, 40);
  }
  moveMotor(13, 0);
  moveMotor(14, 0);
//    moveMotor(2, 27);
//    moveMotor(6, -22);
//    moveMotor(10, 7);
}

void wag(int wags, int period) {
  for (int i=0; i<wags; i++) {
    moveMotor(14, 20);
    delay(period);
    moveMotor(14, -20);
    delay(period);
  }
}

void backbend(int bends, int period) {
  for (int i=0; i<bends; i++) {
    moveMotor(13, 30);
    delay(period);
    moveMotor(13, -30);
    delay(period);
  }
}

void sillywalk(int steps, int period) {
  float yaw[] = {0, 45, 0, -45};
  float shoulder[] = {30, 30, 0, 30};
  float knee[] = {0, 0, -30, 0};
  float body[] = {0, 60, 60, 0};
  float tail[] = {-20, -20, 20, 30};
  float order[] = {3, 1, 4, 2};
  int count = 0;
  while (true) {
    for (int i = 0; i < 4; i++) {
      moveMotor(order[i], yaw[(count+4-i)%4]);
      moveMotor(order[i]+4, shoulder[(count+4-i)%4]);
      moveMotor(order[i]+8, knee[(count+4-i)%4]);
    }
    moveMotor(13, body[count%4]);
    moveMotor(14, tail[count%4]);
    count++;
    if (count >= steps && steps > 0) return;
    delay(period);
  }
}

void grasp() {
//  -27.35, -22.35, 6.66

  float v = 0.1;
  float a1 = 27;
  float a2 = -22;
  float a3 = 7;
  float *da;
  Serial.println("Beginning Grasp");
  for(int i = 0; i < 600; i++) {
    da = getJointVel(0, -1, 0, a1, a2, a3, 2);
    a1 += da[0]*v;
    a2 += da[1]*v;
    a3 += da[2]*v;
    moveMotor(2, round(a1));
    moveMotor(6, round(a2));
    moveMotor(10, round(a3));
//  Serial.print(a1);
//  Serial.print(", ");
//  Serial.print(a2);
//  Serial.print(", ");
//  Serial.println(a3);
  }
}

void graspForce() {
  float v = 0.1;
  float a1 = 27;
  float a2 = -22;
  float a3 = 7;
  float *da;
  float *t;
  Serial.println("Lowering Gripper");
  for(int i = 0; i < 300; i++) {
    da = getJointVel(0, 0, -1, a1, a2, a3, 2);
    t = getJointTorque(0, 0, -5, a1, a2, a3, 2);
    limitTorque(2, abs(t[0]));
    limitTorque(6, abs(t[1]));
    limitTorque(10, abs(t[2]));
    a1 += da[0]*v;
    a2 += da[1]*v;
    a3 += da[2]*v;
    moveMotor(2, a1);
    moveMotor(6, a2);
    moveMotor(10, a3);
//  Serial.print(a1);
//  Serial.print(", ");
//  Serial.print(a2);
//  Serial.print(", ");
//  Serial.println(a3);
  }
  Serial.println("Engaging Spines");
  for(int i = 0; i < 600; i++) {
    da = getJointVel(0, -1, 0, a1, a2, a3, 2);
    t = getJointTorque(10, -20, -5, a1, a2, a3, 2);
    limitTorque(2, abs(t[0]));
    limitTorque(6, abs(t[1]));
    limitTorque(10, abs(t[2]));
    a1 += da[0]*v;
    a2 += da[1]*v;
    a3 += da[2]*v;
    moveMotor(2, a1);
    moveMotor(6, a2);
    moveMotor(10, a3);
//  Serial.print(a1);
//  Serial.print(", ");
//  Serial.print(a2);
//  Serial.print(", ");
//  Serial.println(a3);
  }
}

// Display measured force components for each foot
void printForces() {
  Serial.print(millis());
  Serial.print(",     ");
  for (int i=0; i<4; i++) {
    float *f = getFootForce(i+1);
    Serial.print(f[0]);
    Serial.print(", ");
    Serial.print(f[1]);
    Serial.print(", ");
    Serial.print(f[2]);
    Serial.print(",     ");
  }
  Serial.println("");
}

// Distribute external forces among the feet
void balanceForces(int n) {
  float k = 0.04;
  float kx = 0;
  float ky = 0;
  float kz = k;
  float weights[] = {1,1,1,1};
  if (n > 0) {
    weights[n-1] = 0;
  }
  float W = weights[0]+weights[1]+weights[2]+weights[3];
  for (int i = 0; i < 4; i++) {
    weights[i] /= W;
  }
  Serial.println(weights[0]);
  Serial.println(weights[1]);
  Serial.println(weights[2]);
  Serial.println(weights[3]);
  Serial.println("--------------------------------");
  float a[4][3];
  for (int i=0; i<4; i++) {
    a[i][0] = getMotorPos(i+1);
    a[i][1] = getMotorPos(i+5);
    a[i][2] = getMotorPos(i+9);
  }
  float offsets[][3] = {{0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}};
  float *f;
  float F[4][3];
  float *w;
  float Fx, Fy, Fz;
  float vx, vy, vz;
  for(int t=0; t<1000; t++) {
    Fx = 0;
    Fy = 0;
    Fz = 0;
    for(int i=0; i<4; i++) {
      f = getFootForce(i+1);
      F[i][0] = f[0];
      F[i][1] = f[1];
      F[i][2] = f[2];
      Fx += f[0] - offsets[i][0];
      Fy += f[1] - offsets[i][1];
      Fz += f[2] - offsets[i][2];
    }
    for(int i=0; i<4; i++) {
      vx = kx*(Fx*weights[i] - F[i][0] + offsets[i][0]);
      vy = ky*(Fy*weights[i] - F[i][1] + offsets[i][1]);
      vz = kz*(Fz*weights[i] - F[i][2] + offsets[i][2]);
      w = getJointVel(vx, vy, vz, a[i][0], a[i][1], a[i][2], i+1);
      a[i][0] += w[0];
      a[i][1] += w[1];
      a[i][2] += w[2];
      moveMotor(i+1, a[i][0]);
      moveMotor(i+5, a[i][1]);
      moveMotor(i+9, a[i][2]);
    }
//    delay(500);
//    printForces();
  }
}
