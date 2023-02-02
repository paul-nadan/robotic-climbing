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
 * Legs are ordered clockwise from front left
 */

int speed = 50;
int state = 0; // 0 = wait, 1 = grasp

Dynamixel Dxl(DXL_BUS_SERIAL);

void setup() {
  SerialUSB.attachInterrupt(teleop);
  Dxl.begin(3);
  enableTorque();
  stand();
  delay(3000);
  speed = 100;
  balanceForces(0);
  
//  delay(1000);
//  wag(3, 500);
//  
//  delay(1000);
//  backbend(3, 500);
//  
//  delay(1000);
//  walk(0, 500);
//
//  delay(1000);
//  sillywalk(0, 500);
//  delay(1000);
//  grasp();
}

void loop() {
  delay(100);
  printForces();
//  for (int i=0; i<4; i++) {
//    SerialUSB.println(getMotorPos(i+1));
//    SerialUSB.println(getMotorPos(i+5));
//    SerialUSB.println(getMotorPos(i+9));
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
    SerialUSB.print("Setting motor ");
    SerialUSB.print(id);
    SerialUSB.print(" to torque ");
    SerialUSB.println(torque);
    return;
  }
  int id = atoi(message);
  float angle = atof(message + (id >= 10 ? 3:2));
  moveMotor(id, angle);
  SerialUSB.print("Moving motor ");
  SerialUSB.print(id);
  SerialUSB.print(" to angle ");
  SerialUSB.println(angle);
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
  return(foot == 1 || foot == 4);
}

// Enable torque on all motors (and reset operating mode)
void enableTorque() {
  for(int i=0; i<N_MOTORS; i++) {
    if(isXM(i+1)) {
      Dxl.writeByte(i+1,TORQUE_ENABLE_XM,0);
      Dxl.writeByte(i+1,OPERATING_MODE_XM,3);
      Dxl.writeByte(i+1,TORQUE_ENABLE_XM,1);
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
    Dxl.setPacketType(2);
    Dxl.writeDword(id,GOAL_POSITION_XM,(angle+180)*4095.0/360.0);
  } else {
    Dxl.setPacketType(1);
    Dxl.setPosition(id, (angle+150)*1023.0/300.0, speed);
  }
}

// Set motor max torque (Nmm)
void limitTorque(int id, float torque) {
  if (isMotorMirrored(id)) torque *= -1;
  if (isXM(id)) { // XM
    Dxl.setPacketType(2);
    Dxl.writeByte(id,TORQUE_ENABLE_XM,0);
    Dxl.writeByte(id,OPERATING_MODE_XM,5);
    Dxl.writeByte(id,TORQUE_ENABLE_XM,1);
    Dxl.writeWord(id,GOAL_CURRENT_XM,torque*2.3/4.1/2.69);
  } else { // AX
    Dxl.setPacketType(1);
    Dxl.writeWord(id,TORQUE_LIMIT_AX,torque/1800*1023);
  }
}

// Get current motor position (deg)
float getMotorPos(int id) {
  float angle;
  if (isXM(id)) { // XM
    Dxl.setPacketType(2);
    int pos = Dxl.readWord(id, PRESENT_POSITION_XM);
    angle = pos/4095.0*360 - 180;
  } else { // AX
    Dxl.setPacketType(1);
    int pos = Dxl.readWord(id, PRESENT_POSITION_AX);
    angle = pos/1023.0*300 - 150;
  }
  if (isMotorMirrored(id)) angle *= -1;
  return angle;
}

// Get current motor output torque (Nmm)
float getMotorTorque(int id) {
  if (isXM(id)) { // XM
    Dxl.setPacketType(2);
    int current = Dxl.readWord(id, PRESENT_CURRENT_XM);
    if (current >= 32768) current -= 32768*2;
    if (isMotorMirrored(id)) current *= -1;
    return(current*2.69*4.1/2.3);
  } else { // AX
    Dxl.setPacketType(1);
    int load = Dxl.readWord(id, PRESENT_LOAD_AX);
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
  float a1 = getMotorPos(foot);
  float a2 = getMotorPos(foot+4);
  float a3 = getMotorPos(foot+8);
  return(getEndForce(-t1, -t2, -t3, a1, a2, a3, foot));
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
  x[0] += 0;
  x[1] += 0;
  x[2] += 0;
  // TODO
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
      moveMotor(i+5, -40);
      moveMotor(i+9, 40);
  }
  moveMotor(13, 0);
  moveMotor(14, 0);
    moveMotor(2, 27);
    moveMotor(6, -22);
    moveMotor(10, 7);
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
  SerialUSB.println("Beginning Grasp");
  for(int i = 0; i < 600; i++) {
    da = getJointVel(0, -1, 0, a1, a2, a3, 2);
    a1 += da[0]*v;
    a2 += da[1]*v;
    a3 += da[2]*v;
    moveMotor(2, round(a1));
    moveMotor(6, round(a2));
    moveMotor(10, round(a3));
//  SerialUSB.print(a1);
//  SerialUSB.print(", ");
//  SerialUSB.print(a2);
//  SerialUSB.print(", ");
//  SerialUSB.println(a3);
  }
}

void graspForce() {
  float v = 0.1;
  float a1 = 27;
  float a2 = -22;
  float a3 = 7;
  float *da;
  float *t;
  SerialUSB.println("Lowering Gripper");
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
//  SerialUSB.print(a1);
//  SerialUSB.print(", ");
//  SerialUSB.print(a2);
//  SerialUSB.print(", ");
//  SerialUSB.println(a3);
  }
  SerialUSB.println("Engaging Spines");
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
//  SerialUSB.print(a1);
//  SerialUSB.print(", ");
//  SerialUSB.print(a2);
//  SerialUSB.print(", ");
//  SerialUSB.println(a3);
  }
}

// Display measured force components for each foot
void printForces() {
  SerialUSB.print(millis());
  SerialUSB.print(",     ");
  for (int i=0; i<4; i++) {
    float *f = getFootForce(i+1);
    SerialUSB.print(f[0]);
    SerialUSB.print(", ");
    SerialUSB.print(f[1]);
    SerialUSB.print(", ");
    SerialUSB.print(f[2]);
    SerialUSB.print(",     ");
  }
  SerialUSB.println("");
}

// Distribute external forces among the feet
void balanceForces(int n) {
  float k = 0.04;
  float kx = 0;
  float ky = 0;
  float kz = k;
  float weights[] = {1,1,0,0};
  if (n > 0) {
    weights[n-1] = 0;
  }
  float W = weights[0]+weights[1]+weights[2]+weights[3];
  for (int i = 0; i < 4; i++) {
    weights[i] /= W;
  }
  SerialUSB.println(weights[0]);
  SerialUSB.println(weights[1]);
  SerialUSB.println(weights[2]);
  SerialUSB.println(weights[3]);
  SerialUSB.println("--------------------------------");
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
    for(int i=0; i<2; i++) {
      f = getFootForce(i+1);
      F[i][0] = f[0];
      F[i][1] = f[1];
      F[i][2] = f[2];
      Fx += f[0] - offsets[i][0];
      Fy += f[1] - offsets[i][1];
      Fz += f[2] - offsets[i][2];
    }
    for(int i=0; i<2; i++) {
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
