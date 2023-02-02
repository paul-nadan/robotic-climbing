#define DXL_BUS_SERIAL 3
#define GOAL_POSITION_XM 116
#define TORQUE_ENABLE_XM 64

#define WIDTH 154 // mm
#define LENGTH 283.4
#define LINK1 23.5
#define LINK2A 71
#define LINK2B 22.5
#define LINK3 53.7

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
 */

int speed = 50;

Dynamixel Dxl(DXL_BUS_SERIAL);

void setup() {
  SerialUSB.attachInterrupt(teleop);
  Dxl.begin(3);
  enableTorque();
  delay(2000);
  stand();
  speed = 100;
  
//  delay(1000);
//  wag(3, 500);
//  
//  delay(1000);
//  backbend(3, 500);
//  
  delay(1000);
  walk(0, 500);
//
//  delay(1000);
//  sillywalk(0, 500);
}

void loop() {
}

/* Utility functions */

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
  if (id==6 || id==10 || id==7 || id==11  || id==14) angle = -angle;
  if (id <= 4 || id == 13) {
    Dxl.setPacketType(2);
    Dxl.writeDword(id,GOAL_POSITION_XM,(angle+180)*4095.0/360.0);
  } else {
    Dxl.setPacketType(1);
    Dxl.setPosition(id, (angle+150)*1023.0/300.0, speed);
  }
}

void enableTorque() {
  Dxl.setPacketType(2);
  Dxl.writeByte(1,TORQUE_ENABLE_XM,1);
  Dxl.writeByte(2,TORQUE_ENABLE_XM,1);
  Dxl.writeByte(3,TORQUE_ENABLE_XM,1);
  Dxl.writeByte(4,TORQUE_ENABLE_XM,1);
  Dxl.writeByte(13,TORQUE_ENABLE_XM,1);
}

void teleop(byte* buffer, byte nCount) {
  char message [nCount];
  for (int i=0; i<20; i++) {
    message[i] = 0;
  }
  for (int i=0; i<nCount; i++) {
    message[i] = (char)buffer[i];
  }
  int id = atoi(message);
  float angle = atof(message + (id >= 10 ? 3:2));
  moveMotor(id, angle);
  SerialUSB.print("Moving motor ");
  SerialUSB.print(id);
  SerialUSB.print(" to angle ");
  SerialUSB.println(angle);
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
