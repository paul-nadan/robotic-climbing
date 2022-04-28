// Determine if motor direction is reversed
bool isMotorMirrored(int id) {
  // Note: XM motors (2 & 3) are instead mirrored in firmware
  return (id == 6 || id == 10 || id == 7 || id == 11  || id == 14);
}

// Determine if motor is XM rather than AX
bool isXM(int id) {
  return (id <= 4 || id == 13);
}

// Enable torque on all motors (and reset operating mode)
void enableTorque() {
  for (int i = 0; i < N_MOTORS; i++) {
    if (isXM(i + 1)) {
      Dxl.setPortProtocolVersion(2.0);
      Dxl.torqueOff(i + 1);
      //      Dxl.setOperatingMode(i+1,OP_POSITION);
      Dxl.setOperatingMode(i + 1, OP_CURRENT_BASED_POSITION);
      Dxl.torqueOn(i + 1);
    } else {
      Dxl.setPortProtocolVersion(1.0);
      Dxl.writeControlTableItem(ControlTableItem::TORQUE_LIMIT, i + 1, 1023);
    }
  }
}

// Set leg angle setpoints
void moveLeg(float a1, float a2, float a3, int foot) {
  setpoints[foot - 1][0] = a1;
  setpoints[foot - 1][1] = a2;
  setpoints[foot - 1][2] = a3;
}

// Set body joint angle setpoint
void moveBodyJoint(float angle) {
  setpoints[4][0] = angle;
}

// Set tail angle setpoint
void moveTail(float angle) {
  setpoints[4][1] = angle;
}

// Move all motors to their setpoints (TODO: batch write)
void moveMotors() {
  for (int i = 0; i < 4; i++) {
    moveMotor(i + 1, setpoints[i][0]);
    moveMotor(i + 5, setpoints[i][1]);
    moveMotor(i + 9, setpoints[i][2]);
  }
  moveMotor(13, setpoints[4][0]);
//  Serial << setpoints[4][1] << "\n";
  moveMotor(14, setpoints[4][1]);
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
    Dxl.setGoalPosition(id, (angle + 180) * 4095.0 / 360.0);
    Dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, id, speed);
  } else {
    Dxl.setPortProtocolVersion(1.0);
    Dxl.setGoalPosition(id, (angle + 150) * 1023.0 / 300.0);
    Dxl.writeControlTableItem(ControlTableItem::MOVING_SPEED, id, speed);
  }
}

// Set motor max torque (Nmm)
void limitTorque(int id, float torque) {
  if (isXM(id)) { // XM
    Dxl.setPortProtocolVersion(2.0);
//    Dxl.torqueOff(id);
//    Dxl.setOperatingMode(id,OP_CURRENT);
//    Dxl.torqueOn(id);
    Dxl.setGoalCurrent(id, torque * 2.3 / 4.1 / 2.69);
  } else { // AX
    Dxl.setPortProtocolVersion(1.0);
    Dxl.writeControlTableItem(ControlTableItem::TORQUE_LIMIT, id, torque / 1800 * 1023);
  }
}

void limitFootForce(int foot, float fx, float fy, float fz) {
  float *t = getJointTorque(fx, fy, fz, angles[foot - 1][0], angles[foot - 1][1], angles[foot - 1][2], foot);
  limitTorque(foot, (t[0]));
  limitTorque(foot + 4, abs(t[1]));
  limitTorque(foot + 8, abs(t[2]));
}

void limitTailForce(float fz) {
  float t = getTailTorque(fz, angles[4][1]);
  limitTorque(14, abs(t));
}

// Get current motor position (deg)
float getMotorPos(int id) {
  float angle;
  if (isXM(id)) { // XM
    Dxl.setPortProtocolVersion(2.0);
    int pos = Dxl.readControlTableItem(ControlTableItem::PRESENT_POSITION, id);
    angle = pos / 4095.0 * 360 - 180;
  } else { // AX
    Dxl.setPortProtocolVersion(1.0);
    int pos = Dxl.readControlTableItem(ControlTableItem::PRESENT_POSITION, id);
    angle = pos / 1023.0 * 300 - 150;
  }
  if (isMotorMirrored(id)) angle *= -1;
  return angle;
}

// updates the global array of current motor angles (TODO: batch read)
void updateMotorPos() {
  for (int i = 0; i < 4; i++) {
    angles[i][0] = getMotorPos(i + 1);
    angles[i][1] = getMotorPos(i + 5);
    angles[i][2] = getMotorPos(i + 9);
  }
  angles[4][0] = getMotorPos(13);
  angles[4][1] = getMotorPos(14);
}

// Get current motor output torque (Nmm)
float getMotorTorque(int id) {
  if (isXM(id)) { // XM
    Dxl.setPortProtocolVersion(2.0);
    int current = Dxl.readControlTableItem(ControlTableItem::PRESENT_CURRENT, id);
    if (current >= 32768) current -= 32768 * 2;
    if (isMotorMirrored(id)) current *= -1;
    return (current * 2.69 * 4.1 / 2.3);
  } else { // AX
    Dxl.setPortProtocolVersion(1.0);
    int load = Dxl.readControlTableItem(ControlTableItem::PRESENT_LOAD, id);
    if (load >= 1024) load = -(load - 1024);
    if (isMotorMirrored(id)) load *= -1;
    return (load / 1023.0 * 1800);
  }
}
