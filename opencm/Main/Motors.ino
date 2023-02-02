#define GOAL_POSITION_AX 30
#define GOAL_POSITION_XM 116
#define PRESENT_POSITION_AX 36
#define PRESENT_POSITION_XM 132
#define TORQUE_LIMIT_AX 34
#define GOAL_CURRENT_XM 102
#define PRESENT_LOAD_AX 40
#define PRESENT_CURRENT_XM 126
#define PROFILE_VELOCITY_XM 112
#define MOVING_SPEED_AX 32
#define PRESENT_VELOCITY_XM 128
#define PRESENT_SPEED_AX 38
#define PRESENT_INPUT_VOLTAGE_XM 144

ParamForSyncWriteInst_t sync_write_param;
ParamForSyncReadInst_t sync_read_param;
RecvInfoFromStatusInst_t read_result;

// Note: XM motors (2 & 3) are instead mirrored in firmware
const int mirror[] = {1, 1, 1, 1, 1, -1, -1, 1, 1, -1, -1, 1, 1, -1}; // -1 for mirrored, 1 otherwise
const bool xm[] = {1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0};         // 1 for XM, 0 for AX
const int xmid[] = {1, 2, 3, 4, 13};                                  // indices of XM motors
const int nxm = 5;                                                    // number of XM motors
const int axid[] = {5, 6, 7, 8, 9, 10, 11, 12, 14};                   // indices of AX motors
const int nax = 9;                                                    // number of AX motors
const int map1[] = {0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 4, 4};        // first index in motor value array
const int map2[] = {0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 0, 1};        // second index in motor value array
const int lower_limit[] = {YAW_LIMIT_MIN, YAW_LIMIT_MIN, YAW_LIMIT_MIN, YAW_LIMIT_MIN,
                            SHOULDER_LIMIT_MIN, SHOULDER_LIMIT_MIN, SHOULDER_LIMIT_MIN, SHOULDER_LIMIT_MIN,
                            KNEE_LIMIT_MIN, KNEE_LIMIT_MIN, KNEE_LIMIT_MIN, KNEE_LIMIT_MIN,
                            BODY_LIMIT_MIN, TAIL_LIMIT_MIN
                           };
const int upper_limit[] = {YAW_LIMIT_MAX, YAW_LIMIT_MAX, YAW_LIMIT_MAX, YAW_LIMIT_MAX,
                            SHOULDER_LIMIT_MAX, SHOULDER_LIMIT_MAX, SHOULDER_LIMIT_MAX, SHOULDER_LIMIT_MAX,
                            KNEE_LIMIT_MAX, KNEE_LIMIT_MAX, KNEE_LIMIT_MAX, KNEE_LIMIT_MAX,
                            BODY_LIMIT_MAX, TAIL_LIMIT_MAX
                           };

// Enable torque on all motors and reset operating mode
void enable() {
  for (int i = 0; i < N_MOTORS; i++) {
    if (xm[i]) {
      Dxl.setPortProtocolVersion(2.0);
      Dxl.torqueOff(i + 1);
      Dxl.setOperatingMode(i + 1, OP_CURRENT_BASED_POSITION);
      Dxl.torqueOn(i + 1);
    } else {
      Dxl.setPortProtocolVersion(1.0);
      Dxl.torqueOn(i + 1);
    }
  }
  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < 3; j++) {
      goal_speeds[i][j] = motor_speed;
      goal_torques[i][j] = motor_torque;
      goal_angles[i][j] = angles[i][j];
      setpoints[i][j] = angles[i][j];
    }
    foot_weights[i] = 1;
  }
  writeAngles();
  writeSpeeds();
  // writeTorques();
}

// Disable torque on all motors
void disable() {
  for (int i = 0; i < N_MOTORS; i++) {
    if (xm[i]) {
      Dxl.setPortProtocolVersion(2.0);
    } else {
      Dxl.setPortProtocolVersion(1.0);
    }
    Dxl.torqueOff(i + 1);
  }
  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < 3; j++) {
      goal_torques[i][j] = 0;
      goal_angles[i][j] = angles[i][j];
      setpoints[i][j] = angles[i][j];
    }
  }
}

// Sync read motor angles (deg)
void readAngles() {
  Dxl.setPortProtocolVersion(2.0); // XM
  sync_read_param.addr = PRESENT_POSITION_XM;
  sync_read_param.length = 4;
  sync_read_param.id_count = nxm;
  for (int i = 0; i < nxm; i++) {
    sync_read_param.xel[i].id = xmid[i];
  }
  Dxl.syncRead(sync_read_param, read_result);
  int id = 0;
  int raw = 0;
  for (int i = 0; i < nxm; i++) {
    id = xmid[i];
    memcpy(&raw, read_result.xel[i].data, read_result.xel[i].length);
    angles[map1[id - 1]][map2[id - 1]] = (raw / 4095.0 * 360 - 180) * mirror[id - 1];
  }

  Dxl.setPortProtocolVersion(1.0); // AX (sync read not supported)
  for (int i = 0; i < nax; i++) {
    id = axid[i];
    raw = Dxl.readControlTableItem(ControlTableItem::PRESENT_POSITION, id);
    angles[map1[id - 1]][map2[id - 1]] = (raw / 1023.0 * 300 - 150) * mirror[id - 1];
  }
}

// Sync write motor angles (deg)
void writeAngles() {
  Dxl.setPortProtocolVersion(2.0); // XM
  sync_write_param.addr = GOAL_POSITION_XM;
  sync_write_param.length = 4;
  sync_write_param.id_count = nxm;
  int id;
  float val;
  float setpoint;
  int raw;
  for (int i = 0; i < nxm; i++) {
    id = xmid[i];
    sync_write_param.xel[i].id = id;
    val = goal_angles[map1[id - 1]][map2[id - 1]];
    setpoint = setpoints[map1[id - 1]][map2[id - 1]];
    if (val < lower_limit[id - 1]) val = lower_limit[id - 1];
    if (val > upper_limit[id - 1]) val = upper_limit[id - 1];
    if (val < setpoint - max_overshoot) val = setpoint - max_overshoot;
    if (val > setpoint + max_overshoot) val = setpoint + max_overshoot;
    raw = (val * mirror[id - 1] + 180) * 4095.0 / 360.0;
    memcpy(sync_write_param.xel[i].data, &raw, 4);
  }
  Dxl.syncWrite(sync_write_param);

  Dxl.setPortProtocolVersion(1.0); // AX
  sync_write_param.addr = GOAL_POSITION_AX;
  sync_write_param.length = 2;
  sync_write_param.id_count = nax;
  for (int i = 0; i < nax; i++) {
    id = axid[i];
    sync_write_param.xel[i].id = id;
    val = goal_angles[map1[id - 1]][map2[id - 1]];
    if (val < lower_limit[id - 1]) val = lower_limit[id - 1];
    if (val > upper_limit[id - 1]) val = upper_limit[id - 1];
    raw = (val * mirror[id - 1] + 150) * 1023.0 / 300.0;
    memcpy(sync_write_param.xel[i].data, &raw, 2);
  }
  Dxl.syncWrite(sync_write_param);
}

// Sync read motor torques (Nmm)
void readTorques() {
  Dxl.setPortProtocolVersion(2.0); // XM
  sync_read_param.addr = PRESENT_CURRENT_XM;
  sync_read_param.length = 2;
  sync_read_param.id_count = nxm;
  for (int i = 0; i < nxm; i++) {
    sync_read_param.xel[i].id = xmid[i];
  }
  Dxl.syncRead(sync_read_param, read_result);
  int id = 0;
  int raw = 0;
  for (int i = 0; i < nxm; i++) {
    id = xmid[i];
    memcpy(&raw, read_result.xel[i].data, read_result.xel[i].length);
    if (raw >= 32768) raw -= 32768 * 2;
    torques[map1[id - 1]][map2[id - 1]] = raw * 2.69 * 4.1 / 2.3 * mirror[id - 1];
  }

  Dxl.setPortProtocolVersion(1.0); // AX (sync read not supported)
  for (int i = 0; i < nax; i++) {
    id = axid[i];
    raw = Dxl.readControlTableItem(ControlTableItem::PRESENT_LOAD, id);
    if (raw >= 1024) raw = -(raw - 1024);
    torques[map1[id - 1]][map2[id - 1]] = raw / 1023.0 * 1800 * mirror[id - 1];
  }
}

// Sync write motor torques (Nmm)
void writeTorques() {
  Dxl.setPortProtocolVersion(2.0); // XM
  sync_write_param.addr = GOAL_CURRENT_XM;
  sync_write_param.length = 2;
  sync_write_param.id_count = nxm;
  int id = 0;
  float val = 0;
  int raw = 0;
  for (int i = 0; i < nxm; i++) {
    id = xmid[i];
    sync_write_param.xel[i].id = id;
    val = goal_torques[map1[id - 1]][map2[id - 1]];
    raw = int(min(val * 2.3 / 4.1 / 2.69, 1193));
    // raw = val * 2.3 / 4.1 / 2.69;
    memcpy(sync_write_param.xel[i].data, &raw, 2);
  }
  Dxl.syncWrite(sync_write_param);

  Dxl.setPortProtocolVersion(1.0); // AX
  sync_write_param.addr = TORQUE_LIMIT_AX;
  sync_write_param.length = 2;
  sync_write_param.id_count = nax;
  for (int i = 0; i < nax; i++) {
    id = axid[i];
    sync_write_param.xel[i].id = id;
    val = goal_torques[map1[id - 1]][map2[id - 1]];
    raw = val / 1800 * 1023;
    memcpy(sync_write_param.xel[i].data, &raw, 2);
  }
  Dxl.syncWrite(sync_write_param);
}

// Sync read motor speeds (deg/s)
void readSpeeds() {
  Dxl.setPortProtocolVersion(2.0); // XM
  sync_read_param.addr = PRESENT_VELOCITY_XM;
  sync_read_param.length = 4;
  sync_read_param.id_count = nxm;
  for (int i = 0; i < nxm; i++) {
    sync_read_param.xel[i].id = xmid[i];
  }
  Dxl.syncRead(sync_read_param, read_result);
  int id = 0;
  int raw = 0;
  for (int i = 0; i < nxm; i++) {
    id = xmid[i];
    memcpy(&raw, read_result.xel[i].data, read_result.xel[i].length);
    if (raw >= 32768) raw -= 32768 * 2;
    speeds[map1[id - 1]][map2[id - 1]] = raw * 0.229 * 6 * mirror[id - 1];
  }

  Dxl.setPortProtocolVersion(1.0); // AX (sync read not supported)
  for (int i = 0; i < nax; i++) {
    id = axid[i];
    raw = Dxl.readControlTableItem(ControlTableItem::PRESENT_SPEED, id);
    if (raw >= 1024) raw = -(raw - 1024);
    speeds[map1[id - 1]][map2[id - 1]] = raw * 0.111 * 6 * mirror[id - 1];
  }
}

// Sync write motor speeds (deg/s)
void writeSpeeds() {
  Dxl.setPortProtocolVersion(2.0); // XM
  sync_write_param.addr = PROFILE_VELOCITY_XM;
  sync_write_param.length = 4;
  sync_write_param.id_count = nxm;
  int id = 0;
  float val = 0;
  int raw = 0;
  for (int i = 0; i < nxm; i++) {
    id = xmid[i];
    sync_write_param.xel[i].id = id;
    val = goal_speeds[map1[id - 1]][map2[id - 1]];
    raw = val / 0.229 / 6;
    memcpy(sync_write_param.xel[i].data, &raw, 4);
  }
  Dxl.syncWrite(sync_write_param);

  Dxl.setPortProtocolVersion(1.0); // AX
  sync_write_param.addr = MOVING_SPEED_AX;
  sync_write_param.length = 2;
  sync_write_param.id_count = nax;
  for (int i = 0; i < nax; i++) {
    id = axid[i];
    sync_write_param.xel[i].id = id;
    val = goal_speeds[map1[id - 1]][map2[id - 1]];
    raw = val / 0.111 / 6;
    memcpy(sync_write_param.xel[i].data, &raw, 2);
  }
  Dxl.syncWrite(sync_write_param);
}

float readVoltage() {
  Dxl.setPortProtocolVersion(2.0); // XM
  return(Dxl.readControlTableItem(ControlTableItem::PRESENT_INPUT_VOLTAGE, 1)/10.0);
}

// Limit maximum contact force applied by the foot
void limitFootForce(int foot, float fx, float fy, float fz) {
  float *t = getJointTorque(fx, fy, fz, angles[foot - 1][0], angles[foot - 1][1], angles[foot - 1][2], foot);
  goal_torques[foot - 1][0] = abs(t[0]);
  goal_torques[foot - 1][1] = abs(t[1]);
  goal_torques[foot - 1][2] = abs(t[2]);
}

// Set desired contact force applied by the foot
void setFootForce(int foot, float fx, float fy, float fz) {
  float *t = getJointTorque(fx, fy, fz, angles[foot - 1][0], angles[foot - 1][1], angles[foot - 1][2], foot);
  goal_torques[foot - 1][0] = abs(t[0]);
  goal_torques[foot - 1][1] = abs(t[1]);
  goal_torques[foot - 1][2] = abs(t[2]);

  // goal_angles[foot - 1][0] = setpoints[foot - 1][0] + kF * (t[0] - torques[foot - 1][0]);
  // goal_angles[foot - 1][1] = setpoints[foot - 1][1] + kF * (t[1] - torques[foot - 1][1]);
  // goal_angles[foot - 1][2] = setpoints[foot - 1][2] + kF * (t[2] - torques[foot - 1][2]);

  // goal_angles[foot - 1][0] = setpoints[foot-1][0];
  // goal_angles[foot - 1][1] = setpoints[foot-1][1];
  // goal_angles[foot - 1][2] = setpoints[foot-1][2];

  // float *w = getJointVel(fx, fy, fz, angles[foot - 1][0], angles[foot - 1][1], angles[foot - 1][2], foot);
  // goal_angles[foot - 1][0] += w[0]*overshoot*dt/1000;
  // goal_angles[foot - 1][1] += w[1]*overshoot*dt/1000;
  // goal_angles[foot - 1][2] += w[2]*overshoot*dt/1000;
  
  // goal_angles[foot - 1][0] = t[0] > 0 ? 1000 : -1000;
  // goal_angles[foot - 1][1] = t[1] > 0 ? 1000 : -1000;
  // goal_angles[foot - 1][2] = t[2] > 0 ? 1000 : -1000;
  // goal_angles[foot - 1][0] = t[0] > 0 ? goal_angles[foot - 1][0] + overshoot : goal_angles[foot - 1][0] - overshoot;
  // goal_angles[foot - 1][1] = t[1] > 0 ? goal_angles[foot - 1][1] + overshoot : goal_angles[foot - 1][1] - overshoot;
  // goal_angles[foot - 1][2] = t[2] > 0 ? goal_angles[foot - 1][2] + overshoot : goal_angles[foot - 1][2] - overshoot;
  
  for (int j = 0; j < 3; j++) {
    goal_angles[foot - 1][j] = setpoints[foot - 1][j] + kF * (t[j] - torques[foot - 1][j]);

    // goal_angles[foot - 1][j] = t[j] > 0 ? min(goal_angles[foot - 1][j] + overshoot, angles[foot - 1][j] + max_overshoot) : 
    //   max(goal_angles[foot - 1][j] - overshoot, angles[foot - 1][j] - max_overshoot);
  }
}

// Limit maximum contact force applied by the tail
void limitTailForce(float fz) {
  float t = getTailTorque(fz, angles[4][1]);
  goal_torques[4][1] = abs(t);
}

// Set desired contact force applied by the tail
void setTailForce(float fz) {
  float t = getTailTorque(fz, angles[4][1]);
  float tmax = 300;
  if (t > tmax) t = tmax;
  if (t < -tmax) t = -tmax;
  goal_torques[4][1] = abs(t);
  // goal_angles[4][1] = setpoints[4][1] + kF * (t - torques[4][1]);
  // goal_angles[4][1] = t > 0 ? goal_angles[4][1] + overshoot : goal_angles[4][1] - overshoot;
  goal_angles[4][1] = t > 0 ? goal_angles[4][1] + overshoot : goal_angles[4][1] - overshoot;
  // float w = getTailForce(fz, angles[4][1]); // the inverse jacobian equals its transpose
  // goal_angles[4][1] += w*overshoot*dt/1000;
}

// Set leg angle setpoints
void moveLeg(float a1, float a2, float a3, int foot) {
  setpoints[foot - 1][0] = a1;
  setpoints[foot - 1][1] = a2;
  setpoints[foot - 1][2] = a3;
  goal_speeds[foot - 1][0] = motor_speed;
  goal_speeds[foot - 1][1] = motor_speed;
  goal_speeds[foot - 1][2] = motor_speed;
}

// Set body joint angle setpoint
void moveBodyJoint(float angle) {
  setpoints[4][0] = angle;
  goal_speeds[4][0] = motor_speed;
}

// Set tail angle setpoint
void moveTail(float angle) {
  setpoints[4][1] = angle;
  goal_speeds[4][1] = motor_speed;
}
