// Determine if leg orientation is reversed
bool isLegMirrored(int foot) {
  return (foot == 1 || foot == 3);
}

// Determine foot position (mm) relative to centroid
float* getFootPos(int foot) {
  return (getEndPosCentroidal(angles[foot - 1][0], angles[foot - 1][1], angles[foot - 1][2], foot));
}

// Determine foot position setpoint (mm) relative to centroid
float* getFootPosGoal(int foot) {
  return (getEndPosCentroidal(setpoints[foot - 1][0], setpoints[foot - 1][1], setpoints[foot - 1][2], foot));
}

// Determine foot velocity (mm/s)
float* getFootVel(int foot) {
  return (getEndVel(speeds[foot - 1][0], speeds[foot - 1][1], speeds[foot - 1][2],
                    angles[foot - 1][0], angles[foot - 1][1], angles[foot - 1][2], foot));
}

// Determine current end effector forces (N)
float* getFootForce(int foot) {
  float t1 = torques[foot - 1][0];
  float t2 = torques[foot - 1][1];
  float t3 = torques[foot - 1][2];
  return (getEndForce(-t1, -t2, -t3, angles[foot - 1][0], angles[foot - 1][1], angles[foot - 1][2], foot));
}

// Determine current end effector force setpoint (N)
float* getFootForceGoal(int foot) {
  float t1 = torques[foot - 1][0];
  float t2 = torques[foot - 1][1];
  float t3 = torques[foot - 1][2];
  return (getEndForce(-t1, -t2, -t3, angles[foot - 1][0], angles[foot - 1][1], angles[foot - 1][2], foot));
}

// Determine joint angles position (deg) from end effector position (mm) relative to shoulder
float* getJointPos(float x, float y, float z, int foot) {
  if (isLegMirrored(foot)) x *= -1;
  static float a[3];
  a[0] = (1) * 180 / PI;
  a[1] = (1) * 180 / PI;
  a[2] = (1) * 180 / PI;
  // TODO
  return (a);
}

// Determine joint angles position (deg) from end effector position (mm) relative to centroid
float* getJointPosCentroidal(float x, float y, float z, int foot) {
  x -= 0;
  y -= 0;
  z -= 0;
  // TODO
  float *a = getJointPos(x, y, z, foot);
  return (a);
}

// Determine end effector position (mm) relative to shoulder from joint angles (deg)
float* getEndPos(float a1, float a2, float a3, int foot) {
  a1 = a1 * PI / 180;
  a2 = a2 * PI / 180;
  a3 = a3 * PI / 180;
  static float x[3];
  x[0] = cos(a1) * (LINK1 - LINK3 * sin(a2 + a3) + LINK2A * cos(a2) - LINK2B * sin(a2));
  x[1] = sin(a1) * (LINK1 - LINK3 * sin(a2 + a3) + LINK2A * cos(a2) - LINK2B * sin(a2));
  x[2] = - LINK3 * cos(a2 + a3) - LINK2B * cos(a2) - LINK2A * sin(a2);
  if (isLegMirrored(foot)) x[0] *= -1;
  return (x);
}

// Determine end effector position (mm) relative to centroid from joint angles (deg)
float* getEndPosCentroidal(float a1, float a2, float a3, int foot) {
  float *x = getEndPos(a1, a2, a3, foot);
  x[0] += isLegMirrored(foot) ? -WIDTH / 2 : WIDTH / 2;
  x[1] += foot <= 2 ? LENGTH / 2 : -LENGTH / 2;;
  x[2] += HEIGHT / 2;
  return (x);
}

// Determine joint velocities (deg/s) from end effector velocity (mm/s) and joint angles (deg)
float* getJointVel(float vx, float vy, float vz, float a1, float a2, float a3, int foot) {
  a1 = a1 * PI / 180;
  a2 = a2 * PI / 180;
  a3 = a3 * PI / 180;
  if (isLegMirrored(foot)) vx *= -1;
  static float w[3];
  w[0] = ((vy * cos(a1) - vx * sin(a1)) / (LINK1 - LINK3 * sin(a2 + a3) + LINK2A * cos(a2) - LINK2B * sin(a2))) * 180 / PI;
  w[1] = (-(vz * cos(a2) * cos(a3) - vz * sin(a2) * sin(a3) + vx * cos(a1) * cos(a2) * sin(a3) + vx * cos(a1) * cos(a3) * sin(a2) + vy * cos(a2) * sin(a1) * sin(a3) + vy * cos(a3) * sin(a1) * sin(a2)) / (LINK2A * cos(a3) + LINK2B * sin(a3))) * 180 / PI;
  w[2] = ((LINK3 * vz * cos(a2 + a3) + LINK2B * vz * cos(a2) + LINK2A * vz * sin(a2) + LINK3 * vx * sin(a2 + a3) * cos(a1) + LINK3 * vy * sin(a2 + a3) * sin(a1) - LINK2A * vx * cos(a1) * cos(a2) + LINK2B * vx * cos(a1) * sin(a2) - LINK2A * vy * cos(a2) * sin(a1) + LINK2B * vy * sin(a1) * sin(a2)) / (LINK3 * (LINK2A * cos(a3) + LINK2B * sin(a3)))) * 180 / PI;
  return (w);
}

// Determine end effector velocity (mm/s) from joint velocities (deg/s) and joint angles (deg)
float* getEndVel(float w1, float w2, float w3, float a1, float a2, float a3, int foot) {
  a1 = a1 * PI / 180;
  a2 = a2 * PI / 180;
  a3 = a3 * PI / 180;
  w1 = w1 * PI / 180;
  w2 = w2 * PI / 180;
  w3 = w3 * PI / 180;
  static float v[3];
  v[0] = - w2 * cos(a1) * (LINK3 * cos(a2 + a3) + LINK2B * cos(a2) + LINK2A * sin(a2)) - w1 * sin(a1) * (LINK1 - LINK3 * sin(a2 + a3) + LINK2A * cos(a2) - LINK2B * sin(a2)) - LINK3 * w3 * cos(a2 + a3) * cos(a1);
  v[1] = w1 * cos(a1) * (LINK1 - LINK3 * sin(a2 + a3) + LINK2A * cos(a2) - LINK2B * sin(a2)) - w2 * sin(a1) * (LINK3 * cos(a2 + a3) + LINK2B * cos(a2) + LINK2A * sin(a2)) - LINK3 * w3 * cos(a2 + a3) * sin(a1);
  v[2] = w2 * (LINK3 * sin(a2 + a3) - LINK2A * cos(a2) + LINK2B * sin(a2)) + LINK3 * w3 * sin(a2 + a3);
  if (isLegMirrored(foot)) v[0] *= -1;
  return (v);
}

// Determine joint torques (N*mm) from end effector forces (N) and joint angles (deg)
float* getJointTorque(float fx, float fy, float fz, float a1, float a2, float a3, int foot) {
  a1 = a1 * PI / 180;
  a2 = a2 * PI / 180;
  a3 = a3 * PI / 180;
  if (isLegMirrored(foot)) fx *= -1;
  static float t[3];
  t[0] = (fy * cos(a1) - fx * sin(a1)) * (LINK1 - LINK3 * sin(a2 + a3) + LINK2A * cos(a2) - LINK2B * sin(a2));
  t[1] = fz * (LINK3 * sin(a2 + a3) - LINK2A * cos(a2) + LINK2B * sin(a2)) - fx * cos(a1) * (LINK3 * cos(a2 + a3) + LINK2B * cos(a2) + LINK2A * sin(a2)) - fy * sin(a1) * (LINK3 * cos(a2 + a3) + LINK2B * cos(a2) + LINK2A * sin(a2));
  t[2] = LINK3 * fz * sin(a2 + a3) - LINK3 * fx * cos(a2 + a3) * cos(a1) - LINK3 * fy * cos(a2 + a3) * sin(a1);
  return (t);
}

// Determine tail joint torque (N*mm) from vertical force (N) and tail joint angle (deg)
float getTailTorque(float fz, float a) {
  a = a * PI / 180;
  float t = fz * cos(a) * TAIL1;
  return (t);
}

// Determine tail vertical force (N) from joint torque (N*mm) and joint angle (deg)
float getTailForce(float t, float a) {
  a = a * PI / 180;
  float fz = t / (cos(a) * TAIL1);
  return (fz);
}

// Determine tail position (mm) relative to centroid
float* getTailPos() {
  static float x[3];
  float a = angles[4][1] * PI / 180;
  x[0] = 0;
  x[1] = -LENGTH / 2 - TAIL1 * cos(a);
  x[2] = TAIL1 * sin(a);
  return (x);
}

// Determine tail position (mm) from joint angle (deg)
float* getTailPos(float a) {
  static float x[3];
  a *= PI / 180;
  x[0] = 0;
  x[1] = -LENGTH / 2 - TAIL1 * cos(a);
  x[2] = TAIL1 * sin(a);
  return (x);
}

// Determine tail position setpoint (mm) relative to centroid
float* getTailPosGoal() {
  static float x[3];
  float a = setpoints[4][1] * PI / 180;
  x[0] = 0;
  x[1] = -LENGTH / 2 - TAIL1 * cos(a);
  x[2] = TAIL1 * sin(a);
  return (x);
}

// Determine tail end velocity (mm/s) from joint velocity (deg/s)
float* getTailEndVel(float w) {
  static float v[3];
  w *= PI / 180;
  float a = angles[4][1] * PI / 180;
  v[0] = 0;
  v[1] = TAIL1 * sin(a) * w;
  v[2] = TAIL1 * cos(a) * w;
  return (v);
}

// Determine tail velocity (mm/s)
float* getTailVel() {
  static float v[3];
  float a = angles[4][1] * PI / 180;
  float s = speeds[4][1];
  v[0] = 0;
  v[1] = TAIL1 * sin(a) * s;
  v[2] = TAIL1 * cos(a) * s;
  return (v);
}

// Determine end effector forces (N) from joint torques (N*mm) and joint angles (deg)
float* getEndForce(float t1, float t2, float t3, float a1, float a2, float a3, int foot) {
  a1 = a1 * PI / 180;
  a2 = a2 * PI / 180;
  a3 = a3 * PI / 180;
  static float f[3];
  f[0] = (t3 * cos(a1) * (LINK3 * sin(a2 + a3) - LINK2A * cos(a2) + LINK2B * sin(a2))) / (LINK3 * (LINK2A * cos(a3) + LINK2B * sin(a3))) - (t2 * sin(a2 + a3) * cos(a1)) / (LINK2A * cos(a3) + LINK2B * sin(a3)) - (t1 * sin(a1)) / (LINK1 - LINK3 * sin(a2 + a3) + LINK2A * cos(a2) - LINK2B * sin(a2));
  f[1] = (t1 * cos(a1)) / (LINK1 - LINK3 * sin(a2 + a3) + LINK2A * cos(a2) - LINK2B * sin(a2)) - (t2 * sin(a2 + a3) * sin(a1)) / (LINK2A * cos(a3) + LINK2B * sin(a3)) + (t3 * sin(a1) * (LINK3 * sin(a2 + a3) - LINK2A * cos(a2) + LINK2B * sin(a2))) / (LINK3 * (LINK2A * cos(a3) + LINK2B * sin(a3)));
  f[2] = (LINK3 * t3 * cos(a2 + a3) - LINK3 * t2 * cos(a2 + a3) + LINK2B * t3 * cos(a2) + LINK2A * t3 * sin(a2)) / (LINK3 * (LINK2A * cos(a3) + LINK2B * sin(a3)));
  if (isLegMirrored(foot)) f[0] *= -1;
  return (f);
}

// Determine length of a step given gait leg angles
float getStepLength(float a1, float a2, float a3) {
  float *x = getEndPos(a1, a2, a3, 1);
  return (abs(x[1] * 2));
}
