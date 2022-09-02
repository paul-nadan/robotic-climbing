float yawMax = 30;          // stance maximum shoulder yaw (deg)
float yawMid = 10;          // stance intermediary shoulder yaw (deg)
float shoulderStance = -20; // stance shoulder pitch (deg)
float kneeStance = 20;      // stance knee pitch (deg)
float tailStance = -10;     // stance tail pitch (deg)

int order[] = {3, 1, 4, 2};   // foot sequence
int offsets[] = {2, 0, 3, 1}; // foot sequence offsets
float yaw[] = {yawMax, yawMid, -yawMid, -yawMax}; // yaw positions
// float yaw[] = {15, 0, -15, -30}; // yaw positions

// Move foot linearly towards a desired position at given velocity (mm/s)
bool moveFootToPosition(float x, float y, float z, float v, int foot) {
  float *goal = getFootPosGoal(foot);
  float dx = x - goal[0];
  float dy = y - goal[1];
  float dz = z - goal[2];
  float d = sqrt(dx * dx + dy * dy + dz * dz);
  if (d > POS_TOL) moveFootInDirection(dx, dy, dz, v, foot);
  return (d < POS_TOL);
}

// Move foot linearly in a given direction at given velocity (mm/s)
void moveFootInDirection(float dx, float dy, float dz, float v, int foot) {
  float d = sqrt(dx * dx + dy * dy + dz * dz);
  float *w = getJointVel(dx, dy, dz, angles[foot - 1][0], angles[foot - 1][1], angles[foot - 1][2], foot);
  setpoints[foot - 1][0] += w[0] * dt / 1000 * v / d;
  setpoints[foot - 1][1] += w[1] * dt / 1000 * v / d;
  setpoints[foot - 1][2] += w[2] * dt / 1000 * v / d;
}

// Move tail linearly in a given direction at given velocity (mm/s)
void moveTailInDirection(float dz, float v) {
  float d = abs(dz);
  float w = getTailForce(dz, angles[4][1]);
  setpoints[4][1] += w * dt / 1000 * v / d;
}

// Move body linearly in a given direction at given velocity (mm/s)
void moveBodyInDirection(float dx, float dy, float dz, float v, int swing) {
  for (int i = 0; i < 4; i++) {
    if (i+1 != swing) moveFootInDirection(-dx, -dy, -dz, v, i + 1);
  }
}

// Change all setpoints to current motor positions
void freeze() {
  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < 3; j++) {
      setpoints[i][j] = angles[i][j];
      goal_angles[i][j] = angles[i][j];
    }
  }
}

// Upright posture
void stand() {
  for (int i = 0; i < 4; i++) {
    moveLeg(0, 30, -30, i + 1);
  }
  moveBodyJoint(0);
  moveTail(-20);
}

// Crouched posture that keeps the body close to the ground or wall
void sprawl() {
  for (int i = 0; i < 4; i++) {
    moveLeg(yaw[(step+offsets[i])%4], shoulderStance, kneeStance, i + 1);
  }
  moveBodyJoint(0);
  moveTail(tailStance);
}

// Reduce force so legs can conform to the surface
void stick() {
  for (int i = 0; i < 4; i++) {
    goal_torques[i][0] = motor_torque;
    goal_torques[i][1] = 200;
    goal_torques[i][2] = 200;
  }
  goal_torques[4][0] = motor_torque;
  goal_torques[4][1] = 200;
  writeTorques();
}

// Forward climbing gait
void climb() {
  if ((goalSteps > 0 && step >= goalSteps)) return;
  
  int foot = order[step % 4];

  float ft = 5;           // tangential engagement force (N)
  float fn = 3;           // normal engagement force (N)
  float fb = 2;           // horizontal engagement force (N)
  float v = 40;           // foot velocity (mm/s)
  float w = 50;           // joint angular velocity (mm/s)
  float tau = 300;        // joint torque (Nmm)
  float tauD = 600;       // joint torque during disengagement (Nmm)
  float detach = 40;      // detach distance (mm)
  float place = 120;      // placement distance (mm)
  float engage = 40;      // engage distance (mm)
  float phi1swing = -60;  // swing shoulder pitch (deg)
  float phi2swing = 40;   // swing knee pitch (deg)
  float phi1place = 0;    // place shoulder pitch (deg)
  float phi2place = 0;    // place knee pitch (deg)
  float body = getStepLength(yawMax, shoulderStance, kneeStance) / 4;

  Serial << substep << ", " << int(millis() - tb) << ", " << foot_weights[foot-1] << "\n";
  switch (substep) {
    case 0: // Unload foot
      foot_weights[foot-1] = 1 - (millis()-tb)/1000.0;
      if (foot_weights[foot-1] <= 0) {
        foot_weights[foot-1] = 0;
        substep++;
        tb = millis();
        goal_torques[foot-1][0] = tauD;
        goal_torques[foot-1][1] = tauD;
        goal_torques[foot-1][2] = tauD;
        goal_speeds[foot-1][0] = w;
        goal_speeds[foot-1][1] = w;
        goal_speeds[foot-1][2] = w;
        setpoints[foot-1][0] = angles[foot-1][0];
        setpoints[foot-1][1] = angles[foot-1][1];
        setpoints[foot-1][2] = angles[foot-1][2];
        goal_angles[foot-1][0] = angles[foot-1][0];
        goal_angles[foot-1][1] = angles[foot-1][1];
        goal_angles[foot-1][2] = angles[foot-1][2];
        writeSpeeds();
        writeAngles();
      }
      break;
    case 1: // Disengage foot
      if (millis() - tb < detach * 1000 / v * 0.3) {
        moveFootInDirection(0, 1, 0, v, foot);
      } else {
        moveFootInDirection(0, 1, 1, v, foot);
      }
      if (millis() - tb > detach * 1000 / v) {
        substep++;
        tb = millis();
      }
      break;
    case 2: // Lift foot
      setpoints[foot-1][1] = phi1swing;
      setpoints[foot-1][2] = phi2swing;
      if (millis() - tb > 1000) {
        substep++;
        tb = millis();
        goal_torques[foot-1][0] = tau;
        goal_torques[foot-1][1] = tau;
        goal_torques[foot-1][2] = tau;
      }
      break;
    case 3: // Swing foot and move body
      setpoints[foot-1][0] = yawMax;
      if (!retry) moveBodyInDirection(0, 1, 0, v/2, foot);
      if (millis() - tb > body * 1000 / (v/2)) {
        substep++;
        tb = millis();
      }
      break;
    // case 4: // Swing foot
    //   setpoints[foot-1][0] = yawMax;
    //   if (millis() - tb > 1000) {
    //     substep++;
    //     tb = millis();
    //   }
    //   break;
    case 4: // Place foot
      // moveLeg(yawMax, phi1place, phi2place, foot);
      limitFootForce(foot, fb, ft, fn);
      moveFootInDirection(0, 0, -1, v, foot);
      if (millis() - tb > place * 1000 / v) {
        substep++;
        tb = millis();
        limitFootForce(foot, 0, ft, fn);
        writeTorques();
        goal_angles[foot-1][0] = angles[foot-1][0];
        goal_angles[foot-1][1] = angles[foot-1][1];
        goal_angles[foot-1][2] = angles[foot-1][2];
        setpoints[foot-1][0] = angles[foot-1][0];
        setpoints[foot-1][1] = angles[foot-1][1];
        setpoints[foot-1][2] = angles[foot-1][2];
        writeAngles();
      }
      break;
    case 5: // Engage foot
      limitFootForce(foot, fb, ft, fn);
      moveFootInDirection(0, -1, -1, v, foot);
      if (millis() - tb > engage * 1000 / v) {
        substep++;
        tb = millis();
        goal_speeds[foot-1][0] = motor_speed;
        goal_speeds[foot-1][1] = motor_speed;
        goal_speeds[foot-1][2] = motor_speed;
        writeSpeeds();
      }
      break;
    case 6: // Load foot
      foot_weights[foot-1] = (millis()-tb)/1000.0;
      if (foot_weights[foot-1] >= 1) {
        foot_weights[foot-1] = 1;
        substep++;
        tb = millis();
        goal_torques[foot-1][0] = 1000;
        goal_torques[foot-1][1] = 1000;
        goal_torques[foot-1][2] = 1000;
      }
      break;
    case 7: // Recenter
      // recenter(step+1, 1000);
      // level(0);
      recenter(step+1);
      if (millis() - tb > 2000) {
        substep++;
        tb = millis();
      }
      break;
    default: // Completed step
      substep = 0;
      step++;
      break;
  }
}

// Interpolate controller setpoint towards nominal stance
// void recenter(int step, float duration) {
//   float delta = min(dt/(tb + duration - millis()), 1);
//   for (int i = 0; i < 4; i++) {
//     setpoints[i][0] += (yaw[(step+offsets[i])%4] - setpoints[i][0])*delta;
//     setpoints[i][1] += (shoulderStance - setpoints[i][1])*delta;
//     setpoints[i][2] += (kneeStance - setpoints[i][2])*delta;
//   }
//   setpoints[4][0] += (0 - setpoints[4][0])*delta;
//   setpoints[4][1] += (tailStance - setpoints[4][1])*delta;
// }

// Return the nominal stance position at a given step
BLA::Matrix<12, 1> getNominalStance(int step) {
  BLA::Matrix<12, 1> X;
  for (int i = 0; i < 4; i++) {
    float *x = getEndPosCentroidal(yaw[(step+offsets[i])%4], shoulderStance, kneeStance, i+1);
    X(i*3) = x[0];
    X(i*3+1) = x[1];
    X(i*3+2) = x[2];
  }
  // float *xt = getTailPos(tailStance);
  // X(13) = xt[2];
  return X;
}

// Translate body to maintain constant ground clearance
void level(int foot) {
  float zmax = -1000;
  for (int i = 0; i < 4; i++) {
    float *x = getFootPosGoal(i+1);
    if(i + 1 != foot) zmax = max(zmax, x[2]);
  }
  float offset = zmax + 40;
  float v = 30;
  if (abs(offset) < v*dt/1000) v = abs(offset*1000/dt);
  if (offset == 0) return;
  for (int i = 0; i < 4; i++) {
    if(i + 1 != foot) moveFootInDirection(0, 0, -offset, v, i + 1);
  }
}

// Forward walking gait for level terrain
void walk() {
  if ((goalSteps > 0 && step >= goalSteps) || millis() - tb < gait_period) return;
  int count = step * 2 + substep;
  tb = millis();
  float yaw[] = {60, 60, 0, 0, -60, -60, -60, 60};
  float shoulder[] = {30, 30, 30, 30, 30, 30, 0, 0};
  float knee[] = { -30, -30, 0, 0, -30, -30, 0, 0};
  float body[] = {0, 0, 0, 0, 0, 0, 0, 0};
  float tail[] = { -20, -20, -20, -20, -20, -20, -20, -20};
  int order[] = {3, 1, 4, 2};
  for (int i = 0; i < 4; i++) {
    moveLeg(yaw[(count + 8 - i * 2) % 8], shoulder[(count + 8 - i * 2) % 8], knee[(count + 8 - i * 2) % 8], i + 1);
  }
  moveBodyJoint(body[count % 8]);
  moveTail(tail[count % 8]);
  substep++;
  if (substep >= 2) {
    substep = 0;
    step++;
  }
}

// Inverted turtle gait (inchworming on its back, using the body joint and tail)
void sillywalk() {
  if ((goalSteps > 0 && step >= goalSteps) || millis() - tb < gait_period) return;
  tb = millis();
  float yaw[] = {0, 45, 0, -45};
  float shoulder[] = {30, 30, 0, 30};
  float knee[] = {0, 0, -30, 0};
  float body[] = {0, 60, 60, 0};
  float tail[] = { -20, -20, 20, 30};
  float order[] = {3, 1, 4, 2};
  for (int i = 0; i < 4; i++) {
    moveLeg(yaw[(step + 4 - i) % 4], shoulder[(step + 4 - i) % 4], knee[(step + 4 - i) % 4], i + 1);
  }
  moveBodyJoint(body[step % 4]);
  moveTail(tail[step % 4]);
  step++;
}

// Autonomously take steps with no user input
int lastStep = 0;
void autoclimb() {
  if (control == OFF) {
    if (millis() - t0 > 10000) { // Wait 10 seconds, then turn on controller
      freeze();
      setController(DEFAULT_CONTROL);
      Serial << "Controller on\n";
      lastStep = 0;
      t0 = millis();
    }
  } else {
    if (state != CLIMB && millis() - t0 > 2000) { // Wait 2 seconds, then take a step
      setBehavior(CLIMB);
      retry = false;
      goalSteps = step + 1;
      Serial << "Climb: beginning step " << step+1 << "\n";
    }
    if (state == CLIMB && step != lastStep) { // Reset timer when step completes
      t0 = millis();
      state = IDLE;
      Serial << "Climb: completed step " << step << "\n";
    }
}
  lastStep = step;
}