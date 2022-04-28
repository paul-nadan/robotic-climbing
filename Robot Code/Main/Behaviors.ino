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

// Move body linearly in a given direction at given velocity (mm/s)
void moveBodyInDirection(float dx, float dy, float dz, float v) {
  for (int i = 0; i < 4; i++) {
    moveFootInDirection(-dx, -dy, -dz, v, i + 1);
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
    moveLeg(0, -40, 30, i + 1);
  }
  moveBodyJoint(0);
  moveTail(-20);
}

// Forward climbing gait
void climb() {
  if ((goalSteps > 0 && step >= goalSteps)) return;
  int order[] = {3, 1, 4, 2};
  int foot = order[step % 4];

  float v = 20; // foot velocity (mm/s)
  float detach = 30; // detach distance (mm)
  float engage = 30; // engage distance (mm)
  float body = getStepLength(30, -40, 30) / 4;
  Serial << substep << ", " << int(millis()-tb) << "\n";
  switch (substep) {
    case 0: // Detach foot
      moveFootInDirection(0, 1, 1, v, foot);
      if (millis()-tb > detach * 1000 / v) {
        substep++;
        tb = millis();
      }
      break;
    case 1: // Lift foot
      moveLeg(30, -50, 40, foot);
      if (millis()-tb > 500) {
        substep++;
        tb = millis();
      }
      break;
    case 2: // Place foot
      moveLeg(30, -40, 30, foot);
      if (millis()-tb > 500) {
        substep++;
        tb = millis();
      }
      break;
    case 3: // Engage foot
      moveFootInDirection(0, -1, -1, v, foot);
      if (millis()-tb > engage * 1000 / v) {
        substep++;
        tb = millis();
      }
      break;
    case 4: // Move body
      moveBodyInDirection(0, 1, 0, v);
      if (millis()-tb > body * 1000 / v) {
        substep++;
        tb = millis();
      }
      break;
    default: // finished step
      substep = 0;
      step++;
      break;
  }
}

// Forward walking gait for level terrain
void walk() {
  if ((goalSteps > 0 && step >= goalSteps) || millis() - tb < period) return;
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
  if ((goalSteps > 0 && step >= goalSteps) || millis() - tb < period) return;
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
