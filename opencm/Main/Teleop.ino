// Check for user commands
void checkSerial() {
  if (Serial.available() == 0) return;
  char command = Serial.read();
  int id;
  float val;
  switch (command) {
    case 'o': // toggle controller
      if (control == OFF) {
        freeze();
        setController(DEFAULT_CONTROL);
        Serial << "Controller on\n";
      }
      else {
        setController(OFF);
        freeze();
        writeAngles();
        Serial << "Controller off\n";
      }
      break;
    case ' ': // halt
      freeze();
      disable();
      setBehavior(IDLE);
      setController(OFF);
      step = 0;
      Serial << "Halt\n";
      break;
    case '/': // disable torque
      disable();
      Serial << "Disable torque\n";
      break;
    case 'a': // stand
      setBehavior(IDLE);
      step = 0;
      stand();
      Serial << "Stand\n";
      break;
    case 's': // sprawl
      setBehavior(IDLE);
      // step = 0;
      stick();
      sprawl();
      Serial << "Sprawl\n";
      break;
    case 'w': // walk
      setBehavior(WALK);
      goalSteps = 0;
      Serial << "Beginning walk\n";
      break;
    case 'x': // stick
      setBehavior(IDLE);
      setController(OFF);
      stick();
      Serial << "Stick\n";
      break;
    case 'q': // recenter
      setBehavior(RECENTER);
      Serial << "Recenter\n";
      break;
    case 'c': // climb (1 step)
      setBehavior(CLIMB);
      retry = false;
      goalSteps = step + 1;
      Serial << "Climb: beginning step " << goalSteps << "\n";
      break;
    case 'r': // climb (retry step)
      setBehavior(CLIMB);
      retry = true;
      step -= 1;
      goalSteps = step + 1;
      Serial << "Climb: retrying step " << goalSteps << "\n";
      break;
    case 'i': // sillywalk
      setBehavior(SILLYWALK);
      goalSteps = 0;
      Serial << "Silly walk\n";
      break;
    case 'k': // set gains
      command = Serial.read();
      val = Serial.parseFloat();
      if (command == 'p' || command == 'P') {
        kP = val;
      }
      if (command == 'i' || command == 'I') {
        kI = val;
      }
      if (command == 'd' || command == 'D') {
        kD = val;
      }
      if (command == 'f' || command == 'F') {
        kF = val;
      }
      if (command == 'o' || command == 'O') {
        overshoot = val;
      }
      Serial << "Gains: kP = " << kP << ", kI = " << kI << ", kD = " << kD << ", kF = " << kF << ", kO = " << overshoot << "\n";
      break;
    case 'm': // move motor
      id = Serial.parseInt();
      if (id == 0 || Serial.available() == 0) break;
      val = Serial.parseFloat();
      if (id <= 12) setpoints[(id - 1) % 4][int((id - 1) / 4)] = val;
      else setpoints[4][id - 13] = val;
      Serial << "Moving motor " << id << " to angle " << val << " deg\n";
      break;
    case 't': // set motor torque
      id = Serial.parseInt();
      if (id == 0 || Serial.available() == 0) break;
      val = Serial.parseFloat();
      if (id <= 12) goal_torques[(id - 1) % 4][int((id - 1) / 4)] = val;
      else goal_torques[4][id - 13] = val;
      Serial << "Setting motor " << id << " to torque " << val << " N-mm\n";
      break;
  }
}

// Set the current control strategy
void setController(controller c) {
  integral.Fill(0);
  control = c;
}

// Set the current behavior
void setBehavior(behavior b) {
  state = b;
  substep = 0;
  enable();
  t0 = t;
  tb = t;
}

// Set the current display variable
void setDisplay(display d) {
  output = d;
}

// Display measured positions for each foot (mm)
void printPositions() {
  Serial << int(millis());
  for (int i = 0; i < 4; i++) {
    float *p = getFootPos(i + 1);
    Serial << ",   " << p[0] << ", " << p[1] << ", " << p[2];
  }
  Serial << ";\n";
}

// Display measured angles for each motor (deg)
void printAngles() {
  Serial << int(millis());
  for (int i = 0; i < 4; i++) {
    Serial << ",   " << angles[i][0] << ", " << angles[i][1] << ", " << angles[i][2];
  }
  Serial << ",   " << angles[4][0] << ", " << angles[4][1] << ";\n";
}

// Display measured forces for each foot (N)
void printForces() {
  Serial << int(millis());
  for (int i = 0; i < 4; i++) {
    float *f = getFootForce(i + 1);
    Serial << ",   " << f[0] << ", " << f[1] << ", " << f[2];
  }
  Serial << ";\n";
}

// Display measured torques for each motor (Nmm)
void printTorques() {
  Serial << int(millis());
  for (int i = 0; i < 4; i++) {
    Serial << ",   " << torques[i][0] << ", " << torques[i][1] << ", " << torques[i][2];
  }
  Serial << ",   " << torques[4][0] << ", " << torques[4][1] << ";\n";
}

// Display measured end effector velocities for each foot (mm/s)
void printFootVels() {
  Serial << int(millis());
  for (int i = 0; i < 4; i++) {
    float *v = getFootVel(i + 1);
    Serial << ",   " << v[0] << ", " << v[1] << ", " << v[2];
  }
  Serial << ";\n";
}


// Display measured motor velocities for each motor (deg/s)
void printSpeeds() {
  Serial << int(millis());
  for (int i = 0; i < 4; i++) {
    Serial << ",   " << speeds[i][0] << ", " << speeds[i][1] << ", " << speeds[i][2];
  }
  Serial << ",   " << speeds[4][0] << ", " << speeds[4][1] << ";\n";
}
