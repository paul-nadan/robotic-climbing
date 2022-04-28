// Check for user commands
void checkSerial() {
  if (Serial.available() == 0) return;
  char command = Serial.read();
  int id;
  float val;
  switch (command) {
    case 'o': // toggle controller
      if (control == OFF) {
        setController(DEFAULT_CONTROL);
        Serial << "Controller on\n";
      }
      else {
        setController(OFF);
        Serial << "Controller off\n";
      }
      break;
    case ' ': // halt
      setBehavior(IDLE);
      Serial << "Halt\n";
      break;
    case 'a': // stand
      setBehavior(IDLE);
      stand();
      step = 0;
      Serial << "Stand\n";
      break;
    case 's': // sprawl
      setBehavior(IDLE);
      sprawl();
      step = 0;
      Serial << "Sprawl\n";
      break;
    case 'w': // walk
      setBehavior(WALK);
      goalSteps = 0;
      Serial << "Beginning walk\n";
      break;
    case 'c': // climb (1 step)
      setBehavior(CLIMB);
      goalSteps = step + 1;
      Serial << "Climb: beginning step " << goalSteps << "\n";
      break;
    case 'i': // sillywalk
      setBehavior(SILLYWALK);
      goalSteps = 0;
      Serial << "Silly walk\n";
      break;
    case 'm': // move motor
      id = Serial.parseInt();
      if (id == 0 || Serial.available() == 0) break;
      val = Serial.parseFloat();
      moveMotor(id, val);
      if (id <= 12) setpoints[(id - 1) % 4][int((id - 1) / 4)] = val;
      else setpoints[4][id - 13] = val;
      Serial << "Moving motor " << id << " to angle " << val << " deg\n";
      break;
    case 't': // set motor torque
      id = Serial.parseInt();
      if (id == 0 || Serial.available() == 0) break;
      val = Serial.parseFloat();
      limitTorque(id, val);
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
  enableTorque();
  t0 = t;
  tb = t;
}

// Set the current display variable
void setDisplay(display d) {
  output = d;
}

// Display measured positions for each foot
void printPositions() {
  // TODO
}

// Display measured angles for each motor
void printAngles() {
  // TODO
}

// Display measured forces for each foot (17ms)
void printForces() {
  Serial.print(millis());
  Serial.print(",     ");
  for (int i = 0; i < 4; i++) {
    float *f = getFootForce(i + 1);
    Serial.print(f[0]);
    Serial.print(", ");
    Serial.print(f[1]);
    Serial.print(", ");
    Serial.print(f[2]);
    Serial.print(",     ");
  }
  Serial.println("");
}

// Display measured torques for each motor
void printTorques() {
  // TODO
}
