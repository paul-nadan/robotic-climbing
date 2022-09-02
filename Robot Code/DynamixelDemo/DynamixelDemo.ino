/* Demo script for using Dynamixel motors (XM-430-W350-T and AX-18A) */

#include <actuator.h>
#include <Dynamixel2Arduino.h>

// Motor parameters
#define ID 1                                      // ID number of the motor
#define XM true                                   // Whether the motor is XM (true) or AX (false) series
#define MIN_LIMIT -60                             // Lower angle limit
#define MAX_LIMIT 90                              // Upper angle limit

// OpenCM initialization
#define DXL_BUS_SERIAL 3
#define DXL_SERIAL Serial3                        // OpenCM9.04 EXP Board's DXL port Serial. (Serial for the DXL port on the OpenCM 9.04 board)
const uint8_t DXL_DIR_PIN = 22;                   // OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
Dynamixel2Arduino Dxl(DXL_SERIAL, DXL_DIR_PIN);

// State variables
float angle;                    // Current motor angle (deg)
float speed;                    // Current motor angular velocity (deg/s)
float torque;                   // Current motor torque usage (Nmm)
float goal_angle = 0;           // Motor angle output (deg)
float goal_speed = 0;           // Motor angular velocity limit (deg/s)
float goal_torque = 3000;       // Motor torque limit (Nmm)

void setup() {
  Dxl.begin(1000000);
  Dxl.setPortProtocolVersion(2.0);
  Serial.begin(1000000);
  enable(ID, XM);
}

void loop() {
  // Read user input
  checkSerial();

  // Read motor state
  angle = readAngle(ID, XM);
  speed = readSpeed(ID, XM);
  torque = readTorque(ID, XM);

  // Print motor state
  Serial.print("Angle: ");
  Serial.print(angle);
  Serial.print(", Speed: ");
  Serial.print(speed);
  Serial.print(", Torque: ");
  Serial.println(torque);

  // Write motor output
  writeAngle(ID, goal_angle, XM);
  writeSpeed(ID, goal_speed, XM);
  writeTorque(ID, goal_torque, XM);
}

// Enable motor torque and reset mode to current-based position (XM only)
void enable(int id, bool xm) {
  if (xm) {
    Dxl.setPortProtocolVersion(2.0);
    Dxl.torqueOff(id);
    Dxl.setOperatingMode(id, OP_CURRENT_BASED_POSITION);
    Dxl.torqueOn(id);
  } else {
    Dxl.setPortProtocolVersion(1.0);
    Dxl.torqueOn(id);
  }
}

// Disable motor torque
void disable(int id, bool xm) {
  if (xm) {
    Dxl.setPortProtocolVersion(2.0);
    Dxl.torqueOff(id);
  } else {
    Dxl.setPortProtocolVersion(1.0);
    Dxl.torqueOff(id);
  }
}

// Get current motor supply voltage (i.e. battery voltage)
float readVoltage(int id, bool xm) {
  if (xm) {
    Dxl.setPortProtocolVersion(2.0);
    return (Dxl.readControlTableItem(ControlTableItem::PRESENT_INPUT_VOLTAGE, id) / 10.0);
  } else {
    Dxl.setPortProtocolVersion(1.0);
    return (Dxl.readControlTableItem(ControlTableItem::PRESENT_INPUT_VOLTAGE, id) / 10.0);
  }
}

// Get current motor position (deg)
float readAngle(int id, bool xm) {
  if (xm) {
    Dxl.setPortProtocolVersion(2.0);
    int pos = Dxl.readControlTableItem(ControlTableItem::PRESENT_POSITION, id);
    return (pos / 4095.0 * 360 - 180);
  } else {
    Dxl.setPortProtocolVersion(1.0);
    int pos = Dxl.readControlTableItem(ControlTableItem::PRESENT_POSITION, id);
    return (pos / 1023.0 * 300 - 150);
  }
}

// Get current motor angular velocity (deg/s)
float readSpeed(int id, bool xm) {
  if (xm) {
    Dxl.setPortProtocolVersion(2.0);
    int speed = Dxl.readControlTableItem(ControlTableItem::PRESENT_VELOCITY, id);
    if (speed >= 32768) speed -= 32768 * 2;
    return (speed * 0.229 * 6);
  } else {
    Dxl.setPortProtocolVersion(1.0);
    int speed = Dxl.readControlTableItem(ControlTableItem::PRESENT_SPEED, id);
    if (speed >= 1024) speed = -(speed - 1024);
    return (speed * 0.111 * 6);
  }
}

// Get current motor torque output (Nmm)
float readTorque(int id, bool xm) {
  if (xm) {
    Dxl.setPortProtocolVersion(2.0);
    int current = Dxl.readControlTableItem(ControlTableItem::PRESENT_CURRENT, id);
    if (current >= 32768) current -= 32768 * 2;
    return (current * 2.69 * 4.1 / 2.3);
  } else {
    Dxl.setPortProtocolVersion(1.0);
    int load = Dxl.readControlTableItem(ControlTableItem::PRESENT_LOAD, id);
    if (load >= 1024) load = -(load - 1024);
    return (load / 1023.0 * 1800);
  }
}

// Set motor position (deg)
void writeAngle(int id, float angle, bool xm) {
  if (angle < MIN_LIMIT) angle = MIN_LIMIT;
  if (angle > MAX_LIMIT) angle = MAX_LIMIT;
  if (xm) {
    Dxl.setPortProtocolVersion(2.0);
    Dxl.setGoalPosition(id, (angle + 180) * 4095.0 / 360.0);
  } else {
    Dxl.setPortProtocolVersion(1.0);
    Dxl.setGoalPosition(id, (angle + 150) * 1023.0 / 300.0);
  }
}

// Set motor angular velocity limit (deg/s)
void writeSpeed(int id, float speed, bool xm) {
  if (xm) {
    Dxl.setPortProtocolVersion(2.0);
    Dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, id, speed / 0.229 / 6);
  } else {
    Dxl.setPortProtocolVersion(1.0);
    Dxl.writeControlTableItem(ControlTableItem::MOVING_SPEED, id, speed / 0.111 / 6);
  }
}

// Set motor torque limit (Nmm)
void writeTorque(int id, float torque, bool xm) {
  if (xm) {
    Dxl.setPortProtocolVersion(2.0);
    // Dxl.setGoalCurrent(id, torque * 2.3 / 4.1 / 2.69); // for XM430-W350-T
    Dxl.setGoalCurrent(id, torque * 2.3 / 3.0 / 2.69); // for XM430-W210-T
  } else {
    Dxl.setPortProtocolVersion(1.0);
    Dxl.writeControlTableItem(ControlTableItem::TORQUE_LIMIT, id, torque / 1800 * 1023);
  }
}

// Check for user commands
void checkSerial() {
  if (Serial.available() == 0) return;
  char command = Serial.read();
  float val;
  switch (command) {
    case ' ': // E-stop
      disable(ID, XM);
      goal_angle = angle;
      break;
    case 'e': // enable torque
      enable(ID, XM);
      break;
    case 'm': // set motor angle setpoint
      if (Serial.available() == 0) break;
      goal_angle = Serial.parseFloat();
      Serial.print("Moving motor angle to ");
      Serial.print(goal_angle);
      Serial.println(" deg");
      break;
    case 'v': // set motor angular velocity limit
      if (Serial.available() == 0) break;
      goal_speed = Serial.parseFloat();
      Serial.print("Setting motor speed to ");
      Serial.print(goal_speed);
      Serial.println(" deg/s");      
      break;
    case 't': // set motor torque limit
      if (Serial.available() == 0) break;
      goal_torque = Serial.parseFloat();
      Serial.print("Setting motor speed to ");
      Serial.print(goal_torque);
      Serial.println(" Nmm");
      break;
  }
}