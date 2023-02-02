#include <actuator.h>
#include <Dynamixel2Arduino.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;

// Motor parameters
#define ID 1
#define XM true
#define MIN_LIMIT -60
#define MAX_LIMIT 90

// OpenCM initialization
#define DXL_BUS_SERIAL 3
#define DXL_SERIAL Serial3                        // OpenCM9.04 EXP Board's DXL port Serial. (Serial for the DXL port on the OpenCM 9.04 board)
const uint8_t DXL_DIR_PIN = 22;                   // OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
Dynamixel2Arduino Dxl(DXL_SERIAL, DXL_DIR_PIN);

// Output display
bool printStatus = true;        // Print status messages
bool printState = false;        // Print motor state messages
bool printControl = true;       // Print motor state messages

// State variables
int time = 0;                   // Time since last status message (ms)
int loops = 0;                  // Number of loops since last status message
long t;                         // Timestamp of previous loop (ms)
int dt;                         // Duration of previous loop (ms)
float angle;                    // Current motor angle (deg)
float speed;                    // Current motor angular velocity (deg/s)
float torque;                   // Current motor torque usage (Nmm)
float goal_angle;               // Motor angle output (deg)
float goal_speed;               // Motor angular velocity limit (deg/s)
float goal_torque;              // Motor torque limit (Nmm)
float setpoint;                 // Desired motor angle (deg)
float default_angle = 0;        // Default motor angle setpoint (deg)
float default_speed = 40;       // Default motor speed (deg)
float default_torque = 1000;    // Default motor torque (Nmm)

// Controller state
bool impedance = false;         // Whether the impedance controller is active
float kP = .01;                 // Controller proportional gain
float kI = 0;                   // Controller integral gain
float kD = 0;                   // Controller derivative gain
float overshoot = 0.1;         // Controller angle overshoot to induce torque
float integralErr = 0;          // Integral of error over time
float integralErr2 = 0;         // Integral of error over time
float prevErr = 0;              // Previous error
float prevErr2 = 0;             // Previous error
float output = 0;               // Controller output

void setup() {
  Dxl.begin(1000000);
  Dxl.setPortProtocolVersion(2.0);
  Serial.begin(1000000);
  delay(1000);
  setpoint = default_angle;
  goal_speed = default_speed;
  goal_torque = default_torque;
  enable(ID, XM);
}

void loop() {
  // Print status message (voltage | loop frequency | loop duration)
  loops++;
  if (millis() - time > 1000) {
    if (printStatus) Serial << "Status: " << readVoltage(ID, XM) << " V | " << int(loops * 1000.0 / (millis() - time)) << " Hz | " << int((millis() - time) * 1.0 / loops) << " ms\n";
    time = millis();
    loops = 0;
  }

  // Read user input and motor state
  checkSerial();
  angle = readAngle(ID, XM);
  speed = readSpeed(ID, XM);
  torque = readTorque(ID, XM);
  if (printState) Serial << int(millis()) << ", " << angle << ", " << speed << ", " << torque << ";\n";

  // Compute time since last update
  dt = millis() - t;
  t = millis();

  // Compute desired motor output
  if (impedance) {
    impedanceControl();
    torqueControl();
    delay(20);
  } else {
    goal_angle = setpoint;
  }

  // Write motor output
  writeSpeed(ID, goal_speed, XM);
  writeTorque(ID, goal_torque, XM);
  writeAngle(ID, goal_angle, XM);
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
    Dxl.setGoalCurrent(id, torque * 2.3 / 4.1 / 2.69);
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
  int read;
  switch (command) {
    case ' ': // toggle controller
      if (!impedance) {
        impedance = true;
        integralErr = 0;
        Serial << "Controller on\n";
      }
      else {
        impedance = false;
        setpoint = default_angle;
        goal_speed = default_speed;
        goal_torque = default_torque;
        Serial << "Controller off\n";
      }
      break;
    case 'e': // enable torque
      enable(ID, XM);
      break;
    // case 'd': // toggle status display
    //   printStatus = !printStatus;
    //   break;
    case 'p': // toggle motor state display
      printState = !printState;
      break;
    case 'c': // toggle controller output display
      printControl = !printControl;
      break;
    case 'd': // set motor kD gain
      if (Serial.available() == 0) break;
      val = 0.0 + Serial.parseInt();
      Dxl.setPortProtocolVersion(2.0);
      read = Dxl.readControlTableItem(ControlTableItem::POSITION_D_GAIN, ID);
      Serial << "Old = " << read << "\n";
      Dxl.writeControlTableItem(ControlTableItem::POSITION_D_GAIN, ID, int(val));
      Serial << "Motor kD = " << int(val) << "\n";
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
      if (command == 'o' || command == 'O') {
        overshoot = val;
      }
      Serial << "Gains: kP = " << kP << ", kI = " << kI << ", kD = " << kD << ", kO = " << overshoot << "\n";
      break;
    case 'm': // set motor angle setpoint
      if (Serial.available() == 0) break;
      setpoint = Serial.parseFloat();
      Serial << "Moving motor " << ID << " to angle " << setpoint << " deg\n";
      break;
    case 'v': // set motor angular velocity limit
      if (Serial.available() == 0) break;
      goal_speed = Serial.parseFloat();
      Serial << "Setting motor " << ID << " to velocity " << goal_speed << " deg/s\n";
      break;
    case 't': // set motor torque limit
      if (Serial.available() == 0) break;
      goal_torque = Serial.parseFloat();
      Serial << "Setting motor " << ID << " to torque " << goal_torque << " N-mm\n";
      break;
  }
}

// Drive motor angle to the setpoint using impedance-based feedback control
void impedanceControl() {
  float err = setpoint - angle;
  integralErr += err * dt / 1000;
  output = 100 * err + 0 * integralErr + 2 * (err - prevErr) * 1000 / dt;
  prevErr = err;
  goal_torque = abs(output);
  // if (-(goal_angle - setpoint) * output < 0) {
  //   goal_angle = goal_angle;
  // } else {
  //   goal_angle = output > 0 ? goal_angle + overshoot : goal_angle - overshoot;
  // }
  // goal_angle = min(max(goal_angle, MIN_LIMIT), MAX_LIMIT);
  // if (printControl) Serial << err << ", " << goal_angle << ", " << goal_torque << ";\n";
}


// Drive motor torque to the setpoint using feedback control
void torqueControl() {
  float err = output - torque;
  integralErr2 += err * dt / 1000;
  float output2 = kP * err + kI * integralErr2 + kD * (err - prevErr2) * 1000 / dt - goal_angle;
  prevErr2 = err;
  // output = max(min(output, overshoot), -overshoot);
  // goal_angle += output2;
  goal_angle = setpoint + output2;
  // if ((goal_angle - setpoint) * output < 0) {
  //   goal_angle = goal_angle;
  // } else {
  //   goal_angle = output > 0 ? goal_angle + overshoot : goal_angle - overshoot;
  // }
  goal_angle = min(max(goal_angle, MIN_LIMIT), MAX_LIMIT);
  if (printControl) Serial << err << ", " << goal_angle << ", " << goal_torque << ";\n";
}
