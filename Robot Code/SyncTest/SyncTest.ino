#include <Dynamixel2Arduino.h>

#define DXL_BUS_SERIAL 3
#define DXL_SERIAL Serial3                        // OpenCM9.04 EXP Board's DXL port Serial. (Serial for the DXL port on the OpenCM 9.04 board)
const uint8_t DXL_DIR_PIN = 22;                   // OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
ParamForSyncWriteInst_t sync_write_param;
ParamForSyncReadInst_t sync_read_param;
RecvInfoFromStatusInst_t read_result;
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

#define PRESENT_POSITION_AX 36
#define PRESENT_POSITION_XM 132
#define GOAL_POSITION_AX 30
#define GOAL_POSITION_XM 116

int pos = 0;

int ids[] = {1};
int n = 1;
int t = 0;
int y[4];

void setup() {
  dxl.begin(1000000);
  dxl.setPortProtocolVersion(2.0);
  Serial.begin(9600);
  delay(1000);

  sync_write_param.addr = GOAL_POSITION_XM;
  sync_write_param.length = 4;
  
  sync_read_param.addr = PRESENT_POSITION_XM;
  sync_read_param.length = 4;
  
  for (int i=0;i<n;i++) {
    sync_write_param.xel[i].id = ids[i];
    sync_read_param.xel[i].id = ids[i];
//    dxl.torqueOff(ids[i]);
//    dxl.setOperatingMode(ids[i], OP_POSITION);
//    dxl.torqueOn(ids[i]);
  }
  sync_write_param.id_count = n;
  sync_read_param.id_count = n;
}

void loop() {
  
//  delay(1000);
  pos = -pos;
//  int val = (pos + 180) * 4095.0 / 360.0; // XM
  int val = (pos + 150) * 1023.0 / 300.0; // AX
  t = millis();
  for (int i=0; i<1000; i++) {
   // dxl.syncRead(sync_read_param, read_result);
    // for (int i=0;i<n;i++) {
    //   memcpy(sync_write_param.xel[i].data, &val, 4);
    // }
    // dxl.syncWrite(sync_write_param);
//    dxl.writeControlTableItem(ControlTableItem::GOAL_POSITION, ids[0], val);
//    dxl.writeControlTableItem(ControlTableItem::GOAL_POSITION, ids[1], val);
//    dxl.writeControlTableItem(ControlTableItem::GOAL_POSITION, ids[2], val);
//    dxl.writeControlTableItem(ControlTableItem::GOAL_POSITION, ids[3], val);
   y[0] = dxl.readControlTableItem(ControlTableItem::PRESENT_POSITION, ids[0]);
//    y[1] = dxl.readControlTableItem(ControlTableItem::PRESENT_POSITION, ids[1]);
//    y[2] = dxl.readControlTableItem(ControlTableItem::PRESENT_POSITION, ids[2]);
//    y[3] = dxl.readControlTableItem(ControlTableItem::PRESENT_POSITION, ids[3]);
  }
  Serial.println(millis()-t);
  // for (int i=0;i<n;i++) {
  //   memcpy(&y[i], read_result.xel[i].data, read_result.xel[i].length);
  // }
  Serial.print(y[0]);Serial.print(", ");
  Serial.print(y[1]);Serial.print(", ");
  Serial.print(y[2]);Serial.print(", ");
  Serial.print(y[3]);Serial.println("");
  delay(1000);
//  dxl.writeControlTableItem(ControlTableItem::GOAL_POSITION, 1, val);
//  dxl.writeControlTableItem(ControlTableItem::GOAL_POSITION, 2, val);
}
