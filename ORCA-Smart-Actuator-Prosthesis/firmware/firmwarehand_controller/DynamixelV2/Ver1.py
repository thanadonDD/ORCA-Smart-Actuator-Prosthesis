#include <Arduino.h> 
#include <Dynamixel2Arduino.h>

// --- Config ---
#define DXL_RX_PIN 16
#define DXL_TX_PIN 17
#define DXL_DIR_PIN 4 

const float DXL_PROTOCOL_VERSION = 2.0;
const long DXL_BAUDRATE = 1000000;

Dynamixel2Arduino dxl(Serial2, DXL_DIR_PIN);

// --- Function Prototypes ---
void performClench();
void performOpen();
void performHome();
void performPose1(); // <-- ท่าใหม่ 1
void performPose2(); // <-- ท่าใหม่ 2
int deg2val(float degree);
void moveMotor(int id, float angle);

// --- Data Structure (เพิ่ม 2 ช่องใหม่) ---
struct FingerJoint {
  uint8_t id;
  float angle_open;   
  float angle_close;
  float angle_home;   
  float angle_pose1;  // <-- เพิ่มช่อง Pose 1
  float angle_pose2;  // <-- เพิ่มช่อง Pose 2
};

// ==========================================================================================
// ตารางข้อมูลมอเตอร์ 17 ตัว (เพิ่มคอลัมน์ขวาสุด 2 ช่อง)
// ==========================================================================================
// Mapping ตัวอย่าง:
// Pose 1 = ชูนิ้วชี้ (Pointing) 
// Pose 2 = ชูสองนิ้ว (Peace) 
// ==========================================================================================
FingerJoint joints[] = {
  // ID,  Open,    Close,   HOME,    POSE 1 grip(กด3), POSE 2(กด4)
  {1,   179.0,   75.94,   150.0,   80.00,       179.0},  // Middle Tip (P1=กำ, P2=แบ)
  {2,   178.0,   71.54,   150.0,   80.00,       71.54},  // Pinky Mid  (P1=กำ, P2=กำ)
  {3,   177.0,   73.0,    150.0,   80.0,        73.0 },  // Pinky Tip  (P1=กำ, P2=กำ)
  {4,   178.0,   121.0,   135.0,   150.0,       121.0},  // Pinky Base (Safe)
  {5,   179.0,   121.0,   160.0,   160.0,       179.0},  // Index Base (P1=ตรง, P2=กางออก)
  {6,   178.0,   121.0,   150.0,   150.0,       160.0},  // Middle Base(P1=หุบ, P2=กางออก)
  {7,   178.0,   75.3,    150.0,   80.00,       178.0},  // Index Mid  (P1=แบ, P2=แบ)
  {8,   178.0,   76.29,   150.0,   100.0,       178.0},  // Index Tip  (P1=แบ, P2=แบ)
  {9,   179.0,   81.65,   150.0,   100.0,       81.65},  // Thumb Mid  (กำโป้งทับ)
  {10,  179.0,   79.0,    150.0,   80.0,        79.0 },  // Thumb Tip  (กำโป้งทับ)
  {11,  134.0,   178.0,   145.0,   140.0,       178.0},  // Thumb Pivot(พับเก็บ)
  {12,  179.0,   79.0,    150.0,   130.0,        79.0 },  // Thumb Base (พับเก็บ)
  {13,  179.0,   76.2,    150.0,   80.0,        76.2 },  // Ring Tip   (P1=กำ, P2=กำ)
  {14,  179.0,   75.4,    150.0,   80.0,        75.4 },  // Ring Mid   (P1=กำ, P2=กำ)
  {15,  180.0,   238.0,   215.0,   215.0,       238.0},  // Ring Base  (Safe)
  {16,  179.0,   75.3,    130.0,   80.0,        179.0},  // Middle Mid (P1=กำ, P2=แบ)
  {17,  60.0,    178.0,   120.0,   120.0,       120.0}   // Wrist
};

const int TOTAL_JOINTS = sizeof(joints) / sizeof(joints[0]);

int deg2val(float degree) {
  return (int)((degree / 360.0) * 4095);
}

void moveMotor(int id, float angle) {
  dxl.setGoalPosition(id, deg2val(angle));
}

// ฟังก์ชันช่วยหมุนไปตามท่าที่ต้องการ (เพิ่ม case 3 และ 4)
void setJointState(int id, int state) {
  for(int i=0; i<TOTAL_JOINTS; i++) {
    if(joints[i].id == id) {
      float target;
      if(state == 0) target = joints[i].angle_open;
      else if(state == 1) target = joints[i].angle_close;
      else if(state == 2) target = joints[i].angle_home;
      else if(state == 3) target = joints[i].angle_pose1; // Pose 1
      else if(state == 4) target = joints[i].angle_pose2; // Pose 2
      
      moveMotor(id, target);
      return;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.setPins(DXL_RX_PIN, DXL_TX_PIN);
  dxl.begin(DXL_BAUDRATE);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  Serial.setTimeout(10); 

  Serial.println("Initializing Orca Hand...");
  
  for (int i = 0; i < TOTAL_JOINTS; i++) {
    uint8_t id = joints[i].id;
    if(dxl.ping(id)) {
       dxl.torqueOff(id);
       dxl.setOperatingMode(id, OP_POSITION);
       dxl.torqueOn(id);
       dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, id, 300); 
       Serial.print("ID: "); Serial.print(id); Serial.println(" OK!");
    }
  }
  
  Serial.println("-----------------------------------------");
  Serial.println("CONTROL MENU:");
  Serial.println(" [1] -> CLENCH (กำมือ)");
  Serial.println(" [0] -> OPEN (แบมือ)");
  Serial.println(" [h] -> HOME (ท่าพัก)");
  Serial.println(" [3] -> POSE 1 (Custom 1)");
  Serial.println(" [4] -> POSE 2 (Custom 2)");
  Serial.println(" [ID,Angle] -> Manual (e.g. 5,120)");
  Serial.println("-----------------------------------------");
}

void loop() {
  if (Serial.available() > 0) {
    char incoming = Serial.peek(); 

    if (isDigit(incoming)) {
      int val1 = Serial.parseInt(); 
      
      if (Serial.read() == ',') {
        int val2 = Serial.parseInt();
        Serial.print("Manual Move ID: "); Serial.print(val1);
        Serial.print(" to Angle: "); Serial.println(val2);
        moveMotor(val1, (float)val2);
      } 
      else {
        // --- ส่วนควบคุมปุ่มตัวเลข ---
        if (val1 == 1) {
          Serial.println("Action: CLENCH");
          performClench();
        } else if (val1 == 0) {
          Serial.println("Action: OPEN");
          performOpen();
        } else if (val1 == 3) {
          Serial.println("Action: POSE 1");
          performPose1();
        } else if (val1 == 4) {
          Serial.println("Action: POSE 2");
          performPose2();
        }
      }
      while(Serial.available()) Serial.read(); 
    }
    else {
      char cmd = Serial.read();
      if (cmd == 'h' || cmd == 'H') {
        Serial.println("Action: HOME");
        performHome();
      }
      while(Serial.available()) Serial.read();
    }
  }
}

// =========================================================
// ACTION Functions
// =========================================================

void performHome() {
  // สั่งทุกตัวไปที่ angle_home
  // ยกเว้นตัว Spread ให้ทำทีหลังเพื่อความปลอดภัย
  for(int i=0; i<TOTAL_JOINTS; i++) {
    int id = joints[i].id;
    if(id != 4 && id != 5 && id != 6 && id != 15) {
      setJointState(id, 2); // 2 = Home
    }
  }
  delay(200);
  int spreads[] = {4, 15, 6, 5};
  for(int id : spreads) setJointState(id, 2);
}

void performPose1() {
  // POSE 1: สั่งทุกตัวไปที่ angle_pose1 (State 3)
  // เพื่อความปลอดภัย: สั่ง Spread ก่อน แล้วค่อยตามด้วยนิ้ว
  // (หรือถ้ากลัวชน สั่งนิ้วก่อนก็ได้ ขึ้นอยู่กับท่า)
  // อันนี้ผมสั่งนิ้วก่อนครับ
  for(int i=0; i<TOTAL_JOINTS; i++) {
    int id = joints[i].id;
    if(id != 4 && id != 5 && id != 6 && id != 15) {
      setJointState(id, 3); 
    }
  }
  delay(200);
  int spreads[] = {4, 15, 6, 5};
  for(int id : spreads) setJointState(id, 3);
}

void performPose2() {
  // POSE 2: สั่งทุกตัวไปที่ angle_pose2 (State 4)
  for(int i=0; i<TOTAL_JOINTS; i++) {
    int id = joints[i].id;
    if(id != 4 && id != 5 && id != 6 && id != 15) {
      setJointState(id, 4); 
    }
  }
  delay(200);
  int spreads[] = {4, 15, 6, 5};
  for(int id : spreads) setJointState(id, 4);
}

void performClench() {
  int spreads[] = {4, 15, 6, 5};
  for(int id : spreads) setJointState(id, 1);
  delay(300);

  setJointState(2, 1); setJointState(3, 1); delay(100);
  setJointState(14, 1); setJointState(13, 1); delay(100);
  setJointState(16, 1); setJointState(1, 1); delay(100);
  setJointState(7, 1); setJointState(8, 1); delay(100);

  setJointState(11, 1); delay(150);
  setJointState(12, 1); setJointState(9, 1); setJointState(10, 1);
}

void performOpen() {
  setJointState(10, 0); setJointState(9, 0); setJointState(12, 0);
  delay(100);
  setJointState(11, 0); 
  delay(100);

  int fingers[] = {8,7, 1,16, 13,14, 3,2};
  for(int id : fingers) setJointState(id, 0);
  
  delay(300); 

  int spreads[] = {4, 15, 6, 5};
  for(int id : spreads) setJointState(id, 0);
}