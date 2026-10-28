#include <Dynamixel2Arduino.h>

// ============================================================================
// --- CONFIGURATION ZONE ---
// ============================================================================

// 1. PIN & SERIAL
#define DXL_RX_PIN 16
#define DXL_TX_PIN 17
#define DXL_DIR_PIN 4 
#define BAUDRATE 1000000
const float PROTOCOL = 2.0;

// 2. DIMENSIONS
const float L1_SHORT = 19.64; const float L2_SHORT = 36.63; const float L3_SHORT = 33.7;
const float L1_LONG  = 19.7;  const float L2_LONG  = 46.0;  const float L3_LONG  = 42.0;
const float L1_THUMB = 39.16; const float L2_THUMB = 30.13; const float L3_THUMB = 30.74;

// 3. FINGER OFFSETS (เหลือแค่ 4 นิ้ว)
const float OFF_INDEX_X = 64.62;  const float OFF_INDEX_Y = 26.48; const float OFF_INDEX_Z = 0;
const float OFF_MIDDLE_X = 67.68; const float OFF_MIDDLE_Y = 0.0;  const float OFF_MIDDLE_Z = 0;
const float OFF_RING_X = 60.14;   const float OFF_RING_Y = -19.11; const float OFF_RING_Z = 0;
const float OFF_THUMB_X = 33.19;  const float OFF_THUMB_Y = 36.66; const float OFF_THUMB_Z = 10.64;

// 4. REFERENCE FRAME
const float REF_X = 130.0;  
const float REF_Y = 0.0;
const float REF_Z = -50.0;  

// 5. SAFETY
const float MIN_RADIUS_MM = 12.0; 

// ============================================================================
// STRUCTS & LIMITS
// ============================================================================
struct Point3D { float x; float y; float z; };
struct JointAngles { float theta1; float theta2; float theta3; };
struct MotorLimit { int id; float min_angle; float max_angle; bool invert; };
MotorLimit motors[18]; 

void setupLimits() {
  // นิ้วชี้/กลาง/นาง
  motors[1] = {1, 77.94, 178.0, false}; 
  motors[2] = {2, 73.16, 178.0, false}; 
  motors[3] = {3, 73.8, 178.0, false};
  motors[4] = {4, 122.0, 179.0, false};
  motors[5] = {5, 121.0, 179.0, true};
  motors[6] = {6, 121.0, 178.0, true};
  motors[7] = {7, 78.0, 178.0, false};
  motors[8] = {8, 85.90, 178.0, false};
  motors[13] = {13, 76.2, 179.0, false};
  motors[14] = {14, 88.8, 179.0, false};
  motors[15] = {15, 132.7, 177.0, false};
  motors[16] = {16, 110.0, 179.0, false};

  // --- นิ้วโป้ง (THUMB) ---
  // ID 11: Base Yaw (Invert)
  motors[11] = {11, 135.0, 178.0, true};
  
  // ID 12: Base Pitch (Normal) -> แก้ Min เป็น 75 เพื่อให้งอได้เยอะขึ้น
  motors[12] = {12, 75.0, 179.0, false}; 

  // ID 9: Mid Pitch (Normal) -> พระเอกของเรา (ข้อกลาง)
  motors[9] = {9, 81.65, 179.0, false}; 
  
  // ID 10: Tip Pitch (Normal) -> ข้อปลาย
  motors[10] = {10, 86.00, 177.0, false}; 

  // Wrist
  motors[17] = {17, 28.0, 178.0, false};
}

// ============================================================================
// LOGIC
// ============================================================================

float simulateServoAngle(int id, float ik_angle_deg) {
  float min_limit = motors[id].min_angle;
  float max_limit = motors[id].max_angle;
  bool invert = motors[id].invert;
  float target = 0.0;

  if (id == 5 || id == 6 || id == 15 || id == 4 || id == 11 || id == 12) {
    float center = (max_limit + min_limit) / 2.0;
    if (invert) target = center - ik_angle_deg; 
    else target = center + ik_angle_deg;
  }
  else {
    float bend = abs(ik_angle_deg);
    if (!invert) target = max_limit - bend;
    else target = min_limit + bend;
  }
  return target;
}

JointAngles computeIK(float x, float y, float z, float l1, float l2, float l3) {
  JointAngles sol;
  sol.theta1 = atan2(y, x);
  float r_proj = sqrt(x*x + y*y);
  float r_target = r_proj - l1; 
  float z_target = z;
  float dist_sq = r_target*r_target + z_target*z_target;
  float cos_theta3 = (dist_sq - l2*l2 - l3*l3) / (2 * l2 * l3);
  if (cos_theta3 > 1.0) cos_theta3 = 1.0; if (cos_theta3 < -1.0) cos_theta3 = -1.0;
  sol.theta3 = acos(cos_theta3); 
  float k1 = l2 + l3 * cos(sol.theta3); float k2 = l3 * sin(sol.theta3);
  float alpha = atan2(z_target, r_target); float beta = atan2(k2, k1);
  sol.theta2 = alpha + beta; 
  sol.theta1 *= 57.2958; sol.theta2 = -sol.theta2 * 57.2958; sol.theta3 *= 57.2958;
  return sol;
}

Point3D transformWithRefFrame(Point3D raw_spring, float finger_off_x, float finger_off_y, float finger_off_z) {
  Point3D target;
  float raw_x_mm = raw_spring.x * 1000.0;
  float raw_y_mm = raw_spring.y * 1000.0;
  float raw_z_mm = raw_spring.z * 1000.0;

  // Auto Flip Left->Right
  raw_y_mm = -raw_y_mm; 

  float radius = sqrt(raw_x_mm*raw_x_mm + raw_y_mm*raw_y_mm);
  bool safety_triggered = false;
  if (radius < MIN_RADIUS_MM) {
    float scale = MIN_RADIUS_MM / radius;
    raw_x_mm *= scale;
    raw_y_mm *= scale;
    safety_triggered = true;
  }

  float world_pos_x = REF_X + raw_x_mm;
  float world_pos_y = REF_Y + raw_y_mm; 
  float world_pos_z = REF_Z + raw_z_mm;

  target.x = world_pos_x - finger_off_x;
  target.y = world_pos_y - finger_off_y;
  target.z = world_pos_z - finger_off_z;
  
  if(safety_triggered) target.z += 0.0001; 
  return target;
}

// ============================================================================
// DEBUGGER (รองรับ ID 9 ของนิ้วโป้ง)
// ============================================================================
void debugFingerAngles(String name, Point3D raw, int id_base, int id_mid, int id_tip, 
                       float l1, float l2, float l3, 
                       float off_x, float off_y, float off_z) {
                
  Point3D target = transformWithRefFrame(raw, off_x, off_y, off_z);
  
  bool safety_hit = false;
  float fraction = target.z - (int)target.z;
  if (abs(fraction) > 0.00005 && abs(fraction) < 0.0002) { 
      safety_hit = true; target.z -= 0.0001; 
  }

  JointAngles ang = computeIK(target.x, target.y, target.z, l1, l2, l3);

  Serial.println("------------------------------------------------");
  Serial.print("👉 FINGER: "); Serial.print(name);
  if(safety_hit) Serial.println(" (⚠️ Safety Cylinder Triggered!)");
  else Serial.println("");

  if (isnan(ang.theta1) || isnan(ang.theta2) || isnan(ang.theta3)) {
    Serial.println("   ❌ STATUS: UNREACHABLE (Math Error)");
    return;
  }

  // --- คำนวณค่า Servo ---
  float sv_base = simulateServoAngle(id_base, ang.theta1);
  float sv_mid  = simulateServoAngle(id_mid, ang.theta2);
  float sv_tip  = simulateServoAngle(id_tip, ang.theta3);
  
  // กรณีพิเศษ: THUMB ให้คำนวณ ID 10 (Tip) ด้วย โดยให้ขยับตาม ID 9
  float sv_thumb_tip_extra = 0;
  if (name == "THUMB") {
     // ให้ ID 10 (ปลาย) ขยับด้วยมุมเดียวกับ ID 9 (ข้อกลาง)
     sv_thumb_tip_extra = simulateServoAngle(10, ang.theta3); 
  }

  // Check Limits & Display
  // 1. Base
  String st1 = (sv_base < motors[id_base].min_angle || sv_base > motors[id_base].max_angle) ? " [CLAMPED] ⚠️" : " [OK]";
  if(sv_base < motors[id_base].min_angle) sv_base = motors[id_base].min_angle; 
  if(sv_base > motors[id_base].max_angle) sv_base = motors[id_base].max_angle;

  // 2. Mid
  String st2 = (sv_mid < motors[id_mid].min_angle || sv_mid > motors[id_mid].max_angle) ? " [CLAMPED] ⚠️" : " [OK]";
  if(sv_mid < motors[id_mid].min_angle) sv_mid = motors[id_mid].min_angle; 
  if(sv_mid > motors[id_mid].max_angle) sv_mid = motors[id_mid].max_angle;

  // 3. Tip (Joint 3)
  String st3 = (sv_tip < motors[id_tip].min_angle || sv_tip > motors[id_tip].max_angle) ? " [CLAMPED] ⚠️" : " [OK]";
  if(sv_tip < motors[id_tip].min_angle) sv_tip = motors[id_tip].min_angle; 
  if(sv_tip > motors[id_tip].max_angle) sv_tip = motors[id_tip].max_angle;

  Serial.print("   Target(mm): "); Serial.print(target.x,1); Serial.print(", "); Serial.print(target.y,1); Serial.print(", "); Serial.println(target.z,1);
  Serial.println("   [MATH vs REALITY]");
  
  Serial.print("   1. Base (ID "); Serial.print(id_base); Serial.print("): Math="); Serial.print(ang.theta1, 1); 
  Serial.print(" deg -> Real="); Serial.print(sv_base, 1); Serial.println(st1);
  
  Serial.print("   2. Mid  (ID "); Serial.print(id_mid); Serial.print("): Math="); Serial.print(ang.theta2, 1); 
  Serial.print(" deg -> Real="); Serial.print(sv_mid, 1); Serial.println(st2);
  
  Serial.print("   3. Joint(ID "); Serial.print(id_tip); Serial.print("): Math="); Serial.print(ang.theta3, 1); 
  Serial.print(" deg -> Real="); Serial.print(sv_tip, 1); Serial.println(st3);

  // แสดงผล ID 10 ของนิ้วโป้ง
  if (name == "THUMB") {
    String st4 = (sv_thumb_tip_extra < motors[10].min_angle || sv_thumb_tip_extra > motors[10].max_angle) ? " [CLAMPED] ⚠️" : " [OK]";
    if(sv_thumb_tip_extra < motors[10].min_angle) sv_thumb_tip_extra = motors[10].min_angle;
    if(sv_thumb_tip_extra > motors[10].max_angle) sv_thumb_tip_extra = motors[10].max_angle;
    
    Serial.print("   4. Tip  (ID 10): Coupled -> Real="); Serial.print(sv_thumb_tip_extra, 1); Serial.println(st4);
  }
  
  Serial.println("------------------------------------------------");
}

void setup() {
  Serial.begin(115200);
  while(!Serial);
  
  Serial.println("\n=== SAFETY DRY RUN SIMULATION (NO PINKY, FIX THUMB ID9) ===");
  setupLimits(); 

  // Input Data (4 Points)
  Point3D raw_index  = { 0.0167, -0.0233,  0.0867};
  Point3D raw_middle = { 0.0290,  0.0196,  0.1001};
  Point3D raw_ring   = { 0.0178,  0.0407,  0.0954};
  Point3D raw_thumb  = {-0.0473,  0.0097,  0.0957};

  // Run Simulation (4 Fingers)
  // Index: Base(5), Mid(7), Tip(8)
  debugFingerAngles("INDEX",  raw_index,  5, 7, 8,    L1_SHORT, L2_SHORT, L3_SHORT, OFF_INDEX_X, OFF_INDEX_Y, OFF_INDEX_Z);
  
  // Middle: Base(6), Mid(16), Tip(1)
  debugFingerAngles("MIDDLE", raw_middle, 6, 16, 1,   L1_LONG,  L2_LONG,  L3_LONG,  OFF_MIDDLE_X, OFF_MIDDLE_Y, OFF_MIDDLE_Z);
  
  // Ring: Base(15), Mid(14), Tip(13)
  debugFingerAngles("RING",   raw_ring,   15, 14, 13, L1_LONG,  L2_LONG,  L3_LONG,  OFF_RING_X, OFF_RING_Y, OFF_RING_Z);
  
  // Thumb: Base(11), Base2(12), Mid(9) <-- แก้ตรงนี้เป็น ID 9
  // ส่วน ID 10 จะถูกคำนวณอัตโนมัติในฟังก์ชัน debugFingerAngles เมื่อชื่อเป็น "THUMB"
  debugFingerAngles("THUMB",  raw_thumb,  11, 12, 9,  L1_THUMB, L2_THUMB, L3_THUMB, OFF_THUMB_X, OFF_THUMB_Y, OFF_THUMB_Z);
}

void loop() {
  // ไม่ทำอะไร
}