#include <Dynamixel2Arduino.h>

// ============================================================================
// --- 1. CONFIGURATION ZONE ---
// ============================================================================

// 🔥 HARDWARE CONNECTION SETTINGS 🔥
#define DXL_RX_PIN 16
#define DXL_TX_PIN 17
#define DXL_DIR_PIN 4 
#define BAUDRATE 1000000  // ความเร็ว DYNAMIXEL (1 Mbps)
const float PROTOCOL = 2.0;

// DIMENSIONS
const float L1_SHORT = 19.64; const float L2_SHORT = 36.63; const float L3_SHORT = 33.7;
const float L1_LONG  = 19.7;  const float L2_LONG  = 46.0;  const float L3_LONG  = 42.0;
const float L1_THUMB = 39.16; const float L2_THUMB = 30.13; const float L3_THUMB = 30.74;

// FINGER OFFSETS
const float OFF_INDEX_X = 64.62;  const float OFF_INDEX_Y = 26.48; const float OFF_INDEX_Z = 0;
const float OFF_MIDDLE_X = 67.68; const float OFF_MIDDLE_Y = 0.0;  const float OFF_MIDDLE_Z = 0;
const float OFF_RING_X = 60.14;   const float OFF_RING_Y = -19.11; const float OFF_RING_Z = 0;
const float OFF_THUMB_X = 33.19;  const float OFF_THUMB_Y = 36.66; const float OFF_THUMB_Z = 10.64;

// REF FRAME (ระยะจับ 95mm)
const float REF_X = 115;  
const float REF_Y = 0.0;
const float REF_Z = -50.0; 

// SAFETY
const float MIN_RADIUS_MM = 12.0; 

// ============================================================================
// --- 2. GLOBAL OBJECTS & STRUCTS ---
// ============================================================================ 

// สร้าง Object เพื่อคุยกับมอเตอร์ผ่าน Serial2 (ESP32)
Dynamixel2Arduino dxl(Serial2, DXL_DIR_PIN);

struct Point3D { float x; float y; float z; };
struct JointAngles { float theta1; float theta2; float theta3; };
struct MotorLimit { int id; float min_angle; float max_angle; bool invert; };
MotorLimit motors[18]; 

void setupLimits() {
  motors[5] = {5, 121.0, 179.0, true};   
  motors[7] = {7, 78.0, 178.0, false};   
  motors[8] = {8, 85.90, 178.0, false};  

  motors[6] = {6, 121.0, 178.0, true};   
  motors[16] = {16, 110.0, 179.0, false};
  motors[1] = {1, 77.94, 178.0, false};  

  motors[15] = {15, 132.7, 177.0, false};
  motors[14] = {14, 88.8, 179.0, false}; 
  motors[13] = {13, 76.2, 179.0, false}; 

  motors[4] = {4, 122.0, 179.0, false};
  motors[2] = {2, 73.16, 178.0, false};
  motors[3] = {3, 73.8, 178.0, false};

  // THUMB (ID 11, 12, 9, 10)
  motors[11] = {11, 135.0, 178.0, true}; 
  motors[12] = {12, 75.0, 179.0, false}; 
  motors[9]  = {9, 81.65, 179.0, false}; 
  motors[10] = {10, 86.00, 177.0, false};

  motors[17] = {17, 28.0, 178.0, false};
}

// ============================================================================
// --- 3. LOGIC FUNCTIONS ---
// ============================================================================

int32_t convertIKToServo(int id, float ik_angle_deg) {
  float min_limit = motors[id].min_angle;
  float max_limit = motors[id].max_angle;
  bool invert = motors[id].invert;
  float target_angle_servo = 0.0;

  if (id == 5 || id == 6 || id == 15 || id == 4 || id == 11 || id == 12) {
    float center_servo = (max_limit + min_limit) / 2.0;
    if (invert) target_angle_servo = center_servo - ik_angle_deg; 
    else target_angle_servo = center_servo + ik_angle_deg;
  } else {
    float bend_amount = abs(ik_angle_deg);
    if (!invert) target_angle_servo = max_limit - bend_amount;
    else target_angle_servo = min_limit + bend_amount;
  }
  
  if (target_angle_servo > max_limit) target_angle_servo = max_limit;
  if (target_angle_servo < min_limit) target_angle_servo = min_limit;
  
  return (int32_t)(target_angle_servo * 4096.0 / 360.0);
}

JointAngles computeIK(float x, float y, float z, float l1, float l2, float l3) {
  JointAngles sol;
  sol.theta1 = atan2(y, x);
  
  float r_proj = sqrt(x*x + y*y);
  float r_target = r_proj - l1; 
  float z_target = z;
  float dist_sq = r_target*r_target + z_target*z_target;
  float cos_theta3 = (dist_sq - l2*l2 - l3*l3) / (2 * l2 * l3);
  
  if (cos_theta3 > 1.0) cos_theta3 = 1.0; 
  if (cos_theta3 < -1.0) cos_theta3 = -1.0;
  
  sol.theta3 = acos(cos_theta3); 
  float k1 = l2 + l3 * cos(sol.theta3); 
  float k2 = l3 * sin(sol.theta3);
  float alpha = atan2(z_target, r_target); 
  float beta = atan2(k2, k1);
  sol.theta2 = alpha + beta; 

  sol.theta1 *= 57.2958; 
  sol.theta2 = -sol.theta2 * 57.2958; 
  sol.theta3 *= 57.2958;
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
  if (radius < MIN_RADIUS_MM) {
    float scale = MIN_RADIUS_MM / radius;
    raw_x_mm *= scale;
    raw_y_mm *= scale;
  }

  float world_pos_x = REF_X + raw_x_mm;
  float world_pos_y = REF_Y + raw_y_mm; 
  float world_pos_z = REF_Z + raw_z_mm;

  target.x = world_pos_x - finger_off_x;
  target.y = world_pos_y - finger_off_y;
  target.z = world_pos_z - finger_off_z;
  
  return target;
}

void moveFinger(String name, Point3D raw, int id_base, int id_mid, int id_tip, 
                float l1, float l2, float l3, 
                float off_x, float off_y, float off_z, int mode) {
                
  Point3D target = transformWithRefFrame(raw, off_x, off_y, off_z);
  JointAngles ang = computeIK(target.x, target.y, target.z, l1, l2, l3);

  if (isnan(ang.theta1) || isnan(ang.theta2) || isnan(ang.theta3)) return;
  
  int32_t p_base = convertIKToServo(id_base, ang.theta1);
  int32_t p_mid  = convertIKToServo(id_mid, ang.theta2);
  int32_t p_tip  = convertIKToServo(id_tip, ang.theta3);

  if (mode == 0 || mode == 1) {
    dxl.setGoalPosition(id_base, p_base);
  }
  
  if (mode == 0 || mode == 2) {
    dxl.setGoalPosition(id_mid, p_mid);
    dxl.setGoalPosition(id_tip, p_tip);

    if (name == "THUMB") {
        int32_t p_thumb_tip_extra = convertIKToServo(10, ang.theta3);
        dxl.setGoalPosition(10, p_thumb_tip_extra);
    }
  }
}

// ============================================================================
// --- 4. SETUP (เชื่อมต่อฮาร์ดแวร์ตรงนี้) ---
// ============================================================================
void setup() {
  // 1. เริ่ม Debug Serial (จอคอม)
  Serial.begin(115200);
  
  // 2. ตั้งค่าขา DYNAMIXEL (ESP32)
  Serial2.setPins(DXL_RX_PIN, DXL_TX_PIN);
  
  // 3. เริ่มสื่อสารกับมอเตอร์
  dxl.begin(BAUDRATE);
  dxl.setPortProtocolVersion(PROTOCOL);
  
  // 4. โหลด Limit และเปิด Torque
  setupLimits();
  for(int i=1; i<=17; i++) {
    dxl.torqueOn(i);
    // ❌ ลบบรรทัดที่ Error ทิ้งไปแล้วครับ 
    // dxl.writeControlTableItem(ControlTableItem::Profile_Velocity, i, 30); 
  }
  
  Serial.println("System Ready! Homing...");
  
  // Homing
  for(int i=1; i<=17; i++) dxl.setGoalPosition(i, convertIKToServo(i, 0));
  delay(1000);
}

// ============================================================================
// --- 5. MAIN LOOP ---
// ============================================================================

void loop() {
  Point3D raw_index  = { 0.0167, -0.0233,  0.0867};
  Point3D raw_middle = { 0.0290,  0.0196,  0.1001};
  Point3D raw_ring   = { 0.0178,  0.0407,  0.0954};
  Point3D raw_thumb  = {-0.0473,  0.0097,  0.0957};

  Serial.println("\n--- STEP 1: SPREAD ---");
  moveFinger("INDEX",  raw_index,  5, 7, 8,    L1_SHORT, L2_SHORT, L3_SHORT, OFF_INDEX_X, OFF_INDEX_Y, OFF_INDEX_Z, 1);
  moveFinger("MIDDLE", raw_middle, 6, 16, 1,   L1_LONG,  L2_LONG,  L3_LONG,  OFF_MIDDLE_X, OFF_MIDDLE_Y, OFF_MIDDLE_Z, 1);
  moveFinger("RING",   raw_ring,   15, 14, 13, L1_LONG,  L2_LONG,  L3_LONG,  OFF_RING_X, OFF_RING_Y, OFF_RING_Z, 1);
  moveFinger("THUMB",  raw_thumb,  11, 12, 9,  L1_THUMB, L2_THUMB, L3_THUMB, OFF_THUMB_X, OFF_THUMB_Y, OFF_THUMB_Z, 1);
  
  delay(800);

  Serial.println("--- STEP 2: GRASP ---");
  moveFinger("INDEX",  raw_index,  5, 7, 8,    L1_SHORT, L2_SHORT, L3_SHORT, OFF_INDEX_X, OFF_INDEX_Y, OFF_INDEX_Z, 2);
  moveFinger("MIDDLE", raw_middle, 6, 16, 1,   L1_LONG,  L2_LONG,  L3_LONG,  OFF_MIDDLE_X, OFF_MIDDLE_Y, OFF_MIDDLE_Z, 2);
  moveFinger("RING",   raw_ring,   15, 14, 13, L1_LONG,  L2_LONG,  L3_LONG,  OFF_RING_X, OFF_RING_Y, OFF_RING_Z, 2);
  moveFinger("THUMB",  raw_thumb,  11, 12, 9,  L1_THUMB, L2_THUMB, L3_THUMB, OFF_THUMB_X, OFF_THUMB_Y, OFF_THUMB_Z, 2);

  delay(5000);
  
  Serial.println("--- RELEASE ---");
  for(int i=1; i<=17; i++) {
    // 🔥 เพิ่มบรรทัดนี้: บังคับเปิดแรงใหม่ กันเหนียวเผื่อมันหลุด
    dxl.torqueOn(i); 
    
    // สั่งกลับไปที่ 0
    dxl.setGoalPosition(i, convertIKToServo(i, 0));
  }
  delay(3000);
}