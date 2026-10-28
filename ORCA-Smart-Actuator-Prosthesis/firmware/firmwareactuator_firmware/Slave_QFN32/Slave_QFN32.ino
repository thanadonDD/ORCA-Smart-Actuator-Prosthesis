/* * =============================================================================
 * PROJECT: KPServo Smart Actuator (Professional Slave Node)
 * ARCHITECT: The Polyglot Architect x ธนดล พูลสระคู
 * VERSION: 6.4.0 (Dev Mode & Execution Profiler)
 * =============================================================================
 */

#include <Wire.h>
#include <EEPROM.h>

// ==========================================
// 🛠️ 1. DEV MODE CONFIGURATION
// ==========================================
#define DEV_MODE 1  // เซ็ตเป็น 0 เมื่อต้องการนำไปใช้งานจริง (Production)

#ifdef DEV_MODE
  #define DEBUG_PRINT(msg) printDebug(msg)
  #define DEBUG_PRINT_VAL(label, val) printDebugVal(label, val)
#else
  #define DEBUG_PRINT(msg)
  #define DEBUG_PRINT_VAL(label, val)
#endif

#define INST_PING       0x01
#define INST_READ       0x02
#define INST_WRITE      0x03
#define RETURN_DELAY_US 250   

const int SLAVE_TX_CONTROL = 8;
const int BIN1 = 2;            
const int BIN2 = 3;             
const int AS5600_ADDRESS = 0x36;

// --- Control Table ---
struct __attribute__((packed)) {
  uint8_t  id;                   
  float    kp;                   
  float    ki;                   
  float    kd;                   
  float    minLimit;             
  float    maxLimit;             
  uint8_t  pwmLimit;             
  uint8_t  torqueEnable;         
  float    goalPos;              
  float    presentPos;           
  float    presentError;         
  uint8_t  moving;               
  float    profileVelocity;      
  float    profileAcceleration;  
} reg;

// --- Trajectory & System Variables ---
float profilePos = 0;            
float profileVel = 0;
float lastError = 0, integral = 0;
unsigned long lastMicros = 0; 
long rotations = 0;
float lastRawDeg = -1.0;
bool wasTorqueEnabled = false;   

#ifdef DEV_MODE
unsigned long maxExecTime = 0;
unsigned long lastDebugPrint = 0;
#endif

// ==========================================
// --- Config Management ---
// ==========================================
void loadConfig() {
  EEPROM.get(0, reg.id);
  if(reg.id == 255) reg.id = 1;
  EEPROM.get(1, reg.kp);         if(isnan(reg.kp)) reg.kp = 8.5;
  EEPROM.get(5, reg.ki);         if(isnan(reg.ki)) reg.ki = 2.0;
  EEPROM.get(9, reg.kd);         if(isnan(reg.kd)) reg.kd = 1.5;
  EEPROM.get(13, reg.minLimit);  if(isnan(reg.minLimit)) reg.minLimit = 0;
  EEPROM.get(17, reg.maxLimit);  if(isnan(reg.maxLimit)) reg.maxLimit = 40960; 
  EEPROM.get(21, reg.pwmLimit);  if(reg.pwmLimit == 255 || reg.pwmLimit == 0) reg.pwmLimit = 255;
  EEPROM.get(36, reg.profileVelocity);     if(isnan(reg.profileVelocity)) reg.profileVelocity = 0;
  EEPROM.get(40, reg.profileAcceleration); if(isnan(reg.profileAcceleration)) reg.profileAcceleration = 0;
}

void saveConfig() {
  EEPROM.put(0, reg.id);
  EEPROM.put(1, reg.kp);
  EEPROM.put(5, reg.ki);
  EEPROM.put(9, reg.kd);
  EEPROM.put(13, reg.minLimit);
  EEPROM.put(17, reg.maxLimit);
  EEPROM.put(21, reg.pwmLimit);
  EEPROM.put(36, reg.profileVelocity);
  EEPROM.put(40, reg.profileAcceleration);
}

// 🛠️ 3. ระบบพ่นสถานะออก TX สำหรับ Logic Analyzer
void printDebug(const char* msg) {
  digitalWrite(SLAVE_TX_CONTROL, HIGH);
  delayMicroseconds(50);
  Serial.print(msg);
  Serial.flush();
  delayMicroseconds(50);
  digitalWrite(SLAVE_TX_CONTROL, LOW);
}

void printDebugVal(const char* label, long val) {
  digitalWrite(SLAVE_TX_CONTROL, HIGH);
  delayMicroseconds(50);
  Serial.print(label);
  Serial.print(val);
  Serial.print("\n");
  Serial.flush();
  delayMicroseconds(50);
  digitalWrite(SLAVE_TX_CONTROL, LOW);
}

// ==========================================
// --- Boot Sequence ---
// ==========================================
void setup() {
  Serial.begin(9600); // 📉 ลดเหลือ 9600 ตามที่คุณต้องการ
  Wire.begin();
  Wire.setClock(400000); 
  
  pinMode(SLAVE_TX_CONTROL, OUTPUT);
  digitalWrite(SLAVE_TX_CONTROL, LOW); 
  
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  stopMotor();
  
  loadConfig();
  reg.torqueEnable = 0;
  reg.moving = 0;
  
  reg.presentPos = getUnwrappedAngle();
  reg.goalPos = reg.presentPos;
  profilePos = reg.presentPos;
  lastMicros = micros();
  
  DEBUG_PRINT("ST:BOOT_OK\n");
}

// ==========================================
// --- Core RTOS-like Loop (1kHz) ---
// ==========================================
void loop() {
  unsigned long currentMicros = micros();
  unsigned long dt_micros = currentMicros - lastMicros;
  
  reg.presentPos = getUnwrappedAngle();

  if (Serial.available() >= 0) {
    processSerial();
  }

  if (dt_micros >= 1000) { 
    float dt = dt_micros / 1000000.0;
    lastMicros = currentMicros;

    // 🛠️ DEV MODE: ระบบ Tracker ตรวจสอบ State
    #ifdef DEV_MODE
    static unsigned long lastTrace = 0;
    bool doTrace = false;
    
    // ยอมให้พ่น Trace แค่ทุกๆ 1 วินาที ป้องกัน Serial คอขวด
    if (millis() - lastTrace > 1000) {
      doTrace = true;
      lastTrace = millis();
      DEBUG_PRINT("\n-- LOOP --\n");
    }
    #endif

    // เช็คว่าเปิด Torque หรือยัง
    if (reg.torqueEnable && !wasTorqueEnabled) {
      profilePos = reg.presentPos;
      profileVel = 0;
      reg.goalPos = reg.presentPos;
      DEBUG_PRINT("EV:TRQ_ON\n");
    } 
    wasTorqueEnabled = reg.torqueEnable;

    if (reg.torqueEnable) {
      #ifdef DEV_MODE
      if(doTrace) DEBUG_PRINT(">1.TRQ=1\n");
      #endif
      
      reg.goalPos = constrain(reg.goalPos, reg.minLimit, reg.maxLimit);
      
      // ---------------------------------
      // ด่าน 1: Trajectory Profile 
      // ---------------------------------
      if (reg.profileVelocity > 0 && reg.profileAcceleration > 0) {
        float max_a = reg.profileAcceleration * dt;
        float max_v = reg.profileVelocity;
        float stop_dist = (profileVel * profileVel) / (2.0 * reg.profileAcceleration);
        float dist_to_goal = reg.goalPos - profilePos;
        float desired_v = 0;
        
        if (abs(dist_to_goal) > stop_dist) {
          desired_v = (dist_to_goal > 0) ? max_v : -max_v;
        } else {
          desired_v = 0;
        }

        if (profileVel < desired_v) {
          profileVel += max_a;
          if (profileVel > desired_v) profileVel = desired_v;
        } else if (profileVel > desired_v) {
          profileVel -= max_a;
          if (profileVel < desired_v) profileVel = desired_v;
        }

        profilePos += profileVel * dt;
        if (abs(reg.goalPos - profilePos) < (abs(profileVel) * dt + 1.0)) {
           profilePos = reg.goalPos;
           profileVel = 0;
        }
      } else {
        profilePos = reg.goalPos;
        profileVel = 0;
      }
      
      #ifdef DEV_MODE
      if(doTrace) DEBUG_PRINT(">2.TRJ_OK\n");
      #endif

      // ---------------------------------
      // ด่าน 2: PID Control 
      // ---------------------------------
      reg.presentError = profilePos - reg.presentPos;
      float P = reg.kp * reg.presentError;
      integral = constrain(integral + (reg.presentError * dt), -150, 150);
      float I = reg.ki * integral;
      float D = reg.kd * (reg.presentError - lastError) / dt;
      lastError = reg.presentError;
      
      float output = P + I + D;
      int currentPwm = constrain(abs(output), 0, reg.pwmLimit);
      
      if (abs(reg.presentError) > 0.8 && currentPwm < 45 && reg.pwmLimit >= 45) {
        currentPwm = 45;
      }

      #ifdef DEV_MODE
      if(doTrace) DEBUG_PRINT(">3.PID_OK\n");
      #endif

      // ---------------------------------
      // ด่าน 3: Execution (PWM Output)
      // ---------------------------------
      if (abs(reg.goalPos - reg.presentPos) < 1.0) { 
        // มอเตอร์ถึงเป้าหมายแล้ว เลยสั่งหยุด
        if(reg.moving == 1) DEBUG_PRINT("EV:ARRIVED\n");
        stopMotor();
        integral = 0;
        reg.moving = 0; 
        
        #ifdef DEV_MODE
        if(doTrace) DEBUG_PRINT(">4.STOP(IN_POS)\n");
        #endif
        
      } else {
        // มอเตอร์ยังไม่ถึงเป้าหมาย ต้องยิง PWM
        reg.moving = 1;
        (output > 0) ? motorForward(currentPwm) : motorBackward(currentPwm);
        
        #ifdef DEV_MODE
        if(doTrace) {
          DEBUG_PRINT(">4.DRV_OUT\n");
          DEBUG_PRINT_VAL("PWM:", currentPwm);
        }
        #endif
      }
      
    } else {
      stopMotor();
      integral = 0;
      reg.moving = 0;
      
      #ifdef DEV_MODE
      if(doTrace) DEBUG_PRINT(">1.TRQ=0\n");
      #endif
    }
  }
}

// ==========================================
// --- Dynamixel-like Protocol Parsing ---
// ==========================================
void processSerial() {
  while (Serial.available() >= 2) {
    if (Serial.read() == 0xFF) {
      if (Serial.peek() == 0xFF) {
        Serial.read();
        
        unsigned long t = millis();
        while(Serial.available() < 2) { 
          if(millis() - t > 10) { DEBUG_PRINT("ERR:T1\n"); return; }
        } 
        
        uint8_t id = Serial.read();
        uint8_t len = Serial.read();
        
        t = millis();
        while(Serial.available() < len) { 
          if(millis() - t > 10) { DEBUG_PRINT("ERR:T2\n"); return; }
        }
        
        uint8_t inst = Serial.read();
        uint8_t params[len - 2];
        uint8_t checksum = id + len + inst;
        for (int i = 0; i < len - 2; i++) {
          params[i] = Serial.read();
          checksum += params[i];
        }
        
        uint8_t rx_checksum = Serial.read();
        if ((uint8_t)(~checksum) != rx_checksum) { DEBUG_PRINT("ERR:CSUM\n"); return; }
        if (id != reg.id && id != 0xFE) { DEBUG_PRINT("ERR:ID\n"); return; }
        
        if (inst == INST_WRITE) {
          uint8_t addr = params[0];
          memcpy((uint8_t*)&reg + addr, &params[1], len - 3);
          if (addr <= 21 || (addr >= 36 && addr <= 40)) saveConfig();
          sendStatusPacket(0x00, NULL, 0); 
          
        } else if (inst == INST_READ) {
          uint8_t addr = params[0];
          uint8_t readLen = params[1];
          sendStatusPacket(0x00, (uint8_t*)&reg + addr, readLen);
          
        } else if (inst == INST_PING) {
          sendStatusPacket(0x00, NULL, 0);
        }
        return; 
      }
    }
  }
}

void sendStatusPacket(uint8_t err, uint8_t* p, uint8_t pLen) {
  uint8_t chk = reg.id + (pLen + 2) + err;
  delayMicroseconds(RETURN_DELAY_US);
  
  digitalWrite(SLAVE_TX_CONTROL, HIGH);
  delayMicroseconds(50); 

  Serial.write(0xFF); 
  Serial.write(0xFF);
  Serial.write(reg.id); 
  Serial.write(pLen + 2); 
  Serial.write(err);
  for (int i = 0; i < pLen; i++) { 
    Serial.write(p[i]); 
    chk += p[i];
  }
  Serial.write((uint8_t)(~chk));

  Serial.flush(); 
  delayMicroseconds(50);
  
  digitalWrite(SLAVE_TX_CONTROL, LOW); 
}

// ==========================================
// --- Hardware Drivers ---
// ==========================================
float getUnwrappedAngle() {
  Wire.beginTransmission(AS5600_ADDRESS); 
  Wire.write(0x0C); 
  Wire.endTransmission();
  Wire.requestFrom(AS5600_ADDRESS, 2);
  
  if (Wire.available() == 2) {
    int raw = (Wire.read() << 8) | Wire.read();
    float currentDeg = (raw / 4096.0) * 360.0;
    
    if (lastRawDeg == -1.0) lastRawDeg = currentDeg;
    float diff = currentDeg - lastRawDeg;
    if (diff > 180.0) rotations--; 
    else if (diff < -180.0) rotations++;
    
    lastRawDeg = currentDeg;
    return (rotations * 4096.0) + raw; 
  }
  return reg.presentPos; 
}

void motorForward(int s) { 
  digitalWrite(BIN2, LOW); 
  analogWrite(BIN1, s);
}

void motorBackward(int s) { 
  digitalWrite(BIN1, LOW); 
  analogWrite(BIN2, s); 
}

void stopMotor() { 
  digitalWrite(BIN1, LOW); 
  digitalWrite(BIN2, LOW); 
}