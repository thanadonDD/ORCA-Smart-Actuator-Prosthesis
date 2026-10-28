/* * =============================================================================
 * PROJECT: KPServo Master Node (V8 - Dev Mode & Timer Stepper)
 * =============================================================================
 */

#include <SoftwareSerial.h>

const int RS485_RX = 2;
const int RS485_TX = 3;
const int RS485_CTRL = 8;
SoftwareSerial rs485(RS485_RX, RS485_TX);

#define INST_WRITE 0x03
#define INST_READ 0x02
const uint8_t SLAVE_ID = 1;

typedef union { float f; uint8_t bytes[4]; } FloatUnion;

// --- ⚙️ State Machine สำหรับการหมุนอัตโนมัติ ---
float targetAngles[] = {0.0, 90.0, 180.0, 270.0, 360.0};
const int numAngles = 5;
int currentTargetIndex = 0;

unsigned long lastMoveTime = 0;

void setup() {
  Serial.begin(9600);       // 📉 ปรับความเร็วคุยกับคอมเป็น 9600
  rs485.begin(9600);        // 📉 ปรับความเร็วคุยกับ Slave เป็น 9600 (สำคัญมาก!)
  
  pinMode(RS485_CTRL, OUTPUT);
  digitalWrite(RS485_CTRL, LOW);
  
  Serial.println("=== 🎛️ KPSERVO LIVE PID TUNER BOOTING ===");
  Serial.println("How to use:");
  Serial.println(" - Type 'P 15.0' to set Kp to 15.0");
  Serial.println(" - Type 'I 0.1'  to set Ki to 0.1");
  Serial.println(" - Type 'D 0.5'  to set Kd to 0.5");
  Serial.println(" - Type 'R'      to Read current PID values");
  Serial.println("==========================================");
  delay(3000); 

  writeByte(SLAVE_ID, 0x15, 180);       // PWM Limit
  delay(50);
  writeFloat(SLAVE_ID, 0x24, 3000.0);   // Velocity
  delay(50);
  writeFloat(SLAVE_ID, 0x28, 1000.0);   // Acceleration
  delay(50);
  writeByte(SLAVE_ID, 0x16, 1);         // Enable Torque
  delay(50);

  readAndPrintPID();
  
  sendMoveCommand(targetAngles[currentTargetIndex]);
  lastMoveTime = millis();
}

void loop() {
  // ---------------------------------------------------------
  // 1. ระบบรับคำสั่งจูน PID กลางอากาศ (Non-blocking)
  // ---------------------------------------------------------
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() > 0) {
      char cmd = toupper(input[0]);
      float val = input.substring(1).toFloat();

      Serial.println("\n----------------------------------");
      if (cmd == 'P') {
        writeFloat(SLAVE_ID, 0x01, val);
        Serial.print("🛠️ TUNING -> Kp updated to: "); Serial.println(val, 3);
      } 
      else if (cmd == 'I') {
        writeFloat(SLAVE_ID, 0x05, val);
        Serial.print("🛠️ TUNING -> Ki updated to: "); Serial.println(val, 3);
      } 
      else if (cmd == 'D') {
        writeFloat(SLAVE_ID, 0x09, val);
        Serial.print("🛠️ TUNING -> Kd updated to: "); Serial.println(val, 3);
      } 
      else if (cmd == 'R') {
        readAndPrintPID();
      }
      else {
        Serial.println("❌ Unknown Command! Use P, I, D, or R.");
      }
      Serial.println("----------------------------------\n");
    }
  }

  // ---------------------------------------------------------
  // 2. ระบบ Auto-Stepper แบบบังคับเปลี่ยนเป้าหมาย (ยิงทุกๆ 3 วินาที)
  // ---------------------------------------------------------
  if (millis() - lastMoveTime > 3000) { 
    lastMoveTime = millis();
    
    currentTargetIndex++;
    if (currentTargetIndex >= numAngles) {
      currentTargetIndex = 0;
      Serial.println("\n🔄 Sequence Complete. Restarting from 0°");
    }
    
    sendMoveCommand(targetAngles[currentTargetIndex]);
  }
}

// ==========================================
// --- Smart Functions ---
// ==========================================
void sendMoveCommand(float degree) {
  float targetTicks = (degree / 360.0) * 4096.0;
  Serial.print("🚀 Auto-Step: Moving to "); 
  Serial.print(degree, 1); Serial.println("°");
  
  writeFloat(SLAVE_ID, 0x17, targetTicks);
}

void readAndPrintPID() {
  float p = readFloat(SLAVE_ID, 0x01); 
  float i = readFloat(SLAVE_ID, 0x05);
  float d = readFloat(SLAVE_ID, 0x09);
  
  Serial.println("--- 📊 CURRENT PID VALUES ---");
  Serial.print("   Kp = "); Serial.println(p, 3);
  Serial.print("   Ki = "); Serial.println(i, 3);
  Serial.print("   Kd = "); Serial.println(d, 3);
  Serial.println("-----------------------------");
}

// ==========================================
// --- Low-Level Protocol Drivers ---
// ==========================================
uint8_t readByte(uint8_t id, uint8_t addr) {
  uint8_t params[2] = {addr, 1};
  sendPacket(id, INST_READ, params, 2);
  
  unsigned long startTime = millis();
  while (rs485.available() < 7) { 
    if (millis() - startTime > 150) return 255;
  }
  
  while (rs485.available() >= 7) {
    if (rs485.read() == 0xFF) {
      if (rs485.peek() == 0xFF) {
        rs485.read();
        
        uint8_t rx_id  = rs485.read();
        uint8_t rx_len = rs485.read();
        uint8_t rx_err = rs485.read();
        uint8_t data   = rs485.read();
        uint8_t checksum = rs485.read();
        
        return data;
      }
    }
  }
  return 255;
}

float readFloat(uint8_t id, uint8_t addr) {
  uint8_t params[2] = {addr, 4};
  sendPacket(id, INST_READ, params, 2);
  unsigned long startTime = millis();
  while (rs485.available() == 0) { 
    if (millis() - startTime > 150) return 0.0;
  }
  
  delay(10); 
  
  while (rs485.available() >= 4) {
    if (rs485.read() == 0xFF) {
      if (rs485.peek() == 0xFF) {
        rs485.read();
        rs485.read();
        rs485.read();
        rs485.read();
        
        FloatUnion val;
        val.bytes[0] = rs485.read();
        val.bytes[1] = rs485.read();
        val.bytes[2] = rs485.read();
        val.bytes[3] = rs485.read();
        
        return val.f;
      }
    }
  }
  return 0.0;
}

void writeByte(uint8_t id, uint8_t addr, uint8_t value) {
  uint8_t params[2] = {addr, value};
  sendPacket(id, INST_WRITE, params, 2);
}

void writeFloat(uint8_t id, uint8_t addr, float value) {
  FloatUnion val; val.f = value;
  uint8_t params[5] = {addr, val.bytes[0], val.bytes[1], val.bytes[2], val.bytes[3]};
  sendPacket(id, INST_WRITE, params, 5);
}

void sendPacket(uint8_t id, uint8_t inst, uint8_t *params, uint8_t paramLen) {
  uint8_t length = paramLen + 2;
  uint8_t checksum = id + length + inst;
  
  digitalWrite(RS485_CTRL, HIGH); delayMicroseconds(50); 
  rs485.write(0xFF); rs485.write(0xFF); rs485.write(id);
  rs485.write(length); rs485.write(inst);
  for (uint8_t i = 0; i < paramLen; i++) { rs485.write(params[i]); checksum += params[i]; }
  rs485.write((uint8_t)(~checksum));
  rs485.flush(); 
  delayMicroseconds(50);
  digitalWrite(RS485_CTRL, LOW); 
}