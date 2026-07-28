/*
 * LEGACY — не прошивать на боевого робота.
 * Актуальная прошивка: firmware/metalinspector_robot/metalinspector_robot.ino
 * (PI-прямолинейность, ToF-край, змейка, LN invert, DRIVE_ONLY).
 */
#include <NimBLEDevice.h>
#include <stdio.h>
#include <string>

#define SERVICE_UUID        "12345678-1234-1234-1234-1234567890ab"
#define CHARACTERISTIC_UUID "abcdefab-1234-5678-1234-abcdefabcdef"
#define TELEMETRY_CHAR_UUID "fedcbaab-1234-5678-1234-abcdefabcdef"

/************ ПИНЫ ДРАЙВЕРОВ (не менять!) ************/
#define DRIVER1_MOTOR1_DIR 11  // LV
#define DRIVER1_MOTOR1_PWM 10
#define DRIVER1_MOTOR2_DIR 37  // LN
#define DRIVER1_MOTOR2_PWM 36
#define DRIVER2_MOTOR1_DIR 1   // RN
#define DRIVER2_MOTOR1_PWM 2
#define DRIVER2_MOTOR2_DIR 7   // RV
#define DRIVER2_MOTOR2_PWM 6

/************ ПИНЫ ЭНКОДЕРОВ (A, B) ************/
#define ENC_LV_A 40
#define ENC_LV_B 39
#define ENC_LN_A 20
#define ENC_LN_B 21
#define ENC_RN_A 41
#define ENC_RN_B 42
#define ENC_RV_A 4
#define ENC_RV_B 5

static const int TELEM_HZ = 20;
static const uint32_t TELEM_US = 1000000UL / TELEM_HZ;

int speedVal = 150;

volatile long encLV = 0;
volatile long encLN = 0;
volatile long encRN = 0;
volatile long encRV = 0;

NimBLEServer* pServer = nullptr;
NimBLEAdvertising* pAdvertising = nullptr;
NimBLECharacteristic* pTelemChar = nullptr;

uint32_t lastTelemUs = 0;

// --- Encoders ---------------------------------------------------------------

void IRAM_ATTR isrEncLV() {
  if (digitalRead(ENC_LV_B) != digitalRead(ENC_LV_A)) encLV++;
  else encLV--;
}

void IRAM_ATTR isrEncLN() {
  if (digitalRead(ENC_LN_B) != digitalRead(ENC_LN_A)) encLN++;
  else encLN--;
}

void IRAM_ATTR isrEncRN() {
  if (digitalRead(ENC_RN_B) != digitalRead(ENC_RN_A)) encRN++;
  else encRN--;
}

void IRAM_ATTR isrEncRV() {
  if (digitalRead(ENC_RV_B) != digitalRead(ENC_RV_A)) encRV++;
  else encRV--;
}

void initEncoders() {
  pinMode(ENC_LV_A, INPUT_PULLUP);
  pinMode(ENC_LV_B, INPUT_PULLUP);
  pinMode(ENC_LN_A, INPUT_PULLUP);
  pinMode(ENC_LN_B, INPUT_PULLUP);
  pinMode(ENC_RN_A, INPUT_PULLUP);
  pinMode(ENC_RN_B, INPUT_PULLUP);
  pinMode(ENC_RV_A, INPUT_PULLUP);
  pinMode(ENC_RV_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_LV_A), isrEncLV, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_LN_A), isrEncLN, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RN_A), isrEncRN, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RV_A), isrEncRV, CHANGE);
}

// --- Motors -----------------------------------------------------------------

int clampSpeed(int s) {
  if (s > 255) return 255;
  if (s < -255) return -255;
  return s;
}

void driveMotorSigned(int dirPin, int pwmPin, int speed) {
  speed = clampSpeed(speed);
  if (speed >= 0) {
    analogWrite(dirPin, 0);
    analogWrite(pwmPin, speed);
  } else {
    analogWrite(dirPin, -speed);
    analogWrite(pwmPin, 0);
  }
}

void setMotors(int leftSpeed, int rightSpeed) {
  leftSpeed = clampSpeed(leftSpeed);
  rightSpeed = clampSpeed(rightSpeed);
  driveMotorSigned(DRIVER1_MOTOR1_DIR, DRIVER1_MOTOR1_PWM, leftSpeed);  // LV
  driveMotorSigned(DRIVER1_MOTOR2_DIR, DRIVER1_MOTOR2_PWM, leftSpeed);  // LN
  driveMotorSigned(DRIVER2_MOTOR1_DIR, DRIVER2_MOTOR1_PWM, rightSpeed); // RN
  driveMotorSigned(DRIVER2_MOTOR2_DIR, DRIVER2_MOTOR2_PWM, rightSpeed); // RV
}

void stopMotors() {
  setMotors(0, 0);
}

// --- BLE telemetry ----------------------------------------------------------

void sendEncoderTelemetry() {
  if (pTelemChar == nullptr || pServer->getConnectedCount() == 0) return;

  long lv, ln, rn, rv;
  noInterrupts();
  lv = encLV;
  ln = encLN;
  rn = encRN;
  rv = encRV;
  interrupts();

  char buf[64];
  int len = snprintf(buf, sizeof(buf), "E,%lu,%ld,%ld,%ld,%ld",
                     millis(), lv, ln, rn, rv);
  if (len > 0 && len < (int)sizeof(buf)) {
    pTelemChar->setValue((uint8_t*)buf, len);
    pTelemChar->notify();
  }
}

// --- Commands ---------------------------------------------------------------

void handleCommand(char c) {
  switch (toupper(c)) {
    case 'F': setMotors( speedVal,  speedVal); break;
    case 'B': setMotors(-speedVal, -speedVal); break;
    case 'L': setMotors(-speedVal,  speedVal); break;
    case 'R': setMotors( speedVal, -speedVal); break;
    case 'S': stopMotors(); break;
    default:  stopMotors(); break;
  }
}

bool handleMotorCommand(const std::string& s) {
  if (s.empty() || (s[0] != 'M' && s[0] != 'm')) return false;
  int l = 0, r = 0;
  if (sscanf(s.c_str(), "%*c%d,%d", &l, &r) == 2) {
    setMotors(l, r);
  } else {
    stopMotors();
  }
  return true;
}

class TelemCallbacks : public NimBLECharacteristicCallbacks {
  void onSubscribe(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo, uint16_t subValue) override {
    if (subValue != 0) {
      sendEncoderTelemetry();
    }
  }
};

class CharCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
    NimBLEAttValue value = pCharacteristic->getValue();
    if (value.length() == 0) return;
    std::string s(value.c_str(), value.length());

    if (handleMotorCommand(s)) {
      Serial.printf("CMD: %s\n", s.c_str());
      return;
    }

    char c = (char)toupper(s[0]);
    Serial.printf("CMD: %c\n", c);
    handleCommand(c);
  }
};

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
    Serial.printf("Client connected: %s\n", connInfo.getAddress().toString().c_str());
    stopMotors();
    pServer->updateConnParams(connInfo.getConnHandle(), 24, 40, 0, 400);
    if (pTelemChar != nullptr) {
      sendEncoderTelemetry();
    }
  }

  void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
    Serial.printf("Client disconnected (reason %d) -> stop + re-advertise\n", reason);
    stopMotors();
    if (!NimBLEDevice::getAdvertising()->isAdvertising()) {
      NimBLEDevice::startAdvertising();
    }
  }
};

void setup() {
  Serial.begin(115200);

  pinMode(DRIVER1_MOTOR1_DIR, OUTPUT);
  pinMode(DRIVER1_MOTOR1_PWM, OUTPUT);
  pinMode(DRIVER1_MOTOR2_DIR, OUTPUT);
  pinMode(DRIVER1_MOTOR2_PWM, OUTPUT);
  pinMode(DRIVER2_MOTOR1_DIR, OUTPUT);
  pinMode(DRIVER2_MOTOR1_PWM, OUTPUT);
  pinMode(DRIVER2_MOTOR2_DIR, OUTPUT);
  pinMode(DRIVER2_MOTOR2_PWM, OUTPUT);
  stopMotors();
  initEncoders();

  NimBLEDevice::init("ESP32_ROBOT");
  NimBLEDevice::setDeviceName("ESP32_ROBOT");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEDevice::setMTU(247);

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());
  pServer->advertiseOnDisconnect(true);

  NimBLEService* pService = pServer->createService(SERVICE_UUID);
  NimBLECharacteristic* pChar = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  pChar->setCallbacks(new CharCallbacks());

  pTelemChar = pService->createCharacteristic(
      TELEMETRY_CHAR_UUID,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  pTelemChar->setCallbacks(new TelemCallbacks());
  pTelemChar->createDescriptor(NIMBLE_DESCRIPTOR::CLIENT_CHARACTERISTIC_CONFIGURATION);

  pService->start();

  pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setName("ESP32_ROBOT");
  pAdvertising->addTxPower();
  pAdvertising->setPreferredParams(0x06, 0x12);
  pAdvertising->enableScanResponse(true);
  pAdvertising->start();

  lastTelemUs = micros();
  Serial.println("ESP32_ROBOT advertising. Encoder telemetry: E,ms,lv,ln,rn,rv");
}

void loop() {
  uint32_t now = micros();
  if ((uint32_t)(now - lastTelemUs) >= TELEM_US) {
    lastTelemUs = now;
    sendEncoderTelemetry();
  }
}
