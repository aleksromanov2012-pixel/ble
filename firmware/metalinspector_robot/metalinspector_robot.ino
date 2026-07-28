/*
 * МеталИнспектор / ESP32_ROBOT
 * Корпус 294×220 мм. Старт: ЛЕВЫЙ НИЖНИЙ угол, курс ВВЕРХ.
 *
 * Змейка (G):
 *   полоса чётная ↑ до края → назад → 90° ВПРАВО → сдвиг на ширину (220) →
 *               90° ВПРАВО → полоса ↓
 *   полоса нечётная ↓ до края → назад → 90° ВЛЕВО → сдвиг 220 →
 *               90° ВЛЕВО → полоса ↑
 *   покрытие готово → стоп (DRIVE_ONLY: без HOME/SCAN/камеры).
 *
 * go каждые 147 мм только на прямой полосе (не на развороте).
 *
 * Callers: arduino-cli flash; control/bridge.py; (опц.) Pi run_capture via go.
 */
#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <NimBLEDevice.h>
#include "esp_bt.h"
#include <stdio.h>
#include <string>
#include <math.h>

#define SERVICE_UUID        "12345678-1234-1234-1234-1234567890ab"
#define CHARACTERISTIC_UUID "abcdefab-1234-5678-1234-abcdefabcdef"
#define TELEMETRY_CHAR_UUID "fedcbaab-1234-5678-1234-abcdefabcdef"

#define DRIVER1_MOTOR1_DIR 11
#define DRIVER1_MOTOR1_PWM 10
#define DRIVER1_MOTOR2_DIR 37
#define DRIVER1_MOTOR2_PWM 36
#define DRIVER2_MOTOR1_DIR 1
#define DRIVER2_MOTOR1_PWM 2
#define DRIVER2_MOTOR2_DIR 7
#define DRIVER2_MOTOR2_PWM 6

#define ENC_LV_A 40
#define ENC_LV_B 39
#define ENC_LN_A 20
#define ENC_LN_B 21
#define ENC_RN_A 41
#define ENC_RN_B 42
#define ENC_RV_A 4
#define ENC_RV_B 5

#define PIN_SDA 8
#define PIN_SCL 9
static const int XSHUT[4] = { 12, 13, 14, 15 };
static const uint8_t TOF_ADDR[4] = { 0x30, 0x31, 0x32, 0x33 };
static const char* SIDE_NAME[4] = { "fl", "fr", "rl", "rr" };

static const uint16_t CLIFF_MM = 80;   // над металлом ~50–70; край/пустота выше
static const uint16_t BAD_MM = 8000;
static const uint32_t TELEM_MS = 50;
static const uint32_t CTRL_MS = 10;
static const uint32_t ODO_REPORT_MS = 200;

// Камера на Pi сейчас offline — после покрытия / H не включаем SCAN.
#ifndef DRIVE_ONLY
#define DRIVE_ONLY 1
#endif

static const float ROBOT_LEN_MM = 294.0f;
static const float ROBOT_WID_MM = 220.0f;
static const float WHEEL_DIAMETER_MM = 42.0f;
static const float ENCODER_CPR = 7.0f;
static const float GEAR_RATIO = 100.0f;
static const float MM_PER_TICK =
    (PI * WHEEL_DIAMETER_MM) / (ENCODER_CPR * GEAR_RATIO * 4.0f);
static const float STEP_MM = ROBOT_LEN_MM * 0.5f;   // 147 — фото
static const float LANE_PITCH_MM = ROBOT_WID_MM;    // 220 — как /api/passes/plan
static const float TRACK_WIDTH_MM = ROBOT_WID_MM;
static const float FRAME_CROSS_MM = 225.0f;

static const float BACKUP_MM = 130.0f;   // заметно назад от края
static const float TURN_90_DEG = 115.0f; // недоворот на магн. колёсах → перекрут
static const float NUDGE_MAX_DEG = 14.0f;
static const float HOME_RADIUS_MM = 140.0f;

static const float KP_STRAIGHT = 0.35f;  // слабее PI — trim важнее
static const float KI_STRAIGHT = 0.006f;
static const int CORR_MAX = 35;
static const int TRIM_LEFT = 0;
static const int TRIM_RIGHT = 95;  // сильно вправо против увода влево
static const int TURN_PWM = 145;
static const int NUDGE_PWM = 95;
static const int BACKUP_PWM = 75;
static const int SPEED_DEFAULT = 55;
static const int SPEED_AUTO_MAX = 75;
// Время — только ДЛИННЫЙ запас (раньше 650/850 обрывали манёвр слишком рано)
static const uint32_t TURN90_MS = 1600;
static const uint32_t BACKUP_MS = 1600;

int speedVal = SPEED_DEFAULT;
volatile long encLV = 0, encLN = 0, encRN = 0, encRV = 0;

NimBLEServer* pServer = nullptr;
NimBLECharacteristic* pCmdChar = nullptr;
NimBLECharacteristic* pTelemChar = nullptr;
volatile bool bleConnected = false;
uint32_t lastTelemMs = 0, lastCtrlMs = 0, lastOdoMs = 0;

VL53L0X tof[4];
bool tofAlive[4] = { false, false, false, false };
uint16_t lastMm[4] = { 0, 0, 0, 0 };
uint8_t badStreak[4] = { 0, 0, 0, 0 };
uint8_t groundStreak[4] = { 0, 0, 0, 0 };
uint8_t toStreak[4] = { 0, 0, 0, 0 };
uint32_t lastRecoverMs[4] = { 0, 0, 0, 0 };

bool cliffLatched = false;
int cliffCulprit = -1;
bool cliffEnabled = true;   // защита от падения ВКЛ; выкл временно: команда X
uint8_t cliffHitStreak = 0;
static const uint8_t CLIFF_NEED = 3;   // стоп по краю ощутимый
static const uint8_t BAD_KILL = 200;  // НЕ убивать датчик над пропастью (timeout=край)
// шум моторов вешает I2C: 100 кГц устойчивее, плюс реинициализация вместо вечного края
static const uint32_t I2C_HZ = 100000;
static const uint8_t TO_RECOVER = 15;
static const uint32_t RECOVER_GAP_MS = 400;

enum NavState : uint8_t {
  NAV_IDLE = 0, NAV_CRUISE, NAV_BACKUP, NAV_TURN1, NAV_SHIFT, NAV_TURN2,
  NAV_NUDGE, NAV_HOME_TURN, NAV_HOME_DRIVE, NAV_HOME_ALIGN, NAV_SCAN_CRUISE
};
enum MissionPhase : uint8_t { PHASE_IDLE = 0, PHASE_MAP, PHASE_HOME, PHASE_SCAN };

NavState nav = NAV_IDLE;
MissionPhase phase = PHASE_IDLE;
bool autoMission = false, mappingMode = false, turnRightDir = true, nudgeRightDir = true;
bool scanPass = false;
int laneIndex = 0;
// План с сайта / пульта: P<width_mm>,<height_mm>
float surfaceW = 2000.0f;
float surfaceH = 800.0f;
float laneLenMm = 800.0f;
float lanePitchMm = LANE_PITCH_MM;
int lanesPlanned = 1;
float laneStartPathMm = 0.0f;
bool planReady = false;

long baseL = 0, baseR = 0;
float integErr = 0.0f, lastStepMm = 0.0f, pathMm = 0.0f;
float poseX = 0.0f, poseY = 0.0f, poseYaw = 0.0f;
long poseL0 = 0, poseR0 = 0, manL0 = 0, manR0 = 0;
float manTargetMm = 0.0f;
uint32_t manStartMs = 0;
int lastLeftPwm = 0, lastRightPwm = 0;
bool braking = false, ignoreCliffLatchGate = false;
uint32_t cliffGraceUntilMs = 0;

void IRAM_ATTR isrEncLV() { if (digitalRead(ENC_LV_B) != digitalRead(ENC_LV_A)) encLV++; else encLV--; }
void IRAM_ATTR isrEncLN() { if (digitalRead(ENC_LN_B) != digitalRead(ENC_LN_A)) encLN--; else encLN++; }
void IRAM_ATTR isrEncRN() { if (digitalRead(ENC_RN_B) != digitalRead(ENC_RN_A)) encRN++; else encRN--; }
void IRAM_ATTR isrEncRV() { if (digitalRead(ENC_RV_B) != digitalRead(ENC_RV_A)) encRV++; else encRV--; }

void initEncoders() {
  pinMode(ENC_LV_A, INPUT_PULLUP); pinMode(ENC_LV_B, INPUT_PULLUP);
  pinMode(ENC_LN_A, INPUT_PULLUP); pinMode(ENC_LN_B, INPUT_PULLUP);
  pinMode(ENC_RN_A, INPUT_PULLUP); pinMode(ENC_RN_B, INPUT_PULLUP);
  pinMode(ENC_RV_A, INPUT_PULLUP); pinMode(ENC_RV_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_LV_A), isrEncLV, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_LN_A), isrEncLN, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RN_A), isrEncRN, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RV_A), isrEncRV, CHANGE);
}

void readGuideTicks(long* L, long* R) {
  noInterrupts();
  long lv = encLV, ln = encLN, rn = encRN, rv = encRV;
  interrupts();
  // Average both wheels per side (LV/LN, RV/RN). Dead encoder ≈ 0 and does not hurt much.
  *L = (labs(lv) + labs(ln)) / 2;
  *R = (labs(rv) + labs(rn)) / 2;
}
void resetStraightBaseline() { readGuideTicks(&baseL, &baseR); integErr = 0; }
float arcMmForDeg(float deg) { return (PI * TRACK_WIDTH_MM * fabsf(deg)) / 360.0f; }
void beginManeuver(float t) { manStartMs=millis(); readGuideTicks(&manL0, &manR0); manTargetMm = t; }
float maneuverProgressMm() {
  long L=0,R=0; readGuideTicks(&L,&R);
  return ((float)(labs(L-manL0)+labs(R-manR0))/2.0f)*MM_PER_TICK;
}
bool maneuverDone() {
  if (!manStartMs) return false;
  uint32_t elapsed = millis() - manStartMs;
  if (elapsed > 7000) return true;
  // Главный критерий — путь по энкодерам
  if (maneuverProgressMm() >= manTargetMm) return true;
  // Время только как длинный fallback (короткий таймер раньше недоворачивал / мало откатывал)
  if (nav == NAV_BACKUP && elapsed >= BACKUP_MS) return true;
  if ((nav == NAV_TURN1 || nav == NAV_TURN2) && elapsed >= TURN90_MS) return true;
  if (nav == NAV_NUDGE && elapsed >= 500) return true;
  return false;
}

// Чётная полоса (0,2,…) → два поворота ВПРАВО; нечётная → два ВЛЕВО.
bool serpTurnRight() { return (laneIndex % 2) == 0; }

void emitLine(const char* line) {
  Serial.println(line);
  if (bleConnected && pTelemChar) {
    pTelemChar->setValue((uint8_t*)line, strlen(line));
    pTelemChar->notify();
  }
}
void emitGo() { emitLine("go"); }
void emitOdo() { char b[48]; snprintf(b,sizeof(b),"ODO %.1f",pathMm); emitLine(b); }
void emitPose() {
  char b[96];
  snprintf(b,sizeof(b),"POSE %.1f %.1f %.1f LANE %d", poseX,poseY,poseYaw*180/PI,laneIndex);
  emitLine(b);
}
void emitCliffEvent(const char* side) {
  char b[64]; snprintf(b,sizeof(b),"CLIFF %s %.1f",side,pathMm); emitLine(b);
}

const char* navName(NavState s);
const char* phaseName(MissionPhase p) {
  switch(p){case PHASE_MAP:return "map";case PHASE_HOME:return "home";case PHASE_SCAN:return "scan";default:return "idle";}
}
void emitPhase(){ char b[32]; snprintf(b,sizeof(b),"PHASE %s",phaseName(phase)); emitLine(b); }
void emitState(){ char b[40]; snprintf(b,sizeof(b),"STATE %s",navName(nav)); emitLine(b); }
void setNav(NavState s){ if(nav==s)return; nav=s; emitState(); }
void setPhase(MissionPhase p){ phase=p; emitPhase(); }
void snapPoseBaseline(){ readGuideTicks(&poseL0,&poseR0); }

void updatePoseFromDrive() {
  long L=0,R=0; readGuideTicks(&L,&R);
  float dL=(float)(L-poseL0)*MM_PER_TICK, dR=(float)(R-poseR0)*MM_PER_TICK;
  poseL0=L; poseR0=R;
  if(dL==0&&dR==0)return;
  float dist=0.5f*(dL+dR), dYaw=(dR-dL)/TRACK_WIDTH_MM;
  if(lastLeftPwm>0&&lastRightPwm>0){
    dist=fabsf(dist); poseYaw+=dYaw;
    poseX+=dist*cosf(poseYaw); poseY+=dist*sinf(poseYaw); pathMm+=dist;
  } else if(lastLeftPwm<0&&lastRightPwm<0){
    dist=fabsf(dist); poseYaw+=dYaw;
    poseX-=dist*cosf(poseYaw); poseY-=dist*sinf(poseYaw); pathMm+=dist;
  } else if(lastLeftPwm>0&&lastRightPwm<0){
    poseYaw-=fabsf(dYaw); pathMm+=0.5f*(fabsf(dL)+fabsf(dR));
  } else if(lastLeftPwm<0&&lastRightPwm>0){
    poseYaw+=fabsf(dYaw); pathMm+=0.5f*(fabsf(dL)+fabsf(dR));
  }
  while(poseYaw>PI)poseYaw-=2*PI; while(poseYaw<-PI)poseYaw+=2*PI;
}
void resetMapPose(){ pathMm=lastStepMm=0; poseX=poseY=poseYaw=0; snapPoseBaseline(); }
void pollMappingSteps(){
  if(!mappingMode)return;
  if(!(nav==NAV_CRUISE||nav==NAV_SCAN_CRUISE))return;
  while(pathMm-lastStepMm>=STEP_MM){ lastStepMm+=STEP_MM; emitGo(); emitPose(); }
}
float distToHome(){ return sqrtf(poseX*poseX+poseY*poseY); }
float wrapPi(float a){ while(a>PI)a-=2*PI; while(a<-PI)a+=2*PI; return a; }
int clampSpeed(int s){ if(s>255)return 255; if(s<-255)return -255; return s; }
int cruiseSpeed(){ int v=speedVal; if(autoMission&&v>SPEED_AUTO_MAX)v=SPEED_AUTO_MAX; if(v<40)v=40; return v; }

void driveMotorSigned(int dirPin,int pwmPin,int speed){
  speed=clampSpeed(speed); braking=false;
  if(speed>=0){ analogWrite(dirPin,0); analogWrite(pwmPin,speed); }
  else { analogWrite(dirPin,-speed); analogWrite(pwmPin,0); }
}
void brakeMotor(int d,int p){ analogWrite(d,255); analogWrite(p,255); }
void applyBrakeAll(){
  braking=true; lastLeftPwm=lastRightPwm=0;
  brakeMotor(DRIVER1_MOTOR1_DIR,DRIVER1_MOTOR1_PWM);
  brakeMotor(DRIVER1_MOTOR2_DIR,DRIVER1_MOTOR2_PWM);
  brakeMotor(DRIVER2_MOTOR1_DIR,DRIVER2_MOTOR1_PWM);
  brakeMotor(DRIVER2_MOTOR2_DIR,DRIVER2_MOTOR2_PWM);
}
void setMotorsRaw(int leftSpeed,int rightSpeed){
  if(cliffEnabled&&cliffLatched&&!ignoreCliffLatchGate){
    if(!(leftSpeed<0&&rightSpeed<0)){ applyBrakeAll(); return; }
  }
  leftSpeed=clampSpeed(leftSpeed); rightSpeed=clampSpeed(rightSpeed);
  lastLeftPwm=leftSpeed; lastRightPwm=rightSpeed;
  driveMotorSigned(DRIVER1_MOTOR1_DIR,DRIVER1_MOTOR1_PWM,leftSpeed);
  driveMotorSigned(DRIVER1_MOTOR2_DIR,DRIVER1_MOTOR2_PWM,-leftSpeed);
  driveMotorSigned(DRIVER2_MOTOR1_DIR,DRIVER2_MOTOR1_PWM,rightSpeed);
  driveMotorSigned(DRIVER2_MOTOR2_DIR,DRIVER2_MOTOR2_PWM,rightSpeed);
}
void stopAll(){ autoMission=false; mappingMode=false; ignoreCliffLatchGate=false; setPhase(PHASE_IDLE); setNav(NAV_IDLE); applyBrakeAll(); }
void updateStraightDrive(){
  if(!(nav==NAV_CRUISE||nav==NAV_SCAN_CRUISE||nav==NAV_SHIFT||nav==NAV_HOME_DRIVE)||cliffLatched)return;
  long L=0,R=0; readGuideTicks(&L,&R);
  float err=(float)((L-baseL)-(R-baseR));
  integErr+=err*KI_STRAIGHT; if(integErr>30)integErr=30; if(integErr<-30)integErr=-30;
  int corr=(int)(err*KP_STRAIGHT+integErr);
  if(corr>CORR_MAX)corr=CORR_MAX; if(corr<-CORR_MAX)corr=-CORR_MAX;
  int v=cruiseSpeed(); int l=v+TRIM_LEFT-corr; int r=v+TRIM_RIGHT+corr;
  if(l<30)l=30; if(r<30)r=30;
  if(l>255)l=255; if(r>255)r=255;
  setMotorsRaw(l,r);
}
int turnPwm(){
  int p = TURN_PWM;
  if (p < 100) p = 100;
  if (p > 170) p = 170;
  return p;
}
void driveTurn(bool right, int pwm) {
  ignoreCliffLatchGate = true;
  if (pwm < 100) pwm = 100;
  if (pwm > 170) pwm = 170;
  if (right) setMotorsRaw(pwm, -pwm);
  else setMotorsRaw(-pwm, pwm);
}

void sensorPower(int i,bool on){ if(on)pinMode(XSHUT[i],INPUT); else{pinMode(XSHUT[i],OUTPUT);digitalWrite(XSHUT[i],LOW);} }
bool i2cPresent(uint8_t a){ Wire.beginTransmission(a); return Wire.endTransmission()==0; }
void killSensor(int i,const char* w){ if(!tofAlive[i])return; tofAlive[i]=false; Serial.printf("[cliff %s] off:%s\n",SIDE_NAME[i],w); }
bool initOneSensor(int i){
  sensorPower(i,true); delay(25); tof[i].setTimeout(50);
  if(!tof[i].init()){ sensorPower(i,false); return false; }
  tof[i].setAddress(TOF_ADDR[i]);
  if(!i2cPresent(TOF_ADDR[i]))return false;
  tof[i].setMeasurementTimingBudget(50000); tof[i].startContinuous(0); delay(30);
  return true;
}
// вечный timeout = зависшая шина, а не пропасть: поднимаем I2C и датчик заново
void recoverSensor(int i){
  uint32_t now=millis();
  if(now-lastRecoverMs[i]<RECOVER_GAP_MS)return;
  lastRecoverMs[i]=now; toStreak[i]=0;
  Wire.end(); delay(2); Wire.begin(PIN_SDA,PIN_SCL); Wire.setClock(I2C_HZ);
  sensorPower(i,false); delay(10);
  bool ok=initOneSensor(i);
  tofAlive[i]=ok;
  Serial.printf("[cliff %s] recover %s\n",SIDE_NAME[i],ok?"OK":"FAIL");
}
bool sensorSeesCliff(int i){
  if(!tofAlive[i])return false;
  uint16_t mm=tof[i].readRangeContinuousMillimeters(); bool timeout=tof[i].timeoutOccurred();
  lastMm[i]=timeout?65535:mm;
  // металл под днищем: короткое валидное расстояние
  if(!timeout && mm>0 && mm<CLIFF_MM){
    badStreak[i]=0; toStreak[i]=0;
    if(groundStreak[i]<255) groundStreak[i]++;
    return false;
  }
  if(timeout){ if(toStreak[i]<255)toStreak[i]++; if(toStreak[i]>=TO_RECOVER) recoverSensor(i); }
  else toStreak[i]=0;
  groundStreak[i]=0;
  // далеко/0 = край; одиночный timeout — шум I2C, край только после серии
  bool looks = (!timeout && (mm>=CLIFF_MM || mm==0)) || (timeout && toStreak[i]>=2);
  // убиваем только явно мёртвый (вечный 0), и только если сосед стабильно видит землю
  if(!timeout && mm==0){
    if(badStreak[i]<255) badStreak[i]++;
    if(i<2 && badStreak[i]>=BAD_KILL){
      int o=1-i;
      if(tofAlive[o] && groundStreak[o]>=20){ killSensor(i,"zero"); return false; }
    }
  } else {
    badStreak[i]=0;
  }
  return looks;
}
uint8_t readFrontCliffMask(){
  uint8_t m=0;
  if(tofAlive[0]&&sensorSeesCliff(0))m|=1;
  if(tofAlive[1]&&sensorSeesCliff(1))m|=2;
  // оба мертвы → НЕ считать краем (иначе змейка сразу в backup и «не едет»).
  // живые датчики по-прежнему стопорят/нёджат.
  return m;
}
void refreshFrontMm(){ for(int i=0;i<2;i++) if(tofAlive[i]) sensorSeesCliff(i); }
void initCliffSensors(){
  Wire.begin(PIN_SDA,PIN_SCL); Wire.setClock(I2C_HZ);
  for(int i=0;i<4;i++) sensorPower(i,false); delay(20);
  for(int i=0;i<4;i++){
    sensorPower(i,true); delay(25); tof[i].setTimeout(50);
    if(!tof[i].init()){ tofAlive[i]=false; sensorPower(i,false); continue; }
    tof[i].setAddress(TOF_ADDR[i]);
    if(!i2cPresent(TOF_ADDR[i])){ tofAlive[i]=false; continue; }
    tof[i].setMeasurementTimingBudget(50000); tof[i].startContinuous(0); delay(30);
    int bad=0; for(int k=0;k<5;k++){ uint16_t mm=tof[i].readRangeContinuousMillimeters(); if(tof[i].timeoutOccurred()||mm>=BAD_MM||mm==0)bad++; delay(10); }
    tofAlive[i]=(bad<4); Serial.printf("[cliff %s] %s\n",SIDE_NAME[i],tofAlive[i]?"OK":"брак");
  }
}
void clearCliffLatch(const char*){ cliffLatched=false; cliffCulprit=-1; cliffHitStreak=0; }
const char* navName(NavState s){
  switch(s){
    case NAV_CRUISE:return "cruise"; case NAV_BACKUP:return "backup";
    case NAV_TURN1:return "turn1"; case NAV_SHIFT:return "shift"; case NAV_TURN2:return "turn2";
    case NAV_NUDGE:return "nudge"; case NAV_HOME_TURN:return "home_turn";
    case NAV_HOME_DRIVE:return "home_drive"; case NAV_HOME_ALIGN:return "home_align";
    case NAV_SCAN_CRUISE:return "scan"; default:return "idle";
  }
}

void enterCruise();
void beginTurn90(bool right, NavState st);
void beginNudge(bool right);
void beginBackup();
void beginShift();
void beginScanPass();
void beginHome();
void onLaneEnd();
void finishCoverage();
void handleCliffOnCruise(uint8_t mask);

void enterCruise(){
  ignoreCliffLatchGate=false; clearCliffLatch("c");
  cliffGraceUntilMs = millis()+300;
  resetStraightBaseline(); snapPoseBaseline();
  laneStartPathMm = pathMm;
  setNav(phase==PHASE_SCAN?NAV_SCAN_CRUISE:NAV_CRUISE);
  updateStraightDrive();
  char b[48]; snprintf(b,sizeof(b),"LANE %d /%d",laneIndex,lanesPlanned); emitLine(b);
}
void beginTurn90(bool right, NavState st){
  turnRightDir=right; ignoreCliffLatchGate=true; clearCliffLatch("t");
  beginManeuver(arcMmForDeg(TURN_90_DEG)); snapPoseBaseline(); setNav(st);
  driveTurn(right,turnPwm());
  emitLine(right ? "SERP_RIGHT" : "SERP_LEFT");
  Serial.printf("%s 90 %s lane=%d pwm=%d\n",navName(st),right?"RIGHT":"LEFT",laneIndex,turnPwm());
}
void beginNudge(bool right){
  nudgeRightDir=right; ignoreCliffLatchGate=true; clearCliffLatch("n");
  beginManeuver(arcMmForDeg(NUDGE_MAX_DEG)); snapPoseBaseline(); setNav(NAV_NUDGE);
  driveTurn(right, turnPwm()>NUDGE_PWM?turnPwm()-40:NUDGE_PWM);
}
void beginBackup(){
  ignoreCliffLatchGate=true; clearCliffLatch("b");
  beginManeuver(BACKUP_MM); snapPoseBaseline(); setNav(NAV_BACKUP);
  setMotorsRaw(-BACKUP_PWM,-BACKUP_PWM);
  emitLine("SERP_BACK");
}
void beginShift(){
  ignoreCliffLatchGate=true; clearCliffLatch("s");
  beginManeuver(lanePitchMm); resetStraightBaseline(); snapPoseBaseline();
  setNav(NAV_SHIFT); updateStraightDrive();
  emitLine("SERP_SHIFT");
}
void onLaneEnd(){
  cliffLatched=true; cliffCulprit=0; applyBrakeAll();
  emitCliffEvent("both"); emitLine("LANE_END");
  Serial.printf("LANE_END odo lane=%d dist=%.0f/%.0f turn=%s\n",
                laneIndex, pathMm-laneStartPathMm, laneLenMm,
                serpTurnRight()?"RIGHT":"LEFT");
  beginBackup();
}
void finishCoverage(){
  emitLine("SERP_DONE"); emitLine("MAP_DONE"); applyBrakeAll();
  // home-развороты опрокидывали робота — после покрытия просто стоп
  stopAll();
  if(scanPass) emitLine("SCAN_DONE");
}
void beginScanPass(){
#if DRIVE_ONLY
  // Pi camera broken — coverage done, stay stopped (no second photo pass).
  emitLine("SCAN_SKIPPED");
  emitLine("SCAN_DONE");
  stopAll();
  return;
#else
  scanPass=true; setPhase(PHASE_SCAN); mappingMode=true; laneIndex=0; lastStepMm=pathMm;
  emitLine("CAMERA_ON"); emitLine("SCAN_START"); emitPose(); emitGo(); enterCruise();
#endif
}
void beginHome(){
  setPhase(PHASE_HOME); mappingMode=false; ignoreCliffLatchGate=true; clearCliffLatch("h");
  emitLine("HOME_START");
  float want=atan2f(-poseY,-poseX); float dyaw=wrapPi(want-poseYaw); float deg=fabsf(dyaw)*180/PI;
  if(distToHome()<HOME_RADIUS_MM){
    float a=wrapPi(0-poseYaw); float ad=fabsf(a)*180/PI;
    if(ad<12){ beginScanPass(); return; }
    turnRightDir=a<0; beginManeuver(arcMmForDeg(ad)); snapPoseBaseline();
    setNav(NAV_HOME_ALIGN); driveTurn(turnRightDir,turnPwm()); return;
  }
  if(deg<10){ resetStraightBaseline(); snapPoseBaseline(); setNav(NAV_HOME_DRIVE); updateStraightDrive(); return; }
  turnRightDir=dyaw<0; beginManeuver(arcMmForDeg(deg)); snapPoseBaseline();
  setNav(NAV_HOME_TURN); driveTurn(turnRightDir,turnPwm());
}
void handleCliffOnCruise(uint8_t mask){
  if(mask==3){ onLaneEnd(); return; }
  if(mask==1) beginNudge(true); else if(mask==2) beginNudge(false);
}

void pollCliff(){
  if(!cliffEnabled)return;
  if(millis()<cliffGraceUntilMs){ refreshFrontMm(); return; }
  if(nav==NAV_BACKUP||nav==NAV_TURN1||nav==NAV_TURN2||nav==NAV_NUDGE||nav==NAV_HOME_TURN||nav==NAV_HOME_ALIGN){
    refreshFrontMm(); return;
  }
  bool going=(nav==NAV_CRUISE||nav==NAV_SCAN_CRUISE||nav==NAV_SHIFT||nav==NAV_HOME_DRIVE)||
             (lastLeftPwm>0&&lastRightPwm>0&&!braking);
  if(!going){ refreshFrontMm(); return; }
  uint8_t mask=readFrontCliffMask();
  if(mask==0){ cliffHitStreak=0; return; }
  if(cliffHitStreak<255)cliffHitStreak++;
  if(cliffLatched||cliffHitStreak<CLIFF_NEED)return;

  // Сдвиг полосы: край = доехали до края плиты поперёк → сразу 2-й поворот в новую полосу
  if(nav==NAV_SHIFT){
    cliffLatched=true;
    emitCliffEvent(mask==1?"fl":(mask==2?"fr":"both"));
    applyBrakeAll();
    emitLine("CLIFF_SHIFT");
    if(autoMission){
      beginTurn90(serpTurnRight(), NAV_TURN2);
    } else {
      setNav(NAV_IDLE);
    }
    return;
  }

  if(nav==NAV_CRUISE||nav==NAV_SCAN_CRUISE||nav==NAV_HOME_DRIVE){
    float progressed = pathMm - laneStartPathMm;
    cliffLatched=true;
    emitCliffEvent(mask==1?"fl":(mask==2?"fr":"both"));
    applyBrakeAll();
    Serial.printf("CLIFF STOP mask=%u odo=%.0f/%.0f auto=%d\n",
                  mask, progressed, laneLenMm, (int)autoMission);
    emitLine(mask==3?"CLIFF_BOTH":(mask==1?"CLIFF_FL":"CLIFF_FR"));

    if(autoMission && (nav==NAV_CRUISE || nav==NAV_SCAN_CRUISE)){
      // Всегда змейка у края: назад → 90 → сдвиг → 90 → следующая полоса
      // (без nudge — иначе «просто остановился»)
      emitLine("LANE_END");
      beginBackup();
    } else {
      setNav(NAV_IDLE);
    }
  }
}

void tickNav(){
  if(!autoMission)return;
  updatePoseFromDrive();
  switch(nav){
    case NAV_CRUISE: case NAV_SCAN_CRUISE:
      updateStraightDrive();
      // конец полосы по одометру ИЛИ (датчик в pollCliff). Оба пути → backup→turn→shift
      if (planReady && laneLenMm > 0 && (pathMm - laneStartPathMm) >= laneLenMm) {
        Serial.printf("LANE_END plan dist=%.0f need=%.0f\n", pathMm-laneStartPathMm, laneLenMm);
        onLaneEnd();
      }
      break;
    case NAV_BACKUP:
      setMotorsRaw(-BACKUP_PWM,-BACKUP_PWM);
      // полоса 0,2,… → ВПРАВО; 1,3,… → ВЛЕВО
      if(maneuverDone()){ applyBrakeAll(); beginTurn90(serpTurnRight(), NAV_TURN1); }
      break;
    case NAV_TURN1:
      driveTurn(turnRightDir,turnPwm());
      if(maneuverDone()){ applyBrakeAll(); beginShift(); }
      break;
    case NAV_SHIFT:
      updateStraightDrive();
      if(maneuverDone()&&!cliffLatched){ applyBrakeAll(); beginTurn90(serpTurnRight(), NAV_TURN2); }
      break;
    case NAV_TURN2:
      driveTurn(turnRightDir,turnPwm());
      if(maneuverDone()){
        applyBrakeAll();
        laneIndex++;
        emitLine(laneIndex % 2 == 0 ? "SERP_LANE_UP" : "SERP_LANE_DOWN");
        Serial.printf("SERP next lane %d / %d\n", laneIndex, lanesPlanned);
        if (planReady && laneIndex >= lanesPlanned) finishCoverage();
        else enterCruise();
      }
      break;
    case NAV_NUDGE:{
      driveTurn(nudgeRightDir, turnPwm()>NUDGE_PWM?turnPwm()-30:NUDGE_PWM);
      uint8_t m=readFrontCliffMask();
      bool clear=nudgeRightDir?((m&1)==0):((m&2)==0);
      if(clear||maneuverDone()){
        applyBrakeAll(); m=readFrontCliffMask();
        if(m) onLaneEnd();
        else enterCruise();
      }
      break;}
    case NAV_HOME_TURN:
      driveTurn(turnRightDir,turnPwm());
      if(maneuverDone()){ applyBrakeAll(); resetStraightBaseline(); snapPoseBaseline(); setNav(NAV_HOME_DRIVE); updateStraightDrive(); }
      break;
    case NAV_HOME_DRIVE:{
      updateStraightDrive();
      if(distToHome()<=HOME_RADIUS_MM){
        applyBrakeAll();
        float dyaw=wrapPi(0-poseYaw); float deg=fabsf(dyaw)*180/PI;
        if(deg<12) beginScanPass();
        else { turnRightDir=dyaw<0; beginManeuver(arcMmForDeg(deg)); snapPoseBaseline(); setNav(NAV_HOME_ALIGN); driveTurn(turnRightDir,turnPwm()); }
      } else {
        float want=atan2f(-poseY,-poseX); float err=wrapPi(want-poseYaw);
        if(fabsf(err)>30*PI/180){
          applyBrakeAll(); turnRightDir=err<0;
          beginManeuver(arcMmForDeg(fabsf(err)*180/PI)); snapPoseBaseline();
          setNav(NAV_HOME_TURN); driveTurn(turnRightDir,turnPwm());
        }
      }
      break;}
    case NAV_HOME_ALIGN:
      driveTurn(turnRightDir,turnPwm());
      if(maneuverDone()){ applyBrakeAll(); beginScanPass(); }
      break;
    default: break;
  }
}

void applyPlan(float w, float h){
  surfaceW = w; surfaceH = h;
  lanePitchMm = LANE_PITCH_MM;           // 220 мм — ширина робота
  // ход полосы = высота, минус запас под разворот у края
  // ход полосы = высота поверхности (как в плане сайта). Запас под разворот минимальный.
  laneLenMm = surfaceH - 30.0f;
  if (laneLenMm < 120.0f) laneLenMm = surfaceH > 80.0f ? surfaceH * 0.92f : surfaceH;
  // как inspector/server/app.py: ceil((W-cross)/pitch)+1
  if (surfaceW <= FRAME_CROSS_MM) lanesPlanned = 1;
  else lanesPlanned = (int)ceil((surfaceW - FRAME_CROSS_MM) / lanePitchMm) + 1;
  if (lanesPlanned < 1) lanesPlanned = 1;
  if (surfaceW >= lanePitchMm && lanesPlanned < 2) lanesPlanned = 2;
  planReady = true;
  char b[96];
  snprintf(b,sizeof(b),"PLAN %.0fx%.0f lanes=%d pitch=%.0f len=%.0f",
           surfaceW,surfaceH,lanesPlanned,lanePitchMm,laneLenMm);
  emitLine(b);
  Serial.println(b);
}

void startMission(){
  if (!planReady) applyPlan(surfaceW, surfaceH);
  autoMission=true; mappingMode=true; scanPass=false; laneIndex=0;
  if(speedVal>SPEED_AUTO_MAX)speedVal=SPEED_AUTO_MAX;
  if(speedVal<30)speedVal=SPEED_DEFAULT;
  // край остаётся ВКЛ; сбрасываем ложный latch, чтобы Авто не стояло на старте
  cliffEnabled=true; clearCliffLatch("G");
  resetMapPose(); setPhase(PHASE_MAP);
  emitLine("MAP_START"); emitLine("SERP_START"); emitOdo(); emitPose(); emitGo();
  // при плане не стартуем с «края» — едем по одометру + живые ToF
  enterCruise();
  Serial.printf("SERP start plan %dx%d lanes=%d v=%d cliff=%d tof=%d%d%d%d\n",
                (int)surfaceW,(int)surfaceH,lanesPlanned,speedVal,(int)cliffEnabled,
                (int)tofAlive[0],(int)tofAlive[1],(int)tofAlive[2],(int)tofAlive[3]);
}

bool handlePlanCommand(const String& s){
  if (s.length()<3 || (s[0]!='P' && s[0]!='p')) return false;
  float w=0,h=0;
  if (sscanf(s.c_str(), "%*c%f,%f", &w, &h) != 2) return false;
  if (w < 100 || h < 100 || w > 100000 || h > 100000) {
    Serial.println("PLAN bad size");
    return true;
  }
  applyPlan(w,h);
  return true;
}

void handleCommand(char c){
  switch(toupper(c)){
    case 'G': startMission(); break;
    case 'F':
      autoMission=false; mappingMode=false; setPhase(PHASE_IDLE); ignoreCliffLatchGate=false;
      if(!cliffEnabled){ clearCliffLatch("F"); setNav(NAV_CRUISE); resetStraightBaseline(); updateStraightDrive(); break; }
      if(cliffLatched&&readFrontCliffMask()==0) clearCliffLatch("F");
      if(cliffLatched){ applyBrakeAll(); break; }
      setNav(NAV_CRUISE); resetStraightBaseline(); updateStraightDrive(); break;
    case 'B': stopAll(); setMotorsRaw(-cruiseSpeed(),-cruiseSpeed()); break;
    case 'L': stopAll(); setMotorsRaw(-(cruiseSpeed()+TRIM_LEFT),cruiseSpeed()+TRIM_RIGHT); break;
    case 'R': stopAll(); setMotorsRaw(cruiseSpeed()+TRIM_LEFT,-(cruiseSpeed()+TRIM_RIGHT)); break;
    case 'S': stopAll(); break;
    case 'H': autoMission=true; beginHome(); break;
    case 'C':
      clearCliffLatch("C");
      if(autoMission && (nav==NAV_IDLE || cliffLatched)){
        ignoreCliffLatchGate=false;
        enterCruise();
      }
      break;
    case 'Z': resetMapPose(); emitOdo(); emitPose(); break;
    case 'X': cliffEnabled=true; break; // полный X1/X0 — в handleCliffCommand
    default: break;
  }
}
bool handleMotorCommand(const String& s){
  if(!s.length()||(s[0]!='M'&&s[0]!='m'))return false;
  stopAll(); int l=0,r=0; if(sscanf(s.c_str(),"%*c%d,%d",&l,&r)==2) setMotorsRaw(l,r); return true;
}
bool handleSpeedCommand(const String& s){
  if(s.length()<2||(s[0]!='V'&&s[0]!='v'))return false;
  int v=s.substring(1).toInt(); if(v<30)v=30; if(v>90)v=90; speedVal=v; return true;
}
// X1/X = край ON, X0 = OFF (явное, не toggle)
bool handleCliffCommand(const String& s){
  if(!s.length()||(s[0]!='X'&&s[0]!='x'))return false;
  if(s.length()>=2 && s[1]=='0'){ cliffEnabled=false; clearCliffLatch("off"); }
  else cliffEnabled=true;
  Serial.printf("cliff %s\n", cliffEnabled?"ON":"OFF");
  return true;
}

void sendTelemetry(){
  long lv,ln,rn,rv; noInterrupts(); lv=encLV;ln=encLN;rn=encRN;rv=encRV; interrupts();
  char buf[240]; int len;
  if(cliffLatched)
    len=snprintf(buf,sizeof(buf),"E,%lu,%ld,%ld,%ld,%ld,CLIFF,%s,PWM,%d,%d,ODO,%.1f,NAV,%s,PHASE,%s,LANE,%d,TOF,%u,%u,%u,%u%s",
      millis(),lv,ln,rn,rv,cliffCulprit>=0?SIDE_NAME[cliffCulprit]:"?",lastLeftPwm,lastRightPwm,pathMm,navName(nav),phaseName(phase),laneIndex,lastMm[0],lastMm[1],lastMm[2],lastMm[3],autoMission?",AUTO":"");
  else
    len=snprintf(buf,sizeof(buf),"E,%lu,%ld,%ld,%ld,%ld,PWM,%d,%d,ODO,%.1f,NAV,%s,PHASE,%s,LANE,%d,TOF,%u,%u,%u,%u%s",
      millis(),lv,ln,rn,rv,lastLeftPwm,lastRightPwm,pathMm,navName(nav),phaseName(phase),laneIndex,lastMm[0],lastMm[1],lastMm[2],lastMm[3],autoMission?",AUTO":"");
  if(bleConnected&&pTelemChar&&len>0){ pTelemChar->setValue((uint8_t*)buf,len); pTelemChar->notify(); }
  if(len>0) Serial.println(buf);
}

class ServerCB:public NimBLEServerCallbacks{
  void onConnect(NimBLEServer*, NimBLEConnInfo&)override{ bleConnected=true; /* не stopAll — Pi часто реконнектит */ }
  void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int)override{
    bleConnected=false; stopAll();
    NimBLEDevice::startAdvertising();
    Serial.println("BLE re-adv");
  }
};
class CmdCB:public NimBLECharacteristicCallbacks{
  void onWrite(NimBLECharacteristic*c, NimBLEConnInfo&)override{
    std::string v=c->getValue(); if(v.empty())return;
    String s(v.c_str());
    if(handleMotorCommand(s)||handleSpeedCommand(s)||handlePlanCommand(s)||handleCliffCommand(s))return;
    handleCommand((char)toupper(s[0]));
  }
};

void setup(){
  Serial.begin(115200); Serial.setTimeout(20); delay(150);
  pinMode(DRIVER1_MOTOR1_DIR,OUTPUT); pinMode(DRIVER1_MOTOR1_PWM,OUTPUT);
  pinMode(DRIVER1_MOTOR2_DIR,OUTPUT); pinMode(DRIVER1_MOTOR2_PWM,OUTPUT);
  pinMode(DRIVER2_MOTOR1_DIR,OUTPUT); pinMode(DRIVER2_MOTOR1_PWM,OUTPUT);
  pinMode(DRIVER2_MOTOR2_DIR,OUTPUT); pinMode(DRIVER2_MOTOR2_PWM,OUTPUT);
  applyBrakeAll(); initEncoders(); initCliffSensors(); resetMapPose();
  // Classic BT память не нужна — больше места под BLE controller (S3)
  esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);

  // NimBLE: эфир работает (проверено diag RX+TX). Имя только в adv, без 128-bit UUID.
  NimBLEDevice::init("ESP32_ROBOT");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCB());
  NimBLEService* svc = pServer->createService(SERVICE_UUID);
  pCmdChar = svc->createCharacteristic(
    CHARACTERISTIC_UUID,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  pCmdChar->setCallbacks(new CmdCB());
  pTelemChar = svc->createCharacteristic(
    TELEMETRY_CHAR_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  svc->start();
  pServer->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->setName("ESP32_ROBOT");
  adv->enableScanResponse(false);
  adv->setConnectableMode(BLE_GAP_CONN_MODE_UND);
  adv->setDiscoverableMode(BLE_GAP_DISC_MODE_GEN);
  bool ok = adv->start(0);
  if (!ok) { delay(50); ok = adv->start(0); }
  Serial.printf("BLE adv=%s active=%d addr=%s\n",
    ok ? "OK" : "FAIL", (int)adv->isAdvertising(),
    NimBLEDevice::getAddress().toString().c_str());

  lastTelemMs = lastCtrlMs = lastOdoMs = millis();
  Serial.println("ready: cliff+serpentine+antiR; Pw,h then G");
}

void loop(){
  while(Serial.available()){
    String line=Serial.readStringUntil('\n'); line.trim(); if(!line.length())continue;
    if(handleMotorCommand(line)||handleSpeedCommand(line)||handlePlanCommand(line)||handleCliffCommand(line))continue;
    handleCommand((char)toupper(line[0]));
  }
  uint32_t now=millis();
  if(now-lastCtrlMs>=CTRL_MS){
    lastCtrlMs=now; pollCliff();
    if(autoMission) tickNav();
    else if(nav==NAV_CRUISE){ updatePoseFromDrive(); updateStraightDrive(); }
    pollMappingSteps();
  }
  if(now-lastTelemMs>=TELEM_MS){ lastTelemMs=now; sendTelemetry(); }
  if(mappingMode&&now-lastOdoMs>=ODO_REPORT_MS){ lastOdoMs=now; emitOdo(); emitPose(); }
  // если BLE отвалил рекламу — поднять снова (без этого эфир пустой)
  static uint32_t lastAdvCheck=0;
  if(!bleConnected && now-lastAdvCheck>1000){
    lastAdvCheck=now;
    NimBLEAdvertising* a=NimBLEDevice::getAdvertising();
    if(a && !a->isAdvertising()){ a->start(0); Serial.println("BLE re-adv tick"); }
  }
}
