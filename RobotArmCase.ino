#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <SoftwareSerial.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

// ── ESP-01 무선 브리지 (SoftwareSerial: 핀2=RX<-ESP_TX, 핀3=TX->ESP_RX) ──
//   ESP-01이 MQTT로 받은 명령을 시리얼로 넘겨줌. USB(하드웨어 Serial)는 디버그용 유지.
SoftwareSerial espSerial(2, 3);  // RX, TX

// ── 채널 정의 (집게=5, 손목회전=7, 손목굽힘=9, 팔꿈치=11, 어깨=13, 베이스=15) ──
#define CH_GRIPPER    5
#define CH_WRIST_R    7
#define CH_WRIST_P    9
#define CH_ELBOW     11
#define CH_SHOULDER  13
#define CH_BASE      15

// ── 베이스 방향 (CH15 서보값) — 실제 마운트 보고 테스트로 조정 ──
//   팔 좌측후면 마운트 기준. '15 X' 명령으로 팔이 어디 가리키는지 확인 후 숫자 수정.
#define BASE_PICK    40   // 물건 바라보는 방향 (집기) ← 좌측 물건
#define BASE_PLACE  150   // 후면 차량 방향 (홈/이동/놓기)

// ── 링크 길이(mm) & 베이스 위험영역 ──
#define L1     130.0   // 어깨 -> 팔꿈치
#define L2      65.0   // 팔꿈치 -> 손목
#define L3      60.0   // 손목 -> 집게 끝
#define R_BASE  70.0   // 디스크 반경 + 안전마진
#define H_SHLDR 40.0   // 어깨축 높이(디스크 윗면 기준, 추정 — 실측시 교체)

// ── 채널별 보정 (pulseMin, pulseMax, safeMin, safeMax, home) ──
struct ServoCal { int pMin, pMax, sMin, sMax, home; };
ServoCal cal[16];

void initCal() {
  for (int i = 0; i < 16; i++) cal[i] = {500, 2500, 0, 180, 90};
  //                               pMin  pMax  sMin sMax home
  cal[CH_GRIPPER]   = {500, 2500,   0,  50,   0};  // 0=열림  50=닫힘
  cal[CH_WRIST_R]   = {500, 2500,  40, 130, 130};  // 40=세로 130=가로
  cal[CH_WRIST_P]   = {500, 2500,  70, 170,  90};  // 170=일자 70=숙임
  cal[CH_ELBOW]     = {400, 2600,   0,  90,  60};  // 0=일자  90=굽힘
  cal[CH_SHOULDER]  = {400, 2600,  20, 140, 120};  // 100=수직 20=아래 140=뒤로젖힘
  cal[CH_BASE]      = {400, 2600,   0, 180,  90};  // 0=왼  90=뒤  180=오른
}

int  curAngle[16];
bool initialized[16];
int  moveDelay = 25;   // 1도당 ms (클수록 느림)
bool ems = false;

// ── 각도 -> PCA 틱 ──
int angleToTicks(int ch, int angle) {
  int us = map(angle, 0, 180, cal[ch].pMin, cal[ch].pMax);
  return (int)((long)us * 4096L * 50L / 1000000L);
}

// ── FK: 어깨 기준 집게 끝 좌표 (수직평면) ──
void fkTip(int a13, int a11, int a9, float &gx, float &gz) {
  float ps = radians(a13 - 10);
  float pe = ps - radians(a11);
  float pw = pe - radians(170 - a9);
  float ex = L1 * cos(ps),         ez = L1 * sin(ps);
  float wx = ex + L2 * cos(pe),    wz = ez + L2 * sin(pe);
  gx = wx + L3 * cos(pw);          gz = wz + L3 * sin(pw);
}

// ── 충돌 판정: 집게 끝이 베이스 위험구역(아래+반경 안) 진입 ──
bool tipCollide(int a13, int a11, int a9) {
  float gx, gz;
  fkTip(a13, a11, a9, gx, gz);
  return (gz < -H_SHLDR) && (fabs(gx) < R_BASE);
}

// ── 긴급정지 확인 (USB + ESP 무선 양쪽 감시) ──
void checkEMS() {
  if (Serial.available()) {
    String peek = Serial.readStringUntil('\n');
    peek.trim();
    if (peek == "0") { ems = true; Serial.println("!!! EMS !!!"); }
  }
  if (espSerial.available()) {
    String peek = espSerial.readStringUntil('\n');
    peek.trim();
    if (peek == "0") { ems = true; Serial.println("!!! EMS (무선) !!!"); }
  }
}

// ── 부드러운 이동 (EMS + 팔관절 실시간 충돌가드) ──
void smoothMove(int ch, int target) {
  if (ems) return;
  target = constrain(target, cal[ch].sMin, cal[ch].sMax);

  if (!initialized[ch]) {
    pca.setPWM(ch, 0, angleToTicks(ch, target));
    curAngle[ch] = target; initialized[ch] = true;
    Serial.print("CH"); Serial.print(ch); Serial.print(" 초기 -> "); Serial.println(target);
    return;
  }

  bool armJoint = (ch == CH_SHOULDER || ch == CH_ELBOW || ch == CH_WRIST_P);
  int step = (target > curAngle[ch]) ? 1 : -1;

  for (int a = curAngle[ch]; a != target; a += step) {
    // 팔 관절이면 이동 전 충돌 예측
    if (armJoint) {
      int a13 = (ch == CH_SHOULDER) ? a : curAngle[CH_SHOULDER];
      int a11 = (ch == CH_ELBOW)    ? a : curAngle[CH_ELBOW];
      int a9  = (ch == CH_WRIST_P)  ? a : curAngle[CH_WRIST_P];
      if (tipCollide(a13, a11, a9)) {
        Serial.print("!! 충돌가드 정지 @ CH"); Serial.print(ch);
        Serial.print(" "); Serial.println(a);
        curAngle[ch] = a;
        return;
      }
    }
    pca.setPWM(ch, 0, angleToTicks(ch, a));
    delay(moveDelay);
    checkEMS();
    if (ems) { curAngle[ch] = a; return; }
  }
  pca.setPWM(ch, 0, angleToTicks(ch, target));
  curAngle[ch] = target;
  Serial.print("CH"); Serial.print(ch); Serial.print(" -> "); Serial.println(target);
}

// ── 팔(어깨/팔꿈치/손목) 안전 순서 이동 ──
//    펴는 방향(올림)은 어깨먼저, 접는 방향(내림)은 손목먼저 -> 충돌 최소화
void moveArm(int a13, int a11, int a9) {
  bool lowering = (a13 < curAngle[CH_SHOULDER]);  // 어깨 내리는 동작인가
  if (lowering) {                                  // 내릴 땐 끝단부터 접기
    smoothMove(CH_WRIST_P, a9);  if (ems) return;
    smoothMove(CH_ELBOW,   a11); if (ems) return;
    smoothMove(CH_SHOULDER,a13);
  } else {                                         // 올릴 땐 어깨부터 펴기
    smoothMove(CH_SHOULDER,a13); if (ems) return;
    smoothMove(CH_ELBOW,   a11); if (ems) return;
    smoothMove(CH_WRIST_P, a9);
  }
}

// ────────── 자세 라이브러리 (case별) ──────────
// 형식: moveArm(어깨13, 팔꿈치11, 손목9)
void poseHome() {                    // 대기: 위로 컴팩트
  moveArm(120, 60, 90); if (ems) return;
  smoothMove(CH_WRIST_R, 130);
  smoothMove(CH_BASE, BASE_PLACE);
  smoothMove(CH_GRIPPER, 0);
  Serial.println(">> HOME");
}
void poseReady(int baseDir) {        // 준비: 방향 조준 + 팔 들고 집게 열기
  smoothMove(CH_GRIPPER, 0);
  smoothMove(CH_BASE, baseDir);
  moveArm(90, 30, 120);
  Serial.println(">> READY");
}
void posePickDown() {                // 물건 향해 팔 뻗기 (집게 열린 채)
  smoothMove(CH_GRIPPER, 0);
  moveArm(30, 30, 180);              // 실측 도달 자세 (수평 ~246mm)
  Serial.println(">> PICK_DOWN");
}
void doGrip()    { smoothMove(CH_GRIPPER, 50); Serial.println(">> GRIP"); }
void doRelease() { smoothMove(CH_GRIPPER, 0);  Serial.println(">> RELEASE"); }
void poseLift()  { moveArm(120, 40, 120); Serial.println(">> LIFT"); }   // 들어올리기
void poseCarry() {                   // 이동중: 뒤보고 컴팩트(물건 쥔 채)
  smoothMove(CH_BASE, BASE_PLACE);
  moveArm(120, 60, 100);
  Serial.println(">> CARRY");
}
void posePlaceBack() {               // 후면 차량에 내려놓기 (수평 ~246mm 뻗음)
  smoothMove(CH_BASE, BASE_PLACE);
  moveArm(30, 30, 180); if (ems) return;
  smoothMove(CH_GRIPPER, 0);
  Serial.println(">> PLACE_BACK");
}
void posePlaceSide(int baseDir) {    // 좌/우 내려놓기 (한 칸 더 뻗어 차체 밖 ~197mm)
  smoothMove(CH_BASE, baseDir);      // 0=왼쪽, 180=오른쪽
  moveArm(60, 30, 120); if (ems) return;
  smoothMove(CH_GRIPPER, 0);
  Serial.println(">> PLACE_SIDE");
}

// ── 자동 피킹 시퀀스 (지정 방향에서 집기 -> 항상 후면 차량에 싣기) ──
//    pickDir: 0=왼쪽 90=뒤 180=오른쪽
void pickSequence(int pickDir) {
  poseReady(pickDir); if (ems) return;   // 집을 방향 조준
  posePickDown();     if (ems) return;
  doGrip();           if (ems) return;
  poseLift();         if (ems) return;
  poseCarry();        if (ems) return;   // 뒤로 회전(컴팩트)
  posePlaceBack();    if (ems) return;   // 후면 차량에 싣기
  poseHome();
  Serial.println(">>> 시퀀스 완료");
}

void setup() {
  Serial.begin(9600);
  espSerial.begin(9600);   // ESP-01 무선 브리지
  initCal();
  pca.begin();
  pca.setPWMFreq(50);
  delay(10);
  for (int i = 0; i < 16; i++) { curAngle[i] = 90; initialized[i] = false; }

  Serial.println("=== 로봇팔 제어 (FK 충돌가드 + EMS) ===");
  Serial.println("채널 직접 : '13 90'");
  Serial.println("자세 명령 : H=홈 RL/RB/RR=준비(좌/뒤/우) P=내리기 G=집기 O=놓기 L=들기 C=이동");
  Serial.println("놓기      : B=뒤 BL=왼쪽 BR=오른쪽");
  Serial.println("자동      : K(=KR)=집기시퀀스(오른쪽 집기 -> 후면 싣기)");
  Serial.println("속도      : 'SP 30'   긴급정지: '0'   해제: 'R'");

  // ── 시작 보정: 첫 동작이 한 번에 확 튀는 것 방지 ──
  //   1) 부팅 직후 서보 위치를 모르므로 '홈 값'으로 한 번 즉시 초기화(initialized=true)
  //   2) 그 뒤 RB -> H 를 부드럽게(smoothMove) 실행해 정렬
  Serial.println(">> 시작 보정 중 (홈 초기화 -> RB -> H)...");
  int homeCh[6]  = {CH_BASE,    CH_SHOULDER, CH_ELBOW, CH_WRIST_P, CH_WRIST_R, CH_GRIPPER};
  int homeAng[6] = {BASE_PLACE, 120,         60,       90,         130,        0};
  for (int i = 0; i < 6; i++) {
    int ch = homeCh[i];
    pca.setPWM(ch, 0, angleToTicks(ch, homeAng[i]));
    curAngle[ch]    = homeAng[i];
    initialized[ch] = true;          // 이제부터 모든 이동은 부드럽게
  }
  delay(500);
  poseReady(BASE_PLACE);   // 후면(놓기) 방향 준비 자세로 부드럽게
  delay(300);
  poseHome();              // H: 홈 정렬
  Serial.println(">> 보정 완료. 명령 대기.");
}

// ── 명령 한 줄 처리 (USB/ESP 공통) ──
void processCommand(String line) {
  line.trim();
  if (line.length() == 0) return;

  // 긴급정지
  if (line == "0") { ems = true; Serial.println("!!! EMS !!! — 'R'로 재개"); return; }

  String up = line; up.toUpperCase();

  if (up == "R")  { ems = false; Serial.println("EMS 해제"); return; }
  if (ems)        { Serial.println("EMS 중 — 'R'로 재개"); return; }

  // 자세 명령
  if (up == "H")  { poseHome();       return; }
  if (up == "RL") { poseReady(0);     return; }  // 왼쪽놓기
  if (up == "RB") { poseReady(90);    return; }  // 뒤
  if (up == "RR") { poseReady(180);   return; }  // 오른쪽
  if (up == "P")  { posePickDown();   return; }
  if (up == "G")  { doGrip();         return; }
  if (up == "O")  { doRelease();      return; }
  if (up == "L")  { poseLift();       return; }
  if (up == "C")  { poseCarry();      return; }
  if (up == "BL") { posePlaceSide(0);   return; }  // 왼쪽 놓기
  if (up == "BR") { posePlaceSide(180); return; }  // 오른쪽 놓기
  if (up == "B")  { posePlaceBack();  return; }
  if (up == "KR") { pickSequence(BASE_PICK); return; }   // 집기 -> 후면 싣기
  if (up == "K")  { pickSequence(BASE_PICK); return; }   // K = KR 동일 (집기 한 방향만)

  // 속도
  if (up.startsWith("SP")) {
    int v = up.substring(2).toInt();
    if (v >= 5 && v <= 200) { moveDelay = v; Serial.print("속도 -> "); Serial.println(moveDelay); }
    return;
  }

  // 채널 직접 제어 (13 90)
  line.replace(",", " ");
  up = line; up.toUpperCase(); up.replace("CH", ""); up.trim();
  int sp = up.indexOf(' ');
  if (sp < 0) { Serial.println("형식: 13 90"); return; }
  int ch  = up.substring(0, sp).toInt();
  int ang = up.substring(sp + 1).toInt();
  if (ch < 0 || ch > 15) { Serial.println("채널 0~15"); return; }
  smoothMove(ch, ang);
}

void loop() {
  // USB(시리얼 모니터/Python) 명령
  if (Serial.available()) {
    processCommand(Serial.readStringUntil('\n'));
  }
  // ESP-01 무선(MQTT) 명령
  if (espSerial.available()) {
    processCommand(espSerial.readStringUntil('\n'));
  }
}
