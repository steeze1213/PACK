#include <AFMotor.h>

AF_DCMotor motor_FR(1);  // M1: 오른쪽 정면, 엔코더 있음
AF_DCMotor motor_FL(2);  // M2: 왼쪽 정면, 엔코더 있음
AF_DCMotor motor_RL(3);  // M3: 왼쪽 후면
AF_DCMotor motor_RR(4);  // M4: 오른쪽 후면

#define ENC_LEFT_A  A0
#define ENC_LEFT_B  A1

#define ENC_RIGHT_A A2
#define ENC_RIGHT_B A3

volatile long leftCount = 0;
volatile long rightCount = 0;

volatile byte lastLeftState = 0;
volatile byte lastRightState = 0;

long prevLeftCount = 0;
long prevRightCount = 0;

unsigned long lastEncoderPrintTime = 0;
const unsigned long ENCODER_PRINT_INTERVAL = 100;

int baseLeftFrontSpeed  = 100;  // M2
int baseRightFrontSpeed = 100;  // M1
int baseLeftRearSpeed   = 60;   // M3
int baseRightRearSpeed  = 60;   // M4

int leftCorrection_LF = 70;    // M2
int leftCorrection_RF = 180;   // M1
int leftCorrection_LR = 55;    // M3
int leftCorrection_RR = 155;   // M4

int rightCorrection_LF = 180;  // M2
int rightCorrection_RF = 70;   // M1
int rightCorrection_LR = 155;  // M3
int rightCorrection_RR = 55;   // M4

int pivotFrontSpeed = 130;
int pivotRearSpeed  = 110;

int rearKickSpeed = 130;
unsigned long rearKickTime = 350;
unsigned long rearKickStartTime = 0;
bool rearKickActive = false;

char currentCommand = 's';
char prevCommand = 's';

const uint8_t M1_FORWARD_DIR = FORWARD;
const uint8_t M2_FORWARD_DIR = FORWARD;
const uint8_t M3_FORWARD_DIR = FORWARD;
const uint8_t M4_FORWARD_DIR = FORWARD;

const uint8_t M1_BACKWARD_DIR = BACKWARD;
const uint8_t M2_BACKWARD_DIR = BACKWARD;
const uint8_t M3_BACKWARD_DIR = BACKWARD;
const uint8_t M4_BACKWARD_DIR = BACKWARD;

void setup() {
  Serial.begin(9600);

  pinMode(ENC_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_LEFT_B, INPUT_PULLUP);
  pinMode(ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_RIGHT_B, INPUT_PULLUP);

  lastLeftState = readLeftStateFast();
  lastRightState = readRightStateFast();

  setupPinChangeInterrupts();

  stopAllMotors();

  Serial.println("READY,F2_4WD_ARUCO_TRUE_PIVOT_MODE");
  Serial.println("M1=RF,M2=LF,M3=LR,M4=RR");
  Serial.println("ENC_LEFT=A0/A1,ENC_RIGHT=A2/A3");
  Serial.println("CMD,w=forward,a=left_curve,d=right_curve,q=pivot_left,e=pivot_right,s=stop,c=clear");
}

void loop() {
  readSerialCommand();

  if (
    currentCommand == 'w'
    || currentCommand == 'a'
    || currentCommand == 'd'
  ) {
    if (
      prevCommand == 's'
      && currentCommand == 'w'
    ) {
      rearKickActive = true;
      rearKickStartTime = millis();
    }

    forwardOrCurve();
  }
  else if (currentCommand == 'q') {
    rearKickActive = false;
    pivotLeft();
  }
  else if (currentCommand == 'e') {
    rearKickActive = false;
    pivotRight();
  }
  else {
    stopAllMotors();
    rearKickActive = false;
  }

  prevCommand = currentCommand;

  publishEncoderData();
}

void setupPinChangeInterrupts() {
  PCICR |= (1 << PCIE1);

  PCMSK1 |= (1 << PCINT8);   // A0: 왼쪽 A
  PCMSK1 |= (1 << PCINT9);   // A1: 왼쪽 B
  PCMSK1 |= (1 << PCINT10);  // A2: 오른쪽 A
  PCMSK1 |= (1 << PCINT11);  // A3: 오른쪽 B
}

ISR(PCINT1_vect) {
  byte leftState = readLeftStateFast();
  byte rightState = readRightStateFast();

  int leftStep = quadratureStep(
    lastLeftState,
    leftState
  );

  int rightStep = quadratureStep(
    lastRightState,
    rightState
  );

  if (leftStep != 0) {
    leftCount += leftStep;
    lastLeftState = leftState;
  }

  if (rightStep != 0) {
    rightCount += rightStep;
    lastRightState = rightState;
  }
}

byte readLeftStateFast() {
  byte pins = PINC;

  byte a = (
    pins & (1 << PC0)
  ) ? 1 : 0;

  byte b = (
    pins & (1 << PC1)
  ) ? 1 : 0;

  return (a << 1) | b;
}

byte readRightStateFast() {
  byte pins = PINC;

  byte a = (
    pins & (1 << PC2)
  ) ? 1 : 0;

  byte b = (
    pins & (1 << PC3)
  ) ? 1 : 0;

  return (a << 1) | b;
}

int quadratureStep(
  byte lastState,
  byte currentState
) {
  if (lastState == currentState) {
    return 0;
  }

  if (
    lastState == 0
    && currentState == 1
  ) {
    return 1;
  }

  if (
    lastState == 1
    && currentState == 3
  ) {
    return 1;
  }

  if (
    lastState == 3
    && currentState == 2
  ) {
    return 1;
  }

  if (
    lastState == 2
    && currentState == 0
  ) {
    return 1;
  }

  if (
    lastState == 0
    && currentState == 2
  ) {
    return -1;
  }

  if (
    lastState == 2
    && currentState == 3
  ) {
    return -1;
  }

  if (
    lastState == 3
    && currentState == 1
  ) {
    return -1;
  }

  if (
    lastState == 1
    && currentState == 0
  ) {
    return -1;
  }

  return 0;
}

void readSerialCommand() {
  if (Serial.available() <= 0) {
    return;
  }

  char cmd = Serial.read();

  if (
    cmd == '\n'
    || cmd == '\r'
    || cmd == ' '
  ) {
    return;
  }

  if (
    cmd == 'w'
    || cmd == 'W'
  ) {
    currentCommand = 'w';
    Serial.println("ACK,FORWARD");
  }
  else if (
    cmd == 'a'
    || cmd == 'A'
  ) {
    currentCommand = 'a';
    rearKickActive = false;
    Serial.println("ACK,LEFT_CURVE_SOFT");
  }
  else if (
    cmd == 'd'
    || cmd == 'D'
  ) {
    currentCommand = 'd';
    rearKickActive = false;
    Serial.println("ACK,RIGHT_CURVE_SOFT");
  }
  else if (
    cmd == 'q'
    || cmd == 'Q'
  ) {
    currentCommand = 'q';
    rearKickActive = false;
    Serial.println("ACK,TRUE_PIVOT_LEFT");
  }
  else if (
    cmd == 'e'
    || cmd == 'E'
  ) {
    currentCommand = 'e';
    rearKickActive = false;
    Serial.println("ACK,TRUE_PIVOT_RIGHT");
  }
  else if (
    cmd == 's'
    || cmd == 'S'
  ) {
    currentCommand = 's';
    rearKickActive = false;
    Serial.println("ACK,STOP");
  }
  else if (
    cmd == 'c'
    || cmd == 'C'
  ) {
    clearEncoderCounts();
    Serial.println("ACK,CLEAR");
  }
}

void forwardOrCurve() {
  int m2Speed = baseLeftFrontSpeed;
  int m1Speed = baseRightFrontSpeed;
  int m3Speed = baseLeftRearSpeed;
  int m4Speed = baseRightRearSpeed;

  if (currentCommand == 'a') {
    m2Speed = leftCorrection_LF;
    m1Speed = leftCorrection_RF;
    m3Speed = leftCorrection_LR;
    m4Speed = leftCorrection_RR;
  }
  else if (currentCommand == 'd') {
    m2Speed = rightCorrection_LF;
    m1Speed = rightCorrection_RF;
    m3Speed = rightCorrection_LR;
    m4Speed = rightCorrection_RR;
  }

  if (
    currentCommand == 'w'
    && rearKickActive
  ) {
    if (
      millis() - rearKickStartTime
      <= rearKickTime
    ) {
      m3Speed = rearKickSpeed;
      m4Speed = rearKickSpeed;
    }
    else {
      rearKickActive = false;
    }
  }

  motor_FL.setSpeed(m2Speed);
  motor_FR.setSpeed(m1Speed);
  motor_RL.setSpeed(m3Speed);
  motor_RR.setSpeed(m4Speed);

  motor_FL.run(M2_FORWARD_DIR);
  motor_FR.run(M1_FORWARD_DIR);
  motor_RL.run(M3_FORWARD_DIR);
  motor_RR.run(M4_FORWARD_DIR);
}

void pivotLeft() {
  motor_FL.setSpeed(pivotFrontSpeed);
  motor_FR.setSpeed(pivotFrontSpeed);

  motor_RL.setSpeed(pivotRearSpeed);
  motor_RR.setSpeed(pivotRearSpeed);

  motor_FL.run(M2_BACKWARD_DIR);
  motor_RL.run(M3_BACKWARD_DIR);

  motor_FR.run(M1_FORWARD_DIR);
  motor_RR.run(M4_FORWARD_DIR);
}

void pivotRight() {
  motor_FL.setSpeed(pivotFrontSpeed);
  motor_FR.setSpeed(pivotFrontSpeed);

  motor_RL.setSpeed(pivotRearSpeed);
  motor_RR.setSpeed(pivotRearSpeed);

  motor_FL.run(M2_FORWARD_DIR);
  motor_RL.run(M3_FORWARD_DIR);

  motor_FR.run(M1_BACKWARD_DIR);
  motor_RR.run(M4_BACKWARD_DIR);
}

void stopAllMotors() {
  motor_FL.run(RELEASE);
  motor_FR.run(RELEASE);
  motor_RL.run(RELEASE);
  motor_RR.run(RELEASE);
}

void clearEncoderCounts() {
  noInterrupts();

  leftCount = 0;
  rightCount = 0;

  prevLeftCount = 0;
  prevRightCount = 0;

  lastLeftState = readLeftStateFast();
  lastRightState = readRightStateFast();

  interrupts();
}

long getLeftCount() {
  long value;

  noInterrupts();
  value = leftCount;
  interrupts();

  return value;
}

long getRightCount() {
  long value;

  noInterrupts();
  value = rightCount;
  interrupts();

  return value;
}

void publishEncoderData() {
  if (
    millis() - lastEncoderPrintTime
    < ENCODER_PRINT_INTERVAL
  ) {
    return;
  }

  long left = getLeftCount();
  long right = getRightCount();

  long leftDelta = (
    left - prevLeftCount
  );

  long rightDelta = (
    right - prevRightCount
  );

  prevLeftCount = left;
  prevRightCount = right;

  Serial.print("ENC,");
  Serial.print(left);
  Serial.print(",");
  Serial.print(right);
  Serial.print(",");
  Serial.print(leftDelta);
  Serial.print(",");
  Serial.print(rightDelta);
  Serial.print(",");
  Serial.println(currentCommand);

  lastEncoderPrintTime = millis();
}
