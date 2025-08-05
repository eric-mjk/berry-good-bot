#include <Arduino.h>
#include <Servo.h>

/* ────── 하드웨어 핀 ────── */
#define DIR_PIN     9   // 스텝퍼 DIR
#define STEP_PIN    6   // 스텝퍼 STEP
#define LIMIT_PIN   7   // 리밋 스위치 (눌리면 LOW)

const int wristServoPin  = 11;
const int gripServoPin   = 3;

/* ────── 스텝퍼 모션 파라미터 ────── */
const float STEPS_PER_REV = 2415.0;   // 한 회전 스텝
const float SCREW_LEAD_MM = 4.0;      // 한 회전 이동(mm)
const float STEPS_PER_MM  = STEPS_PER_REV / SCREW_LEAD_MM;

const float MAX_VEL_STEP_PER_S   = 16000.0;   // ★조정: 최대 속도(스텝/s)
const float MAX_ACCEL_STEP_PER_S2 = 8000.0;  // ★조정: 최대 가속(스텝/s²)

/* ────── 서보 파라미터 ────── */
const int  ANGLE_GRIPPER_OPEN   = 150;
const int  ANGLE_GRIPPER_CLOSED = 75;
const float MAX_SERVO_DEG_PER_SEC = 180.0;  // ★조정: 서보 속도 한계

/* ────── 런타임 상태 ────── */
volatile long  curStepPos = 0;   // 절대 위치(스텝)
volatile long  targetStepPos = 0;

float   curStepVel  = 0;         // 현재 속도(스텝/s)
unsigned long lastStepTimeUs = 0; // 마지막 스텝 발생 시각

Servo wristServo, gripServo;
volatile float curWristDeg = 0;      // -90~90°
volatile float targetWristDeg = 0;

volatile bool  curGripOpen = false;
volatile bool  targetGripOpen = false;

/* ────── 통신 출력 타이머 ────── */
const unsigned long STATUS_INTERVAL_MS = 100;
unsigned long lastStatusMs = 0;

/* ────── 수신 버퍼 (비블로킹 파싱용) ────── */
char recvBuf[32];
uint8_t recvPos = 0;

/* ───────────────────────────────────────────────────────────── */
/*                        기본 함수 선언                         */
void homeAxis();
void processSerial();
void updateStepper();
void updateServos();
void sendStatus();

/* ───────────────────────────────────────────────────────────── */
void setup() {
  /* 핀 초기화 */
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(LIMIT_PIN, INPUT_PULLUP);

  digitalWrite(DIR_PIN, LOW);
  digitalWrite(STEP_PIN, LOW);

  Serial.begin(115200);
  while (!Serial) {}

  /* 서보 초기화 */
  wristServo.attach(wristServoPin);
  gripServo.attach(gripServoPin);

  /* Homing */
  homeAxis();

  /* 초기 서보 위치 */
  curWristDeg = targetWristDeg = 0;
  wristServo.write(90);               // 중앙
  curGripOpen = targetGripOpen = false;
  gripServo.write(ANGLE_GRIPPER_CLOSED);

  Serial.println(F("\n=== 명령 형식: \"Z_mm W_deg G(0/1)\" ==="));
  Serial.println(F("   예) -120.0  30  1"));
  Serial.println(F("----------------------------------------"));
}

void loop() {
  processSerial();   // (1) 새 명령 수신
  updateStepper();   // (2) 스텝퍼 한-스텝/가속도 관리
  updateServos();    // (3) 서보 각도 점진 이동
  sendStatus();      // (4) 주기적 상태 전송
}

/* ───────────────────────────────────────────────────────────── */
/*                     1) 시리얼 명령 처리                       */
// void processSerial() {
//   if (!Serial.available()) return;

//   // String line = Serial.readStringUntil('\n');
//   // line.trim();
//   // if (line.length() == 0) return;

//   // // 공백 구분자 세 개 값 기대
//   // float zMm=0, wDeg=0;
//   // int   gCmd=0;
//   // int n = sscanf(line.c_str(), "%f %f %d", &zMm, &wDeg, &gCmd);
//   // if (n != 3) {
//   //   Serial.print(F("❌ 잘못된 입력: ")); Serial.println(line); Serial.println(n);
//   //   return;
//   // }
  
//   String line = Serial.readStringUntil('\n');
//   line.trim();
//   if (line.length() == 0) return;

//   // 공백 구분자로 세 개 값 분리
//   int sep1 = line.indexOf(' ');
//   int sep2 = line.indexOf(' ', sep1 + 1);
//   if (sep1 == -1 || sep2 == -1) {
//     Serial.print(F("❌ 잘못된 입력: "));
//     Serial.println(line);
//     return;
//   }
//   float zMm = line.substring(0, sep1).toFloat();
//   float wDeg = line.substring(sep1 + 1, sep2).toFloat();
//   int   gCmd = line.substring(sep2 + 1).toInt();

//   /* Z 축 범위 체크 */
//   if (zMm > 0 || zMm < -590) {
//     Serial.println(F("⚠️ Z 범위(0 ~ -590 mm) 초과"));
//   } else {
//     targetStepPos = lround(zMm * STEPS_PER_MM);
//   }

//   /* 손목 범위 체크 */
//   if (wDeg < -90 || wDeg > 90) {
//     Serial.println(F("⚠️ W 범위(-90 ~ 90°) 초과"));
//   } else {
//     targetWristDeg = wDeg;
//   }

//   /* 그리퍼 */
//   targetGripOpen = (gCmd == 1);

//   // Serial.print(F("🆗 목표 → Z:")); Serial.print(zMm,1);
//   // Serial.print(F("mm  W:"));      Serial.print(wDeg,0);
//   // Serial.print(F("°  G:"));       Serial.println(targetGripOpen?1:0);
// }

void processSerial() {
  // 시리얼 버퍼에 데이터가 있으면 한 글자씩 읽어 처리
  while (Serial.available()) {
    // // ▶ 디버그: 읽은 문자 출력
    // char dbgChar = Serial.peek();
    // Serial.print(F("[DBG] Serial.peek(): '"));
    // Serial.print(dbgChar);
    // Serial.println(F("'"));

    char c = Serial.read();
    // 'a' 문자를 명령 종료자로 사용 (또는 버퍼 풀 시)
    if (c == 'a' || recvPos >= sizeof(recvBuf) - 1) {
      recvBuf[recvPos] = '\0';           // 문자열 종료
      // // ▶ 디버그: 완성된 버퍼 내용 출력
      // Serial.print(F("[DBG] Complete recvBuf: \""));
      // Serial.print(recvBuf);
      // Serial.println(F("\""));

      // 빈 줄 무시
      if (recvPos > 0) {
        // 공백으로 세 토큰 분리
        // // ▶ 디버그: 토큰 분리 전 원본 버퍼 복사
        // char tmpBuf[32];
        // strcpy(tmpBuf, recvBuf);
        // Serial.print(F("[DBG] Tokenizing tmpBuf: \""));
        // Serial.print(tmpBuf);
        // Serial.println(F("\""));

        char *p1 = strtok(recvBuf, " ");
        char *p2 = strtok(NULL, " ");
        char *p3 = strtok(NULL, " ");
        // // ▶ 디버그: 파싱된 토큰 출력
        // Serial.print(F("[DBG] p1=\"")); Serial.print(p1); Serial.println(F("\""));
        // Serial.print(F("[DBG] p2=\"")); Serial.print(p2); Serial.println(F("\""));
        // Serial.print(F("[DBG] p3=\"")); Serial.print(p3); Serial.println(F("\""));

        if (!p1 || !p2 || !p3) {
          Serial.print(F("❌ 잘못된 입력: "));
          Serial.println(recvBuf);
        } else {
          float zMm = atof(p1);
          float wDeg = atof(p2);
          int   gCmd = atoi(p3);
          // Z 축 범위 체크
          if (zMm > 0 || zMm < -590) {
            Serial.println(F("⚠️ Z 범위(0 ~ -590 mm) 초과"));
          } else {
            targetStepPos = lround(zMm * STEPS_PER_MM);
          }
          // 손목 범위 체크
          if (wDeg < -90 || wDeg > 90) {
            Serial.println(F("⚠️ W 범위(-90 ~ 90°) 초과"));
          } else {
            targetWristDeg = wDeg;
          }
          // 그리퍼
          targetGripOpen = (gCmd == 1);
        }
      }
      recvPos = 0;                       // 버퍼 리셋
    } else if (c >= 32 && c != 'a') {   // 'a'는 저장하지 않음
      // // ▶ 디버그: 버퍼에 추가될 문자 출력
      // Serial.print(F("[DBG] Appending to recvBuf: '"));
      // Serial.print(c);
      // Serial.println(F("'"));

      recvBuf[recvPos++] = c;           // 가시문자만 저장
    }
    
  }
}

/* ───────────────────────────────────────────────────────────── */
/*                     2) 스텝퍼 업데이트                         */
void updateStepper() {
  long   delta = targetStepPos - curStepPos;
  // if (delta == 0 && curStepVel == 0) return;          // 정지 상태
  if (delta == 0) {
    curStepVel = 0;                                  // 완전 정지
    return;
  }

  /* 이동 방향 & 목표 속도 */
  int dirSign = (delta > 0) ? 1 : -1;
  float desiredVel = dirSign * MAX_VEL_STEP_PER_S;
  // Serial.print(targetStepPos);
  // Serial.print(curStepPos);
  // Serial.println(dirSign);

  /* 가속도 제한으로 속도 업데이트 */
  unsigned long nowUs = micros();
  float dt = (nowUs - lastStepTimeUs) / 1e6;          // s
  if (dt == 0) dt = 1e-6;
  if (curStepVel < desiredVel) {
    curStepVel += MAX_ACCEL_STEP_PER_S2 * dt;
    if (curStepVel > desiredVel) curStepVel = desiredVel;
  } else if (curStepVel > desiredVel) {
    curStepVel -= MAX_ACCEL_STEP_PER_S2 * dt;
    if (curStepVel < desiredVel) curStepVel = desiredVel;
  }

  /* 마지막 단계에서 감속(목표까지 남은 거리 대비) */
  float reqBrakingDist = (curStepVel*curStepVel) / (2*MAX_ACCEL_STEP_PER_S2);
  if (abs(delta) < reqBrakingDist) {
    desiredVel = dirSign * sqrt(2 * MAX_ACCEL_STEP_PER_S2 * abs(delta));
    if (curStepVel > desiredVel)
      curStepVel -= MAX_ACCEL_STEP_PER_S2 * dt;
  }

  /* 스텝 펄스 타이밍 계산 */
  float stepIntervalUsF = 1e6 / abs(curStepVel);
  static float intervalCarry = 0;  // 소수 누적
  unsigned long stepIntervalUs = (unsigned long)(stepIntervalUsF + intervalCarry);
  intervalCarry = (stepIntervalUsF + intervalCarry) - stepIntervalUs; // 잔여

  if (abs(curStepVel) < 1) return;  // 아직 너무 느려 스텝 안찍음

  if (nowUs - lastStepTimeUs >= stepIntervalUs) {
    /* 리밋 스위치 보호 */
    if (digitalRead(LIMIT_PIN) == HIGH && dirSign > 0) { // 아래쪽으로 더 가면 안 됨
      curStepVel = 0;
      targetStepPos = curStepPos;      // 목표도 여기로
      Serial.println(F("⚠️ 리밋 스위치 감지 → Z 정지"));
      return;
    }

    /* DIR 설정 */
    digitalWrite(DIR_PIN, (dirSign > 0) ? HIGH : LOW);

    /* 펄스 출력 */
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(50);
    digitalWrite(STEP_PIN, LOW);

    curStepPos += dirSign;
    lastStepTimeUs = nowUs;
  }
}

/* ───────────────────────────────────────────────────────────── */
/*                     3) 서보 업데이트                           */
void updateServos() {
  /* 손목 */
  if (curWristDeg != targetWristDeg) {
    float dt = (millis() - lastStatusMs) / 1000.0;      // rough loop dt
    if (dt == 0) dt = 0.001;
    float step = MAX_SERVO_DEG_PER_SEC * dt;
    if (abs(targetWristDeg - curWristDeg) < step)
      curWristDeg = targetWristDeg;
    else
      curWristDeg += (targetWristDeg > curWristDeg ? step : -step);

    int servoVal = constrain(lround(curWristDeg + 90), 0, 180);
    wristServo.write(servoVal);
  }

  /* 그리퍼 */
  if (curGripOpen != targetGripOpen) {
    curGripOpen = targetGripOpen;
    gripServo.write(curGripOpen ? ANGLE_GRIPPER_OPEN
                                : ANGLE_GRIPPER_CLOSED);
  }
}

/* ───────────────────────────────────────────────────────────── */
/*                     4) 상태 메시지 전송                        */
void sendStatus() {
  unsigned long nowMs = millis();
  if (nowMs - lastStatusMs < STATUS_INTERVAL_MS) return;
  lastStatusMs = nowMs;

  float zmm = curStepPos / STEPS_PER_MM;
  Serial.print(F("POS Z:")); Serial.print(zmm,1);
  Serial.print(F(" W:"));    Serial.print(curWristDeg,0);
  Serial.print(F(" G:"));    Serial.println(curGripOpen?1:0);
}

/* ───────────────────────────────────────────────────────────── */
/*                         Homing 함수                           */
void homeAxis() {
  Serial.println(F(">>> Homing 시작"));
  digitalWrite(DIR_PIN, HIGH);    // 스위치 방향(위쪽)으로
  delay(100);

  while (digitalRead(LIMIT_PIN) == LOW) {     // LOW면 아직 미감지
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(50);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(50);
  }

  curStepPos = targetStepPos = 0;
  curStepVel = 0;
  Serial.println(F(">>> 리밋 스위치 감지 → 원점 설정"));
  
  /* 스위치에서 10 mm 이탈 */
  digitalWrite(DIR_PIN, LOW);
  for (long i=0;i< (long)(100*STEPS_PER_MM); i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(50);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(50);
    curStepPos--;
  }
  targetStepPos = curStepPos;
  Serial.println(F(">>> Homing 완료"));
}
