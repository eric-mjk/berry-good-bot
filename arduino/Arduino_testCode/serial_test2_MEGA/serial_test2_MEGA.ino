/* ───────────────────── 핀 매핑 ───────────────────── */
const int StepPin[3]  = {2, 3, 4};
const int DirPin[3]   = {5, 6, 7};
const int LimitPin[3] = {9,45,11};      // LOW = 스위치 ON

/* ───────────────── 스텝/기어 파라미터 ─────────────── */
const int   STEPS_PER_REV = 200;         // 모터 1회전 스텝
const float gearRatio[3]  = {20.0, 20.0, 5.0}; // ★수정
const float initAngle[3]  = {0, 0, 0};

/* ──────── 조인트 최대 속도·가속도(각도 기준) ──────── */
const float maxVelDeg[3]  = {40, 40, 40};     // °/s (★조정)
const float maxAccDeg[3]  = {20, 20, 20};  // °/s² (★조정)

/* ─────────── 허용 각도 범위 (min, max) ──────────── */
const float angleLim[3][2] = {{0,130}, {0,250}, {-200,0}};

/* ─────────── 내부 상태 변수 ─────────── */
struct Axis {
  long   curSteps      = 0;     // 현재 위치(스텝)
  long   targetSteps   = 0;
  float  curVelSteps   = 0;     // 현재 속도(스텝/s)
  unsigned long lastUs = 0;     // 마지막 스텝 시각
  float  carry         = 0;     // 소수 누적
} axes[3];

float   stepsPerDeg[3];         // 런타임 계산
float   curAngle[3];            // 현재 각도(°)

/* ─────── 상태 전송 주기 ─────── */
const unsigned long STATUS_MS = 500;
unsigned long lastStatusMs = 0;

/* ────── 비블로킹 수신 버퍼 ────── */
char recvBuf[32];
uint8_t recvPos = 0;
 
// ── 축별 “리밋 스위치 쪽” 방향( dirSign ) 기록 ──
int limitDirSign[3] = { -1, -1, -1 };   // 기본값은 -1(LOW 쪽이 스위치)로 두고 homing 때 갱신

/* ───────────────────────────── */
void setup() {
  Serial.begin(115200);
  while(!Serial){}

  for(int i=0;i<3;i++){
    pinMode(StepPin[i],OUTPUT);
    pinMode(DirPin[i], OUTPUT);
    pinMode(LimitPin[i],INPUT_PULLUP);

    stepsPerDeg[i] = STEPS_PER_REV * gearRatio[i] / 360.0;
    axes[i].curSteps = axes[i].targetSteps =
        lround(initAngle[i]*stepsPerDeg[i]);
    curAngle[i] = initAngle[i];
  }

  Serial.println(F("=== 3-Axis Controller Ready ==="));
  Serial.println(F("명령: X Y Z (deg)"));
  Serial.println(F("------------------------------"));

  /* Homing */
  Serial.println(F(">>> Homing..."));
  homeAxis(0,true);
  homeAxis(1,true);
  homeAxis(2,false);
  Serial.println(F(">>> Homing done"));
}

void loop(){
  // Serial.println(digitalRead(LimitPin[1]));
  processSerial();
  updateAxes();
  sendStatus();
}

/* ───────────────────────────────────────────────────
 *                      시리얼 파싱
 * ─────────────────────────────────────────────────*/
void processSerial() {
  // 한 문자씩 받아 'a' 종단자로 3토큰 파싱
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'a' || recvPos >= sizeof(recvBuf) - 1) {
      recvBuf[recvPos] = '\0';
      if (recvPos > 0) {
        // 세 개 토큰 분리
        char *p1 = strtok(recvBuf, " ");
        char *p2 = strtok(NULL,  " ");
        char *p3 = strtok(NULL,  " ");
        if (!p1 || !p2 || !p3) {
          Serial.println(F("❌ 형식: X Y Z a"));
        } else {
          float tgt[3] = { atof(p1), atof(p2), atof(p3) };
          // 범위 체크 후 목표 설정
          bool ok = true;
          for (int i = 0; i < 3; i++) {
            if (tgt[i] < angleLim[i][0] || tgt[i] > angleLim[i][1]) {
              Serial.print(F("⚠️ 범위 오류 Axis "));
              Serial.println(i);
              ok = false;
              break;
            }
          }
          if (ok) {
            for (int i = 0; i < 3; i++) {
              axes[i].targetSteps = lround(tgt[i] * stepsPerDeg[i]);
              // 새 명령이 들어오면 모션 상태 초기화
//              axes[i].curVelSteps = 0;
//              axes[i].carry       = 0;
//              axes[i].lastUs      = micros();
            }
            Serial.print(F("🆗 목표 → X:")); Serial.print(tgt[0]);
            Serial.print(F(" Y:"));       Serial.print(tgt[1]);
            Serial.print(F(" Z:"));       Serial.println(tgt[2]);
          }
        }
      }
      recvPos = 0;
    }
    else if (c >= 32 && c != 'a') {
      recvBuf[recvPos++] = c;
    }
  }
}

/* ───────────────────────────────────────────────────
 *                각 축 업데이트 (비차단)
 * ─────────────────────────────────────────────────*/
void updateAxes(){
  unsigned long nowUs = micros();
  for(int i=0;i<3;i++){
    long delta = axes[i].targetSteps - axes[i].curSteps;
    // // ▶▶ 디버그: 매 사이클 delta 확인
    // Serial.print(F("DEBUG updateAxes Axis ")); Serial.print(i);
    // Serial.print(F(" | delta=")); Serial.print(delta);
    // Serial.print(F(" curVel=")); Serial.print(axes[i].curVelSteps);
    // Serial.print(F(" lastUs(us)=")); Serial.println(axes[i].lastUs);

    // if(delta==0 && abs(axes[i].curVelSteps)<1) continue;
    /* ── 목표에 이미 도달했으면 정지 ── */
    if (delta == 0) {
      axes[i].curVelSteps = 0;   // 잔여 속도 클리어
      axes[i].carry       = 0;
     continue;                  // 더 계산하지 않음
    }

    int dirSign = (delta>0)?1:-1;

    /* 조인트 기준 → 스텝 기준 최대속도/가속도 */
    float maxVel = maxVelDeg[i]*stepsPerDeg[i];
    float maxAcc = maxAccDeg[i]*stepsPerDeg[i];

    /* 가속/감속 */
    float dt = (nowUs - axes[i].lastUs)/1e6;
    if(dt<=0) dt = 1e-6;

    float desiredVel = dirSign * maxVel;
    if(axes[i].curVelSteps < desiredVel){
      axes[i].curVelSteps += maxAcc*dt;
      if(axes[i].curVelSteps > desiredVel) axes[i].curVelSteps = desiredVel;
    }else if(axes[i].curVelSteps > desiredVel){
      axes[i].curVelSteps -= maxAcc*dt;
      if(axes[i].curVelSteps < desiredVel) axes[i].curVelSteps = desiredVel;
    }

    /* 브레이크 거리 고려 감속 */
    float brakeDist = (axes[i].curVelSteps*axes[i].curVelSteps)/(2*maxAcc);
    if(abs(delta)<brakeDist){
      desiredVel = dirSign*sqrt(2*maxAcc*abs(delta));
      if(abs(axes[i].curVelSteps) > abs(desiredVel)){
        axes[i].curVelSteps += (axes[i].curVelSteps>0?-1:1)*maxAcc*dt;
      }
    }

    // if(abs(axes[i].curVelSteps)<1) continue; // 아직 너무 느림

    float stepIntervalF = 1e6 / abs(axes[i].curVelSteps);
    unsigned long stepIntervalUs = (unsigned long)(stepIntervalF+axes[i].carry);
    axes[i].carry = (stepIntervalF+axes[i].carry)-stepIntervalUs;

    if(nowUs - axes[i].lastUs >= stepIntervalUs){
      // /* 리밋 스위치 한계 보호 (스위치 쪽 이동 차단) */
      // if(digitalRead(LimitPin[i])==LOW && dirSign<0){
    /* ── 리밋 스위치 한계 보호 ──
       스위치가 LOW(눌림)이고, 지금 가려는 방향이
       해당 축의 “스위치 쪽” 방향과 같으면 정지 */
      // if ( digitalRead(LimitPin[i]) == HIGH &&
      //      dirSign == limitDirSign[i] )
      // {
      //   axes[i].curVelSteps = 0;
      //   axes[i].targetSteps = axes[i].curSteps;
      //   continue;
      // }
      digitalWrite(DirPin[i], dirSign>0?HIGH:LOW);
      digitalWrite(StepPin[i],HIGH);
      delayMicroseconds(5);
      digitalWrite(StepPin[i],LOW);

      axes[i].curSteps += dirSign;
      axes[i].lastUs = nowUs;
      curAngle[i] = axes[i].curSteps/stepsPerDeg[i];
    }
  }
}

/* ───────────────────────────────────────────────────
 *               상태 출력 (100 ms 주기)
 * ─────────────────────────────────────────────────*/
void sendStatus(){
  unsigned long nowMs = millis();
  if(nowMs - lastStatusMs < STATUS_MS) return;
  lastStatusMs = nowMs;

  Serial.print(F("POS X:"));Serial.print(curAngle[0],1);
  Serial.print(F(" Y:"));   Serial.print(curAngle[1],1);
  Serial.print(F(" Z:"));   Serial.println(curAngle[2],1);
  
  // // ▶▶ 디버그: 실제 step 카운터 값도 함께 출력
  // Serial.print(F("DEBUG Steps X:")); Serial.print(axes[0].curSteps);
  // Serial.print(F(" Y:"));             Serial.print(axes[1].curSteps);
  // Serial.print(F(" Z:"));             Serial.println(axes[2].curSteps);
}

/* ───────────────────────────────────────────────────
 *                       Homing
 * ─────────────────────────────────────────────────*/
void homeAxis(int idx, bool cw){
  Serial.print(F("Axis "));Serial.print(idx);Serial.println(F(" homing..."));

  digitalWrite(DirPin[idx], cw?LOW:HIGH);
  delay(100);

  // ── homing 속도를 maxVelDeg[idx] 바탕으로 계산 ──
  float maxVelSteps = maxVelDeg[idx] * stepsPerDeg[idx] / gearRatio[idx];
  unsigned long homingIntervalUs = maxVelSteps > 0
      ? (unsigned long)(40000.0 / maxVelSteps)
      : 1000;  // 안전한 기본값

  // while(digitalRead(LimitPin[idx])==LOW){   // LOW = 아직 스위치 안풀림
  //   digitalWrite(StepPin[idx],HIGH);
  //   delayMicroseconds(homingIntervalUs);
  //   digitalWrite(StepPin[idx],LOW);
  //   delayMicroseconds(homingIntervalUs);
  // }
  // Debounce 적용 homing: HIGH(스위치 해제) 10회 연속 확인 시 homing 완료로 간주
  int highCount = 0;
  while (true) {
    // 한 스텝 펄스
    digitalWrite(StepPin[idx], HIGH);
    delayMicroseconds(homingIntervalUs);
    digitalWrite(StepPin[idx], LOW);
    delayMicroseconds(homingIntervalUs);

    // 연속 HIGH 체크
    int limit = digitalRead(LimitPin[idx]);
    // Serial.println(limit);

    if (limit == HIGH) {
        highCount = 0;
      for(int step=0; step<100; step++){
        int limit = digitalRead(LimitPin[idx]);
        Serial.println(limit);
        if(limit==HIGH) highCount++;
        delayMicroseconds(3000000);
      }
      if (++highCount >= 100) {
        // 10번 연속 HIGH면 스위치 해제 완료
        Serial.print("스위치 high수 : ");
        Serial.println(highCount);
        break;
      }
    } else {
      highCount = 0;
    }
  }

  axes[idx].curSteps = axes[idx].targetSteps =
      lround(initAngle[idx]*stepsPerDeg[idx]);
  axes[idx].curVelSteps = 0;
  axes[idx].lastUs = micros();
  curAngle[idx] = initAngle[idx];

  // // ▶▶ 디버그: homing 직후 상태 출력
  // Serial.print(F("DEBUG Homed Axis ")); Serial.print(idx);
  // Serial.print(F(" | curSteps=")); Serial.print(axes[idx].curSteps);
  // Serial.print(F(" targetSteps=")); Serial.print(axes[idx].targetSteps);
  // Serial.print(F(" curVel=")); Serial.print(axes[idx].curVelSteps);
  // Serial.print(F(" curAngle=")); Serial.println(curAngle[idx]);

  /* 스위치에서 5° 만큼 이탈 */
  long backSteps = lround(5*stepsPerDeg[idx]);
  digitalWrite(DirPin[idx], cw?HIGH:LOW);
  for(long s=0;s<backSteps;s++){
    digitalWrite(StepPin[idx],HIGH);delayMicroseconds(homingIntervalUs);
    digitalWrite(StepPin[idx],LOW);delayMicroseconds(homingIntervalUs);
    axes[idx].curSteps += (cw?1:-1);
  }
  curAngle[idx] = axes[idx].curSteps/stepsPerDeg[idx];
  
  // ▶ 리트랙션 후, 현재 위치를 새로운 목표로 설정하여 다시 돌아가지 않도록
  axes[idx].targetSteps = axes[idx].curSteps;
  Serial.println(F("Homing OK (stopped at retract)"));
  
  // ▶ “스위치 쪽 dirSign” 저장
  //   cw == true  → 스위치 쪽 DirPin = LOW → dirSign = -1
  //   cw == false → 스위치 쪽 DirPin = HIGH → dirSign = +1
  limitDirSign[idx] = cw ? -1 : +1;

  // ▶ 디버그: retraction 후 targetSteps 확인
  Serial.print(F("DEBUG Post-Homing Target Axis ")); Serial.print(idx);
  Serial.print(F(" targetSteps=")); Serial.println(axes[idx].targetSteps);
  
  // // ▶▶ 디버그: 리트랙션(5°) 후 상태 출력
  // Serial.print(F("DEBUG Retract Axis ")); Serial.print(idx);
  // Serial.print(F(" | curSteps=")); Serial.print(axes[idx].curSteps);
  // Serial.print(F(" curAngle=")); Serial.println(curAngle[idx]);
}
