/* ───────────────────── 핀 매핑 ───────────────────── */
const int StepPin[3]  = {2, 3, 4};
const int DirPin[3]   = {5, 6, 7};
const int LimitPin[3] = {9,10,11};      // LOW = 스위치 ON

/* ───────────────── 스텝/기어 파라미터 ─────────────── */
const int   STEPS_PER_REV = 200;         // 모터 1회전 스텝
const float gearRatio[3]  = {20.0, 20.0, 5.0}; // ★수정
const float initAngle[3]  = {113.4, -142.5, 91.4};

/* ──────── 조인트 최대 속도·가속도(각도 기준) ──────── */
const float maxVelDeg[3]  = {60, 60, 60};     // °/s (★조정)
const float maxAccDeg[3]  = {120, 120, 120};  // °/s² (★조정)

/* ─────────── 허용 각도 범위 (min, max) ──────────── */
const float angleLim[3][2] = {{-110,110}, {-140,140}, {-90,90}};

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
const unsigned long STATUS_MS = 100;
unsigned long lastStatusMs = 0;

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
  processSerial();
  updateAxes();
  sendStatus();
}

/* ───────────────────────────────────────────────────
 *                      시리얼 파싱
 * ─────────────────────────────────────────────────*/
void processSerial(){
  if(!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if(line.length()==0) return;

  float tgt[3];
  if(sscanf(line.c_str(),"%f %f %f",&tgt[0],&tgt[1],&tgt[2])!=3){
    Serial.println(F("❌ 형식: X Y Z (deg)"));
    return;
  }

  for(int i=0;i<3;i++){
    if(tgt[i]<angleLim[i][0] || tgt[i]>angleLim[i][1]){
      Serial.print(F("⚠️ 범위 오류 Axis "));
      Serial.println(i);
      return;
    }
    axes[i].targetSteps = lround(tgt[i]*stepsPerDeg[i]);
  }
  Serial.print(F("🆗 목표 → X:"));Serial.print(tgt[0]);
  Serial.print(F(" Y:"));Serial.print(tgt[1]);
  Serial.print(F(" Z:"));Serial.println(tgt[2]);
}

/* ───────────────────────────────────────────────────
 *                각 축 업데이트 (비차단)
 * ─────────────────────────────────────────────────*/
void updateAxes(){
  unsigned long nowUs = micros();
  for(int i=0;i<3;i++){
    long delta = axes[i].targetSteps - axes[i].curSteps;
    if(delta==0 && abs(axes[i].curVelSteps)<1) continue;

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

    if(abs(axes[i].curVelSteps)<1) continue; // 아직 너무 느림

    float stepIntervalF = 1e6 / abs(axes[i].curVelSteps);
    unsigned long stepIntervalUs = (unsigned long)(stepIntervalF+axes[i].carry);
    axes[i].carry = (stepIntervalF+axes[i].carry)-stepIntervalUs;

    if(nowUs - axes[i].lastUs >= stepIntervalUs){
      /* 리밋 스위치 한계 보호 (스위치 쪽 이동 차단) */
      if(digitalRead(LimitPin[i])==LOW && dirSign<0){
        axes[i].curVelSteps = 0;
        axes[i].targetSteps = axes[i].curSteps;
        continue;
      }
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
}

/* ───────────────────────────────────────────────────
 *                       Homing
 * ─────────────────────────────────────────────────*/
void homeAxis(int idx, bool cw){
  Serial.print(F("Axis "));Serial.print(idx);Serial.println(F(" homing..."));

  digitalWrite(DirPin[idx], cw?LOW:HIGH);
  delay(100);

  while(digitalRead(LimitPin[idx])==LOW){   // LOW = 아직 스위치 안풀림
    digitalWrite(StepPin[idx],HIGH);
    delayMicroseconds(700);
    digitalWrite(StepPin[idx],LOW);
    delayMicroseconds(700);
  }

  axes[idx].curSteps = axes[idx].targetSteps =
      lround(initAngle[idx]*stepsPerDeg[idx]);
  axes[idx].curVelSteps = 0;
  axes[idx].lastUs = micros();
  curAngle[idx] = initAngle[idx];

  /* 스위치에서 5° 만큼 이탈 */
  long backSteps = lround(5*stepsPerDeg[idx]);
  digitalWrite(DirPin[idx], cw?HIGH:LOW);
  for(long s=0;s<backSteps;s++){
    digitalWrite(StepPin[idx],HIGH);delayMicroseconds(700);
    digitalWrite(StepPin[idx],LOW);delayMicroseconds(700);
    axes[idx].curSteps += (cw?-1:1);
  }
  curAngle[idx] = axes[idx].curSteps/stepsPerDeg[idx];
  Serial.println(F("Homing OK"));
}
