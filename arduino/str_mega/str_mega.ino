// ---------- 스텝 모터 핀 정의 ----------
const int StepX = 2, DirX = 5, LimitX = 9;
const int StepY = 3, DirY = 6, LimitY = 10;
const int StepZ = 4, DirZ = 7, LimitZ = 11;

// ---------- 스텝 모터 설정 ----------
const int stepsPerRev = 200;
const float gearRatioX = 20.0;
const float gearRatioY = 20.0;
const float gearRatioZ = 5.0;

// ---------- 초기 기준 각도 ----------
const float initAngleX = 113.4;
const float initAngleY = -142.5;
const float initAngleZ = 91.4;

float currentAngleX = initAngleX;
float currentAngleY = initAngleY;
float currentAngleZ = initAngleZ;

void setup() {
  Serial.begin(9600);     // 사용자 입력용
  Serial1.begin(9600);    // Uno 통신용 (TX1: Pin 18)

  // 핀 설정
  pinMode(StepX, OUTPUT); pinMode(DirX, OUTPUT); pinMode(LimitX, INPUT_PULLUP);
  pinMode(StepY, OUTPUT); pinMode(DirY, OUTPUT); pinMode(LimitY, INPUT_PULLUP);
  pinMode(StepZ, OUTPUT); pinMode(DirZ, OUTPUT); pinMode(LimitZ, INPUT_PULLUP);

  Serial.println("3축 Homing 시작...");
  homeAxis(StepX, DirX, LimitX, false, gearRatioX, initAngleX, "X");
  homeAxis(StepY, DirY, LimitY, true, gearRatioY, initAngleY, "Y");
  homeAxis(StepZ, DirZ, LimitZ, false, gearRatioZ, initAngleZ, "Z");

  Serial.println("초기화 완료. 입력 형식: X Y Z A B (예: 90 80 40 -30 1)");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    int s1 = input.indexOf(' ');
    int s2 = input.indexOf(' ', s1 + 1);
    int s3 = input.indexOf(' ', s2 + 1);
    int s4 = input.indexOf(' ', s3 + 1);

    if (s1 == -1 || s2 == -1 || s3 == -1 || s4 == -1) {
      Serial.println("❌ 형식: X Y Z A B (예: 90 80 40 -30 1)");
      return;
    }

    float targetX = input.substring(0, s1).toFloat();
    float targetY = input.substring(s1 + 1, s2).toFloat();
    float targetZ = input.substring(s2 + 1, s3).toFloat();
    int servoAngleA = input.substring(s3 + 1, s4).toInt();  // -90 ~ 90
    String servoBState = input.substring(s4 + 1);            // "0" 또는 "1"

    if (!isValidX(targetX) || !isValidY(targetY) || !isValidZ(targetZ) ||
        servoAngleA < -90 || servoAngleA > 90 ||
        (servoBState != "0" && servoBState != "1")) {
      Serial.println("❌ 유효 범위 오류: X[-110~110], Y[-140~140], Z[-90~90], A[-90~90], B[0|1]");
      return;
    }

    // 모터 이동
    moveAxesTogether(targetX, targetY, targetZ);

    // Uno로 서보 명령 전송
    Serial1.print(String(servoAngleA) + " " + servoBState + "\n");

    Serial.print("✅ 현재: X="); Serial.print(currentAngleX);
    Serial.print(" Y="); Serial.print(currentAngleY);
    Serial.print(" Z="); Serial.print(currentAngleZ);
    Serial.print(" A="); Serial.print(servoAngleA);
    Serial.print(" B="); Serial.println(servoBState);
  }
}

// ----------------------------
// 유효성 검사
bool isValidX(float a) { return a >= -110 && a <= 110; }
bool isValidY(float a) { return a >= -140 && a <= 140; }
bool isValidZ(float a) { return a >= -90  && a <= 90;  }

// ----------------------------
// Homing
void homeAxis(int stepPin, int dirPin, int limitPin, bool cw, float gearRatio, float initAngle, String label) {
  digitalWrite(dirPin, cw ? LOW : HIGH);
  delay(100);
  long count = 0;

  while (digitalRead(limitPin) == LOW) {
    digitalWrite(stepPin, HIGH); delayMicroseconds(1400);
    digitalWrite(stepPin, LOW);  delayMicroseconds(1400);
    count++;
  }

  Serial.print("[" + label + "] 리밋 감지, 총 스텝: ");
  Serial.println(count);
}

// ----------------------------
// 동시 축 이동
void moveAxesTogether(float targetX, float targetY, float targetZ) {
  float deltaX = targetX - currentAngleX;
  float deltaY = targetY - currentAngleY;
  float deltaZ = targetZ - currentAngleZ;

  long stepsX = long((deltaX / 360.0) * gearRatioX * stepsPerRev);
  long stepsY = long((deltaY / 360.0) * gearRatioY * stepsPerRev);
  long stepsZ = long((deltaZ / 360.0) * gearRatioZ * stepsPerRev);

  digitalWrite(DirX, stepsX > 0 ? HIGH : LOW);
  digitalWrite(DirY, stepsY > 0 ? HIGH : LOW);
  digitalWrite(DirZ, stepsZ > 0 ? HIGH : LOW);

  long maxSteps = max(abs(stepsX), max(abs(stepsY), abs(stepsZ)));

  for (long i = 0; i < maxSteps; i++) {
    if (i < abs(stepsX)) digitalWrite(StepX, HIGH);
    if (i < abs(stepsY)) digitalWrite(StepY, HIGH);
    if (i < abs(stepsZ)) digitalWrite(StepZ, HIGH);
    delayMicroseconds(1400);
    digitalWrite(StepX, LOW);
    digitalWrite(StepY, LOW);
    digitalWrite(StepZ, LOW);
    delayMicroseconds(1400);
  }

  currentAngleX = targetX;
  currentAngleY = targetY;
  currentAngleZ = targetZ;
}
