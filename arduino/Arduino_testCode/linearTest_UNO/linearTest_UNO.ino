#include <Arduino.h>

#define DIR_PIN        9    // 방향 제어 핀
#define STEP_PIN       6    // 스텝 펄스 핀
#define LIMIT_PIN      7    // 리밋 스위치 핀
// 지지 위치 : -580
// 모터·볼스크류 특성
const float STEPS_PER_REV  = 2415.0;         // 모터 1회전 당 스텝 수
const float SCREW_LEAD_MM  = 4.0;            // 볼스크류 1회전 당 이송 길이 (mm)
const float STEPS_PER_MM   = STEPS_PER_REV / SCREW_LEAD_MM;

// 스텝 펄스 폭·속도 (마이크로초)
const unsigned int STEP_PULSE_US = 50;
const unsigned int STEP_DELAY_US = 50;

// 현재 위치 (스텝 단위, 절대 좌표)
long currentPositionSteps = 0;

void setup() {
  // 핀 모드
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(LIMIT_PIN, INPUT_PULLUP);  // 스위치 누르면 LOW

  digitalWrite(DIR_PIN, LOW);
  digitalWrite(STEP_PIN, LOW);

  Serial.begin(9600);
  while (!Serial) { /* 시리얼 연결 대기 */ }

  // 1) 리밋 스위치까지 호밍
  homeAxis();

  // 2) 시리얼 입력 안내
  Serial.println();
  Serial.println("=== MM 단위 이동 제어 ===");
  Serial.println("목표 위치(mm) 입력 → Enter");
  Serial.println("예) 0.0  → 초기 위치");
  Serial.println("    4.0  → 볼스크류 1회전 (4 mm)");
  Serial.println("---------------------------------");
}

void loop() {
  // Serial.println(digitalRead(LIMIT_PIN));
  
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    float targetMm = line.toFloat();
    long targetSteps = lround(targetMm * STEPS_PER_MM);
    // 현재 위치(mm) 계산
    float currentMm   = currentPositionSteps / STEPS_PER_MM;
    // 목표 범위 제한: 0 ~ -590 mm 사이만 허용
    if (targetMm > 0 || targetMm < -590) {
      Serial.println("→ 오류: 목표 위치가 허용 범위를 벗어났습니다. (0 ~ -590 스텝)");
      return;
    }
    long deltaSteps   = targetSteps - currentPositionSteps;

    if (deltaSteps == 0) {
      // 실시간 위치 기반 출력
      currentMm = currentPositionSteps / STEPS_PER_MM;
      Serial.print("→ 이미 "); Serial.print(currentMm, 3);
      Serial.println(" mm 에 있습니다.");
      return;
    }

    // 이동 방향 설정
    bool dir = (deltaSteps > 0);
    digitalWrite(DIR_PIN, dir ? HIGH : LOW);

    Serial.print("→ 이동: ");
    Serial.print(targetMm, 3);
    Serial.print(" mm (");
    Serial.print(deltaSteps);
    Serial.println(" 스텝)");

    moveSteps(abs(deltaSteps), dir);

    // 이동 완료 후 실시간 위치 기반 mm 계산 및 출력
    currentMm = currentPositionSteps / STEPS_PER_MM;
    Serial.print("✔ 완료. 현재 위치: ");
    Serial.print(currentMm, 3);
    Serial.println("---------------------------------");
  }
}

// ——————————————
// 호밍 함수: 리밋 스위치 눌릴 때까지 이동 → 기준 스텝 0으로 설정
void homeAxis() {
  Serial.println(">>> Homing 시작: 리밋 스위치 감지 전까지 이동");
  // 스위치 방향으로 이동 (스위치 눌리면 digitalRead == LOW)
  digitalWrite(DIR_PIN, HIGH);  
  delay(100);

  // 스위치가 눌리지 않은 동안 계속 스텝
  while (digitalRead(LIMIT_PIN) == LOW) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_PULSE_US);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_DELAY_US);
  }

  Serial.println(">>> 리밋 스위치 감지됨. 기준점 설정 완료.");
  currentPositionSteps = 0;

  // 스위치에서 살짝 이탈 (옵션, 너무 가까우면 이후 이동 시 접촉 유지될 수 있으므로)
  digitalWrite(DIR_PIN, LOW);
  for (int i = 0; i < 10000; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_PULSE_US);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_DELAY_US);
    currentPositionSteps--;
  }
  
  float currentMm   = currentPositionSteps / STEPS_PER_MM;
  Serial.print(">>> Homing 완료. 현재 위치 = "); Serial.print(currentMm);
  Serial.println(" mm");
}

// ——————————————
// 지정된 스텝 수만큼 펄스 출력
void moveSteps(unsigned long steps, bool dir) {
  for (unsigned long i = 0; i < steps; i++) {
    // 리밋 스위치 감지 시 즉시 중지
    if (digitalRead(LIMIT_PIN) == HIGH) {
      Serial.println("⚠️ 이동 중 리밋 스위치 감지! 중지합니다.");
    // 스위치에서 살짝 이탈 (옵션, 너무 가까우면 이후 이동 시 접촉 유지될 수 있으므로)
    digitalWrite(DIR_PIN, LOW);
    for (int i = 0; i < 10000; i++) {
      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(STEP_PULSE_US);
      digitalWrite(STEP_PIN, LOW);
      delayMicroseconds(STEP_DELAY_US);
      currentPositionSteps--;
    }
      break;
    }
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_PULSE_US);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_DELAY_US);
    // 실시간 위치 업데이트
    // DIR_PIN이 HIGH면 양(+), LOW면 음(-) 방향으로 스텝했으므로 currentPositionSteps를 증감
    if (dir) {
      currentPositionSteps++;
    } else {
      currentPositionSteps--;
    }
  }
}
