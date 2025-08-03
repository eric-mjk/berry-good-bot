#include <Servo.h>

Servo wristleServo;
Servo gripperServo;

// 핀 설정
const int wristleServoPin = 11;
const int gripperServoPin = 3;

// 그리퍼 서보 각도 설정
const int ANGLE_GRIPPER_OPEN   = 150;
const int ANGLE_GRIPPER_CLOSED = 75;

void setup() {
  Serial.begin(115200);  
  
  wristleServo.attach(wristleServoPin);
  gripperServo.attach(gripperServoPin);

  // 초기 위치
  wristleServo.write(90);                    // 손목 서보: 중앙(0°)
  gripperServo.write(ANGLE_GRIPPER_CLOSED);  // 그리퍼 서보: 닫힘(150°)

  Serial.println(F("UNO 준비 완료! 형식: \"각도(-90~90) 그리퍼_명령(0/1)\" 예) \"30 1\""));
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    // 공백으로 분리
    int sep = input.indexOf(' ');
    if (sep == -1) {
      Serial.print(F("❌ 잘못된 입력: "));
      Serial.println(input);
      return;
    }

    String angleStr = input.substring(0, sep);
    String gripCmd  = input.substring(sep + 1);

    /* ---------- 손목 서보 (wristleServo) ---------- */
    int inputAngle = angleStr.toInt();
    if (inputAngle < -90 || inputAngle > 90) {
      Serial.println(F("⚠️ 손목 값은 -90 ~ 90° 사이여야 합니다!"));
    } else {
      int mappedAngle = inputAngle + 90;  
      wristleServo.write(mappedAngle);
      Serial.print(F("✅ 손목 서보 이동: "));
      Serial.println(mappedAngle);
    }

    /* ---------- 그리퍼 서보 (gripperServo) ---------- */
    if (gripCmd == "1") {
      gripperServo.write(ANGLE_GRIPPER_OPEN);
      Serial.println(F("✅ 그리퍼 열림 (75°)"));
    } else if (gripCmd == "0") {
      gripperServo.write(ANGLE_GRIPPER_CLOSED);
      Serial.println(F("✅ 그리퍼 닫힘 (150°)"));
    } else {
      Serial.print(F("⚠️ 그리퍼 명령 오류: "));
      Serial.println(gripCmd);
    }

    Serial.println(F("----"));
  }
}
