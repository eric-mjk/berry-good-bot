#include <Servo.h>
#include <SoftwareSerial.h>

Servo servoA;
Servo servoB;

// 핀 설정
const int servoAPin = 9;
const int servoBPin = 8;
const int rxPin = 2;   // Mega TX1와 연결
const int txPin = 3;   // 사용 안 해도 됨

SoftwareSerial megaSerial(rxPin, txPin);  // RX, TX

// 서보B 각도 설정
const int ANGLE_FOR_ONE = 75;
const int ANGLE_FOR_ZERO = 150;

void setup() {
  Serial.begin(115200);
  megaSerial.begin(9600);

  servoA.attach(servoAPin);
  servoB.attach(servoBPin);

  // 초기값
  servoA.write(90);  // 0도에 해당
  servoB.write(ANGLE_FOR_ZERO);

  Serial.println("UNO 준비 완료. 형식: \"각도 -90~90 서보B_명령 0/1\" (예: \"30 1\")");
}

void loop() {
  if (megaSerial.available()) {
    String input = megaSerial.readStringUntil('\n');
    input.trim();

    // 공백으로 분리
    int spaceIndex = input.indexOf(' ');
    if (spaceIndex == -1) {
      Serial.print("❌ 잘못된 입력 형식: ");
      Serial.println(input);
      return;
    }

    String angleStr = input.substring(0, spaceIndex);
    String valB = input.substring(spaceIndex + 1);

    // 첫 번째 서보 각도 계산
    int inputAngle = angleStr.toInt();
    if (inputAngle < -90 || inputAngle > 90) {
      Serial.println("⚠️ A 서보 값은 -90 ~ 90 사이여야 합니다.");
    } else {
      int servoAngle = inputAngle + 90;  // 변환
      servoA.write(servoAngle);
      Serial.print("✅ A 서보 이동: ");
      Serial.println(servoAngle);
    }

    // 두 번째 서보 제어
    if (valB == "1") {
      servoB.write(ANGLE_FOR_ONE);
      Serial.println("✅ B 서보: 열림 (75도)");
    } else if (valB == "0") {
      servoB.write(ANGLE_FOR_ZERO);
      Serial.println("✅ B 서보: 닫힘 (150도)");
    } else {
      Serial.print("⚠️ B 서보 입력 오류: ");
      Serial.println(valB);
    }

    Serial.println("----");
  }
}
