void setup() {
  Serial.begin(115200);    // PC ↔ Mega
  Serial1.begin(115200);   // Mega ↔ UNO
  Serial.println(F("Mega 준비 완료. PC에서 명령 입력하세요."));
}

void loop() {
  // PC → Mega → UNO
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    Serial1.println(cmd);
  }
  // UNO → Mega → PC
  if (Serial1.available()) {
    String resp = Serial1.readStringUntil('\n');
    Serial.println(resp);
  }
}
