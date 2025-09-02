#include <Arduino.h>
#include "umbrella.h"

static int pinIn1, pinIn2, pinEna;

static const int pwmChannel    = 0;     // 0~15
static const int pwmFreq       = 2000;  // 2kHz
static const int pwmResolution = 10;    // 10-bit (0~1023)
static const int DUTY_MAX      = 950;   // 필요시 1023까지 가능

// 요청 사양: 정방향 5초 → 15초 대기 → 역방향 5초
static const unsigned long FWD_MS  = 5000;
static const unsigned long WAIT_MS = 15000;
static const unsigned long REV_MS  = 5000;

static inline void setDir(bool forward) {
  digitalWrite(pinIn1, forward ? HIGH : LOW);
  digitalWrite(pinIn2, forward ? LOW  : HIGH);
}

static inline void stopMotor() {
  ledcWrite(pwmChannel, 0);   // PWM 0
  digitalWrite(pinIn1, LOW);  // 코스트 정지
  digitalWrite(pinIn2, LOW);
}

void setupUmbrella(int in1, int in2, int ena) {
  pinIn1 = in1;
  pinIn2 = in2;
  pinEna = ena;

  pinMode(pinIn1, OUTPUT);
  pinMode(pinIn2, OUTPUT);

  // PWM 설정
  ledcSetup(pwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(pinEna, pwmChannel);
  ledcWrite(pwmChannel, 0);

  // 초기 정지
  stopMotor();
}

void moveUmbrella() {
  // 정방향 5초 (열기)
  setDir(true);
  ledcWrite(pwmChannel, DUTY_MAX);
  delay(FWD_MS);
  stopMotor();

  // 열린 상태 유지 15초
  delay(WAIT_MS);

  // 역방향 5초 (닫기)
  setDir(false);
  ledcWrite(pwmChannel, DUTY_MAX);
  delay(REV_MS);
  stopMotor();
}
