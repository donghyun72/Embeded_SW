#include <Arduino.h>
#include "umbrella.h"

static int pinIn1, pinIn2, pinEna;

static const int pwmChannel    = 0;      // 0~15
static const int pwmFreq       = 20000;  // 20kHz (가청대역 밖)
static const int pwmResolution = 10;     // 10-bit (0~1023)
static const int DUTY_MAX      = 1023;   // 최대 듀티

// 요청 사양: 정방향 2초 → 10초 대기 → 역방향 2초
static const unsigned long FWD_MS  = 2000;
static const unsigned long WAIT_MS = 10000;
static const unsigned long REV_MS  = 2000;

static inline void setDir(bool forward) {
  digitalWrite(pinIn1, forward ? HIGH : LOW);
  digitalWrite(pinIn2, forward ? LOW  : HIGH);
}

static inline void stopMotor() {
  ledcWrite(pwmChannel, 0);   // PWM 0
  digitalWrite(pinIn1, LOW);  // 코스트 정지
  digitalWrite(pinIn2, LOW);
}

// 시동 토크 확보용 킥/램프업 (accelMs 동안 startDuty→dutyPeak로 빠르게 상승)
static void kickStart(bool forward,
                      int dutyPeak      = DUTY_MAX,
                      unsigned long accelMs = 300,
                      int startDuty     = 700) {
  if (dutyPeak > DUTY_MAX) dutyPeak = DUTY_MAX;
  if (startDuty < 0) startDuty = 0;
  if (startDuty > dutyPeak) startDuty = dutyPeak;

  setDir(forward);

  unsigned long t0 = millis();
  int duty = startDuty;

  while (millis() - t0 < accelMs) {
    ledcWrite(pwmChannel, duty);
    duty += 20;                           // 상승 속도(환경에 맞게 조정 가능)
    if (duty > dutyPeak) duty = dutyPeak;
    delay(5);                             // 너무 짧으면 CPU 점유↑
  }

  // 가속 종료 후 정점 유지
  ledcWrite(pwmChannel, dutyPeak);
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
  // 정방향 (열기) - 킥/램프업 후 잔여 시간 유지
  kickStart(true, DUTY_MAX, 300, 700);
  if (FWD_MS > 300) delay(FWD_MS - 300);
  stopMotor();

  // 열린 상태 유지
  delay(WAIT_MS);

  // 역방향 (닫기) - 킥/램프업 후 잔여 시간 유지
  kickStart(false, DUTY_MAX, 300, 700);
  if (REV_MS > 300) delay(REV_MS - 300);
  stopMotor();
}
 
