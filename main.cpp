#include <Arduino.h>
#include <WiFi.h>
#include <DFRobotDFPlayerMini.h>
#include "weather.h"
#include "umbrella.h"

// ===== Wi-Fi & API =====
const char* ssid     = " ";  // 사용자 ssid
const char* password = " ";  // 비밀번호
const String apiKey  = " ";  // 사용자 apikey
const char* city     = "Seoul";

// ===== Pins =====
const int pirPin = 26;   // PIR out -> ESP32
const int in1Pin = 27;   // L298N IN1
const int in2Pin = 25;   // L298N IN2
const int enaPin = 13;   // L298N ENA (PWM)

// ===== DFPlayer (UART2) =====
HardwareSerial dfSerial(2);
DFRobotDFPlayerMini mp3;
bool dfReady = false;

// ===== PIR 감지 파라미터 =====
const unsigned long WARMUP_MS         = 30000; // PIR 워밍업
const unsigned long MIN_EDGE_INTERVAL = 200;   // 에지 채터링 필터
const unsigned long cooldownMs        = 5000;  // 연속 동작 쿨다운

// ===== 감지 차단(락아웃) =====
const unsigned long POST_LOCK_MS = 15000;
bool pirAttached = true;
unsigned long lockReleaseAt = 0;

//  재활성화 조건
const unsigned long STABLE_LOW_MS = 3000;  // LOW가 3초 연속 유지돼야 attach
bool waitingStableLow = false;
unsigned long lowSince = 0;
bool ignoreFirstRising = false;            // attach 직후 첫 RISING 무시

// ===== 전역 타임스탬프 =====
unsigned long bootMs    = 0;
unsigned long lastRunMs = 0;

// ===== PIR 인터럽트 =====
volatile bool pirRisingFlag = false;
volatile unsigned long pirIsrCount = 0;

void IRAM_ATTR pirISR() {
  pirRisingFlag = true;
  pirIsrCount++;
}

// ===== 주기성 노이즈 필터 =====
bool rejectPeriodicNoise(unsigned long dt) {
  static unsigned long lastDt = 0;
  static int streak = 0;
  if (dt == 0) return false;
  if (lastDt == 0) { lastDt = dt; streak = 0; return false; }
  long diff = (long)dt - (long)lastDt;
  if (abs(diff) <= (long)(lastDt / 10)) streak++; else streak = 0;
  lastDt = dt;
  return streak >= 3;
}

// === 디버깅 ===
void logPirEdges() {
  static int prev = -1;
  static unsigned long lastEdgeMs = 0;
  int cur = digitalRead(pirPin);
  if (prev == -1) { prev = cur; lastEdgeMs = millis(); return; }
  if (cur != prev) {
    unsigned long now = millis();
    unsigned long dur = now - lastEdgeMs;
    Serial.printf("[PIR] %s for %lu ms\n", prev ? "HIGH" : "LOW", dur);
    lastEdgeMs = now;
    prev = cur;
  }
}

// ===== PIR attach/detach  =====
inline void attachPir() {
  if (!pirAttached) {
    attachInterrupt(digitalPinToInterrupt(pirPin), pirISR, RISING);
    pirAttached = true;
    Serial.println("🔔 PIR re-enabled");
  }
}
inline void detachPir() {
  if (pirAttached) {
    detachInterrupt(digitalPinToInterrupt(pirPin));
    pirAttached = false;
    Serial.println("🔕 PIR disabled");
  }
  // 재활성화 대기 상태 초기화
  waitingStableLow = false;
  lowSince = 0;
  ignoreFirstRising = false;
}

void setup() {
  Serial.begin(115200);

  // PIR 입력
  pinMode(pirPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(pirPin), pirISR, RISING);

  bootMs = millis();

  // Wi-Fi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Wi-Fi 연결 중");
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(300);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n✅ Wi-Fi 연결 완료: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\n⚠️ Wi-Fi 연결 실패(오프라인 모드)");
  }

  // DFPlayer
  Serial.println("DFPlayer 연결 중...");
  dfSerial.begin(9600, SERIAL_8N1, 16, 17); // RX2=16, TX2=17
  delay(1200);
  if (!mp3.begin(dfSerial, /*isACK=*/false, /*doReset=*/true)) {
    Serial.println("❌ DFPlayer 초기화 실패 - 전원/배선/SD 확인");
    dfReady = false;
  } else {
    mp3.volume(28); // 0~30
    dfReady = true;
    Serial.println("✅ DFPlayer 연결 완료");
  }

  // 모터
  setupUmbrella(in1Pin, in2Pin, enaPin);
}

void loop() {
  // 디버그
  logPirEdges();

  // 워밍업 동안 PIR 무시
  if (millis() - bootMs < WARMUP_MS) { delay(10); return; }

  // 락아웃 중이면 재활성화 조건 검사 (LOW가 STABLE_LOW_MS 유지될 때까지 대기)
  if (!pirAttached) {
    if (millis() >= lockReleaseAt) {
      int level = digitalRead(pirPin);
      if (level == LOW) {
        if (!waitingStableLow) { waitingStableLow = true; lowSince = millis(); }
        if (millis() - lowSince >= STABLE_LOW_MS) {
          // LOW 안정 확보 → 인터럽트 재등록
          pirRisingFlag = false;
          pirIsrCount   = 0;
          ignoreFirstRising = true;   // 첫 RISING은 버림
          attachPir();
        }
      } else {
        // 여전히 HIGH면 카운트 리셋하고 계속 기다림
        waitingStableLow = false;
      }
    }
    delay(10);
    return; // attach될 때까지 loop 탈출
  }

  // 인터럽트로 들어온 RISING 처리
  static unsigned long lastEdgeHandled = 0;
  static unsigned long lastTriggerMs   = 0;

  if (pirRisingFlag) {
    // 재활성화 직후 첫 RISING은 무시
    if (ignoreFirstRising) {
      pirRisingFlag = false;
      ignoreFirstRising = false;
      // 작은 여유
      delay(5);
      return;
    }

    pirRisingFlag = false;                 // 플래그 클리어
    unsigned long now = millis();

    // 너무 촘촘한 에지는 무시(채터링/리플)
    if (now - lastEdgeHandled < MIN_EDGE_INTERVAL) {
      // skip
    } else {
      unsigned long dt = (lastTriggerMs == 0) ? 0 : (now - lastTriggerMs);
      lastEdgeHandled = now;

      // 주기성 노이즈 필터
      if (rejectPeriodicNoise(dt)) {
        Serial.println("⚠️ 주기성 노이즈 패턴 감지 → 무시");
      } else {
        // 쿨다운 확인
        if (now - lastRunMs > cooldownMs) {
          lastTriggerMs = now;
          lastRunMs     = now;

          Serial.printf("🚶 PIR 감지! (ISR cnt=%lu)\n", pirIsrCount);

          bool rain = false;
          if (WiFi.status() == WL_CONNECTED) {
            rain = isRaining(city, apiKey);
          } else {
            Serial.println("🌐 오프라인 - 날씨 확인 불가(기본값: 비 아님)");
          }

          // ── 비 여부에 따라 항상 음성 재생 ──
          if (dfReady) {
            if (rain) {
              Serial.println("🔊 MP3: 비가 옵니다 (0001.mp3)");
              mp3.play(1);                 // 0001.mp3 : 비 옴
            } else {
              Serial.println("🔊 MP3: 비가 오지 않습니다 (0002.mp3)");
              mp3.play(2);                 // 0002.mp3 : 비 안 옴
            }
            delay(1200);                   // 필요 시 파일 길이에 맞게 조정
          } else {
            Serial.println("⚠️ DFPlayer 미동작 - 음성 생략");
          }

          // ── 우산 동작은 '비 올 때만' 수행 ──
          if (rain) {
            Serial.println("🌧️ 비 감지! 우산 서랍 동작 시작");

            // 감지 차단: 인터럽트 분리 & 플래그 초기화
            detachPir();
            pirRisingFlag = false;
            pirIsrCount   = 0;

            // 정방향 5s → 대기 15s → 역방향 5s
            moveUmbrella();

            // 닫힘 완료 후 추가 락아웃
            lockReleaseAt = millis() + POST_LOCK_MS;
            return;  // 락아웃 유지
          } else {
            Serial.println("☀️ 비 없음 - 우산 동작 생략");
          }
        }
      }
    }
  }

  delay(10);
}
