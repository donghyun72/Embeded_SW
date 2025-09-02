// weather.cpp
#include "weather.h"
#include <Arduino.h>       // Arduino 매크로/함수 사용
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

bool isRaining(const char* city, const String& apiKey) {
  if (WiFi.status() != WL_CONNECTED) return false;

  HTTPClient http;
  http.setTimeout(7000); // ms

  String url = "http://api.openweathermap.org/data/2.5/weather?q="
               + String(city) + "&appid=" + apiKey
               + "&lang=kr&units=metric";
  Serial.println("🌐 GET " + url);

  if (!http.begin(url)) {
    Serial.println("❌ http.begin 실패");
    return false;
  }

  int httpCode = http.GET();
  if (httpCode != HTTP_CODE_OK) {
    Serial.printf("❌ HTTP 응답 코드: %d\n", httpCode);
    http.end();
    return false;
  }

  String payload = http.getString();
  http.end();

  // 응답 일부만 출력 (길면 200자까지만)
  Serial.println("📦 날씨 응답(일부):");
  unsigned int previewLen = payload.length();
  if (previewLen > 200U) previewLen = 200U;
  Serial.println(payload.substring(0, previewLen) + "...");

  // JSON 파싱
  StaticJsonDocument<2048> doc;
  DeserializationError err = deserializeJson(doc, payload);
  if (err) {
    Serial.print("❌ JSON 파싱 에러: ");
    Serial.println(err.c_str());
    return false;
  }

  // 안전하게 필드 접근
  const char* mainWeather = nullptr;
  if (doc["weather"].is<JsonArray>() && doc["weather"][0]["main"].is<const char*>()) {
    mainWeather = doc["weather"][0]["main"];
  }
  float rain1h = doc["rain"]["1h"] | 0.0f;
  float rain3h = doc["rain"]["3h"] | 0.0f;

  String mw = mainWeather ? String(mainWeather) : String("(unknown)");
  Serial.printf("🔍 main=%s, rain1h=%.2f, rain3h=%.2f\n", mw.c_str(), rain1h, rain3h);

  // 비 판단: main이 비 계열이거나 강수량이 양수
  bool rainFlag =
      (mw == "Rain" || mw == "Drizzle" || mw == "Thunderstorm") ||
      (rain1h > 0.01f || rain3h > 0.01f);

  return rainFlag;
}
