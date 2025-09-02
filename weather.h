// weather.h
#ifndef WEATHER_H
#define WEATHER_H
#include <Arduino.h>

bool isRaining(const char* city, const String& apiKey);

#endif
