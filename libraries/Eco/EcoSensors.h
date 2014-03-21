#ifndef EcoSensors_h
#define EcoSensors_h

#include "Arduino.h"

struct EcoSensors
{
  uint32_t rtc;
  long ultrasonic0;
  long ultrasonic45;
  long ultrasonic90;
  long ultrasonic135;
  long ultrasonic180;
  long ultrasonic225;
  long ultrasonic270;
  long ultrasonic315;
  float temperatureLeft;
  float temperatureRight;
  long lightLeft;
  long lightRight;
  long compassBearing;
  long COppm;
};

#endif
