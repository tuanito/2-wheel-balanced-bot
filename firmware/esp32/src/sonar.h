// HC-SR04, non-blocking: trigger every 60 ms, measure the echo with a pin-change interrupt.
#pragma once
#include <Arduino.h>
#include "config.h"

class Sonar {
 public:
  void begin() {
    pinMode(PIN_SONAR_TRIG, OUTPUT); digitalWrite(PIN_SONAR_TRIG, LOW);
    pinMode(PIN_SONAR_ECHO, INPUT);
    self_ = this;
    attachInterrupt(digitalPinToInterrupt(PIN_SONAR_ECHO), isr, CHANGE);
  }
  void update(uint32_t now_ms) {               // call from the main loop
    if (now_ms - last_trig_ >= 60) {
      if (waiting_) dist_ = -1;                 // previous echo never ended: nothing in range
      last_trig_ = now_ms; waiting_ = true;
      digitalWrite(PIN_SONAR_TRIG, HIGH); delayMicroseconds(10); digitalWrite(PIN_SONAR_TRIG, LOW);
    }
  }
  float distance() const { return dist_; }      // m, -1 = none
 private:
  static void isr();
  static Sonar* self_;
 public:
  volatile uint32_t rise_ = 0;
  volatile float dist_ = -1;
  volatile bool waiting_ = false;
 private:
  uint32_t last_trig_ = 0;
};
