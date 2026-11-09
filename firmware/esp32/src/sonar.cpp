#include "sonar.h"

Sonar* Sonar::self_ = nullptr;

void IRAM_ATTR Sonar::isr() {
  const uint32_t t = micros();
  if (digitalRead(PIN_SONAR_ECHO)) self_->rise_ = t;
  else {
    const uint32_t us = t - self_->rise_;
    self_->dist_ = (us > 100 && us < 25000) ? us * 343e-6f / 2.f : -1.f;   // 2 cm .. 4.3 m
    self_->waiting_ = false;
  }
}
