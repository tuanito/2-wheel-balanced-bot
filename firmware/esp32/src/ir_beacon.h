// 4 x TSOP38238 on GPIO13..16 (front-left, front-right, back-left, back-right): pin-change interrupts time the
// low pulses, ir_decode.h classifies them. bits() = what each receiver heard in the last 100 ms window.
#pragma once
#include <Arduino.h>
#include "config.h"
#include "ir_decode.h"

class IrBeacon {
 public:
  void begin() {
    self_ = this;
    for (int i = 0; i < 4; ++i) pinMode(PINS[i], INPUT);      // TSOP has its own pull-up
    attachInterrupt(digitalPinToInterrupt(PINS[0]), isr0, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PINS[1]), isr1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PINS[2]), isr2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PINS[3]), isr3, CHANGE);
  }
  void update(uint32_t now_ms) {                 // call from the comms loop
    if (now_ms - win_start_ < 100) return;
    win_start_ = now_ms;
    portENTER_CRITICAL(&mux_); bits_ = ir::window_bits(rx_); portEXIT_CRITICAL(&mux_);
  }
  uint8_t bits() const { return bits_; }
 private:
  static const int PINS[4];
  static IrBeacon* self_;
  static void on(int i);                         // in ir_beacon.cpp (IRAM)
  static void isr0(); static void isr1(); static void isr2(); static void isr3();
 public:
  ir::Receiver rx_[4];
  portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
 private:
  uint32_t win_start_ = 0;
  volatile uint8_t bits_ = 0;
};
