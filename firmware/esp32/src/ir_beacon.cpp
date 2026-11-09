#include "ir_beacon.h"

IrBeacon* IrBeacon::self_ = nullptr;
const int IrBeacon::PINS[4] = {PIN_IR_FL, PIN_IR_FR, PIN_IR_BL, PIN_IR_BR};

void IRAM_ATTR IrBeacon::on(int i) {
  const uint32_t t = micros();
  const bool hi = digitalRead(PINS[i]);
  portENTER_CRITICAL_ISR(&self_->mux_); self_->rx_[i].edge(hi, t); portEXIT_CRITICAL_ISR(&self_->mux_);
}
void IRAM_ATTR IrBeacon::isr0() { on(0); }
void IRAM_ATTR IrBeacon::isr1() { on(1); }
void IRAM_ATTR IrBeacon::isr2() { on(2); }
void IRAM_ATTR IrBeacon::isr3() { on(3); }
