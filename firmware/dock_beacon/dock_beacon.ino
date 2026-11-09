// Dock controller: IR beacon + safe charge-rod switch. ATtiny85 @ 8 MHz internal (Arduino IDE: ATTinyCore,
// "ATtiny85", clock "8 MHz (internal)", millis on Timer0; flash with an ISP programmer or an Arduino as ISP).
// Schematic and wiring: docs "Dock wiring" (v2site/dock_wiring.html), parts in the BOM rows "Dock controller board".
//
// Pins (DIP-8):  pin 5 PB0 ROD_EN  -> 2N7000 -> gate of the high-side P-MOSFET that feeds the rods (HIGH = rods live)
//                pin 6 PB1 OC1A    -> IRLB8721 -> 'A' all-round LEDs (2 series pairs, 22 ohm each)
//                pin 7 PB2 STATUS  -> green LED + 1 k (off: no robot; on: charging; blinking: fault)
//                pin 2 PB3 ADC3    <- rod+ sense (220 k / 47 k divider, 100 nF)
//                pin 3 PB4 OC1B    -> IRLB8721 -> 'C' centre LED (33 ohm), in the 22 mm tube
// IR codes (match firmware/esp32/src/ir_decode.h): 38 kHz carrier from Timer1 (PLL off),
//   'A' all round: 600 us burst, 2.4 ms off;  'C' centre: 1200 us burst, 2.4 ms off.
// Rod switch logic: dock_logic.h (rods dead unless the robot's 112 k detect divider is on them; short -> off).
#include <avr/io.h>
#include "dock_logic.h"

static dock::Switch sw;

static void carrier_a(bool on) { if (on) TCCR1 |= _BV(COM1A1); else TCCR1 &= ~_BV(COM1A1); }
static void carrier_c(bool on) { if (on) GTCCR |= _BV(COM1B1); else GTCCR &= ~_BV(COM1B1); }

static void rods(bool on) { if (on) PORTB |= _BV(PB0); else PORTB &= ~_BV(PB0); }

static void status_led(uint32_t now) {
  bool l;
  switch (sw.st) {
    case dock::ON: case dock::PROBE: l = true; break;              // charging
    case dock::FAULT:   l = (now / 250) & 1; break;                 // 2 Hz: short / overload, retrying in 5 s
    case dock::LATCHED: l = (now / 62) & 1; break;                  // fast: 3 faults, unplug the dock to reset
    default:            l = (now % 2000) < 40; break;               // short blip: waiting for the robot
  }
  if (l) PORTB |= _BV(PB2); else PORTB &= ~_BV(PB2);
}

static void tick() {                                                 // ~every 3 ms, between IR bursts
  uint32_t now = millis();
  uint16_t adc = analogRead(A3);
  sw.step(now, adc);
  rods(sw.on());
  status_led(now);
}

void setup() {
  PORTB &= ~(_BV(PB0) | _BV(PB1) | _BV(PB2) | _BV(PB4));             // rods OFF from the first instruction
  DDRB |= _BV(PB0) | _BV(PB1) | _BV(PB2) | _BV(PB4);
  TCCR1 = _BV(PWM1A) | _BV(CS10);                                    // PWM on OCR1C period, clock / 1
  GTCCR = _BV(PWM1B);
  OCR1C = 209;                                                       // 8 MHz / 210 = 38.1 kHz
  OCR1A = 70; OCR1B = 70;                                            // 1/3 duty inside a burst
  analogReference(DEFAULT);                                          // Vcc = 5 V (D24V5F5)
}

void loop() {
  carrier_a(true);  delayMicroseconds(600);  carrier_a(false); PORTB &= ~_BV(PB1);
  tick(); delayMicroseconds(2200);
  carrier_c(true);  delayMicroseconds(1200); carrier_c(false); PORTB &= ~_BV(PB4);
  tick(); delayMicroseconds(2200);
}
