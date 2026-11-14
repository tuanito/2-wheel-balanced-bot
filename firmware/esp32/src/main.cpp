// ESP32-S3 firmware: 500 Hz balance/stand-up control (core 1) + Pi link, sonar, battery, charging, LED (core 0).
// The controller (lib/botcore) is byte-for-byte the code validated in the MuJoCo simulation.
#include <Arduino.h>
#include <Preferences.h>
#include <cmath>
#include <algorithm>
#include "config.h"
#include "link_proto.h"
#include "lk_motor.h"
#include "imu.h"
#include "sonar.h"
#include "tof.h"
#include "ir_beacon.h"
#include "power_off.h"
#include "power_off.h"
#include "controller.hpp"
#include "gains.hpp"

static_assert(CONTROL_PERIOD_US == (uint32_t)(bot::Params::DT * 1e6f + 0.5f), "control period must match the LQR design");

// One robot, one motor, and the choice is made in two files: config.h's FITTED_MOTOR carries the
// DRIVE's side (Kt, what an iq command means), core/motor.hpp's BOT_MOTOR_* carries the
// CONTROLLER's (torque limit, speed ceiling, rotor inertia). Setting one without the other builds a
// robot that commands torques its controller does not believe in. Measured cost of exactly that
// mismatch in the simulator: the stand-up scored 0/30 instead of 30/30, and it looked for all the
// world like a physical motor speed ceiling. Make it a compile error instead.
static_assert(KT == bot::motor::KT,
              "config.h's FITTED_MOTOR and core/motor.hpp's BOT_MOTOR_* disagree about which motor "
              "is fitted. Set both: -D FITTED_MOTOR=MOTOR_MF7015 -D BOT_MOTOR_MF7015");

static LkMotors motors;
static Imu imu;
#if FRONT_SENSOR == FRONT_TOF
static Tof tof;                  // the zone array, instead of the sonar: same aperture, same job
#else
static Sonar sonar;
#endif
static IrBeacon irb;
static bot::Controller ctl;
static Preferences prefs;
#ifdef BENCH_CAN
static HardwareSerial& PI_UART = Serial;      // bench: the PC plays the Pi on the same port as the IMU frames
#else
static HardwareSerial& PI_UART = Serial1;
#endif

// ---- shared between the two tasks ----
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static bot::Command g_cmd;              // written by comms, read by control
static bot::Outputs g_out;              // written by control
static volatile float g_sonar = -1, g_drop = -1, g_vbat = 22.2f;
static volatile float g_pitch = 0;   // last body pitch, for the zone array on core 0
static volatile bool g_charging = false, g_motor_fault = false, g_batt_low = false, g_ov_event = false;
static volatile bool g_armed = false;
static TaskHandle_t control_task = nullptr;
static volatile uint32_t g_fault_age_l = 0, g_fault_age_r = 0, g_fault_seq = 0;   // why the last CAN fault happened

// ------------------------------------------------------------------ control (core 1, 500 Hz)
static void tick(void*) { xTaskNotifyGive(control_task); }   // esp_timer task context (not an ISR)

static void control_loop(void*) {
  uint32_t last_us = micros();
  bool motors_on = false;
  uint32_t off_cycles = 0;
  float saved_adj = prefs.getFloat("e_adj", 0.f);
  ctl.set_standup_energy_adj(saved_adj);
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    const uint32_t now_us = micros(), now_ms = millis();
    const float dt = (now_us - last_us) * 1e-6f; last_us = now_us;

    motors.poll();                                   // replies to the previous command (angle, speed, temp)
    const ImuSample s = imu.read();

    bot::Command c;
    portENTER_CRITICAL(&mux); c = g_cmd; portEXIT_CRITICAL(&mux);
    const bool can_ok = motors.healthy(now_ms);
    if (!can_ok && motors_on && !g_motor_fault) {
      g_motor_fault = true;
      g_fault_age_l = motors.left.seen ? (uint32_t)std::max<int32_t>(0, (int32_t)(now_ms - motors.left.last_ms)) : 9999;
      g_fault_age_r = motors.right.seen ? (uint32_t)std::max<int32_t>(0, (int32_t)(now_ms - motors.right.last_ms)) : 9999;
      g_fault_seq = g_fault_seq + 1;
    }
    c.arm = g_armed && s.ok && !g_motor_fault && !g_batt_low;
    ctl.set_command(c);

    bot::Inputs in{};
    in.dt = dt;
    in.gx = s.gx; in.gy = s.gy; in.gz = s.gz; in.ax = s.ax; in.ay = s.ay; in.az = s.az;
    in.ql = motors.left.angle; in.qr = motors.right.angle;
    in.wl = motors.left.speed; in.wr = motors.right.speed;
    in.sonar_m = g_sonar; in.drop_m = g_drop; in.vbat = g_vbat; in.charging = g_charging;
    const bot::Outputs o = ctl.step(in);
    g_pitch = o.pitch;

    const bool want_on = o.motors_on;                // balance, stand-up, flip, dock landing, wheel hold on the dock
    if (want_on && !motors_on) { motors.run(); motors_on = true; }
    if (!want_on && motors_on) { motors.off(); motors_on = false; }
    if (motors_on) motors.torque(o.tl, o.tr);
    else if (++off_cycles % 25 == 0) motors.off();  // re-assert "off" every 50 ms: a single 0x80 lost on the bus would leave
                                                     // the motors on their last torque command
    else motors.request_state();                     // keep the encoders/speeds flowing while off (0x9C = read state 2)
    portENTER_CRITICAL(&mux); g_out = o; portEXIT_CRITICAL(&mux);

    // remember what the stand-up learned (only when resting, flash writes are slow)
    if (std::fabs(o.standup_energy_adj - saved_adj) > 0.5f && o.state == bot::ST_BALANCE && std::fabs(o.v) < 0.05f) {
      saved_adj = o.standup_energy_adj; prefs.putFloat("e_adj", saved_adj);
    }
  }
}

// ------------------------------------------------------------------ comms (core 0)
static void send_frame(uint8_t type, const void* p, uint8_t len) {
  uint8_t b[4 + 255 + 1] = {pilink::SYNC0, pilink::SYNC1, type, len};
  memcpy(b + 4, p, len);
  b[4 + len] = pilink::crc8(b + 2, 2 + len);
  PI_UART.write(b, 5 + len);                    // one write: frames never interleave on a shared port
}

static void event(const char* txt) {
  send_frame(pilink::MSG_EVENT, txt, strnlen(txt, 200));
  Serial.println(txt);
}

static uint32_t last_cmd_ms = 0;
static uint8_t prev_flags = 0;
static PowerOff g_pwr;              // flat battery -> power switch OFF (power_off.h)
static bool g_pwr_pending = false;

static void handle_cmd(const pilink::CmdMsg& m) {
  last_cmd_ms = millis();
  if ((m.flags & pilink::F_ARM) && !(prev_flags & pilink::F_ARM)) { g_armed = true; g_motor_fault = false; }
  if (m.flags & pilink::F_DISARM) g_armed = false;
  prev_flags = m.flags;
  portENTER_CRITICAL(&mux);
  g_cmd.mode = m.mode; g_cmd.v = m.v; g_cmd.yaw_rate = m.yaw_rate; g_cmd.angle = m.angle; g_cmd.seq = m.seq;
  g_cmd.auto_standup = (m.flags & pilink::F_AUTO_STANDUP) ? 1 : 0;
  g_cmd.obstacle_stop = (m.flags & pilink::F_OBST_STOP) ? 1 : 0;
  portEXIT_CRITICAL(&mux);
}

static void poll_uart() {
#ifdef BENCH_CAN
  return;                                       // commands arrive through simhil::on_cmd
#endif
  static uint8_t buf[64]; static int n = 0, need = -1;
  while (PI_UART.available()) {
    const uint8_t b = PI_UART.read();
    if (n == 0 && b != pilink::SYNC0) continue;
    if (n == 1 && b != pilink::SYNC1) { n = 0; continue; }
    buf[n++] = b;
    if (n == 4) need = 4 + buf[3] + 1;
    if (need > (int)sizeof(buf)) { n = 0; need = -1; continue; }
    if (need > 0 && n == need) {
      const uint8_t crc = pilink::crc8(buf + 2, 2 + buf[3]);
      if (crc == buf[n - 1] && buf[2] == pilink::MSG_CMD && buf[3] == sizeof(pilink::CmdMsg)) {
        pilink::CmdMsg m; memcpy(&m, buf + 4, sizeof m); handle_cmd(m);
      }
      n = 0; need = -1;
    }
  }
}

#if defined(SIM_HIL) || defined(BENCH_CAN)   // no battery sense wire in QEMU / on the bench: nominal battery, no charger
static float adc_volts(int pin, float) { return pin == PIN_VBAT ? 22.2f : 0.f; }
#else
static float adc_volts(int pin, float div) { return analogReadMilliVolts(pin) * 1e-3f * div; }
#endif
#if defined(SIM_HIL) || defined(QEMU_NO_LEDC)   // QEMU has no LEDC/GPIO input: LED off, button released
static void led(uint8_t, uint8_t, uint8_t) {}
#define BUTTON_PRESSED() false
#else
static void led(uint8_t r, uint8_t g, uint8_t b) { analogWrite(PIN_LED_R, r); analogWrite(PIN_LED_G, g); analogWrite(PIN_LED_B, b); }
#define BUTTON_PRESSED() (digitalRead(PIN_ARM_BTN) == LOW)
#endif

static void comms_loop(void*) {
  uint32_t last_telem = 0, low_since = 0, btn_down = 0;
  bool btn_prev = false;
  uint8_t prev_state = 255;
  for (;;) {
    const uint32_t now = millis();
    poll_uart();
#ifndef SIM_HIL
#if FRONT_SENSOR == FRONT_TOF
    tof.update(g_pitch);              // the array is bolted to the body: zone elevations move with it
    g_sonar = tof.obstacle_m();
    g_drop = tof.drop_m();
#else
    sonar.update(now);
    g_sonar = sonar.distance();       // an HC-SR04 cannot say where the floor ended, only that it did
    g_drop = -1;
#endif
    irb.update(now);
#endif

    // command watchdog: Pi silent -> stand still (keep balancing; arming state unchanged)
    if ((int32_t)(now - last_cmd_ms) > (int32_t)CMD_TIMEOUT_MS) {      // signed: last_cmd_ms can be newer than `now`
      portENTER_CRITICAL(&mux); g_cmd.v = 0; g_cmd.yaw_rate = 0; g_cmd.mode = bot::CMD_VELOCITY; portEXIT_CRITICAL(&mux);
    }
    // arm button: short press toggles arm (works without the Pi)
    const bool btn = BUTTON_PRESSED();
    if (btn && !btn_prev) btn_down = now;
    if (!btn && btn_prev && now - btn_down > 30) { g_armed = !g_armed; g_motor_fault = false; }
    btn_prev = btn;

    // battery and charger
    const float vb = adc_volts(PIN_VBAT, VBAT_DIV);
    // over-voltage: the 6S rail above a full pack + braking sag means braking current is not going into the battery
    // (BMS charge FET open). The TVS on the PDB holds the rail for a moment; switching the motors off ends the braking.
    static uint8_t ov_n = 0;
    if (vb > VBAT_OVER) { if (++ov_n >= 2 && !g_motor_fault) { g_motor_fault = true; g_ov_event = true; } } else ov_n = 0;
    g_vbat = 0.95f * g_vbat + 0.05f * vb;
    if (g_vbat < VBAT_LOW) { if (!low_since) low_since = now; if (now - low_since > 5000) g_batt_low = true; }
    else { low_since = 0; if (g_vbat > VBAT_LOW + 0.6f) g_batt_low = false; }
    g_charging = adc_volts(PIN_DOCK_SENSE, DOCK_DIV) > CHARGE_DETECT || adc_volts(PIN_USB_SENSE, USB_DIV) > CHARGE_DETECT;
    {
      const bool was = g_pwr_pending;
      const bool off = g_pwr.step(now, g_vbat, g_charging, g_armed);
      g_pwr_pending = g_pwr.pending;
      if (g_pwr_pending && !was) event("battery empty: switching off in 20 s");
      digitalWrite(PIN_PWR_OFF, off ? HIGH : LOW);
    }

    if (motors.enc_event) {
      char t[80]; snprintf(t, sizeof t, "encoder range detected: %d counts/turn", motors.enc_event); motors.enc_event = 0; event(t);
    }
    if (g_ov_event) { g_ov_event = false; char t[64]; snprintf(t, sizeof t, "6S over-voltage %.1f V: motors off", (double)vb); event(t); }
    if (motors.can_event) {
      char t[96];
      if (motors.can_event == 1) snprintf(t, sizeof t, "CAN bus-off #%u: restarting the controller", (unsigned)motors.bus_off_count);
      else snprintf(t, sizeof t, "CAN back after %u ms (%u frames dropped so far)", (unsigned)motors.recovered_after_ms, (unsigned)motors.tx_drop_count);
      motors.can_event = 0; event(t);
    }
    static uint32_t fault_seq_seen = 0;
    if (g_fault_seq != fault_seq_seen) {
      fault_seq_seen = g_fault_seq;
      char t[80]; snprintf(t, sizeof t, "motor fault: no CAN reply for %u ms (L) / %u ms (R)", (unsigned)g_fault_age_l, (unsigned)g_fault_age_r);
      event(t);
    }
    bot::Outputs o;
    portENTER_CRITICAL(&mux); o = g_out; portEXIT_CRITICAL(&mux);
    if (o.state != prev_state) {
      static const char* names[] = {"DISARMED", "LYING", "LIFT", "COAST", "BALANCE", "FALLEN", "CHARGING", "FAULT",
                                    "FLIP_RUN", "FLIP_BRAKE", "FLIP_AIR", "DOCK_LAND"};
      char t[64]; snprintf(t, sizeof t, "state %s side %d tries %u", o.state < 12 ? names[o.state] : "?", o.side, o.standup_tries);
      event(t); prev_state = o.state;
    }
    // status LED: green balancing, blue charging, yellow lying/standing up, red fault/low battery, dim white disarmed
    if (g_batt_low || o.state == bot::ST_FAULT || g_motor_fault) led(255, 0, 0);
    else if (o.state == bot::ST_CHARGING) led(0, 0, (now / 8) % 255);
    else if (o.state == bot::ST_BALANCE) led(0, o.obstacle ? 120 : 255, 0);
    else if (o.state == bot::ST_DISARMED) led(20, 20, 20);
    else led(255, 160, 0);

    if (now - last_telem >= TELEM_PERIOD_MS) {
      last_telem = now;
      pilink::TelemMsg t{};
      t.t_ms = now; t.state = o.state; t.side = o.side; t.tries = o.standup_tries;
      t.flags = (g_armed ? 1 : 0) | (g_charging ? 2 : 0) | (o.obstacle ? 4 : 0) | (g_batt_low ? 8 : 0) | (g_motor_fault ? 16 : 0) | (g_pwr_pending ? 32 : 0);
      t.pitch = o.pitch; t.roll = o.roll; t.yaw = o.yaw; t.x = o.x; t.y = o.y; t.v = o.v;
      t.vbat = g_vbat; t.sonar = g_sonar; t.tl = o.tl; t.tr = o.tr; t.standup_adj = o.standup_energy_adj;
      t.motor_temp = max(motors.left.temp, motors.right.temp);
#ifndef SIM_HIL
      t.ir = irb.bits();
#endif
      send_frame(pilink::MSG_TELEM, &t, sizeof t);
    }
    vTaskDelay(1);
  }
}

void setup() {
#ifdef SIM_HIL
  Serial.begin(2000000);   // UART0 also carries the HIL frames
#elif defined(BENCH_CAN)
  Serial.begin(921600);    // DevKit "UART" USB port: IMU frames + Pi frames + console to the bench PC
  simhil::on_cmd = [](const uint8_t* p, uint8_t len) {
    if (len == sizeof(pilink::CmdMsg)) { pilink::CmdMsg m; memcpy(&m, p, sizeof m); handle_cmd(m); }
  };
#else
  Serial.begin(115200);
#endif
#ifndef BENCH_CAN
  PI_UART.begin(PI_BAUD, SERIAL_8N1, PIN_PI_RX, PIN_PI_TX);
#endif
  pinMode(PIN_PWR_OFF, OUTPUT); digitalWrite(PIN_PWR_OFF, LOW);   // 100 k pull-down on the board too
  pinMode(PIN_ARM_BTN, INPUT_PULLUP);
  pinMode(PIN_LED_R, OUTPUT); pinMode(PIN_LED_G, OUTPUT); pinMode(PIN_LED_B, OUTPUT);
#ifndef SIM_HIL
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
#endif
  prefs.begin("bot", false);
  g_cmd.arm = 0; g_cmd.auto_standup = 1; g_cmd.obstacle_stop = 1;

  delay(300);
  if (!imu.begin()) event("IMU not found: check I2C wiring (SDA 8, SCL 9)");
  if (!motors.begin()) event("CAN (TWAI) start failed");
#ifndef SIM_HIL
#if FRONT_SENSOR == FRONT_TOF
  if (!tof.begin()) event("VL53L5CX not found: check its I2C bus (SDA 35, SCL 36) and LPn");
#else
  sonar.begin();
#endif
  irb.begin();
#endif
  const ImuSample s = imu.read();
  char t[120];
  snprintf(t, sizeof t, "boot: acc %.2f %.2f %.2f m/s2 (upright & still should be ~0 0 +9.8)  gyro %.3f %.3f %.3f rad/s",
           s.ax, s.ay, s.az, s.gx, s.gy, s.gz);
  event(t);

  xTaskCreatePinnedToCore(control_loop, "control", 6144, nullptr, configMAX_PRIORITIES - 1, &control_task, 1);
  xTaskCreatePinnedToCore(comms_loop, "comms", 6144, nullptr, 5, nullptr, 0);
  const esp_timer_create_args_t ta = {.callback = &tick, .arg = nullptr, .dispatch_method = ESP_TIMER_TASK, .name = "ctl",
                                      .skip_unhandled_events = true};
  esp_timer_handle_t timer;
  esp_timer_create(&ta, &timer);
  esp_timer_start_periodic(timer, CONTROL_PERIOD_US);
}

void loop() { vTaskDelay(portMAX_DELAY); }
