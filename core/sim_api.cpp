// C API so the simulation (Python ctypes) runs exactly this controller code.
#include "controller.hpp"
extern "C" {
void* bot_new() { return new bot::Controller(); }
void bot_free(void* p) { delete static_cast<bot::Controller*>(p); }
void bot_set_cmd(void* p, const bot::Command* c) { static_cast<bot::Controller*>(p)->set_command(*c); }
void bot_tune(void* p, float lift, float time, float gain, float catch_p) { auto* c = static_cast<bot::Controller*>(p); c->lift_torque = lift; (void)time; c->lift_gain = gain; c->catch_pitch = catch_p; }
void bot_lift_mode(void* p, int m, float eg, float et) { auto* c = static_cast<bot::Controller*>(p); (void)m; c->energy_gain = eg; c->energy_target = et; }
void bot_step(void* p, const bot::Inputs* in, bot::Outputs* out) { *out = static_cast<bot::Controller*>(p)->step(*in); }
}
extern "C" void bot_limits(void* p, float max_speed, float max_accel, float max_yaw_rate, float max_yaw_accel) {
  auto* c = static_cast<bot::Controller*>(p);
  c->max_speed = max_speed; c->max_accel = max_accel; c->max_yaw_rate = max_yaw_rate; c->max_yaw_accel = max_yaw_accel;
}
extern "C" void bot_recover(void* p, float accel, float hold) { auto* c = static_cast<bot::Controller*>(p); c->recover_accel = accel; c->recover_hold = hold; }
extern "C" void bot_torque_limit(void* p, float lim) { static_cast<bot::Controller*>(p)->torque_limit = lim; }
extern "C" void bot_flip(void* p, float lie_tq, float kick_t, float air_tq, float catch_p) {
  auto* c = static_cast<bot::Controller*>(p); c->flip_lie_torque = lie_tq; c->flip_brake_time = kick_t; c->flip_air_torque = air_tq; c->flip_catch = catch_p;
}
extern "C" void bot_dock(void* p, float lean_tgt, float lean_rate, float press_max, float roll_damp) {
  auto* c = static_cast<bot::Controller*>(p); c->dock_lean_tgt = lean_tgt; c->dock_lean_rate = lean_rate; c->dock_press_max = press_max; c->dock_roll_damp = roll_damp;
}
extern "C" void bot_dock_hold(void* p, float kp, float kd, float max_tq) {
  auto* c = static_cast<bot::Controller*>(p); c->dock_hold_kp = kp; c->dock_hold_kd = kd; c->dock_hold_max = max_tq;
}
extern "C" void bot_standup(void* p, int mode, float ext_tsum) {
  auto* c = static_cast<bot::Controller*>(p); c->standup_mode = (uint8_t)mode; c->ext_tsum = ext_tsum;
}
extern "C" void bot_standup_obs(void* p, float* o) { static_cast<bot::Controller*>(p)->standup_obs(o); }
extern "C" void bot_look_gains(void* p, float rate, float ret, float gain) {
  auto* c = static_cast<bot::Controller*>(p);
  c->look_rate = rate; c->look_return = ret; c->look_gain = gain;
}
extern "C" void bot_look_osc(void* p, float amp, float hz, float cycles, float amp_end) {
  auto* c = static_cast<bot::Controller*>(p);
  c->look_osc = amp; c->look_hz = hz; c->look_cycles = cycles; c->look_osc_end = amp_end;
}
extern "C" void bot_look(void* p, int on, float pitch, float time) {
  auto* c = static_cast<bot::Controller*>(p);
  c->look_mode = (uint8_t)on; c->look_pitch = pitch; c->look_time = time; c->look_t_ = 0;
}
