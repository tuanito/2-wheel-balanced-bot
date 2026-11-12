#include "controller.hpp"
#include "standup_policy.hpp"
#include "gains.hpp"
#include <cmath>
#include <algorithm>

namespace bot {

namespace {
constexpr float PI = 3.14159265f;
inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline float wrap(float a) {               // -> (-pi, pi]
  while (a > PI) a -= 2 * PI;
  while (a <= -PI) a += 2 * PI;
  return a;
}
inline float slew(float cur, float target, float rate, float dt) {
  float d = target - cur, m = rate * dt;
  return cur + clampf(d, -m, m);
}
// Stand-up / safety thresholds
constexpr float LYING_PITCH    = 0.60f;   // rad (34°): beyond this at arm time we treat it as lying
constexpr float FALL_PITCH     = 0.85f;   // rad (49°): balancing lost
constexpr float SIDE_ROLL      = 0.80f;   // rad (46°): lying on its side, cannot stand up
constexpr float LIFT_TIMEOUT   = 1.2f;    // s
constexpr float STILL_RATE     = 0.6f;    // rad/s: "not moving" while lying
constexpr float WHEEL_W_MAX    = 120.f;   // rad/s: no-load speed at 22 V (53 rpm/V); cut motors beyond 95% of it
constexpr int   MAX_TRIES      = 5;
}  // namespace

Controller::Controller() { reset(); }

void Controller::reset() {
  pitch_ = pitch_rate_ = roll_ = yaw_ = yaw_rate_ = 0;
  phi_ = phi_rate_ = phi_rate_prev_ = acc_fwd_ = 0;
  x_ = y_ = v_ = 0;
  est_init_ = false;
  v_ref_ = phi_ref_ = yaw_ref_ = yaw_rate_ref_ = yaw_target_ = pos_int_ = 0;
  turning_to_ = false;
  st_ = ST_DISARMED; side_ = 0; t_state_ = 0; tries_ = 0; obstacle_ = 0;
}

void Controller::estimate(const Inputs& in_raw) {
  const float dt = in_raw.dt;
  // gyro bias: learn it whenever the robot is not driving and really still (boot, lying, charging)
  const bool idle = st_ == ST_DISARMED || st_ == ST_LYING || st_ == ST_CHARGING || st_ == ST_FAULT;
  if (idle && std::fabs(in_raw.gx - gbias_[0]) < 0.05f && std::fabs(in_raw.gy - gbias_[1]) < 0.05f &&
      std::fabs(in_raw.gz - gbias_[2]) < 0.05f) {
    const float k = clampf(dt / 0.5f, 0, 1);
    gbias_[0] += k * (in_raw.gx - gbias_[0]); gbias_[1] += k * (in_raw.gy - gbias_[1]); gbias_[2] += k * (in_raw.gz - gbias_[2]);
  }
  Inputs in = in_raw;
  in.gx -= gbias_[0]; in.gy -= gbias_[1]; in.gz -= gbias_[2];
  const float R = Params::WHEEL_R;
  // forward acceleration of the axle, from the wheels (to remove it from the accelerometer)
  const float a = acc_fwd_;
  const float fx = in.ax - a * std::cos(pitch_);
  const float fz = in.az - a * std::sin(pitch_);
  const float pitch_acc = std::atan2(-fx, fz);
  if (!est_init_) {
    pitch_ = pitch_acc; est_init_ = true;
    phi_ = 0.5f * (in.ql + in.qr) + pitch_;
  }
  // complementary filter: gyro short-term, accelerometer long-term (faster while lying still)
  float tau = (st_ == ST_BALANCE || st_ == ST_LIFT || st_ == ST_COAST || st_ == ST_DOCK_LAND || st_ == ST_UNDOCK ||
               (st_ == ST_CHARGING && docked_)) ? 0.8f : 0.15f;   // docked: the wheel hold uses pitch, keep it smooth
  if (st_ == ST_FLIP_BRAKE || st_ == ST_FLIP_AIR) tau = 1e6f;      // flying/tumbling: the accelerometer is useless, gyro only
  pitch_rate_ = in.gy;
  rate_lp_ += clampf(dt / 0.05f, 0, 1) * (in.gy - rate_lp_);   // for 'is it still?' checks
  pitch_ = wrap(pitch_ + in.gy * dt);
  pitch_ = wrap(pitch_ + (dt / tau) * wrap(pitch_acc - pitch_));
  roll_ = std::atan2(in.ay, std::sqrt(in.ax * in.ax + in.az * in.az));
  // heading: world-z rate from the gyro (pitch-compensated)
  yaw_rate_ = -std::sin(pitch_) * in.gx + std::cos(pitch_) * in.gz;
  yaw_ += yaw_rate_ * dt;
  // wheels: absolute angle = encoder (relative to body) + body pitch
  phi_ = 0.5f * (in.ql + in.qr) + pitch_;
  phi_rate_ = 0.5f * (in.wl + in.wr) + pitch_rate_;
  const float v_new = R * phi_rate_;
  const float acc_raw = dt > 0 ? (v_new - v_) / dt : 0;
  acc_fwd_ += clampf(dt / 0.03f, 0, 1) * (clampf(acc_raw, -20, 20) - acc_fwd_);
  v_ = v_new;
  // odometry (rolling without slip: distance = R x absolute wheel rotation, also valid while lying/lifting)
  x_ += v_ * std::cos(yaw_) * dt;
  y_ += v_ * std::sin(yaw_) * dt;
}

// free-fall detection: in free fall the accelerometer reads ~0, and on touchdown it spikes
static constexpr float FALL_A = 6.0f;        // m/s^2: below this the wheels are carrying little
static constexpr float LAND_A = 7.0f;        // m/s^2: weight back on the wheels. NOT the touchdown
                                             // spike: a wheel clipping the edge mid-fall reads 18
                                             // for a tick, and exiting on that released the catch
                                             // in mid-air, after which the balance law drove the
                                             // wheels to -3.8 m/s and put the body on its nose.
static constexpr float LAND_HOLD = 0.008f;   // s it must stay there: sustained support, not a spike
static constexpr float STEP_SETTLE = 0.50f;  // s of support before a STEP hands back. The landing
                                             // transient is still running long after touchdown: an
                                             // exit 8 ms in scores 5%, at 0.35 s it throws the body
                                             // to 112 deg, and only past ~0.5 s is it over.
static constexpr float STEP_RATE = 1.2f;     // rad/s: and the body has to have stopped rotating
static constexpr float FALL_DWELL = 0.006f;  // s. Three ticks, not twelve: unloaded wheels go from
                                             // 0.56 to 5.78 m/s in 50 ms, so a 25 ms dwell latches
                                             // after the runaway it exists to prevent.
static constexpr float AIR_MAX = 0.60f;      // s: a drop this long is not a step, so give up and balance
static constexpr float AIR_KD = 0.060f;      // Nm per rad/s: enough to actually arrest the spin

float Controller::sonar_speed_limit(const Inputs& in) {
  obstacle_ = 0;
  float vmax = max_speed;
  if (cmd_.obstacle_stop && in.sonar_m > 0.02f) {
    const float room = in.sonar_m - stop_dist;
    if (room <= 0) { obstacle_ = 1; return 0; }
    vmax = std::sqrt(2 * brake_decel * room);
    if (vmax < max_speed) obstacle_ = 1;
  }
  // The drop fence. Same shape as the obstacle limit and for the same reason - the speed it allows is
  // the one it can still stop from - but measured to the edge of the floor rather than to a wall.
  // It is what a zone array buys that a sonar cannot: the sonar knows its beam lost the floor, not
  // how far ahead that happened, so with an HC-SR04 fitted drop_m stays negative and nothing changes.
  if (cmd_.drop_stop && in.drop_m >= 0.0f) {
    const float room = in.drop_m - cmd_.drop_margin;
    if (room <= 0) { obstacle_ = 1; return 0; }
    const float vdrop = std::sqrt(2 * brake_decel * room);
    if (vdrop < vmax) { vmax = vdrop; obstacle_ = 1; }
  }
  return vmax;
}

void Controller::standup_obs(float* o) const {
  // side-normalised (+ = towards upright from the side it lies on), roughly scaled to [-1, 1]
  o[0] = pitch_ * side_;
  o[1] = 0.1f * pitch_rate_ * side_;
  o[2] = wheel_w_ * side_ / WHEEL_W_MAX;
  o[3] = t_state_ / STANDUP_TIMEOUT;
  o[4] = 0.5f * (vbat_ - 23.f);
  o[5] = nn_tsum_ / (2 * torque_limit);
}

float Controller::body_energy() const {
  return 0.5f * pitch_rate_ * pitch_rate_ - Params::WN2 * (1 - std::cos(pitch_));
}

void Controller::hold_here() {
  phi_ref_ = phi_;
  pos_int_ = 0;
  v_ref_ = clampf(v_, -max_speed, max_speed);
  yaw_ref_ = yaw_; yaw_rate_ref_ = 0; yaw_target_ = yaw_;
  turning_to_ = false;
  last_seq_ = cmd_.seq;
}

void Controller::references(float dt) {
  float v_target = clampf(cmd_.v, -max_speed, max_speed);
  float yr_target = clampf(cmd_.yaw_rate, -max_yaw_rate, max_yaw_rate);
  if (cmd_.mode == CMD_TURN_TO) {
    yaw_target_ = yaw_ref_ + wrap(cmd_.angle - wrap(yaw_ref_));
    turning_to_ = true;
  } else if (cmd_.mode == CMD_TURN_BY && cmd_.seq != last_seq_) {
    last_seq_ = cmd_.seq;
    yaw_target_ = yaw_ref_ + cmd_.angle;
    turning_to_ = true;
  } else if (cmd_.mode == CMD_VELOCITY || cmd_.mode == CMD_DOCK) {
    turning_to_ = false;
  }
  if (turning_to_) {
    v_target = 0;                                   // rotate on the spot
    const float err = yaw_target_ - yaw_ref_;
    // trapezoidal profile: largest rate that can still stop at the target
    const float vstop = std::sqrt(2 * max_yaw_accel * 0.8f * std::fabs(err));
    yr_target = std::copysign(std::min(max_yaw_rate, vstop), err);
    if (std::fabs(err) < 0.002f) { yr_target = 0; yaw_ref_ = yaw_target_; yaw_rate_ref_ = 0; }
  }
  v_target = std::min(v_target, sonar_speed_limit_cache_);
  if (recovering_) {
    // Just caught after a stand-up: first only stabilise the attitude (follow whatever speed we have),
    // then brake to a stop. Asking for a stop immediately fights the catch and topples it over.
    if (t_state_ < recover_hold && (std::fabs(pitch_) > 0.09f || std::fabs(pitch_rate_) > 1.0f)) v_target = v_ref_;   // hold the catch speed
    else v_target = 0;
    if (std::fabs(v_ref_) < 0.05f) recovering_ = false;
  }
  v_ref_ = slew(v_ref_, v_target, recovering_ ? recover_accel : max_accel, dt);
  phi_ref_ += v_ref_ / Params::WHEEL_R * dt;
  yaw_rate_ref_ = slew(yaw_rate_ref_, yr_target, max_yaw_accel, dt);
  yaw_ref_ += yaw_rate_ref_ * dt;
}

void Controller::balance(float dt, float& t_sum, float& t_diff) {
  const float R = Params::WHEEL_R;
  if (look_mode) {
    // Look-down nod: PD on pitch only, no position or speed terms, exactly as the dock lean does it.
    // Keeping the position terms makes them fight the lean and the pitch never gets past ~14 deg.
    look_t_ += dt;
    // Hold the target, then slew it back to upright. Cutting straight back to the normal balance law
    // at 34 deg does not recover - the position terms re-engage against a big pitch error and it goes
    // over. The nod has to be flown back to vertical before the handover.
    // HALF gains (look_gain), like the dock lean: at the full LQR pitch gains this PD is a ~130 rad/s
    // rate loop, and two control periods of CAN latency - which the randomised robots draw half the
    // time - took it unstable. Measured on 10 randomised robots, a 0.8 rad nod: 3/10 stayed up at
    // gain 1.0 and 10/10 at 0.5, with the same peak pitch. Every failure was a delay-2 robot, and
    // every one ended the same way: the pitch loop rang, the wheels got pumped to 5 m/s and the
    // 114 rad/s runaway guard cut the motors.
    // The target is also ramped in rather than stepped, which is not about stability but about reach:
    // the same 0.8 rad target peaks at 28 deg stepped and 35 deg ramped at 4 rad/s, because the body
    // has time to follow instead of the PD saturating against its own error.
    float tgt;
    if (look_osc > 0.0f) {
      // Symmetric swing: sin() starts at 0 going nose-down, so the first quarter cycle is the look.
      const float t_end = look_cycles / look_hz;
      const float frac = t_end > 0.0f ? look_t_ / t_end : 1.0f;
      const float amp = look_osc_end > 0.0f ? look_osc + (look_osc_end - look_osc) * frac : look_osc;
      tgt = look_t_ < t_end ? amp * std::sin(6.2831853f * look_hz * look_t_) : 0.0f;
    } else {
      const float t_down = look_t_ - (look_pitch / look_rate + look_time);
      tgt = t_down <= 0.0f ? std::min(look_pitch, look_t_ * look_rate)
                           : std::max(0.0f, look_pitch - t_down * look_return);
    }
    t_sum = clampf(-look_gain * (Params::K[0] * (pitch_ - tgt) + Params::K[1] * pitch_rate_),
                   -2 * torque_limit, 2 * torque_limit);
    t_diff = Params::KP_YAW * (yaw_ref_ - yaw_) - Params::KD_YAW * yaw_rate_;
    const bool swing_done = look_osc > 0.0f && look_t_ >= look_cycles / look_hz;
    if ((look_osc > 0.0f ? swing_done : tgt <= 0.0f)
        && std::fabs(pitch_) < 0.12f && std::fabs(pitch_rate_) < 1.5f) {
      // Upright again: hand back. v_ref_ takes the speed the nod left behind, it does NOT get zeroed.
      // A nod to 34 deg ends at ~0.55 m/s, and handing over with v_ref_ = 0 demanded all of that back
      // in one tick: the speed term saturated t_sum forward, the body over-rotated to -20 deg in 30 ms
      // and the wheels ran away to 116 rad/s, past the 114 rad/s runaway guard, which cut the motors.
      // Every deep nod ended in ST_FALLEN that way. The normal ramp (max_accel) now takes the speed
      // down to whatever is commanded, which is what the ramp is for.
      look_mode = 0; look_t_ = 0; phi_ref_ = phi_; pos_int_ = 0; v_ref_ = phi_rate_ * R;
    }
    return;
  }
  look_t_ = 0;
  float e_phi = phi_ - phi_ref_;
  const float e_max = 0.25f / R;                   // never chase more than 25 cm of position error
  if (e_phi > e_max) { phi_ref_ = phi_ - e_max; e_phi = e_max; }
  if (e_phi < -e_max) { phi_ref_ = phi_ + e_max; e_phi = -e_max; }
  pos_int_ = clampf(pos_int_ + e_phi * dt, -2.0f / R * 0.1f, 2.0f / R * 0.1f);
  const float* K = Params::K;
  t_sum = -(K[0] * pitch_ + K[1] * pitch_rate_ + K[2] * e_phi + K[3] * (phi_rate_ - v_ref_ / R) + K[4] * pos_int_ + K[5] * last_t_sum_);
  t_diff = Params::KP_YAW * (yaw_ref_ - yaw_) + Params::KD_YAW * (yaw_rate_ref_ - yaw_rate_);
  dbg_[0] = -K[0] * pitch_; dbg_[1] = -K[1] * pitch_rate_; dbg_[2] = -K[2] * e_phi;
  dbg_[3] = -K[3] * (phi_rate_ - v_ref_ / R); dbg_[4] = -K[4] * pos_int_; dbg_[5] = -K[5] * last_t_sum_;
}

Outputs Controller::step(const Inputs& in) {
  const float dt = in.dt;
  estimate(in);
  t_state_ += dt;
  sonar_speed_limit_cache_ = sonar_speed_limit(in);
  float t_sum = 0, t_diff = 0;

  // ---- global transitions ----
  // charge sense: while backing onto the dock it starts the lean-back; otherwise motors off at once.
  // Right after an undock it is ignored for a moment (the contacts are still touching while it stands up).
  if (undock_t_ > 0) undock_t_ -= dt;
  const bool charge = in.charging && undock_t_ <= 0;
  if (charge) {
    if (st_ == ST_BALANCE && cmd_.mode == CMD_DOCK && cmd_.arm) { still_t_ = 0; contact_seen_ = false; lean_tgt_ = std::min(pitch_, 0.0f); go(ST_DOCK_LAND); }
    else if (st_ != ST_CHARGING && st_ != ST_DOCK_LAND) go(ST_CHARGING);
    nocharge_t_ = 0;
  } else if (st_ == ST_CHARGING) {                // contacts bounce while the plate settles: leave only after 0.15 s
    nocharge_t_ += dt;
    if (nocharge_t_ > 0.15f) { docked_ = false; go(ST_DISARMED); }
  }
  if (st_ != ST_CHARGING) hold_on_ = false;
  if (!cmd_.arm && st_ != ST_CHARGING && st_ != ST_DISARMED) go(ST_DISARMED);
  const float wheel_rel = 0.5f * std::fabs(in.wl + in.wr);
  wheel_w_ = 0.5f * (in.wl + in.wr); vbat_ = in.vbat;
  if (st_ >= ST_LIFT && st_ <= ST_BALANCE && wheel_rel > 0.95f * WHEEL_W_MAX) go(ST_FALLEN);   // runaway: cut, then retry

  switch (st_) {
    case ST_DISARMED:
      tries_ = 0;
      if (cmd_.arm && !in.charging && t_state_ > 0.3f) {   // let the attitude filter settle first
        if (std::fabs(pitch_) < LYING_PITCH) { hold_here(); go(ST_BALANCE); }
        else go(ST_LYING);
      }
      break;
    case ST_CHARGING:
      // on the dock: hold the wheels where they are (weak, and only while it leans on the dock like it should)
      // (free for the first 0.6 s so the plate can seat fully on the dock face, then the hold latches)
      // ... only while it settles: after that the motors are off and the floor wave holds the wheels
      hold_on_ = docked_ && cmd_.arm && t_state_ < dock_settle && pitch_ < -0.03f && pitch_ > -0.35f;
      if (hold_on_) {
        t_sum = clampf(-dock_hold_kp * (phi_ - phi_hold_) - dock_hold_kd * phi_rate_, -2 * dock_hold_max, 2 * dock_hold_max);
        t_diff = 0;
      }
      // undock: stand up off the dock face (it leans back ~8°, well inside the balance controller's catch range)
      if (cmd_.arm && cmd_.mode == CMD_UNDOCK && cmd_.seq != dock_seq_) {
        dock_seq_ = cmd_.seq; last_seq_ = cmd_.seq; undock_t_ = undock_ignore; tries_ = 0;
        docked_ = false; go(ST_UNDOCK);
      }
      break;
    case ST_UNDOCK:
      // Ramp the wheels backwards under the body (the plate slides up and off the dock face), then
      // the balance controller takes over near upright: gentler than catching the 8° lean with the full balance gains.
      // (The column is stepped back low down so the robot has room to roll back under itself.)
      t_sum = clampf(-2 * undock_torque * std::min(t_state_ / 0.4f, 1.0f) - undock_damp * phi_rate_, -2 * undock_torque, 2 * undock_torque);
      t_diff = -Params::KD_YAW * yaw_rate_;
      // hand over with the speed it has (the recovery brakes it smoothly; a hard stop made the wheels hop and spin)
      if (pitch_ > -0.05f) { hold_here(); recovering_ = true; go(ST_BALANCE); break; }
      if (t_state_ > 1.5f || pitch_ < -LYING_PITCH) go(ST_FALLEN);
      break;
    case ST_FAULT:
      break;
    case ST_DOCK_LAND: {
      // Lean back onto the dock under control: the pitch target slews from where it is to a bit past the dock lean
      // (a PD on pitch only, no position terms). Before contact that means rolling back slowly with the body tilted
      // back; once the plate is on the face, the wheels roll forward under it until the contacts are pressed home.
      // (Just letting it fall back does not work: with the CoM a few mm off it may fall forwards instead.)
      // Once the contacts touch, stop steering the lean (the estimate can be several degrees off while the plate bumps
      // onto the face, and pushing on climbs the wave and slides the plate down the face).
      if (charge && !contact_seen_) { contact_seen_ = true; contact_pitch_ = pitch_; }
      lean_tgt_ = std::max(lean_tgt_ - dock_lean_rate * dt, -dock_lean_tgt);
      if (!contact_seen_)
        t_sum = clampf(-0.5f * Params::K[0] * (pitch_ - lean_tgt_) - 0.5f * Params::K[1] * pitch_rate_ - dock_roll_damp * phi_rate_,
                       -2 * dock_press_max, 2 * dock_press_max);
      else   // on the face: stop steering the lean, only damp; weight + the floor wave settle it into the trough
        t_sum = clampf(2 * dock_damp * pitch_rate_ - dock_roll_damp * phi_rate_, -2 * dock_press_max, 2 * dock_press_max);
      t_diff = -0.3f * Params::KD_YAW * yaw_rate_;     // light: the funnels may need to turn it a little
      // seated: leaning back (nearly) the full DOCK_LEAN, or it stopped moving (the plate is on the face)
      if (std::fabs(rate_lp_) > 0.05f) still_t_ = 0; else still_t_ += dt;
      const bool resting = contact_seen_ && pitch_ < -dock_rest_pitch && still_t_ > 0.3f;
      if (pitch_ < -LYING_PITCH || pitch_ > FALL_PITCH) { go(ST_FALLEN); break; }        // missed the dock
      if (charge && resting && t_state_ > 0.2f) { docked_ = true; phi_hold_ = phi_; go(ST_CHARGING); break; }   // docked: hold
      if (t_state_ > dock_timeout && charge) { docked_ = true; phi_hold_ = phi_; go(ST_CHARGING); break; }
      if (t_state_ > dock_timeout) {                                                        // no contact: stand up, the Pi retries
        hold_here(); v_ref_ = 0; recovering_ = true; go(ST_BALANCE);
      }
      break;
    }
    case ST_LYING:
      side_ = pitch_ > 0 ? +1 : -1;                 // +1: lying on its face, -1: on its back
      if (std::fabs(rate_lp_) > STILL_RATE) { t_state_ = 0; break; }                // wait until it is still
      if (t_state_ > 0.2f && std::fabs(roll_) > SIDE_ROLL) { go(ST_FAULT); break; }   // resting on its side
      if (std::fabs(pitch_) < LYING_PITCH) { hold_here(); go(ST_BALANCE); break; }   // someone stood it up
      if (cmd_.mode == CMD_BACKFLIP && cmd_.seq != flip_seq_ && side_ < 0 && t_state_ > 0.3f) {
        flip_seq_ = cmd_.seq; last_seq_ = cmd_.seq; flip_turn_ = 0; go(ST_FLIP_BRAKE); break;
      }
      if (cmd_.auto_standup && t_state_ > 0.3f) {
        if (++tries_ > MAX_TRIES) { go(ST_FAULT); break; }
        yaw_ref_ = yaw_;
        last_side_ = side_;
        nn_tick_ = 0; nn_tsum_ = 0;
        go(ST_LIFT);
      }
      break;
    case ST_LIFT: {
      if (standup_mode == 2 || (standup_mode == 1 && STANDUP_POLICY_TRAINED)) {
        // learned stand-up: the policy sets the push every STANDUP_HOLD ticks (10 ms), from lying until the catch
        if (standup_mode == 2) nn_tsum_ = ext_tsum;
        else if (nn_tick_ % STANDUP_HOLD == 0) { float o[STANDUP_NOBS]; standup_obs(o); nn_tsum_ = standup_policy(o) * 2 * torque_limit; }
        ++nn_tick_;
        t_sum = side_ * clampf(nn_tsum_, -2 * torque_limit, 2 * torque_limit);
        t_diff = Params::KP_YAW * (yaw_ref_ - yaw_) - Params::KD_YAW * yaw_rate_;
        if (std::fabs(pitch_) < catch_pitch) { hold_here(); recovering_ = true; go(ST_BALANCE); }
        else if (pitch_ * side_ < -LYING_PITCH || t_state_ > STANDUP_TIMEOUT) go(ST_FALLEN);   // went over / gave up
        break;
      }
      // Kick: full push towards the side the top points to; the reaction swings the body up.
      // Stop as soon as the body has (almost) the energy to reach upright, the COAST phase trims the rest.
      t_sum = side_ * lift_torque * 2;
      t_diff = Params::KP_YAW * (yaw_ref_ - yaw_) - Params::KD_YAW * yaw_rate_;
      const bool moving_up = pitch_rate_ * side_ < 0;
      if (std::fabs(pitch_) < catch_pitch) { hold_here(); recovering_ = true; go(ST_BALANCE); }
      else if (moving_up && body_energy() >= -lift_gain) go(ST_COAST);
      else if (t_state_ > LIFT_TIMEOUT) go(ST_FALLEN);
      break;
    }
    case ST_COAST: {
      // Energy shaping: steer the body's energy to exactly "just reaches upright" while it swings up.
      // dE/dt = pitch_rate * B * t_sum with B < 0  ->  t_sum = k E pitch_rate drives E to 0.
      const float E = body_energy() - (energy_target + et_adapt_);
      t_sum = clampf(energy_gain * E * pitch_rate_, -2 * lift_torque, 2 * lift_torque);
      t_diff = Params::KP_YAW * (yaw_ref_ - yaw_) - Params::KD_YAW * yaw_rate_;
      if (std::fabs(pitch_) < catch_pitch) { hold_here(); recovering_ = true; go(ST_BALANCE); }
      else if (pitch_rate_ * side_ > 0.3f || t_state_ > 1.0f) {   // falling back: not enough energy
        et_adapt_ = clampf(et_adapt_ + 4.f, -20.f, 20.f);          // learn: aim a bit higher next time
        go(ST_FALLEN);
      }
      break;
    }
    case ST_BALANCE:
      if (std::fabs(pitch_) > FALL_PITCH) {
        // fell over to the other side right after a stand-up: too much energy, aim lower next time
        if (recovering_ && t_state_ < 1.0f && pitch_ * last_side_ < 0) et_adapt_ = clampf(et_adapt_ - 4.f, -20.f, 20.f);
        go(ST_FALLEN); break;
      }
      if (t_state_ > 2.0f) tries_ = 0;
      if (cmd_.mode == CMD_BACKFLIP && cmd_.seq != flip_seq_) {
        flip_seq_ = cmd_.seq; last_seq_ = cmd_.seq;
        if (!recovering_ && std::fabs(v_) < 0.2f) { go(ST_FLIP_RUN); break; }
      }
      if (cmd_.mode == CMD_DOCK && cmd_.seq != dock_seq_) {          // the Pi says: plate at the dock face, lean back now
        dock_seq_ = cmd_.seq; last_seq_ = cmd_.seq;
        if (!recovering_ && std::fabs(v_) < 0.15f) { { still_t_ = 0; contact_seen_ = false; lean_tgt_ = std::min(pitch_, 0.0f); go(ST_DOCK_LAND); } break; }
      }
      // --- a known step down -------------------------------------------------------------
      // The free-fall catch is reactive, and reacting is not enough: rolling off an edge at walking
      // pace there IS no clean free fall, because the wheel stays in contact with the corner the
      // whole way down. |a| never drops far enough to trigger anything, the velocity loop sees its
      // support vanish, drives the wheels to -4.9 m/s, and the wheel slams into the riser face at
      // 58 m/s^2. Measured, repeatedly.
      //
      // So the robot commits instead of reacting. It knows where the edge is - the zone array puts
      // it within 13 mm - so it can enter this mode at a chosen distance and simply hold its
      // attitude across the lip: pitch servo only, at half gains, with the speed loop switched off
      // entirely so nothing can command a lurch while the wheels are unloaded. It leaves on
      // sustained support, handing back the speed it actually has.
      // Latched, and it clears ITSELF. The first version simply ran while the command was held,
      // which meant it re-did its handover every tick - and that accidentally kept the position loop
      // switched off, which is most of why it worked. Releasing the command then handed back to a
      // fresh position loop that immediately fought the robot: 98% became 65%. A step is one event
      // with a beginning and an end, so the controller owns both.
      if (cmd_.step_mode && !step_prev_) { step_active_ = true; step_t_ = 0; land_t_ = 0; }
      step_prev_ = cmd_.step_mode;
      if (step_active_) {
        step_t_ += dt;
        const float a_mag2 = std::sqrt(in.ax * in.ax + in.ay * in.ay + in.az * in.az);
        if (step_t_ > 0.15f && a_mag2 > LAND_A) land_t_ += dt; else land_t_ = 0;
        if ((land_t_ > STEP_SETTLE && std::fabs(pitch_rate_) < STEP_RATE)
            || step_t_ > cmd_.step_max) {
          step_active_ = false;
          step_t_ = 0; land_t_ = 0;
          phi_ref_ = phi_; pos_int_ = 0; v_ref_ = phi_rate_ * Params::WHEEL_R;
          airborne_ = false;
        } else {
          t_sum = clampf(-0.5f * (Params::K[0] * (pitch_ - cmd_.step_pitch) + Params::K[1] * pitch_rate_),
                         -2 * torque_limit, 2 * torque_limit);
          t_diff = Params::KP_YAW * (yaw_ref_ - yaw_) - Params::KD_YAW * yaw_rate_;
          break;
        }
      }

      // --- free fall ---------------------------------------------------------------------
      // Taking a step DOWN, the wheels leave the ground together (one axle), and for ~150 ms there
      // is nothing to push against. The balance law does not know that: it still sees a pitch error
      // and still commands torque, the unloaded wheels spin up to 2 m/s in 200 ms, and the reaction
      // throws the body to -67 deg and onto its back. Measured, repeatedly, on steps as small as
      // 40 mm - the robot was not losing a fight with the drop, it was losing one with its own
      // velocity loop. So while airborne: damp the wheels and leave the attitude alone. Gravity
      // cannot change the body's angular momentum, so an attitude that was upright at the edge is
      // still upright at touchdown.
      {
        const float a_mag = std::sqrt(in.ax * in.ax + in.ay * in.ay + in.az * in.az);
        if (cmd_.fall_catch && a_mag < FALL_A) air_t_ += dt; else air_t_ = 0;
        if (!airborne_ && air_t_ > FALL_DWELL) { airborne_ = true; ground_t_ = 0; land_t_ = 0; }
        if (airborne_) {
          ground_t_ += dt;
          // Leave on the touchdown SPIKE, not on the accelerometer merely recovering: a wheel
          // clipping the edge on the way past reads like ground for a tick and would drop the catch
          // in the middle of the fall.
          if (a_mag > LAND_A) land_t_ += dt; else land_t_ = 0;
          if (land_t_ > LAND_HOLD || ground_t_ > AIR_MAX) {
            airborne_ = false;
            // the same handover the nod uses: never hand back a zero speed reference while moving
            phi_ref_ = phi_; pos_int_ = 0; v_ref_ = phi_rate_ * Params::WHEEL_R;
          }
        }
      }
      if (airborne_) {
        t_sum = clampf(-AIR_KD * phi_rate_, -torque_limit, torque_limit);   // damp only: no servo
        t_diff = 0;
        break;
      }
      references(dt);
      balance(dt, t_sum, t_diff);
      side_ = 0;
      break;
    case ST_FLIP_RUN: {
      // backflip step 1 (KENBOT style): lie down on the back. A short forward push tips the body backwards,
      // then motors off until it rests on its back.
      t_sum = (t_state_ < flip_lie_time) ? 2 * flip_lie_torque : 0;
      t_diff = 0;
      if (pitch_ < -1.2f && std::fabs(rate_lp_) < STILL_RATE && t_state_ > flip_lie_time + 0.4f) { flip_turn_ = 0; go(ST_FLIP_BRAKE); }
      else if (t_state_ > 3.0f) go(ST_FALLEN);
      break;
    }
    case ST_FLIP_BRAKE:
    case ST_FLIP_AIR: {
      // step 2: kick. Lying on the back, drive the wheels forward at full torque: the body levers on its back edge,
      // the wheels are thrown up and over the top. step 3: keep a little torque on while it flies, catch on the wheels.
      flip_turn_ += pitch_rate_ * dt;
      t_diff = 0;
      t_sum = 2 * (st_ == ST_FLIP_BRAKE ? torque_limit : flip_air_torque);
      if (st_ == ST_FLIP_BRAKE && t_state_ >= flip_brake_time) go(ST_FLIP_AIR);
      else if (st_ == ST_FLIP_AIR) {
        if (flip_turn_ < -PI && std::fabs(pitch_) < flip_catch) {         // came over the top onto the wheels: catch
          hold_here(); v_ref_ = 0; recovering_ = true; go(ST_BALANCE);
        } else if (t_state_ > 1.5f) go(ST_FALLEN);                       // missed: normal stand-up
      }
      break;
    }
    case ST_FALLEN:
      if (t_state_ > 0.5f && std::fabs(rate_lp_) < STILL_RATE) go(ST_LYING);
      break;
  }

  // ---- torque split with saturation: balance (sum) has priority over steering (difference) ----
  const float lim = torque_limit;
  float half = clampf(0.5f * t_sum, -lim, lim);
  float dmax = lim - std::fabs(half);
  t_diff = clampf(t_diff, -dmax, dmax);
  Outputs o{};
  const bool on = st_ == ST_LIFT || st_ == ST_COAST || st_ == ST_BALANCE || st_ >= ST_FLIP_RUN || (st_ == ST_CHARGING && hold_on_);
  last_t_sum_ = on ? 2 * half : 0;
  o.tl = on ? half - t_diff : 0;
  o.tr = on ? half + t_diff : 0;
  o.state = st_; o.side = side_;
  o.pitch = pitch_; o.pitch_rate = pitch_rate_; o.roll = roll_; o.yaw = yaw_;
  o.x = x_; o.y = y_; o.v = v_;
  o.v_ref = v_ref_; o.yaw_ref = yaw_ref_;
  o.obstacle = obstacle_; o.standup_tries = tries_; o.standup_energy_adj = et_adapt_;
  o.airborne = airborne_ ? 1 : 0;
  o.stepping = step_active_ ? 1 : 0;
  for (int i = 0; i < 6; ++i) o.dbg[i] = dbg_[i];
  o.motors_on = on;
  return o;
}

}  // namespace bot
