// Balancing-robot controller core: state estimation, LQR balance, speed/heading tracking,
// lying-side detection, stand-up from face-down or back-down, safety.
// Plain C++17, no Arduino/ESP-IDF dependencies: the SAME file runs in the ESP32 firmware
// and in the MuJoCo simulation (compiled as a shared library and called through ctypes).
//
// Robot frame: X forward, Y left, Z up, origin on the wheel axle.
// Pitch > 0 = top leaning forward (towards +X). Lying face-down ≈ +100°, back-down ≈ -97°.
// Wheel angles/speeds: positive = wheel turning so the robot rolls forward (+X), measured
// RELATIVE TO THE BODY (that is what the motor encoder sees).
#pragma once
#include <cstdint>
#include "motor.hpp"   // the fitted motor's torque limit, speed ceiling and rotor inertia

namespace bot {

enum State : uint8_t {
  ST_DISARMED = 0,  // motors off, waiting for arm
  ST_LYING    = 1,  // armed, lying on front or back, about to stand up
  ST_LIFT     = 2,  // stand-up: pushing the body up
  ST_COAST    = 3,  // stand-up: swinging up, torque only trims the body's energy
  ST_BALANCE  = 4,  // balancing / driving
  ST_FALLEN   = 5,  // fell over: motors off for a moment, then back to LYING
  ST_CHARGING = 6,  // on the dock or USB-C: motors forced off
  ST_FAULT    = 7,  // lying on its side, over-speed, repeated failed stand-ups... needs re-arm
  ST_FLIP_RUN   = 8,  // backflip: lie down on the back first
  ST_FLIP_BRAKE = 9,  // backflip: kick, levering on the back
  ST_FLIP_AIR   = 10, // backflip: flying over the top, waiting to come round onto the wheels
  ST_DOCK_LAND  = 11, // docking: backed up to the vertical dock, leaning back onto it (motors damp only)
  ST_UNDOCK     = 12, // leaving the dock: wheels roll back under the body until it is upright, then BALANCE
};

enum CmdMode : uint8_t {
  CMD_VELOCITY = 0,  // follow v (m/s) and yaw_rate (rad/s)
  CMD_TURN_TO  = 1,  // stand still and rotate to an absolute heading (rad)
  CMD_TURN_BY  = 2,  // rotate by a relative angle (rad), once per new seq
  CMD_BACKFLIP = 3,  // do one backflip (lie on the back, kick, flip over the top), once per new seq
  CMD_DOCK     = 4,  // drive like VELOCITY (the Pi reverses slowly); contacts touching -> lean back onto the dock.
                    // A new seq while balancing = 'at the dock face, lean back now'
  CMD_UNDOCK   = 5,  // on the dock: a new seq stands it up off the dock (then drive away with VELOCITY)
};

struct Inputs {
  float dt;                 // s since the previous call
  float gx, gy, gz;         // gyro, rad/s, robot frame
  float ax, ay, az;         // accelerometer, m/s^2, robot frame (reads +9.81 on Z when upright & still)
  float ql, qr;             // wheel angles rad, relative to body, multi-turn
  float wl, wr;             // wheel speeds rad/s, relative to body
  float sonar_m;            // nearest obstacle ahead, m (<= 0: no echo / invalid). Either front
                            // sensor fills this: the HC-SR04's echo, or the zone array's nearest
                            // return above the floor plane.
  float drop_m = -1;        // distance to where the floor AHEAD ends, m (< 0: unknown). Only a
                            // bearing-resolved sensor can fill this honestly - the sonar cone says
                            // "something is gone" without saying where - so it stays -1 with an
                            // HC-SR04 fitted and the fence below simply never arms.
  float vbat;               // battery voltage, V
  uint8_t charging;         // 1 = charger connected
};

struct Command {
  uint8_t mode = CMD_VELOCITY;
  float v = 0;              // m/s
  float yaw_rate = 0;       // rad/s
  float angle = 0;          // rad (TURN_TO: absolute heading, TURN_BY: relative)
  uint16_t seq = 0;         // bump to issue a new TURN_BY
  uint8_t arm = 0;          // 1 = armed (motors allowed)
  uint8_t auto_standup = 1; // stand up by itself when lying
  uint8_t obstacle_stop = 1;// refuse to drive forward into a sonar obstacle
  uint8_t fall_catch = 1;   // stop servoing the wheels while airborne (see the free-fall branch)
  uint8_t step_mode = 0;    // 1 = taking a known step down: hold attitude, do not servo speed
  float step_pitch = 0.0f;  // rad: attitude to hold across the edge (slightly negative = heels first)
  float step_roll = 0.30f;  // m/s: speed to carry over the edge
  float step_max = 1.2f;    // s: give up and go back to balancing if no landing is detected
  uint8_t drop_stop = 1;    // refuse to drive forward over an edge the front sensor can see
  float drop_margin = 0.25f;// m of floor that must remain ahead before forward driving is refused
};

struct Outputs {
  float tl, tr;             // motor torque commands, N·m, positive = drive forward
  uint8_t state;
  int8_t side;              // +1 face-down, -1 back-down, 0 upright/unknown
  float pitch, pitch_rate, roll, yaw;   // rad, rad/s
  float x, y, v;            // odometry, m, m/s
  float v_ref, yaw_ref;
  uint8_t airborne;         // 1 = the free-fall catch is holding the wheels (accelerometer)
  uint8_t stepping;         // 1 = taking a commanded step down (latched; clears itself on landing)
  uint8_t obstacle;         // 1 = sonar is limiting forward speed
  uint8_t standup_tries;
  float standup_energy_adj;  // learned correction, rad^2/s^2 (store it in flash on the real robot)
  float dbg[6];             // LQR terms (debug/telemetry)
  uint8_t motors_on;        // 1 = the drives must be running (torque mode), 0 = motor off
};

struct Params;  // see gains.hpp

class Controller {
 public:
  Controller();
  void reset();
  void set_command(const Command& c) { cmd_ = c; }
  Outputs step(const Inputs& in);
  // learned stand-up energy correction: save it (NVS/flash) when it changes, restore it at boot
  void set_standup_energy_adj(float v) { et_adapt_ = v; }
  // Tunables that are not LQR gains
  float max_speed = 1.5f;       // m/s
  float max_accel = 0.6f;       // m/s^2 for the speed reference (low CoM: 0.6 m/s^2 already means ~9° of lean)
  float max_yaw_rate = 1.2f;    // rad/s (sim: pitch wobble grows above ~1.8 rad/s)
  float max_yaw_accel = 3.0f;   // rad/s^2
  // N·m per motor. From the motor profile, NOT a literal: with the MF7015's plant and the
  // MF6015's 2.3 here, the stand-up scores 0/30 instead of 30/30. See core/motor.hpp.
  float torque_limit = motor::TAU_MAX;   // mf6015 2.3, mf7015 3.8
  float stop_dist = 0.25f;      // m, sonar: closest allowed approach
  float brake_decel = 1.0f;     // m/s^2 used to compute the sonar speed limit
  // Stand-up (tuned in sim/tune_standup.py)
  float lift_torque = 1.6f;     // N·m per motor while pushing the body up (1.9 tried for the +10 mm body: no better)
  float catch_pitch = 0.35f;    // rad (20°): balance controller takes over below this
  float lift_gain = 30.f;       // stop the kick once body energy >= -lift_gain (rad^2/s^2)
  float energy_gain = 0.03f;    // N·m per (rad^2/s^2 · rad/s), coast energy shaping
  float recover_accel = 1.0f;   // m/s^2: braking right after a stand-up
  float recover_hold = 0.0f;    // s: keep the catch speed this long while the attitude settles
  // Learned stand-up (ml/): 0 = the hand-tuned kick + energy shaping above, 1 = the neural policy in standup_policy.hpp
  // (falls back to 0 while that file holds no trained policy), 2 = torque from outside (the trainer, sim only)
  uint8_t standup_mode = 0;
  float ext_tsum = 0;           // mode 2: N·m for both motors together, in the side-normalised frame (+ = push up)
  void standup_obs(float* o) const;   // the policy's inputs (STANDUP_NOBS floats), same in the sim and on the robot
  // Look-down nod: hold a pitch target so the lidar's scan plane tilts down and sees the floor ahead.
  // An inverted pendulum holds a pitch only by accelerating, so this ALWAYS drives the robot towards
  // whatever it is looking at - enter it while reversing and the net travel is still away. The normal
  // balance law cannot do it: commanded velocity tops out around 14 deg of pitch, and the geometry needs
  // 30-40 deg to see over a step nose. So, like the dock lean, this drops the position terms and runs a
  // PD on pitch alone; it is time-limited because it cannot be held.
  uint8_t look_mode = 0;        // 0 = off, 1 = hold look_pitch while look_time has not run out
  float look_pitch = 0.6f;      // rad (+ = nose down / top forward): 0.6 rad = 34 deg
  float look_time = 0.4f;       // s: the nod holds the target this long, then flies back to upright
  float look_rate = 4.0f;       // rad/s the target ramps in at: sets how deep a given target actually goes
  float look_gain = 0.5f;       // fraction of the LQR pitch gains the nod's PD uses (the dock lean: same)
  // Oscillating look: instead of one flick, swing the pitch target through look_cycles full cycles of
  // +-look_osc at look_hz. A flick is one-sided, so gravity acts forward the whole time and the robot
  // ends up metres down range; a symmetric swing spends as long leaning back as forward, so the net
  // travel is close to nothing and the scan can be taken from much closer to the drop.
  float look_osc = 0;           // rad amplitude: 0 = single flick (look_pitch / look_time / look_return)
  float look_hz = 1.5f;         // Hz of the swing
  float look_cycles = 2.0f;     // full cycles before it returns to upright
  float look_osc_end = 0;       // rad: if > 0, the amplitude grows from look_osc to this across the
                                // cycles, so each swing probes a deeper angle than the last. The robot
                                // cannot know how tall the step is before it has measured it, and the
                                // angle that sees a 170 mm riser is not the one that sees an 80 mm one,
                                // so the sweep IS the search for it.
  float look_return = 2.0f;     // rad/s the target slews back at (0.6 rad -> upright in 0.3 s)
  float look_t_ = 0;            // s spent in the current nod
  // Backflip, KENBOT style (tuned in sim/flip_tune2.py)
  float flip_lie_torque = 0.6f; // N·m per motor to tip over onto the back
  float flip_lie_time = 0.15f;  // s
  float flip_brake_time = 0.05f;// s of full kick torque
  float flip_air_torque = 1.0f; // N·m per motor while flying
  float flip_catch = 0.6f;      // rad: catch with the balance controller within this of upright, after a full turn
  float energy_target = 0.f;    // rad^2/s^2 of spare energy at the top (covers model error)
  // Docking (tuned in sim/dock_sim.py)
  float dock_lean_tgt = 0.17f;   // rad (10°): lean-back target before contact (dock at 8°; covers a few degrees of estimate error,
                                 // the floor ridge stops it rolling out further)
  float dock_lean_rate = 0.2f;   // rad/s: how fast the target leans back
  float dock_press_max = 0.5f;   // N·m per motor while landing
  float dock_rest_pitch = 0.05f; // rad (3°): contacts touching, leaning back at least this much and still 0.3 s = docked
  float dock_damp = 0.12f;       // N·m per rad/s of pitch rate: damping only, once the contacts touch
  float dock_roll_damp = 0.15f;  // N·m per rad/s of wheel speed: the wheels roll out while it leans back, slowly
  float dock_timeout = 2.5f;     // s: no contacts by then -> stand up again (the Pi retries)
  float undock_ignore = 1.5f;    // s: ignore the charge sense while the contacts separate
  float undock_damp = 0.5f;     // N·m per rad/s of wheel speed: keeps the roll-back slow (~0.1 m/s)
  float undock_torque = 0.6f;    // N·m per motor, reached in 0.4 s: rights the body (its weight on the face ~0.15 N·m; more just spins the wheels on a slippery floor)
  // Docked: leaning on the dock pushes the robot forward (a ladder on a slippery floor). The floor wave in front of the
  // wheels stops that; for the first seconds the motors also hold/damp the wheels so the landing settles in the trough
  float dock_hold_kp = 6.0f;     // N·m per rad of wheel rotation (sum of both motors)
  float dock_hold_kd = 0.3f;     // N·m per rad/s
  float dock_settle = 0.0f;      // s: hold the wheels this long after docking, then motors off. 0 = motors off at once:
                                 // the floor wave holds it (sim: no difference); kept as an option
  float dock_hold_max = 0.25f;   // N·m per motor (≈ 9 N at the tyres). The phase current for it is
                                 // motor-dependent: 1.8 A peak at Kt 0.14, 0.9 A at Kt 0.28

 private:
  void estimate(const Inputs& in);
  void references(float dt);
  float sonar_speed_limit(const Inputs& in);
  void balance(float dt, float& t_sum, float& t_diff);
  Command cmd_;
  uint16_t last_seq_ = 0;
  // estimator
  float rate_lp_ = 0;
  float gbias_[3] = {0, 0, 0};    // learned gyro bias, rad/s
  float pitch_ = 0, pitch_rate_ = 0, roll_ = 0, yaw_ = 0, yaw_rate_ = 0;
  bool airborne_ = false;           // wheels off the ground: see the free-fall branch in balance()
  float air_t_ = 0, ground_t_ = 0, land_t_ = 0;
  float step_t_ = 0;                // time in the step
  bool step_active_ = false;        // latched on the rising edge of cmd_.step_mode, cleared on landing
  uint8_t step_prev_ = 0;
  float phi_ = 0, phi_rate_ = 0;          // mean absolute wheel angle / rate (rad, rad/s)
  float phi_rate_prev_ = 0, acc_fwd_ = 0;
  float x_ = 0, y_ = 0, v_ = 0;
  bool est_init_ = false;
  // references
  float v_ref_ = 0, phi_ref_ = 0, yaw_ref_ = 0, yaw_rate_ref_ = 0, yaw_target_ = 0;
  bool turning_to_ = false;
  float pos_int_ = 0;
  // state machine
  State st_ = ST_DISARMED;
  int8_t side_ = 0;
  float wheel_w_ = 0, vbat_ = 22.2f, nn_tsum_ = 0;   // learned stand-up: wheel speed (rad/s), battery, held action
  uint16_t nn_tick_ = 0;
  float t_state_ = 0;
  uint8_t tries_ = 0;
  uint8_t obstacle_ = 0;
  float sonar_speed_limit_cache_ = 0;
  bool recovering_ = false;
  float flip_turn_ = 0;           // accumulated pitch rotation since the brake (rad)
  uint16_t flip_seq_ = 0;
  uint16_t dock_seq_ = 0;
  float undock_t_ = 0;            // s left to ignore the charge sense after an undock
  bool docked_ = false;           // CHARGING entered from DOCK_LAND: hold the wheels
  bool hold_on_ = false;
  float still_t_ = 0;
  float lean_tgt_ = 0;
  bool contact_seen_ = false;
  float contact_pitch_ = 0;
  float nocharge_t_ = 0;
  float phi_hold_ = 0;
  float last_t_sum_ = 0;
  float et_adapt_ = 0;            // learned stand-up energy correction (kept across attempts)
  int8_t last_side_ = 0;
  float dbg_[6] = {0, 0, 0, 0, 0, 0};          // torque sent last period (still being applied: actuation delay)
  void go(State s) { st_ = s; t_state_ = 0; }
  void hold_here();
  float body_energy() const;     // 0.5*rate^2 - WN2*(1-cos(pitch)); 0 = just reaches upright
};

}  // namespace bot
