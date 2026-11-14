// The fitted hub motor's limits, as the CONTROLLER needs them.
//
// One robot has one motor, so this is a compile-time choice, the same shape as FRONT_SENSOR in
// firmware/esp32/include/config.h. Default = the MF6015-V2 15T, the robot that exists. Build with
// -DBOT_MOTOR_MF7015 for the bigger Ø69 unit (README, "A bigger motor: MF7015-V2"):
//
//     ROBOT_MOTOR=mf7015 core/build_lib.sh          # the simulator's library
//     build_flags = -DBOT_MOTOR_MF7015              # platformio.ini, for the firmware
//
// These four numbers were literals spread through controller.cpp and controller.hpp, every one of
// them an MF6015 value with an MF6015 derivation in its comment. Measured consequence of leaving
// torque_limit behind when the plant changed: the MF7015 scored 0/30 on the stand-up instead of
// 30/30, and it looked exactly like a physical speed ceiling. See CLAUDE.md.
//
// The single source of truth for the motors themselves is cad_src/case/motors.py (the CAD and
// sim/motor_profile.py read it). This header is the C++ mirror of the control-relevant half - the
// controller has to compile for the ESP32 with no Python - and sim/check_motor.py compiles a probe
// per profile and fails if the two disagree. ./check.sh runs it.
#pragma once

namespace bot {
namespace motor {

#if defined(BOT_MOTOR_MF7015)

constexpr const char* NAME = "mf7015";
constexpr float TAU_MAX = 3.8f;      // N·m per motor: 15 A × 0.28 N·m/A (L-7015 parameter sheet, 23T)
constexpr float KT      = 0.28f;     // N·m/A
constexpr float W_MAX   = 79.0f;     // rad/s: no-load at 22 V (34 rpm/V). The 95% guard below sits at
                                     // 75.1, which the stand-up kick reaches: see the guard in controller.cpp.
constexpr float ROTOR_J = 933e-7f;   // kg·m², the part that turns with the wheel

#else   // the as-built robot

constexpr const char* NAME = "mf6015";
constexpr float TAU_MAX = 2.3f;      // N·m per motor (LK drive: iq 2048 ≙ 16.5 A × 0.14 N·m/A; datasheet 2.31)
constexpr float KT      = 0.14f;     // N·m/A
constexpr float W_MAX   = 120.0f;    // rad/s: no-load at 22 V (53 rpm/V); cut motors beyond 95% of it
constexpr float ROTOR_J = 515e-7f;   // kg·m²

#endif

// Reference rotor inertia the air-damping gain was tuned against, so AIR_KD can scale with it
// instead of being re-guessed per motor.
constexpr float ROTOR_J_REF = 515e-7f;

}  // namespace motor
}  // namespace bot
