// The two halves of the motor choice must agree, and the compiler must say so.
//
// config.h's FITTED_MOTOR carries the DRIVE's side of the choice (Kt, what an iq command of 2048
// means in amps). core/motor.hpp's BOT_MOTOR_* carries the CONTROLLER's (torque limit, speed
// ceiling, rotor inertia). Set one without the other and the firmware commands torques its own
// controller does not believe in - and nothing says so at run time. Measured cost of that exact
// mismatch in the simulator: the stand-up scored 0/30 instead of 30/30, and the explanation that
// fitted the numbers was a physical motor speed ceiling, which was wrong.
//
// src/main.cpp static_asserts KT == bot::motor::KT. This translation unit is that assert on its
// own, so run_tests.sh can compile it four ways - two matched, two crossed - and check that the
// crossed pairs FAIL to compile. A static_assert nobody ever trips is a comment.
#include "config.h"
#include "motor.hpp"

static_assert(KT == bot::motor::KT,
              "config.h's FITTED_MOTOR and core/motor.hpp's BOT_MOTOR_* disagree about which motor "
              "is fitted. Set both: -D FITTED_MOTOR=MOTOR_MF7015 -D BOT_MOTOR_MF7015");

int main() { return 0; }
