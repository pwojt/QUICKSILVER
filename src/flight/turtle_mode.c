#include "turtle_mode.h"

#include <math.h>

#include "core/profile.h"
#include "driver/motor.h"
#include "driver/time.h"
#include "flight/control.h"

// TODO: enable for brushed too
#ifdef BRUSHLESS_TARGET

#define TURTLE_TIMEOUT 1000 // 1 second timeout for auto turtle

typedef enum {
  TURTLE_STAGE_IDLE,
  TURTLE_STAGE_START,
  TURTLE_STAGE_ROTATING,
  TURTLE_STAGE_EXIT,
} turtle_mode_stage_t;

static turtle_mode_stage_t turtle_state = TURTLE_STAGE_IDLE;
static uint32_t turtle_time = 0;
static uint8_t turtle_axis = 0;
static uint8_t turtle_dir = 0;

static bool turtle_should_exit() {
  if (!rx_aux_on(AUX_TURTLE) || rx_aux_on(AUX_MOTOR_TEST)) {
    // turtle deactivated
    return true;
  }
  if (state.GEstG.yaw > 0.5f) {
    // quad was flipped
    return true;
  }
  if (!flags.arm_state) {
    // quad was disarmed
    return true;
  }
  return false;
}

static bool turtle_can_activate() {
  if (!rx_aux_on(AUX_TURTLE) || rx_aux_on(AUX_MOTOR_TEST)) {
    // turtle deactivated
    return false;
  }
  if (state.GEstG.yaw > 0) {
    // quad not upside down
    return false;
  }
  if (flags.in_air) {
    // quad in-air
    return false;
  }
  return true;
}

void turtle_mode_update() {
turtle_do_more:
  switch (turtle_state) {
  case TURTLE_STAGE_IDLE: {
    static uint8_t last_arm_state = 0;
    if (flags.arm_state == last_arm_state) {
      break;
    }
    last_arm_state = flags.arm_state;

    // quad was just armed and upside down, begin the turtle sequence
    if (!flags.arm_state) {
      break;
    }

    if (turtle_can_activate()) {
      motor_set_direction(MOTOR_REVERSE);
      turtle_state = TURTLE_STAGE_START;
      goto turtle_do_more;
    }

    motor_set_direction(MOTOR_FORWARD);
    break;
  }
  case TURTLE_STAGE_START:
    flags.controls_override = 1;
    flags.motortest_override = 1;
    state.rx_override.axis[0] = 0;
    state.rx_override.axis[1] = 0;
    state.rx_override.yaw = 0;
    state.rx_override.axis[3] = 0;

    if (turtle_should_exit()) {
      turtle_state = TURTLE_STAGE_EXIT;
      break;
    }

    if (!motor_direction_change_done()) {
      // wait for the motor to sucessfully change
      break;
    }

    if (fabsf(state.rx.axis[0]) > 0.5f || fabsf(state.rx.axis[1]) > 0.5f) {
      if (fabsf(state.rx.axis[0]) < fabsf(state.rx.axis[1])) {
        turtle_axis = 1;

        if (state.rx.axis[1] > 0) {
          turtle_dir = 1;
        } else {
          turtle_dir = 0;
        }
      } else {
        turtle_axis = 0;

        if (state.rx.axis[0] > 0) {
          turtle_dir = 1;
        } else {
          turtle_dir = 0;
        }
      }

      turtle_state = TURTLE_STAGE_ROTATING;
      turtle_time = time_millis();
    }
    break;

  case TURTLE_STAGE_ROTATING:
    if (turtle_should_exit()) {
      turtle_state = TURTLE_STAGE_EXIT;
      break;
    }

    state.rx_override.throttle = profile.motor.turtle_throttle_percent / 100.0f;
    if (turtle_dir) {
      state.rx_override.axis[turtle_axis] = 1.0f;
    } else {
      state.rx_override.axis[turtle_axis] = -1.0f;
    }

    if (time_millis() - turtle_time > TURTLE_TIMEOUT) {
      turtle_state = TURTLE_STAGE_START;
    }
    break;

  case TURTLE_STAGE_EXIT:
    motor_set_direction(MOTOR_FORWARD);
    flags.controls_override = 0;
    flags.motortest_override = 0;
    flags.arm_safety = 1;
    turtle_state = TURTLE_STAGE_IDLE;
    break;
  }

  flags.turtle = turtle_state != TURTLE_STAGE_IDLE;
}
#else
void turtle_mode_update() {}
#endif