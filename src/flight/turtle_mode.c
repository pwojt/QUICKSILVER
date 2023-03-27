#include "turtle_mode.h"

#include <math.h>

#include "core/profile.h"
#include "driver/motor.h"
#include "driver/time.h"
#include "flight/control.h"

// TODO: enable for brushed too
#ifdef BRUSHLESS_TARGET

#define TURTLE_TIMEOUT 1000 // 1 second timeout for auto turtle
#define TURTLE_DEBOUNCE_TIME 100 // TODO: Figure out timing for this
#define TURTLE_EXIT_TIMEOUT 500 // TODO: Figure out correct value for this 150ms was too short, is 500 too long?

typedef enum {
  NORMAL,
  TURTLE_STAGE_IDLE_DEBOUNCE,
  TURTLE_STAGE_IDLE,
  TURTLE_STAGE_READY,
  TURTLE_STAGE_ROTATING,
  TURTLE_STAGE_EXIT_PENDING,
  TURTLE_STAGE_EXIT,
} turtle_mode_stage_t;

static turtle_mode_stage_t turtle_state = TURTLE_STAGE_IDLE;
static uint32_t turtle_time = 0;
static uint8_t turtle_axis = 0;
static uint8_t turtle_dir = 0;

static bool turtle_flipped_over() {
  return state.GEstG.yaw > 0.5f;
}

static bool turtle_right_side_up() {
  return state.GEstG.yaw > 0;
}

static bool turtle_mode_switch_deactived() {
  return !rx_aux_on(AUX_TURTLE) || rx_aux_on(AUX_MOTOR_TEST);
}

static bool turtle_should_exit() {
  return turtle_mode_switch_deactived() ||
         turtle_flipped_over() ||
         !flags.arm_state;
}

static bool turtle_can_activate() {
  return !turtle_mode_switch_deactived() &&
         !turtle_right_side_up() &&
         !flags.in_air &&
         !flags.arm_state;
}

static turtle_mode_stage_t calculate_next_state() {
  turtle_mode_stage_t calculated_stage = turtle_state;

  switch (turtle_state) {
  case NORMAL:
    if (turtle_can_activate()) {
      // If we can activate, move to next stae.
      calculated_stage = TURTLE_STAGE_IDLE_DEBOUNCE;
    }
  case TURTLE_STAGE_IDLE_DEBOUNCE:
    // TODO: Is this state required? I've seen strange behavior where quad is half between turtle and normal stage 
    if (time_millis() - turtle_time > 100) {
      if (turtle_can_activate()) {
        calculated_stage = TURTLE_STAGE_IDLE;
      } else {
        calculated_stage = NORMAL;
      }
    }
    break;
  case TURTLE_STAGE_IDLE:
    if (motor_direction_change_done()) {
      // Motor direction changed to reverse now
      if (flags.arm_state) {
        // Move to next stage when quad is armed
        calculated_stage = TURTLE_STAGE_READY;
      } else if (turtle_right_side_up()) {
        // We were manually flipped over before arming, we can now exit
        // and return motors to normal direction.
        calculated_stage = TURTLE_STAGE_EXIT_PENDING;
      }
    }
    break;
  case TURTLE_STAGE_READY:
    if (fabsf(state.rx.axis[0]) > 0.5f || fabsf(state.rx.axis[1]) > 0.5f) {
      // User input to rotate was given, start flipping
      calculated_stage = TURTLE_STAGE_ROTATING;
    }
    break;
  case TURTLE_STAGE_ROTATING:
    if (time_millis() - turtle_time > TURTLE_TIMEOUT) {
      // We timed out, so move back to ready state.
      calculated_stage = TURTLE_STAGE_READY;
    }
    break;
  case TURTLE_STAGE_EXIT_PENDING:
    // Make sure we can transition to exit
    if (motor_direction_change_done() && // If motor direction is pending, don't do anything yet
        !flags.arm_switch && // Wait for disarm before trying to exit
        time_millis() - turtle_time > TURTLE_EXIT_TIMEOUT) { // Wait at least exit time out to ensure we are ready.
      calculated_stage = TURTLE_STAGE_EXIT;
    }
    break;
  case TURTLE_STAGE_EXIT:
    if (motor_direction_change_done()) {
      // Motors are now forward, go back to normal state.
      calculated_stage = NORMAL;
    }
    break;
  }

  if (calculated_stage > TURTLE_STAGE_IDLE &&
      calculated_stage < TURTLE_STAGE_EXIT_PENDING &&
      turtle_should_exit()) {
    // Prepare to exit if we are in active turtle mode.
    if (turtle_flipped_over() || turtle_mode_switch_deactived()) {
      calculated_stage = TURTLE_STAGE_EXIT_PENDING;
    } else {
      // We are still upside down and turtle mode switch is active, stay in turtle ready mode.
      calculated_stage = TURTLE_STAGE_READY;
    }
  }
  return calculated_stage;
}

void turtle_mode_update() {
  // Get next stage
  turtle_mode_stage_t next_state = calculate_next_state();
  if (next_state == turtle_state) {
    // No state change, no need to do anything
    return;
  }

  switch (next_state) {
  case NORMAL:
    // We are back to normal opperation, turn off turtle flag.
    flags.turtle = 0;
    break;
  case TURTLE_STAGE_IDLE_DEBOUNCE:
    // Capture time to make sure we are really upside down before proceeding.
    turtle_time = time_millis();
    break;
  case TURTLE_STAGE_IDLE:
    // We are upside down and preparing to go over
    motor_set_direction(MOTOR_REVERSE);
    flags.turtle = 1;
    break;
  case TURTLE_STAGE_READY:
    // Prepare control for Turtle
    flags.controls_override = 1;
    flags.motortest_override = 1;
    state.rx_override.axis[0] = 0;
    state.rx_override.axis[1] = 0;
    state.rx_override.yaw = 0;
    state.rx_override.axis[3] = 0;
    state.rx_override.throttle = 0;
    break;
  case TURTLE_STAGE_ROTATING:
    // We are ready to go over, update rotation parameters
    turtle_axis = fabsf(state.rx.axis[0]) < fabsf(state.rx.axis[1]) ? 1 : 0;
    turtle_dir = state.rx.axis[turtle_axis] > 0 ? 1 : 0;

    turtle_time = time_millis();
    // We are now rotating
    state.rx_override.throttle = profile.motor.turtle_throttle_percent / 100.0f;
    state.rx_override.axis[turtle_axis] = turtle_dir ? 1.0f : -1.0f;
    break;
  case TURTLE_STAGE_EXIT_PENDING:
    // Update turtle flags to prepare for exit
    flags.controls_override = 0;
    flags.motortest_override = 0;
    flags.arm_safety = 1;
    turtle_time = time_millis();
    break;
  case TURTLE_STAGE_EXIT:
    // Return motors to normal operation.
    motor_set_direction(MOTOR_FORWARD);
    break;
  }

  turtle_state = next_state;
}

#else
void turtle_mode_update() {}
#endif