#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "hardware.h"

#define MOTOR_OFF -1.0f

typedef enum {
  MOTOR_FORWARD,
  MOTOR_REVERSE
} motor_direction_t;

#define MOTOR_BL 0
#define MOTOR_FL 1
#define MOTOR_BR 2
#define MOTOR_FR 3

typedef struct {
  gpio_pins_t pin;
} motor_pin_def_t;

#define MAKE_MOTOR_PIN_DEF(port, _pin, pin_af, timer, timer_channel) \
  {                                                                  \
    .pin = PIN_IDENT(port, _pin),                                    \
  }

extern const volatile motor_pin_def_t motor_pin_defs[MOTOR_PIN_MAX];

void motor_init();
void motor_wait_for_ready();
void motor_beep();
void motor_write(float *values);
void motor_set_direction(motor_direction_t dir);
bool motor_direction_change_done();

// generic functions
void motor_set(uint8_t number, float pwm);
void motor_set_all(float pwm);
void motor_update();