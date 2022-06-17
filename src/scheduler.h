#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <cbor.h>

typedef enum {
  TASK_PRIORITY_REALTIME,
  TASK_PRIORITY_HIGH,
  TASK_PRIORITY_MEDIUM,
  TASK_PRIORITY_LOW,
} task_priority_t;

typedef enum {
  TASK_MASK_ON_GROUND = (0x1 << 0),
  TASK_MASK_IN_AIR = (0x1 << 1),

  TASK_MASK_ALWAYS = 0xFF,
} task_mask_t;

typedef void (*task_function_t)();
typedef bool (*task_poll_function_t)();

// This defines the stack that is saved  by the hardware
typedef struct __attribute__((__packed__)) {
  uint32_t r0;
  uint32_t r1;
  uint32_t r2;
  uint32_t r3;
  uint32_t r12;
  uint32_t lr;
  uint32_t pc;
  uint32_t psr;
} task_hw_stack_t;

// This defines the stack that must be saved by the software
typedef struct __attribute__((__packed__)) {
  uint32_t r4;
  uint32_t r5;
  uint32_t r6;
  uint32_t r7;
  uint32_t r8;
  uint32_t r9;
  uint32_t r10;
  uint32_t r11;
  uint32_t lr;
} task_sw_stack_t;

typedef struct {
  const char *name;
  const char *subname;

  uint8_t mask;

  task_priority_t priority;

  task_function_t func;
  void *stack;
  void *sp;
  bool completed;

  uint32_t last_run_time;
  uint32_t period;

  uint32_t execution_time;
  uint32_t exection_start_time;

  uint32_t runtime_current;
  uint32_t runtime_min;
  uint32_t runtime_avg;
  uint32_t runtime_worst;
  uint32_t runtime_max;

  uint32_t runtime_avg_sum;
} task_t;

#define CREATE_TASK(p_name, p_subname, p_stack, p_mask, p_priority, p_period, p_func) \
  {                                                                                   \
    .name = p_name,                                                                   \
    .subname = p_subname,                                                             \
    .mask = p_mask,                                                                   \
    .priority = p_priority,                                                           \
    .func = p_func,                                                                   \
    .stack = p_stack,                                                                 \
    .sp = NULL,                                                                       \
    .completed = true,                                                                \
    .last_run_time = 0,                                                               \
    .period = p_period,                                                               \
    .execution_time = 0,                                                              \
    .exection_start_time = 0,                                                         \
    .runtime_current = 0,                                                             \
    .runtime_min = UINT32_MAX,                                                        \
    .runtime_avg = 0,                                                                 \
    .runtime_worst = 0,                                                               \
    .runtime_max = 0,                                                                 \
    .runtime_avg_sum = 0,                                                             \
  }

void scheduler_init();
void scheduler_update();

cbor_result_t cbor_encode_task_stats(cbor_value_t *enc);

void task_yield();