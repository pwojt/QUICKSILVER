#include "scheduler.h"

#include <stdbool.h>
#include <string.h>

#include "debug.h"
#include "drv_time.h"
#include "flight/control.h"
#include "io/usb_configurator.h"
#include "tasks.h"
#include "util/cbor_helper.h"

#define MSP_RETURN_FLOAT 0xFFFFFFE9
#define MSP_RETURN_NO_FLOAT 0xFFFFFFF9

#define PSP_RETURN_FLOAT 0xFFFFFFED
#define PSP_RETURN_NO_FLOAT 0xFFFFFFFD

#define TASK_AVERAGE_SAMPLES 32
#define TASK_RUNTIME_REDUCTION(val) ((val * 2) / 3)
#define TASK_RUNTIME_MARGIN 1.25
#define TASK_RUNTIME_BUFFER 10

#define STACK_CANARY 0xDEADBEEF

#define US_TO_CYCLES(us) ((us)*TICKS_PER_US)
#define CYCLES_TO_US(cycles) ((cycles) / TICKS_PER_US)

uint8_t looptime_warning;
uint8_t blown_loop_counter;

static uint32_t lastlooptime;

static FAST_RAM task_t *task_queue[TASK_MAX];
static volatile uint32_t task_index = TASK_IDLE;
static volatile uint8_t task_mask = 0;

static volatile uint32_t start_cycles = 0;
volatile bool scheduler_active = false;

#define current_task task_queue[task_index]

void task_return_function() {
  failloop(FAILLOOP_FAULT);
}

void task_wrapper_function(task_t *task) {
  while (true) {
    task->func();

    task->execution_time += time_cycles() - task->exection_start_time;
    task->runtime_current = task->execution_time;

    if (task->runtime_current < task->runtime_min) {
      task->runtime_min = task->runtime_current;
    }

    task->runtime_avg_sum -= task->runtime_avg;
    task->runtime_avg_sum += task->runtime_current;
    task->runtime_avg = task->runtime_avg_sum / TASK_AVERAGE_SAMPLES;

    if (task->runtime_current > task->runtime_max) {
      task->runtime_max = task->runtime_current;
    }

    if (task->runtime_worst < (task->runtime_avg * TASK_RUNTIME_MARGIN)) {
      task->runtime_worst = task->runtime_avg * TASK_RUNTIME_MARGIN;
    }

    task->completed = true;

    task_yield();
  }
}

static inline void task_reset(task_t *task) {
  void *stack_addr = task->stack + TASK_STACK_SIZE;

  const uint32_t stack_size = sizeof(task_hw_stack_t) + sizeof(task_sw_stack_t);
  uint32_t *ptr = (uint32_t *)(stack_addr - stack_size);
  for (uint32_t i = 0; i < stack_size / 4; i++) {
    ptr[i] = 0;
  }

  task_hw_stack_t *hw_stack = (task_hw_stack_t *)(stack_addr - sizeof(task_hw_stack_t));
  hw_stack->r0 = (uint32_t)task;
  hw_stack->pc = (uint32_t)task_wrapper_function;
  hw_stack->lr = (uint32_t)task_return_function;
  hw_stack->psr = 0x21000000;

  task_sw_stack_t *sw_stack = (task_sw_stack_t *)(stack_addr - sizeof(task_hw_stack_t) - sizeof(task_sw_stack_t));
  sw_stack->lr = PSP_RETURN_NO_FLOAT;

  *((uint32_t *)task->stack) = STACK_CANARY;

  task->sp = (uint32_t)stack_addr - stack_size;
}

static uint32_t task_queue_size = 0;

static bool task_queue_contains(task_t *task) {
  for (uint32_t i = 0; i < task_queue_size; i++) {
    if (task_queue[i] == task) {
      return true;
    }
  }
  return false;
}

static bool task_queue_push(task_t *task) {
  if (task_queue_size >= TASK_MAX || task_queue_contains(task)) {
    return false;
  }

  task_reset(task);

  for (uint32_t i = 0; i < TASK_MAX; i++) {
    if (task_queue[i] != NULL && task_queue[i]->priority <= task->priority) {
      continue;
    }

    memcpy(task_queue + i + 1, task_queue + i, (task_queue_size - i) * sizeof(task_t *));
    task_queue[i] = task;
    task_queue_size++;
    return true;
  }
  return false;
}

static inline bool should_run_task(const uint32_t start_cycles, uint8_t task_mask, task_t *task) {
  if ((task_mask & task->mask) == 0) {
    // task shall not run in this firmware state
    return false;
  }

  if ((int32_t)(task->last_run_time - start_cycles) > 0 && current_task->completed) {
    // task was already run this loop
    return false;
  }

  if (task->period != 0 && (time_cycles() - task->last_run_time) < US_TO_CYCLES(task->period)) {
    // task has a period, but its not up yet
    return false;
  }

  const int32_t time_left = US_TO_CYCLES(state.looptime_autodetect - TASK_RUNTIME_BUFFER) - (time_cycles() - start_cycles);
  if (task->priority != TASK_PRIORITY_REALTIME && (int32_t)(task->runtime_worst - task->execution_time) > time_left) {
    // we dont have any time left this loop and task is not realtime
    task->runtime_worst = TASK_RUNTIME_REDUCTION(task->runtime_worst);
    return false;
  }

  return true;
}

static bool select_task() {
  while ((time_cycles() - start_cycles) < US_TO_CYCLES(state.looptime_autodetect - TASK_RUNTIME_BUFFER)) {
    task_index = (task_index + 1) % TASK_MAX;

    if (should_run_task(start_cycles, task_mask, current_task)) {
      return true;
    }
  }
  return false;
}

__attribute__((noinline, used)) uint32_t run_sched(uint32_t sp) {
  if (sp == 0) {
    // intial context switch, task_idle is pre-selected
    current_task->exection_start_time = time_cycles();
    return current_task->sp;
  }

  current_task->execution_time += time_cycles() - current_task->exection_start_time;
  current_task->sp = sp;

  if (*((uint32_t *)current_task->stack) != STACK_CANARY) {
    __BKPT(0);
  }

  if (!select_task()) {
    task_index = TASK_IDLE;
  }

  if (current_task->completed) {
    current_task->last_run_time = time_cycles();
    current_task->execution_time = 0;
    current_task->completed = false;
  }

  current_task->exection_start_time = time_cycles();
  return current_task->sp;
}

static inline __attribute__((always_inline)) void scheduler_run_new_task() {
  asm volatile(
      ".thumb_func\n"
      ".syntax unified\n"

      "mov r0, #0\n"

      "tst lr, #4\n"
      "it eq\n"
      "beq schedule_task\n"

      "mrs r0, psp\n"
      "tst lr, #0x10\n"
      "it eq\n"
      "vstmdbeq r0!, {s16-s31}\n"
      "stmdb r0!, {r4-r11, lr}\n"
      "msr psp, r0\n"

      "schedule_task:\n"
      "cpsid  i\n"
      "bl run_sched\n" // returns sp in r0
      "cpsie  i\n"

      "ldmfd r0!, {r4-r11, lr}\n"
      "tst lr, #0x10\n"
      "it eq\n"
      "vldmiaeq r0!, {s16-s31}\n"

      "msr psp, r0\n"
      "bx lr\n"

      ".ltorg\n"
      :
      :
      :);
}

__attribute__((used)) void SVC_Handler() {
  SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
  __ISB();
}

__attribute__((isr, naked)) void PendSV_Handler() {
  scheduler_run_new_task();
}

void looptime_update() {
  // looptime_autodetect sequence
  static uint8_t loop_delay = 0;
  if (loop_delay < 200) {
    loop_delay++;
  }

  static float loop_avg = 0;
  static uint8_t loop_counter = 0;

  if (loop_delay >= 200 && loop_counter < 200) {
    loop_avg += state.looptime_us;
    loop_counter++;
  }

  if (loop_counter == 200) {
    loop_avg /= 200;

    if (loop_avg < 130.f) {
      state.looptime_autodetect = LOOPTIME_8K;
    } else if (loop_avg < 255.f) {
      state.looptime_autodetect = LOOPTIME_4K;
    } else {
      state.looptime_autodetect = LOOPTIME_2K;
    }

    loop_counter++;
  }

  if (loop_counter == 201) {
    if (state.cpu_load > state.looptime_autodetect + 5) {
      blown_loop_counter++;
    }

    if (blown_loop_counter > 100) {
      blown_loop_counter = 0;
      loop_counter = 0;
      loop_avg = 0;
      looptime_warning++;
    }
  }
}

void scheduler_start() {
  for (uint32_t i = 0; i < TASK_MAX; i++) {
    task_queue_push(&tasks[i]);
  }

  NVIC_SetPriority(SVCall_IRQn, 0);
  NVIC_SetPriority(PendSV_IRQn, 0xFF);

  lastlooptime = time_micros();
  scheduler_active = true;
  task_yield();
}

void idle_task() {
  while (true) {
    start_cycles = time_cycles();

    const uint32_t time = time_micros();
    state.looptime_us = ((uint32_t)(time - lastlooptime));
    lastlooptime = time;

    if (state.looptime_us <= 0) {
      state.looptime_us = 1;
    }

    // max loop 20ms
    if (state.looptime_us > 20000) {
      failloop(FAILLOOP_LOOPTIME);
    }

    looptime_update();

    state.looptime = state.looptime_us * 1e-6f;

    state.uptime += state.looptime;
    if (flags.arm_state) {
      state.armtime += state.looptime;
    }

    task_mask = 0;

    if (flags.on_ground && !flags.arm_state) {
      task_mask |= TASK_MASK_ON_GROUND;
    }
    if (flags.in_air || flags.arm_state) {
      task_mask |= TASK_MASK_IN_AIR;
    }

    scheduler_active = true;

    uint8_t index = 0;
    do {
      if (!task_queue[index]->completed) {
        task_yield();
      }
      index = (index + 1) % TASK_MAX;
    } while ((time_cycles() - start_cycles) < US_TO_CYCLES(state.looptime_autodetect - TASK_RUNTIME_BUFFER - 5));

    state.cpu_load = (time_micros() - lastlooptime);
    state.loop_counter++;
  }
}

void reset_looptime() {
  lastlooptime = time_micros();
}

#ifdef DEBUG

#define ENCODE_CYCLES(val)                                \
  {                                                       \
    const uint32_t us = CYCLES_TO_US(val);                \
    CBOR_CHECK_ERROR(res = cbor_encode_uint32(enc, &us)); \
  }

cbor_result_t cbor_encode_task_stats(cbor_value_t *enc) {
  CBOR_CHECK_ERROR(cbor_result_t res = cbor_encode_array_indefinite(enc));

  for (uint32_t i = 0; i < TASK_MAX; i++) {
    CBOR_CHECK_ERROR(res = cbor_encode_map_indefinite(enc));

    CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "name"));
    CBOR_CHECK_ERROR(res = cbor_encode_str(enc, tasks[i].name));

    CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "last"));
    ENCODE_CYCLES(tasks[i].last_run_time)

    CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "current"));
    ENCODE_CYCLES(tasks[i].runtime_current)

    CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "min"));
    ENCODE_CYCLES(tasks[i].runtime_min)

    CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "avg"));
    ENCODE_CYCLES(tasks[i].runtime_avg)

    CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "max"));
    ENCODE_CYCLES(tasks[i].runtime_max)

    CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "worst"));
    ENCODE_CYCLES(tasks[i].runtime_worst)

    CBOR_CHECK_ERROR(res = cbor_encode_end_indefinite(enc));
  }

  CBOR_CHECK_ERROR(res = cbor_encode_end_indefinite(enc));

  return res;
}

#endif