#include "tasks.h"

#include <stddef.h>

#include "drv_serial.h"
#include "drv_usb.h"

#include "flight/control.h"
#include "flight/gestures.h"
#include "flight/imu.h"
#include "flight/sixaxis.h"
#include "io/blackbox.h"
#include "io/buzzer.h"
#include "io/led.h"
#include "io/usb_configurator.h"
#include "io/vbat.h"
#include "io/vtx.h"
#include "osd_render.h"
#include "profile.h"
#include "project.h"
#include "rx.h"

void task_main() {
  sixaxis_read();

  // all flight calculations and motors
  control();

  // attitude calculations for level mode
  imu_calc();

  // battery low logic
  vbat_calc();

  // check gestures
  if (flags.on_ground && !flags.gestures_disabled) {
    gestures();
  }

  // handle led commands
  led_update();

#if (RGB_LED_NUMBER > 0)
  // RGB led control
  rgb_led_lvc();
#ifdef RGB_LED_DMA
  rgb_dma_start();
#endif
#endif

  buzzer_update();
}

FAST_RAM uint8_t task_stacks[TASK_MAX][TASK_STACK_SIZE];

FAST_RAM task_t tasks[TASK_MAX] = {
    [TASK_MAIN] = CREATE_TASK("MAIN", NULL, task_stacks[TASK_MAIN], TASK_MASK_ALWAYS, TASK_PRIORITY_REALTIME, 0, task_main),
    [TASK_RX] = CREATE_TASK("RX", NULL, task_stacks[TASK_RX], TASK_MASK_ALWAYS, TASK_PRIORITY_HIGH, 0, rx_update),

    [TASK_USB] = CREATE_TASK("USB", NULL, task_stacks[TASK_USB], TASK_MASK_ON_GROUND, TASK_PRIORITY_LOW, 0, usb_configurator),
#ifdef ENABLE_BLACKBOX
    [TASK_BLACKBOX] = CREATE_TASK("BLACKBOX", NULL, task_stacks[TASK_BLACKBOX], TASK_MASK_ALWAYS, TASK_PRIORITY_HIGH, (LOOPTIME * BLACKBOX_RATE), blackbox_update),
#endif
#ifdef ENABLE_OSD
    [TASK_OSD] = CREATE_TASK("OSD", NULL, task_stacks[TASK_OSD], TASK_MASK_ALWAYS, TASK_PRIORITY_MEDIUM, 0, osd_display),
#endif
    [TASK_VTX] = CREATE_TASK("VTX", NULL, task_stacks[TASK_VTX], TASK_MASK_ON_GROUND, TASK_PRIORITY_LOW, 0, vtx_update),
};