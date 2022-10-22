#pragma once

typedef enum {
  LOOPTIME_2K = 500,
  LOOPTIME_4K = 250,
  LOOPTIME_8K = 125,
} looptime_autodetect_t;

#ifdef STM32F411
#define SYS_CLOCK_FREQ_HZ 120000000
#define PWM_CLOCK_FREQ_HZ 120000000
#define SPI_CLOCK_FREQ_HZ (SYS_CLOCK_FREQ_HZ / 2)

#define LOOPTIME LOOPTIME_4K
#endif

#ifdef STM32F405
#define SYS_CLOCK_FREQ_HZ 168000000
#define PWM_CLOCK_FREQ_HZ 84000000
#define SPI_CLOCK_FREQ_HZ (SYS_CLOCK_FREQ_HZ / 4)

#define LOOPTIME LOOPTIME_8K
#endif

#ifdef STM32F7
#define SYS_CLOCK_FREQ_HZ 216000000
#define PWM_CLOCK_FREQ_HZ 216000000
#define SPI_CLOCK_FREQ_HZ (SYS_CLOCK_FREQ_HZ / 4)

#define LOOPTIME LOOPTIME_8K

#define WITHIN_DTCM_RAM(p) (((uint32_t)p & 0xffff0000) == 0x20000000)
#define WITHIN_DMA_RAM(p) (false)
#endif

#ifdef STM32H7
#define SYS_CLOCK_FREQ_HZ 480000000
#define PWM_CLOCK_FREQ_HZ (SYS_CLOCK_FREQ_HZ / 2)
#define SPI_CLOCK_FREQ_HZ (SYS_CLOCK_FREQ_HZ / 4)

#define LOOPTIME LOOPTIME_8K

#define WITHIN_DTCM_RAM(p) (((uint32_t)p & 0xfffe0000) == 0x20000000)
#define WITHIN_DMA_RAM(p) (((uint32_t)p & 0xfffe0000) == 0x30000000)
#endif

#ifdef USE_FAST_RAM
#define FAST_RAM __attribute__((section(".fast_ram"), aligned(4)))
#else
#define FAST_RAM
#endif

#ifdef USE_DMA_RAM
#define DMA_RAM __attribute__((section(".dma_ram"), aligned(32)))
#else
#define DMA_RAM
#endif

#if defined(BUZZER_ENABLE) && !defined(BUZZER_PIN)
#undef BUZZER_ENABLE
#endif

#ifdef BRUSHLESS_TARGET
// dshot pin initialization & usb interface to esc
#define USE_DSHOT_DMA_DRIVER
#define USE_SERIAL_4WAY_BLHELI_INTERFACE
#endif

#ifdef BRUSHED_TARGET
// pwm pin initialization
#define USE_PWM_DRIVER
#endif

#if defined(USE_M25P16) || defined(USE_SDCARD)
#define ENABLE_BLACKBOX
#endif

#define SERIAL_RX

#if defined(USE_CC2500)
#define RX_FRSKY
#endif

#if defined(USE_SX127X) || defined(USE_SX128X)
#define RX_EXPRESS_LRS
#endif

#define PIN_IDENT(port, num) PIN_##port##num
#define GPIO_PIN(port, num) PIN_IDENT(port, num),

typedef enum {
  PIN_NONE,
#include "gpio_pins.in"
  PINS_MAX,
} gpio_pins_t;

#undef GPIO_PIN

typedef enum {
  MOTOR_PIN0,
  MOTOR_PIN1,
  MOTOR_PIN2,
  MOTOR_PIN3,
  MOTOR_MAX
} motor_index_t;

#define SERIAL_IDENT(channel) SERIAL_PORT##channel
#define SERIAL_PORT(channel) SERIAL_IDENT(channel),

typedef enum {
  SERIAL_PORT_INVALID,
#include "serial_ports.in"
  SERIAL_PORT_MAX,
} serial_port_index_t;

#undef SERIAL_PORT

#define SPI_IDENT(spi_prt, dma_prt, chan, rx, tx) SPI_PORT##spi_prt
#define SPI_PORT(spi_prt, dma_prt, chan, rx, tx) SPI_IDENT(spi_prt, dma_prt, chan, rx, tx),

typedef enum {
  SPI_PORT_INVALID,
  SPI_PORT1,
  SPI_PORT2,
  SPI_PORT3,
#if defined(STM32F7) || defined(STM32H7)
  SPI_PORT4,
#endif
  SPI_PORT_MAX,
} spi_port_index_t;

#undef SPI_PORT