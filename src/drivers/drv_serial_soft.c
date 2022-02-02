#include "drv_serial_soft.h"

#include "drv_gpio.h"
#include "project.h"

#define CYCLES_TIMEOUT (10000 * (SYS_CLOCK_FREQ_HZ / 1000000))

static void delay_until_cycles(uint32_t cycles) {
  while (time_cycles() < cycles) {
    __NOP();
  }
}

static int soft_serial_is_1wire(const soft_serial_t *dev) {
  return dev->tx_pin == dev->rx_pin;
}

static void soft_serial_init_rx(const soft_serial_t *dev) {
  if (dev->rx_pin == PIN_NONE) {
    return;
  }

  LL_GPIO_InitTypeDef gpio_init;
  gpio_init.Mode = LL_GPIO_MODE_INPUT;
  gpio_init.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  gpio_init.Pull = LL_GPIO_PULL_UP;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  gpio_pin_init(&gpio_init, dev->rx_pin);

  gpio_pin_set(dev->rx_pin);
}

static void soft_serial_init_tx(const soft_serial_t *dev) {
  if (dev->tx_pin == PIN_NONE) {
    return;
  }

  LL_GPIO_InitTypeDef gpio_init;
  gpio_init.Mode = LL_GPIO_MODE_OUTPUT;
  gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpio_init.Pull = LL_GPIO_PULL_NO;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  gpio_pin_init(&gpio_init, dev->tx_pin);

  gpio_pin_set(dev->tx_pin);
}

uint8_t soft_serial_init(soft_serial_t *dev, gpio_pins_t tx_pin, gpio_pins_t rx_pin, uint32_t baudrate) {
  if (tx_pin == PIN_NONE && rx_pin == PIN_NONE) {
    return 0;
  }

  dev->tx_pin = tx_pin;
  dev->rx_pin = rx_pin;

  soft_serial_init_tx(dev);
  soft_serial_init_rx(dev);

  dev->baud = baudrate;
  dev->cycles_per_bit = SYS_CLOCK_FREQ_HZ / baudrate;
  dev->cycles_per_bit_half = dev->cycles_per_bit * .5;

  return 1;
}

void soft_serial_set_input(const soft_serial_t *dev) {
  if (soft_serial_is_1wire(dev))
    soft_serial_init_rx(dev);

  time_delay_us(20);
}

void soft_serial_set_output(const soft_serial_t *dev) {
  if (soft_serial_is_1wire(dev))
    soft_serial_init_tx(dev);

  time_delay_us(20);
}

uint8_t soft_serial_read_byte(const soft_serial_t *dev, uint8_t *byte) {

  uint32_t time_start = time_cycles();
  uint32_t time_next = time_start;
  while (!gpio_pin_read(dev->rx_pin)) {
    time_next = time_cycles(); // wait for start bit
    if (time_next - time_start > CYCLES_TIMEOUT)
      return 0;
  }

  // start bit falling edge
  while (gpio_pin_read(dev->rx_pin)) {
    time_next = time_cycles(); // wait for start bit
    if (time_next - time_start > CYCLES_TIMEOUT)
      return 0;
  }

  time_next += dev->cycles_per_bit_half; // move away from edge to center of bit

  uint8_t b = 0;
  for (int i = 0; i < 8; ++i) {
    time_next += dev->cycles_per_bit;
    delay_until_cycles(time_next);
    b >>= 1;
    if (gpio_pin_read(dev->rx_pin))
      b |= 0x80;
  }

  time_next += dev->cycles_per_bit;
  delay_until_cycles(time_next); // move away from edge

  // stop bit
  if (!(gpio_pin_read(dev->rx_pin))) {
    // error no stop bit
    *byte = 0;
    return 0;
  }

  *byte = b;
  return 1;
}

void soft_serial_write_byte(const soft_serial_t *dev, uint8_t byte) {
  gpio_pin_reset(dev->tx_pin);

  uint32_t next_time = time_cycles();
  for (int i = 0; i < 8; ++i) {
    next_time += dev->cycles_per_bit;
    delay_until_cycles(next_time);

    if (0x01 & byte)
      gpio_pin_set(dev->tx_pin);
    else
      gpio_pin_reset(dev->tx_pin);

    byte = byte >> 1;
  }
  next_time += dev->cycles_per_bit;
  delay_until_cycles(next_time);

  // stop bit
  gpio_pin_set(dev->tx_pin);
  next_time += dev->cycles_per_bit;
  delay_until_cycles(next_time);
}