#include "config.h"

// PORTS
#define SPI_PORTS   \
  SPI1_PA5PA6PA7    \
  SPI2_PB13PB14PB15 \
  SPI3_PC10PC11PC12

#define USART_PORTS \
  USART1_PA10PA9    \
  USART2_PA3PA2     \
  USART3_PB11PB10   \
  USART4_PA1PA0     \
  USART6_PC7PC6

// LEDS
#define LED_NUMBER 1
#define LED1PIN PIN_D2
#define LED1_INVERT

#define BUZZER_PIN PIN_C8

// GYRO
#define GYRO_SPI_PORT SPI_PORT1
#define GYRO_NSS PIN_B2
#define GYRO_INT PIN_A4
#define GYRO_ORIENTATION GYRO_ROTATE_180

// OSD
#define USE_MAX7456
#define MAX7456_SPI_PORT SPI_PORT3
#define MAX7456_NSS PIN_A15

#define USE_M25P16
#define M25P16_SPI_PORT SPI_PORT2
#define M25P16_NSS_PIN PIN_B12

// VOLTAGE DIVIDER
#define VBAT_PIN PIN_C0
#define VBAT_DIVIDER_R1 10000
#define VBAT_DIVIDER_R2 1000

#define IBAT_PIN PIN_C1
#define IBAT_SCALE 275

// MOTOR PINS
// S3_OUT
#define MOTOR_PIN0 MOTOR_PIN_PB4
// S4_OUT
#define MOTOR_PIN1 MOTOR_PIN_PB5
// S1_OUT
#define MOTOR_PIN2 MOTOR_PIN_PB0
// S2_OUT
#define MOTOR_PIN3 MOTOR_PIN_PB1