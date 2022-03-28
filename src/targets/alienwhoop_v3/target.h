#include "config.h"
#include "config_helper.h"

#define Alienwhoop_V3

// PORTS
#define SPI_PORTS   \
  SPI1_PA5PA6PA7    \
  SPI2_PB13PB14PB15 \
  SPI3_PB3PB4PB5

#define USART_PORTS \
  USART1_PA10PA9    \
  USART2_PA3PA2     \
  USART3_PC11PC10

// LEDS
#define LED_NUMBER 2
#define LED1PIN PIN_C12
#define LED2PIN PIN_D2
#define LED2_INVERT
#define BUZZER_PIN PIN_A14
#define BUZZER_INVERT
#define FPV_PIN PIN_A13

// GYRO
#define GYRO_SPI_PORT SPI_PORT1
#define GYRO_NSS PIN_A4
#define GYRO_INT PIN_C14
#define GYRO_ORIENTATION GYRO_ROTATE_90_CCW

// RADIO
#ifdef SERIAL_RX
#define RX_USART USART_PORT1
#endif

// OSD
#define ENABLE_OSD
#define MAX7456_SPI_PORT SPI_PORT2
#define MAX7456_NSS PIN_B12

// VOLTAGE DIVIDER
#define VBAT_PIN PIN_C2
#define VBAT_DIVIDER_R1 2000
#define VBAT_DIVIDER_R2 1000

// MOTOR PINS
#define MOTOR_PIN0 MOTOR_PIN_PC7
#define MOTOR_PIN1 MOTOR_PIN_PC6
#define MOTOR_PIN2 MOTOR_PIN_PC9
#define MOTOR_PIN3 MOTOR_PIN_PC8
