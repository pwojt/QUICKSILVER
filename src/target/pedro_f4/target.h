#include "config.h"

// PORTS
#define SPI_PORTS   \
  SPI1_PA5PA6PA7    \
  SPI2_PB13PB14PB15 \
  SPI3_PB3PB4PB5

#define USART_PORTS \
  USART1_PA10PA9    \
  USART3_PB11PB10   \
  USART4_PA1PA0

// #define DEBUG_PIN0 PIN_A9
// #define DEBUG_PIN1 PIN_A10

#define HDZERO_USART USART_PORT1

// #define USB_DETECT_PIN LL_GPIO_PIN_5
// #define USB_DETECT_PORT GPIOC

// LEDS
#define LED_NUMBER 1
#define LED1_INVERT
#define LED1PIN PIN_C12
#define LED2PIN PIN_D2

// #define FPV_PIN LL_GPIO_PIN_13
// #define FPV_PORT GPIOA

// GYRO
#define GYRO_SPI_PORT SPI_PORT1
#define GYRO_NSS PIN_A4
#define GYRO_INT PIN_C4
#define GYRO_ORIENTATION GYRO_ROTATE_90_CCW

// RADIO
// #define USART_INVERTER_PIN LL_GPIO_PIN_0 //UART 1
// #define USART_INVERTER_PORT GPIOC

/*
// OSD
#define USE_MAX7456
#define MAX7456_SPI_PORT SPI_PORT3
#define MAX7456_NSS PIN_A15

// SDCARD
#define USE_SDCARD
#define SDCARD_SPI_PORT SPI_PORT3
#define SDCARD_NSS_PIN PIN_A15

#define USE_M25P16
#define M25P16_SPI_PORT SPI_PORT3
#define M25P16_NSS_PIN PIN_A15
*/

#define USE_SX128X
#define USE_SX128X_BUSY_EXTI
#define SX12XX_SPI_PORT SPI_PORT3
#define SX12XX_NSS_PIN PIN_A15
#define SX12XX_DIO0_PIN PIN_B12
#define SX12XX_BUSY_PIN PIN_B13
#define SX12XX_RESET_PIN PIN_B15

// VOLTAGE DIVIDER
// #define VBAT_PIN LL_GPIO_PIN_2
// #define BATTERYPORT GPIOC
//
// #ifndef VOLTAGE_DIVIDER_R1
// #define VBAT_DIVIDER_R1 10000
// #endif
// #ifndef VOLTAGE_DIVIDER_R2
// #define VBAT_DIVIDER_R2 1000
// #endif
// #ifndef ADC_REF_VOLTAGE
// #define ADC_REF_VOLTAGE 3.3
// #endif

// MOTOR PINS
#define MOTOR_PIN0 MOTOR_PIN_PC9
#define MOTOR_PIN1 MOTOR_PIN_PC8
#define MOTOR_PIN2 MOTOR_PIN_PC7
#define MOTOR_PIN3 MOTOR_PIN_PC6
