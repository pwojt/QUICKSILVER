#include "config.h"
#include "config_helper.h"

#define CrazyBee_F4

// PORTS
#define SPI_PORTS   \
  SPI1_PA5PA6PA7    \
  SPI2_PB13PB14PB15 \
  SPI3_PB3PB4PB5

#define USART_PORTS \
  USART1_PA10PA9    \
  USART2_PA3PA2

// LEDS
#define LED_NUMBER 2
#define LED1PIN PIN_C13
#define LED1_INVERT
#define LED2PIN PIN_B9
#define LED2_INVERT
#define BUZZER_PIN PIN_C15

//--------------------------------------------------------------
// RGB LEDs
//--------------------------------------------------------------
#define RGB_LED_NUMBER 2
//--------------------------------------------------------------
// DMA => TIM2-CC1 => DMA1, Channel3, Stream5 (See drv_dma.c)
//--------------------------------------------------------------
#define RGB_LED_DMA       DMA_DEVICE_TIM2_CH1
//--------------------------------------------------------------
// pin / port for the RGB led
//--------------------------------------------------------------
#define RGB_PIN LL_GPIO_PIN_0
#define RGB_PORT GPIOA
#define RGB_PIN_CLOCK LL_AHB1_GRP1_PERIPH_GPIOA
//--------------------------------------------------------------
// Timer for the data signal => TIM2
//--------------------------------------------------------------
#define  RGB_TIM_CLOCK     LL_APB1_GRP1_PERIPH_TIM2
#define  RGB_TIMER         TIM2
#define  RGB_TIM_CHAN      LL_TIM_CHANNEL_CH1
#define  RGB_TIM_CCR       CCR1
#define  RGB_TIM_AF        GPIO_AF1_TIM2

//#define FPV_PIN LL_GPIO_PIN_13
//#define FPV_PORT GPIOA

// GYRO
#define GYRO_SPI_PORT SPI_PORT1
#define GYRO_NSS PIN_A4
#define GYRO_INT PIN_A1

// RADIO
#define USE_CC2500
#define CC2500_SPI_PORT SPI_PORT3
#define CC2500_NSS_PIN PIN_A15
#define CC2500_GDO0_PIN PIN_C14

#ifdef SERIAL_RX
#define RX_USART USART_PORT2
#endif

// OSD
#define USE_MAX7456
#define MAX7456_SPI_PORT SPI_PORT2
#define MAX7456_NSS PIN_B12

// VOLTAGE DIVIDER
#define VBAT_PIN PIN_B0
#define VBAT_DIVIDER_R1 10000
#define VBAT_DIVIDER_R2 1000

#define IBAT_PIN PIN_B1

// MOTOR PINS
#define MOTOR_PIN0 MOTOR_PIN_PB7
#define MOTOR_PIN1 MOTOR_PIN_PB8
#define MOTOR_PIN2 MOTOR_PIN_PB10
#define MOTOR_PIN3 MOTOR_PIN_PB6
