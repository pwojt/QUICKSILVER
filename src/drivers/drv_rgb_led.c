#include "drv_rgb_led.h"

#include <stdbool.h>
#include <string.h>

#include "drv_dma.h"
#include "drv_gpio.h"
#include "drv_interrupt.h"
#include "drv_spi.h"
#include "drv_time.h"
#include "flight/control.h"
#include "profile.h"
#include "project.h"
#include "util/util.h"

#if (RGB_LED_NUMBER > 0)
void rgb_send(int data);

#ifdef RGB_LED_DMA

#define RGB_BIT_TIME ((PWM_CLOCK_FREQ_HZ / 800000) - 1)
#define RGB_T0H_TIME (RGB_BIT_TIME * 0.30 + 0.05)
#define RGB_T1H_TIME (RGB_BIT_TIME * 0.60 + 0.05)
#define RGB_BITS_LED 24
#define RGB_BUFFER_SIZE (RGB_BITS_LED * RGB_LED_NUMBER)
extern int rgb_led_value[];

volatile int rgb_dma_phase = 0; // 3:rgb data ready
                                // 2:rgb dma buffer ready
                                // 1:rgb dma busy
                                // 0:idle

// Must be a better way of doing this instead of having two separate arrays!
volatile uint32_t rgb_timer_buffer32[RGB_BUFFER_SIZE] = {0}; // DMA buffer: 32-bit Array of PWM duty cycle timings 
volatile uint16_t rgb_timer_buffer16[RGB_BUFFER_SIZE] = {0}; // DMA buffer: 16-bit Array of PWM duty cycle timings

const dma_stream_def_t *rgb_dma = &dma_stream_defs[RGB_LED_DMA];


void rgb_init_io()
{
  LL_GPIO_InitTypeDef GPIO_InitStructure;

  // Config pin for digital output
  GPIO_InitStructure.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStructure.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;

  GPIO_InitStructure.Pin = RGB_PIN;

  // Bind IO pin to the alternate function
  gpio_pin_init_af(&GPIO_InitStructure, RGB_PIN, RGB_TIM_AF);
  gpio_pin_reset(RGB_PIN);
}


void rgb_init_tim()
{
  LL_TIM_InitTypeDef TIM_TimeBaseStructure;
  LL_TIM_OC_InitTypeDef TIM_OCInitStructure;

  // Clock enable (TIM)
  LL_APB1_GRP1_EnableClock(RGB_TIM_CLOCK);  
  // Clock Enable (DMA)
  dma_enable_rcc(RGB_LED_DMA);
  
  // Timer init
  LL_TIM_StructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.Autoreload = RGB_BIT_TIME;
  TIM_TimeBaseStructure.Prescaler = 0;
  TIM_TimeBaseStructure.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_TimeBaseStructure.CounterMode = LL_TIM_COUNTERMODE_UP;
  LL_TIM_Init(RGB_TIMER, &TIM_TimeBaseStructure);
  
  TIM_OCInitStructure.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OCInitStructure.OCState = LL_TIM_OCSTATE_ENABLE;
  TIM_OCInitStructure.CompareValue = 0;
  TIM_OCInitStructure.OCPolarity = LL_TIM_OCPOLARITY_HIGH;

  LL_TIM_OC_Init(RGB_TIMER, RGB_TIM_CHAN, &TIM_OCInitStructure);
  LL_TIM_OC_EnablePreload(RGB_TIMER, RGB_TIM_CHAN);
  LL_TIM_EnableARRPreload(RGB_TIMER);
}


void rgb_init_dma()
{
  LL_DMA_InitTypeDef DMA_InitStructure;
  LL_DMA_StructInit(&DMA_InitStructure);

  dma_clear_flag_tc(rgb_dma->port, rgb_dma->stream_index);
  LL_DMA_DeInit(rgb_dma->port, rgb_dma->stream_index);
#ifdef STM32H7
  DMA_InitStructure.PeriphRequest = rgb_dma->request;
#else
  DMA_InitStructure.Channel = rgb_dma->channel;
#endif
  DMA_InitStructure.PeriphOrM2MSrcAddress = (uint32_t) &RGB_TIMER->RGB_TIM_CCR;
  DMA_InitStructure.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;;
  DMA_InitStructure.NbData = RGB_BUFFER_SIZE;
  DMA_InitStructure.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
  DMA_InitStructure.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
  if ((RGB_TIMER == TIM2) || (RGB_TIMER == TIM5)){
    DMA_InitStructure.MemoryOrM2MDstAddress = (uint32_t) rgb_timer_buffer32;
    DMA_InitStructure.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_WORD; // 32bit
    DMA_InitStructure.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_WORD;
  }
  else{
    DMA_InitStructure.MemoryOrM2MDstAddress = (uint32_t) rgb_timer_buffer16;
    DMA_InitStructure.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_HALFWORD; // 16bit
    DMA_InitStructure.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_HALFWORD;
  }
  DMA_InitStructure.Mode = LL_DMA_MODE_NORMAL;
  DMA_InitStructure.Priority = LL_DMA_PRIORITY_HIGH;
  DMA_InitStructure.FIFOMode = LL_DMA_FIFOMODE_DISABLE;
  DMA_InitStructure.FIFOThreshold = LL_DMA_FIFOTHRESHOLD_1_2;
  DMA_InitStructure.MemBurst = LL_DMA_MBURST_SINGLE;
  DMA_InitStructure.PeriphBurst = LL_DMA_PBURST_SINGLE;
  LL_DMA_Init(rgb_dma->port, rgb_dma->stream_index, &DMA_InitStructure);
}


void rgb_init_nvic()
{
  interrupt_enable(rgb_dma->irq, DMA_PRIORITY);
}


void rgb_dma_buffer_making() {
  // generate rgb dma packet of pulse width timings for all LEDs
  for(uint32_t n=0;n<RGB_LED_NUMBER;n++) {
    // rgb_led_value contains a (32bit) int that contains the RGB values in G R B format already
    // so i can just test each bit and assign the T1H or T0H depending on whether it is 1 or 0.
    for (size_t i = 0; i < RGB_BITS_LED; i++) {
      if ((RGB_TIMER == TIM2) || (RGB_TIMER == TIM5))
        rgb_timer_buffer32[(n * 24) + i] = (rgb_led_value[n] & (1 << ((RGB_BITS_LED - 1) - i))) ? RGB_T1H_TIME : RGB_T0H_TIME;
      else
        rgb_timer_buffer16[(n * 24) + i] = (rgb_led_value[n] & (1 << ((RGB_BITS_LED - 1) - i))) ? RGB_T1H_TIME : RGB_T0H_TIME;
    }
  }
}

void rgb_dma_trigger() {
  rgb_init_dma();
  // Enable Transfer-Complete Interrupt
  LL_DMA_EnableIT_TC(rgb_dma->port, rgb_dma->stream_index);
  // Enable DMA 
  LL_DMA_EnableStream(rgb_dma->port, rgb_dma->stream_index);
  // Enable timer
  LL_TIM_EnableDMAReq_CC1(RGB_TIMER);
  LL_TIM_EnableCounter(RGB_TIMER);
}

void rgb_init() {

  rgb_init_io();
  rgb_init_tim();
  rgb_init_nvic();
  rgb_init_dma();

  for (int i = 0; i < RGB_LED_NUMBER; i++) {
    rgb_led_value[i] = 0;
  }
  if (!rgb_dma_phase)
    rgb_dma_phase = 3;
}


void rgb_send(int data) {
  if (!rgb_dma_phase)
    rgb_dma_phase = 3;
}


void rgb_dma_start() {
  if (rgb_dma_phase <= 1)
    return;

  if (rgb_dma_phase == 3) {
    rgb_dma_buffer_making();
    rgb_dma_phase = 2;
    return;
  }

  rgb_dma_phase = 1;
  rgb_dma_trigger();
}


void rgb_dma_isr() {

    // reset TC flag
    dma_clear_flag_tc(rgb_dma->port, rgb_dma->stream_index);
    // Disable Timer
    LL_TIM_DisableCounter(RGB_TIMER);
    // Disable DMA
    LL_TIM_DisableDMAReq_CC1(RGB_TIMER);
    LL_DMA_DisableStream(rgb_dma->port, rgb_dma->stream_index);

    // Set phase to idle
    rgb_dma_phase=0;

}

#endif

#else
// rgb led not found
// some dummy headers just in case
void rgb_init() {
}

void rgb_send(int data) {
}
#endif
