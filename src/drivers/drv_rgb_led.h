#pragma once

void rgb_init();
void rgb_send(int data);

void rgb_dma_start();
void rgb_initIO();
void rgb_initTIM();
void rgb_initNVIC();
void rgb_initDMA();
/*
//--------------------------------------------------------------
// Timer for the data signal => TIM2
//--------------------------------------------------------------
#define  RGB_TIM_CLOCK     LL_APB1_GRP1_PERIPH_TIM2
#define  RGB_TIMER         TIM2
#define  RGB_TIM_CHAN      LL_TIM_CHANNEL_CH1
#define  RGB_TIM_CCR       CCR1
#define  RGB_TIM_AF        GPIO_AF1_TIM2
//#define  RGB_TIM_DMA_TRG1  TIM_DMA_CC1

//--------------------------------------------------------------
// LED Pin
//--------------------------------------------------------------
#define  RGB_PIN_CLOCK        LL_AHB1_GRP1_PERIPH_GPIOA

//--------------------------------------------------------------
// DMA => TIM2-CC1 => DMA1, Channel3, Stream5 (See drv_dma.c)
//--------------------------------------------------------------
#define  RGB_DMA           DMA_DEVICE_TIM2_CH1
//#define  RGB_DMA_CLOCK     LL_AHB1_GRP1_PERIPH_DMA1
//#define  RGB_DMA_PORT      DMA1
//#define  RGB_DMA_STREAM    LL_DMA_STREAM_5
//#define  RGB_DMA_CHANNEL   LL_DMA_CHANNEL_3

//--------------------------------------------------------------
// Transfer-Complete Interrupt
//--------------------------------------------------------------
//#define  RGB_DMA_CH1_IRQn      DMA1_Stream5_IRQn
*/