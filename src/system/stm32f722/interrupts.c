__attribute__((weak)) void Default_Handler();

__attribute__((used)) void Default_Handler_Proxy() {
  Default_Handler();
}

__attribute__((weak, alias("Default_Handler_Proxy"))) void NMI_Handler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void HardFault_Handler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void MemManage_Handler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void BusFault_Handler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void UsageFault_Handler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void SVC_Handler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void DebugMon_Handler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void PendSV_Handler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void SysTick_Handler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void WWDG_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void PVD_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void TAMP_STAMP_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void RTC_WKUP_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void FLASH_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void RCC_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void EXTI0_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void EXTI1_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void EXTI2_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void EXTI3_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void EXTI4_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void DMA1_Stream0_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void DMA1_Stream1_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void DMA1_Stream2_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void DMA1_Stream3_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void DMA1_Stream4_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void DMA1_Stream5_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void DMA1_Stream6_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void ADC_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void CAN1_TX_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void CAN1_RX0_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void CAN1_RX1_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void CAN1_SCE_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void EXTI9_5_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void TIM1_BRK_TIM9_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void TIM1_UP_TIM10_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void TIM1_TRG_COM_TIM11_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void TIM1_CC_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void TIM2_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void TIM3_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void TIM4_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void I2C1_EV_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void I2C1_ER_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void I2C2_EV_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void I2C2_ER_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void SPI1_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void SPI2_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void USART1_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void USART2_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void USART3_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void EXTI15_10_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void RTC_Alarm_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void OTG_FS_WKUP_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void TIM8_BRK_TIM12_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void TIM8_UP_TIM13_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void TIM8_TRG_COM_TIM14_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void TIM8_CC_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void DMA1_Stream7_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void FMC_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void SDMMC1_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void TIM5_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void SPI3_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void UART4_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void UART5_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void TIM6_DAC_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void TIM7_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void DMA2_Stream0_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void DMA2_Stream1_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void DMA2_Stream2_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void DMA2_Stream3_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void DMA2_Stream4_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void OTG_FS_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void DMA2_Stream5_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void DMA2_Stream6_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void DMA2_Stream7_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void USART6_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void I2C3_EV_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void I2C3_ER_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void OTG_HS_EP1_OUT_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void OTG_HS_EP1_IN_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void OTG_HS_WKUP_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void OTG_HS_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void RNG_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void FPU_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void UART7_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void UART8_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void SPI4_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void SPI5_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void SAI1_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void SAI2_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void QUADSPI_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void LPTIM1_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void SDMMC2_IRQHandler();