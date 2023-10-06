//#define SYSCLOCK 72000000U
#ifndef _STM32F103_HPP
#define _STM32F103_HPP
#include <stm32f1xx.h>

//----------------------------------------------------------
#define TIM_EnableIT_UPDATE(TIMx) SET_BIT(TIMx->DIER, TIM_DIER_UIE)
#define TIM_EnableCounter(TIMx) SET_BIT(TIMx->CR1, TIM_CR1_CEN)
#define TIM_DisableCounter(TIMx) CLEAR_BIT(TIMx->CR1, TIM_CR1_CEN)
//----------------------------------------------------------
void SetSysClockTo72(void);
void TIM2_Init(char koef,int period);
void TIM2_IRQHandler(void);
#endif // _STM32F103_HPP