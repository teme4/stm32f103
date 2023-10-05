/**************************************************************************************************
 * @file timers.cpp
 * @author Yakovlev Vladislav (y.yakovlev@quanttelecom.ru)
 * @brief
 * @version 1.0
 * @date 28.12.2022
 *
 * @copyright Copyright (c) 2022
 *
 *************************************************************************************************/
#include "timers.hpp"


#ifdef TESTING
volatile double my_system_time = 0;
#endif

bool Timers::TimeOutFlag  = false;
double Timers::SystemTime = 0;
extern uint32_t MaxStackSize;
extern uint32_t StackSize;


std::unique_ptr<PINx> Timers::GreenLed{};
std::unique_ptr<PINx> Timers::RedLed{};

double Timers::GetSystemTime() { return SystemTime; }
void Timers::IntegrateSystemTime(double n) { SystemTime += n; }
bool Timers::CheckTimeOutFlag() { return TimeOutFlag; }
void Timers::SetTimeOutFlag(bool n) { TimeOutFlag = n; }
void Timers::ResetSystemTime() { SystemTime = 0; }

/**
 * @brief
 * Системный таймер
 */
extern "C" void SysTick_Handler()
{
    Timers::IntegrateSystemTime(0.001);
#if (_LOGGER_EN_)
    Logger::IntegrateSystemTime();
#endif
#ifdef TESTING
    my_system_time += 1;
#endif
}

/**
 * @brief
 *
 * @param type
 * @param time
 * @param time_unit
 */
/*
void Timers::TimeStart(Timers::Type type, uint32_t time, Timers::Units time_unit)
{
    SetTimeOutFlag(false);
    if (!time) time = 1;
    uint32_t prescaller = ClockSystem::TIMxAPB1Clock / static_cast<int>(time_unit) - 1;
    double Time_divider = 1;
    if (prescaller > 0xFFFF)
    {
        Time_divider = static_cast<double>(prescaller) / 0xFFFF;
        time         = roundf(static_cast<double>(time) * Time_divider);
        prescaller   = 0xFFFF;
    }*/
/*
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    TIM6->CNT = 0;
    TIM6->PSC = prescaller;
    TIM6->ARR = time;
    switch (type)
    {
        case Type::TIMEOUT:
            NVIC_EnableIRQ(TIM6_IRQn);
            TIM6->DIER |= TIM_DIER_UIE;
            TIM6->CR1 |= TIM_CR1_CEN;
            break;
        case Type::RESET:
            NVIC_EnableIRQ(TIM6_IRQn);
            TIM6->DIER |= TIM_DIER_UIE;
            TIM6->PSC = prescaller;
            TIM6->ARR = 2069;
            TIM6->CR1 |= TIM_CR1_CEN;
            break;
    }
}*/
/**
 * @brief
 *
 */
/*
void Timers::TimeStop()
{
    TIM6->CR1 &= ~TIM_CR1_CEN;
    TIM6->CNT = 0;
    SetTimeOutFlag(false);
}
*/
/**
 * @brief
 *
 */
/*
extern "C" void TIM6_IRQHandler()
{
    TIM6->SR &= ~TIM_SR_UIF;
    TIM6->CR1 &= ~TIM_CR1_CEN;
    TIM6->CNT = 0;

    if (TIM6->ARR == 2069) NVIC_SystemReset();

    Timers::SetTimeOutFlag(true);
}*/

/**
 * @brief
 *
 * @param delay
 * @param time_unit
 */
/*
void Timers::delay(uint32_t delay, Timers::Units time_unit)
{
    if (delay <= 0) delay = 1;
    uint32_t prescaller = ClockSystem::TIMxAPB1Clock / static_cast<int>(time_unit) - 1;
    double Time_divider = 1;
    if (prescaller > 0xFFFF)
    {
        Time_divider = static_cast<double>(prescaller) / 0xFFFF;
        delay        = roundf(static_cast<double>(delay) * Time_divider);
        prescaller   = 0xFFFF;
    }

    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    TIM4->PSC = prescaller;
    TIM4->ARR = (delay * Time_divider);
    TIM4->EGR |= TIM_EGR_UG;
    TIM4->CR1 |= (TIM_CR1_CEN | TIM_CR1_OPM);
    while ((TIM4->CR1 & TIM_CR1_CEN)) {}
}*/

/*
void Timers::delay(uint32_t delay, uint32_t time_unit)
{
    if (delay <= 0) delay = 1;
    uint32_t prescaller = ClockSystem::TIMxAPB1Clock / static_cast<int>(time_unit) - 1;
    double Time_divider = 1;
    if (prescaller > 0xFFFF)
    {
        Time_divider = static_cast<double>(prescaller) / 0xFFFF;
        delay        = roundf(static_cast<double>(delay) * Time_divider);
        prescaller   = 0xFFFF;
    }

    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    TIM4->PSC = prescaller;
    TIM4->ARR = delay;
    TIM4->EGR |= TIM_EGR_UG;
    TIM4->CR1 |= (TIM_CR1_CEN | TIM_CR1_OPM);
    while ((TIM4->CR1 & TIM_CR1_CEN)) {}
}*/

/*
void Timers::Indication(OnOff mode)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
    if (mode == OnOff::INACTIVE)
    {
        TIM7->CR1  = 0;
        TIM7->CNT  = 0;
        TIM7->DIER = 0;
        NVIC_DisableIRQ(TIM7_IRQn);
        return;
    }
    if (GreenLed) GreenLed->Settings(TYPE::OUTPUT_GPO_Push_Pull);
    if (RedLed) RedLed->Settings(TYPE::OUTPUT_GPO_Push_Pull);

    NVIC_EnableIRQ(TIM7_IRQn);
    uint32_t period     = 100;
    uint32_t prescaller = ClockSystem::TIMxAPB1Clock / static_cast<int>(Units::MILLISECOND) - 1;
    double Time_divider = 1;
    if (prescaller > 0xFFFF)
    {
        Time_divider = static_cast<double>(prescaller) / 0xFFFF;
        period       = roundf(static_cast<double>(period) * Time_divider);
        prescaller   = 0xFFFF;
    }
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
    TIM7->PSC = prescaller;
    TIM7->ARR = period;
    TIM7->DIER |= TIM_DIER_UIE;
    TIM7->CR1 |= TIM_CR1_CEN;
}*/
/*
void Timers::SetColor(Color color)
{
    if (!GreenLed || !RedLed) return;

    switch (color)
    {
        case Color::Green:
            GreenLed->SetPinLevel(LVL::HIGH);
            RedLed->SetPinLevel(LVL::LOW);
            break;
        case Color::Orange:
            GreenLed->SetPinLevel(LVL::HIGH);
            RedLed->SetPinLevel(LVL::HIGH);
            break;
        case Color::Red:
            GreenLed->SetPinLevel(LVL::LOW);
            RedLed->SetPinLevel(LVL::HIGH);
            break;
        case Color::None:
            GreenLed->SetPinLevel(LVL::LOW);
            RedLed->SetPinLevel(LVL::LOW);
            break;
        default: break;
    }
}
*/
/*
void Timers::Blink(Color color)
{
#if (_BOARD_RELEASE == _v3_0)
    GPIOC->ODR ^= GPIO_ODR_ODR7;
#elif (_BOARD_RELEASE == _v3_1)
    TIM8->CCER ^= (TIM_CCER_CC2E | TIM_CCER_CC4E);
#elif (_BOARD_RELEASE >= _v4_0)
    static bool enable{false};
    if (enable)
        SetColor(color);
    else
        SetColor(Color::None);

    enable = !enable;
#endif
}*/
