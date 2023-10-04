/**********************************************************************************************
 * @file rcc.cpp
 * @author Yakovlev Vladislav (y.yakovlev@quanttelecom.ru)
 * @brief
 * @version 1.7
 * @date 06.10.2021
 *
 * @copyright Copyright (c) 2021
 *
 **********************************************************************************************/
#include "rcc.hpp"

uint32_t ClockSystem::SystemCoreClock{SystemCoreClock};
uint32_t ClockSystem::APB1BusClock{APB1BusClock};
uint32_t ClockSystem::APB2BusClock{APB2BusClock};
uint32_t ClockSystem::TIMxAPB1Clock{TIMxAPB1Clock};
uint32_t ClockSystem::TIMxAPB2Clock{TIMxAPB2Clock};


ClockSystem::ClockSystem(PINx &MCO, UARTLines &baremetal)
{
    RCC->CR &= ~RCC_CR_HSEON;
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY)) {}

    RCC->CR &= ~RCC_CR_PLLON;// PLL Disable
    while ((RCC->CR & RCC_CR_PLLON)) {}

#if (_BOARD_RELEASE == _v3_0)
    RCC->CFGR &= ~RCC_CFGR_PLLSRC;// HSI oscillator clock / 2 selected as PLL input clock

    RCC->CFGR |= RCC_CFGR_PLLMULL9;// PLL input clock x9 = 36MHz
    RCC->CR |= RCC_CR_PLLON;       // PLL Enable
    while (!(RCC->CR & RCC_CR_PLLON)) {}

    FLASH->ACR &= ~FLASH_ACR_PRFTBE;  //- 0 wait states, if 0 < SYSCLK ≤ 24 MHz
    FLASH->ACR |= FLASH_ACR_LATENCY_1;//- 1 wait state, if 24 MHz < SYSCLK ≤ 48 MHz
    FLASH->ACR |= FLASH_ACR_PRFTBE;   //- 2 wait states, if 48 MHz < SYSCLK ≤ 72 MHz

    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;  // SYSCLK prescaler(AHB) = 72MHz/1 = 72MHz
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV1; // APB Low-speed prescaler (APB1) = 36MHz/1 = 36MHz
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV1; // APB high-speed prescaler(APB2) = 36MHz/1 = 36MHz
    RCC->CFGR |= RCC_CFGR_ADCPRE_DIV4;// ADC = 36MHz/4 = 9MHz
#else

    RCC->CR |= RCC_CR_HSEON;// HSE Enable
    int i{100000};
    while (!(RCC->CR & RCC_CR_HSERDY))
        if (i-- < 0) break;

    if (RCC->CR & RCC_CR_HSERDY)
    {
        RCC->CFGR |= RCC_CFGR_PLLXTPRE_HSE;// HSE -> PLLXTPRE = 8MHz
        RCC->CFGR |= RCC_CFGR_PLLSRC;      // PLLXTPRE -> PLLSRC -> PLLMULL
        RCC->CFGR |= RCC_CFGR_PLLMULL9;    // PLLMULL x9 = 72MHz
    }
    else
    {
        RCC->CFGR &= ~RCC_CFGR_PLLSRC;  // HSI/2 -> PLLSRC -> PLLMULL
        RCC->CFGR |= RCC_CFGR_PLLMULL16;// PLLMULL x16 = 64MHz
    }

    RCC->CR |= RCC_CR_PLLON;// PLL Enable = 72MHz
    while (!(RCC->CR & RCC_CR_PLLRDY)) {}

    FLASH->ACR &= ~FLASH_ACR_PRFTBE;

    /**************************************************************************************************
     * @brief if 0 < SYSCLK ≤ 24 MHz -- 0 wait states
     * 24 MHz < SYSCLK ≤ 48 MHz -- 1 wait state
     * 48 MHz < SYSCLK ≤ 72 MHz -- 2 wait states
     ***************************************************************************************************/
    FLASH->ACR |= FLASH_ACR_LATENCY_2;
    FLASH->ACR |= FLASH_ACR_PRFTBE;

    /**************************************************************************************************
     * @brief SYSCLK prescaler(AHB) = PLL/1 = 72MHz
     ***************************************************************************************************/
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
    /**************************************************************************************************
     * @brief APB Low-speed prescaler (APB1) = AHB/2 = 36MHz
     ***************************************************************************************************/
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;
    /**************************************************************************************************
     * @brief APB high-speed prescaler(APB2) = AHB/1 = 72MHz
     ***************************************************************************************************/
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;
    /**************************************************************************************************
     * @brief ADC = APB2/6 = 12MHz
     ***************************************************************************************************/
    RCC->CFGR |= RCC_CFGR_ADCPRE_DIV4;

#endif

    RCC->CFGR |= RCC_CFGR_SW_PLL;         // PLL -> SYSCKLK
    while (!(RCC->CFGR & RCC_CFGR_SWS)) {}// wait PLL used as system clock

    SystemCoreClock = SystemCoreClockUpdate();
    if (SysTick_Config(SystemCoreClock / 1000)) RCC->BDCR &= ~RCC_BDCR_BDRST;

    uint32_t APB1Prescaller = 0;
    uint32_t APB2Prescaller = 0;
    if (!(RCC->CFGR & RCC_CFGR_PPRE1))
        APB1Prescaller = 1;
    else
        APB1Prescaller = (1 << (((RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos) - 3));

    if (!(RCC->CFGR & RCC_CFGR_PPRE2))
        APB2Prescaller = 1;
    else
        APB2Prescaller = (1 << (((RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos) - 3));

    APB1BusClock  = SystemCoreClock / APB1Prescaller;
    APB2BusClock  = SystemCoreClock / APB2Prescaller;
    TIMxAPB1Clock = (APB1Prescaller == 1) ? (APB1BusClock) : (APB1BusClock * 2);
    TIMxAPB2Clock = (APB2Prescaller == 1) ? (APB2BusClock) : (APB2BusClock * 2);
    if (TIMxAPB1Clock > 72000000) TIMxAPB1Clock = 72000000;
    if (TIMxAPB2Clock > 72000000) TIMxAPB2Clock = 72000000;


    // RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
    // RCC->APB1ENR |= RCC_APB1ENR_WWDGEN;
    RCC->APB1ENR |= RCC_APB1ENR_BKPEN;
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;

    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPDEN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPEEN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPFEN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPGEN;

    GPIOA->CRL &= ~(GPIO_CRL_CNF | GPIO_CRL_MODE);
    GPIOA->CRH &= ~(GPIO_CRH_CNF | GPIO_CRH_MODE);

    GPIOB->CRL &= ~(GPIO_CRL_CNF | GPIO_CRL_MODE);
    GPIOB->CRH &= ~(GPIO_CRH_CNF | GPIO_CRH_MODE);

    GPIOC->CRL &= ~(GPIO_CRL_CNF | GPIO_CRL_MODE);
    GPIOC->CRH &= ~(GPIO_CRH_CNF | GPIO_CRH_MODE);

    GPIOD->CRL &= ~(GPIO_CRL_CNF | GPIO_CRL_MODE);
    GPIOD->CRH &= ~(GPIO_CRH_CNF | GPIO_CRH_MODE);

    GPIOE->CRL &= ~(GPIO_CRL_CNF | GPIO_CRL_MODE);
    GPIOE->CRH &= ~(GPIO_CRH_CNF | GPIO_CRH_MODE);

    GPIOF->CRL &= ~(GPIO_CRL_CNF | GPIO_CRL_MODE);
    GPIOF->CRH &= ~(GPIO_CRH_CNF | GPIO_CRH_MODE);

    GPIOG->CRL &= ~(GPIO_CRL_CNF | GPIO_CRL_MODE);
    GPIOG->CRH &= ~(GPIO_CRH_CNF | GPIO_CRH_MODE);

    /*Termocontroller PWM*/
    DBGMCU->CR |= DBGMCU_CR_DBG_TIM1_STOP;
    /*Interface Nkk <-> PKU*/
    DBGMCU->CR &= ~DBGMCU_CR_DBG_TIM2_STOP;
    /*Vacancy*/
    DBGMCU->CR &= ~DBGMCU_CR_DBG_TIM3_STOP;
    /*Timers::delay()*/
    DBGMCU->CR |= DBGMCU_CR_DBG_TIM4_STOP;
    /*Vacancy*/
    DBGMCU->CR &= ~DBGMCU_CR_DBG_TIM5_STOP;
    /*Timers::TimeOut()*/
    DBGMCU->CR |= DBGMCU_CR_DBG_TIM6_STOP;
    /*Timers::EnableIndication*/
    DBGMCU->CR &= ~DBGMCU_CR_DBG_TIM7_STOP;
    /*Invertor NVOA PWM*/
    DBGMCU->CR &= ~DBGMCU_CR_DBG_TIM8_STOP;
    /**/
    DBGMCU->CR |= DBGMCU_CR_DBG_IWDG_STOP;

    // setvbuf(stdin, NULL, _IONBF, 0);
    // setvbuf(stdout, NULL, _IONBF, 0);
    // setvbuf(stderr, NULL, _IONBF, 0);

    SetUpDebugInterface(baremetal);

    MCO_Init(MCO, RCC_CFGR_MCOSEL_SYSCLK);

    // ClockGeneration(TIM1, MCO, 6000000, 50);

    // MCO_Init(RCC_CFGR_MCOSEL_PLL_DIV2);
    // MCO_Init(RCC_CFGR_MCOSEL_NOCLOCK);
    return;
}


/**************************************************************************************************
 * @brief Настройка UART1 2Мбод/с
 * Отладочный интерфейс, реализует транспортный уровень
 * функции printf()
 * @param baremetal A pointer to an object of @c element_type
 * @param baud_rate скорость обмена (бод/с)
 *************************************************************************************************/
void ClockSystem::SetUpDebugInterface(UARTLines &baremetal)
{
    static UART debug(baremetal.UARTx,
                      baremetal.BaudRate,
                      std::make_unique<PINx>(baremetal.TxD),
                      std::make_unique<PINx>(baremetal.RxD));
}

/**********************************************************************************************
 * @brief Вывод частоты на ногу PA8
 * @param rcc_cfgr_mcosel Disable, HSI, HSE, SYSCLK, PLL/2
 **********************************************************************************************/
void ClockSystem::MCO_Init(PINx &MCO, int rcc_cfgr_mcosel)// RCC
{
    RCC->CFGR &= ~RCC_CFGR_MCO_Msk;//Сначала устанавливаем все в ноль
    RCC->CFGR |= rcc_cfgr_mcosel;  //Устанавливаем источник тактирования

    if (rcc_cfgr_mcosel == RCC_CFGR_MCOSEL_NOCLOCK) return;

    //Настраиваем порт в режим альтернативной функции
    MCO.Settings(TYPE::OUTPUT_AFO_Push_Pull);
}

uint32_t ClockSystem::SystemCoreClockUpdate()
{
    uint32_t tmp = 0U, pllmull = 0U, pllsource = 0U;
    static const uint32_t HSE_Freq{8000000};
    static const uint32_t HSI_Freq{8000000};

    /* Get SYSCLK source -------------------------------------------------------*/
    tmp = RCC->CFGR & RCC_CFGR_SWS;

    switch (tmp)
    {
        case 0x00U: /* HSI used as system clock */ SystemCoreClock = HSI_Freq; break;
        case 0x04U: /* HSE used as system clock */ SystemCoreClock = HSE_Freq; break;
        case 0x08U: /* PLL used as system clock */

            /* Get PLL clock source and multiplication factor
             * ----------------------*/
            pllmull   = RCC->CFGR & RCC_CFGR_PLLMULL;
            pllsource = RCC->CFGR & RCC_CFGR_PLLSRC;

            pllmull = (pllmull >> 18U) + 2U;

            if (pllsource == 0x00U)
            {
                /* HSI oscillator clock divided by 2 selected as PLL clock entry */
                SystemCoreClock = (HSI_Freq >> 1U) * pllmull;
            }
            else
            {
                /* HSE selected as PLL clock entry */
                if ((RCC->CFGR & RCC_CFGR_PLLXTPRE) != (uint32_t)RESET)
                { /* HSE oscillator clock divided by 2 */
                    SystemCoreClock = (HSE_Freq >> 1U) * pllmull;
                }
                else { SystemCoreClock = HSE_Freq * pllmull; }
            }
            break;

        default: SystemCoreClock = HSI_Freq; break;
    }

    /* Compute HCLK clock frequency ----------------*/
    /* Get HCLK prescaler */
    tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4U)];
    /* HCLK clock frequency */
    SystemCoreClock >>= tmp;
    return SystemCoreClock;
}

void ClockSystem::ClockGeneration(TIM_TypeDef *timer, PINx &gen, double frequency, double duty_cycle)
{
    gen.Settings(TYPE::OUTPUT_AFO_Push_Pull);

    uint32_t prescaller    = 0;
    uint32_t AutoReloadReg = static_cast<double>(TIMxAPB2Clock) / (prescaller + 1) / frequency;

    while (AutoReloadReg > 65535)
    {
        prescaller++;
        if (prescaller > 65535) return;
        AutoReloadReg = static_cast<double>(TIMxAPB2Clock) / (prescaller + 1) / frequency;
    }
    uint16_t CCR1val = roundf(static_cast<double>(AutoReloadReg) * (duty_cycle / 100));

    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    timer->CR1 &= ~TIM_CR1_CEN;
    timer->CNT = 0;
    /*Выход канал 1, активный уровень низкий*/
    timer->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1P);
    /*Использовать выводы таймера как выходы*/
    timer->BDTR |= TIM_BDTR_MOE;
    /*PWM mode 1, прямой ШИМ 1 канал*/
    timer->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1);
    /*считаем вверх*/
    timer->CR1 &= ~TIM_CR1_DIR;
    /*выравнивание по фронту, Fast PWM*/
    timer->CR1 &= ~TIM_CR1_CMS;

    timer->ARR  = AutoReloadReg - 1;
    timer->PSC  = prescaller;
    timer->CCR1 = CCR1val;
    timer->CR1 |= TIM_CR1_CEN;
    return;
}

/**********************************************************************************************
 * @brief Reinterprets the uint32_t as a float.
 *
 * @param recv_bytes
 * @return float
 **********************************************************************************************/
float uint2float(uint32_t recv_bytes)
{
    float *flt_array = reinterpret_cast<float *>(&recv_bytes);
    return flt_array[0];
}

/**********************************************************************************************
 * @brief Reinterprets the float as a uint32_t.
 * @param send_double floating-point number
 * @return uint32_t
 **********************************************************************************************/
uint32_t float2uint(double send_double)
{
    float trans         = static_cast<float>(send_double);
    uint32_t *flt_array = reinterpret_cast<uint32_t *>(&trans);
    return flt_array[0];
}

uint32_t vector2uint(std::vector<uint8_t> &vector, bool erase)
{
    uint32_t Value{0};
    if (erase)
    {
        for (int shift = 0; shift < 32; shift += 8)
        {
            if (vector.empty()) return Value;
            Value |= vector.back() << shift;
            vector.pop_back();
        }
        return Value;
    }

    std::vector<uint8_t> transit{vector};
    for (int shift = 0; shift < 32; shift += 8)
    {
        if (transit.empty()) return Value;
        Value |= transit.back() << shift;
        transit.pop_back();
    }
    return Value;
}

uint32_t BKP_GetAdr(uint8_t adr)
{
    static uint32_t t[44] = {BKP_BASE + 0x00, BKP_BASE + 0x04, BKP_BASE + 0x08, BKP_BASE + 0x0c, BKP_BASE + 0x10,
                             BKP_BASE + 0x14, BKP_BASE + 0x18, BKP_BASE + 0x1c, BKP_BASE + 0x20, BKP_BASE + 0x24,
                             BKP_BASE + 0x28, BKP_BASE + 0x40, BKP_BASE + 0x44, BKP_BASE + 0x48, BKP_BASE + 0x4c,
                             BKP_BASE + 0x50, BKP_BASE + 0x54, BKP_BASE + 0x58, BKP_BASE + 0x5c, BKP_BASE + 0x60,
                             BKP_BASE + 0x64, BKP_BASE + 0x68, BKP_BASE + 0x6c, BKP_BASE + 0x70, BKP_BASE + 0x74,
                             BKP_BASE + 0x78, BKP_BASE + 0x7c, BKP_BASE + 0x80, BKP_BASE + 0x84, BKP_BASE + 0x88,
                             BKP_BASE + 0x8c, BKP_BASE + 0x90, BKP_BASE + 0x92, BKP_BASE + 0x94, BKP_BASE + 0x98,
                             BKP_BASE + 0x9c, BKP_BASE + 0xa0, BKP_BASE + 0xa4, BKP_BASE + 0xa8, BKP_BASE + 0xac,
                             BKP_BASE + 0xb0, BKP_BASE + 0xb4, BKP_BASE + 0xb8, BKP_BASE + 0xbc};
    if (adr <= 42)
        return t[adr];// Вернуть данные
    else
        return t[43];
}

/**
 * @brief  Writes user data to the specified Data Backup Register.
 * @param  BKP_DR: specifies the Data Backup Register.
 *   This parameter can be BKP_DRx where x:[1, 42]
 * @param  Data: data to write
 * @retval None
 */
void BKP_WriteBackupRegister(uint8_t adr, uint16_t Data)
{
    PWR->CR |= PWR_CR_DBP;// Разрешить запись в область BKP
    *(__IO uint16_t *)BKP_GetAdr(adr) = (uint16_t)Data;
    PWR->CR &= ~PWR_CR_DBP;// Запретить запись в область BKP
}

/**
 * @brief  Reads data from the specified Data Backup Register.
 * @param  BKP_DR: specifies the Data Backup Register.
 *   This parameter can be BKP_DRx where x:[1, 42]
 * @retval The content of the specified Data Backup Register
 */
uint16_t BKP_ReadBackupRegister(uint8_t adr) { return (*(__IO uint32_t *)BKP_GetAdr(adr)); }
