/**********************************************************************************************
 * @file rcc.hpp
 * @author Yakovlev Vladislav (y.yakovlev@quanttelecom.ru)
 * @brief
 * @version 1.7
 * @date 06.10.2021
 *
 * @copyright Copyright (c) 2021
 *
 **********************************************************************************************/
#ifndef rcc_H
#define rcc_H

#include "stm32f1xx.h"
#include <cmath>
#include <array>
#include <vector>
#include <PortDriver/pin.hpp>
#include <DigitalInterface/drivers.hpp>

float uint2float(uint32_t recv_bytes);
uint32_t float2uint(double send_double);
uint32_t vector2uint(std::vector<uint8_t> &vector, bool erase);

void BKP_WriteBackupRegister(uint8_t adr, uint16_t Data);
uint16_t BKP_ReadBackupRegister(uint8_t adr);

struct UARTLines;

class ClockSystem
{
  public:
    /**************************************************************************************************
     * @brief System Clock Frequency (Core Clock)
     ***************************************************************************************************/
    static uint32_t SystemCoreClock;
    /**************************************************************************************************
     * @brief APB1 Bus Clock Frequency (Core Clock)
     ***************************************************************************************************/
    static uint32_t APB1BusClock;
    /**************************************************************************************************
     * @brief APB2 Bus Clock Frequency
     ***************************************************************************************************/
    static uint32_t APB2BusClock;
    /**************************************************************************************************
     * @brief APB1 Timers Clock Frequency (TIM<2/3/4/5/6/7/12/13/14> Clock)
     ***************************************************************************************************/
    static uint32_t TIMxAPB1Clock;
    /**************************************************************************************************
     * @brief APB2 Timers Clock Frequency (TIM<1/8/9/10/11> Clock)
     ***************************************************************************************************/
    static uint32_t TIMxAPB2Clock;

    ClockSystem(PINx &MCO, UARTLines &baremetal);
    ClockSystem(ClockSystem const &)            = default;
    ClockSystem(ClockSystem &&)                 = default;
    ClockSystem &operator=(ClockSystem const &) = default;
    ClockSystem &operator=(ClockSystem &&)      = default;
    ~ClockSystem(){};

    static uint32_t SystemCoreClockUpdate();

  private:
    void SetUpDebugInterface(UARTLines &baremetal);
    void MCO_Init(PINx &MCO, int rcc_cfgr_mcosel);
    void init_RCC(PINx &MCO, UARTLines &baremetal);
    void ClockGeneration(TIM_TypeDef *timer, PINx &gen, double frequency, double duty_cycle);
};


#endif
