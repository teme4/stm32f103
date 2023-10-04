
#ifndef _IWDG_HPP
#define _IWDG_HPP
#include "stm32f1xx.h"
#include <cmath>

class IWDoG
{
  public:
    IWDoG(double timeout)
    {
#if (_IDWG_EN_)
        if (timeout > 26214) { return; }

        RCC->CSR |= RCC_CSR_LSION;
        while (!(RCC->CSR & RCC_CSR_LSIRDY)) {}

        _EnableAccessConfig();
        //*Prescaler divider
        if (timeout <= 409) { IWDG->PR = 0; }//*/4
        else if ((timeout <= 819) && (timeout > 409))
        {
            IWDG->PR = IWDG_PR_PR_0;
        }//*/8
        else if ((timeout <= 1638) && (timeout > 819))
            IWDG->PR = IWDG_PR_PR_1;//*/16
        else if ((timeout <= 3276) && (timeout > 1638))
        {
            IWDG->PR = (IWDG_PR_PR_1 | IWDG_PR_PR_0);
        }//*/32
        else if ((timeout <= 6553) && (timeout > 3276))
            IWDG->PR = IWDG_PR_PR_2;//*/64
        else if ((timeout <= 13107) && (timeout > 6553))
        {
            IWDG->PR = IWDG_PR_PR_2 | IWDG_PR_PR_0;
        }//*/128
        else if (timeout <= 26214 && (timeout > 13107))
        {
            IWDG->PR = IWDG_PR_PR_2 | IWDG_PR_PR_1;
        }//*/256

        IWDG->RLR = timeout;
        StartWDG();
#endif
        return;
    }

    IWDoG()                         = delete;
    IWDoG(IWDoG const &)            = default;
    IWDoG(IWDoG &&)                 = default;
    IWDoG &operator=(IWDoG const &) = default;
    IWDoG &operator=(IWDoG &&)      = default;
    ~IWDoG(){};

    /**************************************************************************************************
     * @brief Сброс IWDG для перезагрузки
     * @return true
     * @return false
     *************************************************************************************************/
    static bool ResetWDG()
    {
#if (_IDWG_EN_)
        IWDG->KR = 0xAAAA;
#endif
        return IWDG->KR;
    }

    /***********************************
     * @brief Запуск таймера IWDG
     **********************************/
    static void StartWDG(double time = 0)
    {
#if (_IDWG_EN_)
        if (time > 0.1) IWDG->RLR = time;
        IWDG->KR = 0xCCCC;
#endif
    }

  private:
    /**************************************************************************************************
     * @brief разрешает доступ на запись в регистры PR и RLR
     *************************************************************************************************/
    static void _EnableAccessConfig() { IWDG->KR = 0x5555; }
};

#endif//_IWDG_HPP
