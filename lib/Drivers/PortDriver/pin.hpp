/**************************************************************************************************
 * @file pin.hpp
 * @author Yakovlev Vladislav (y.yakovlev@quanttelecom.ru)
 * @brief
 * @version 1.2
 * @date 04.11.2022
 *
 * @copyright Copyright (c) 2022
 *
 *************************************************************************************************/
#ifndef _PIN_HPP
#define _PIN_HPP

#include "stm32f1xx.h"
#include <cmath>
#include <stdio.h>
#include <logger/logger.hpp>


/******************************************************************************************************************
 * @brief Defines different pin configurations for the GPIO pins. Each configuration is represented by a binary value.
 * Here's a breakdown of each configuration:
 * @param OUTPUT_GPO_Push_Pull MODE[11]; CNF[00]; ODR[0/1];
 * @param OUTPUT_GPO_Open_Drain MODE[11]; CNF[01]; ODR[0/1];
 * @param OUTPUT_AFO_Push_Pull MODE[11]; CNF[10]; ODR[~~~];
 * @param OUTPUT_AFO_Open_Drain MODE[11]; CNF[11]; ODR[~~~];
 * @param INPUT_Analog MODE[00]; CNF[00]; ODR[~~~];
 * @param INPUT_Floating MODE[00]; CNF[01]; ODR[~~~];
 * @param INPUT_Pull_Down MODE[00]; CNF[10]; ODR[0/1];
 * @param INPUT_Pull_Up MODE[00]; CNF[11]; ODR[0/1];
 *****************************************************************************************************************/
enum class TYPE
{
    output_GPO_Push_Pull  = 0b0001,///< MODE[01]; CNF[00]; ODR[0/1];
    OUTPUT_GPO_Push_Pull  = 0b0011,///< MODE[11]; CNF[00]; ODR[0/1];
    OUTPUT_GPO_Open_Drain = 0b0111,///< MODE[11]; CNF[01]; ODR[0/1];
    OUTPUT_AFO_Push_Pull  = 0b1011,///< MODE[11]; CNF[10]; ODR[~~~];
    OUTPUT_AFO_Open_Drain = 0b1111,///< MODE[11]; CNF[11]; ODR[~~~];

    INPUT_Analog    = 0b0000,///< MODE[00]; CNF[00]; ODR[~~~];
    INPUT_Floating  = 0b0100,///< MODE[00]; CNF[01]; ODR[~~~];
    INPUT_Pull_Down = 0b1000,///< MODE[00]; CNF[10]; ODR[0/1];
    INPUT_Pull_Up   = 0b1100 ///< MODE[00]; CNF[11]; ODR[0/1];
};

/******************************************************************************************************************
 * @brief Defining two constants `LOW` and `HIGH` which represent the logical levels of a pin. These constants can be
 used to set or get the level of a pin, indicating whether it is in a low or high state.
 *****************************************************************************************************************/
enum class LVL
{
    LOW,
    HIGH
};

/**************************************************************************************************
 * @brief Контейнер,
 *  предоставляющий нативный интерфейс настройки
 *    и управления пина МК, а так же предоставляющий защиту от окирпичивания.
 *
 * @ingroup hardware
 *
 * @tparam _GPIOx буквенное обозначение порта
 *           @arg GPIOA
 *           @arg GPIOB
 *           @arg GPIOC
 *           @arg GPIOD
 *           @arg GPIOE
 *           @arg GPIOF
 * @tparam PIN номер вывода МК dec(0..15)
 *
 *  При вызове конструктора проверяеся задействованность инициализируемого
 *  вывода в отладочном интерфейсе JTAG/SWD в текущей конфигурации,
 *  в случае задействованности происходит прерывание настройки с
 *  целю недопущения окирпичивания данного девайса
 */
class PINx
{
  public:
    GPIO_TypeDef *_GPIOx;
    uint8_t _number{};
    TYPE pinType{};
    bool _verify{false};
    __IO uint32_t *_ConfigReg;

    explicit PINx(GPIO_TypeDef *port, uint8_t number) :
            _GPIOx(port),
            _number(number),
            _verify((_GPIOx ? true : false) & ((_number < 16) ? true : false))
    {
    }

    explicit PINx(GPIO_TypeDef *port, uint8_t number, TYPE type, LVL level = LVL::LOW) :
            _GPIOx(port),
            _number(number),
            _verify((_GPIOx ? true : false) & ((_number < 16) ? true : false))
    {
        if (_verify) Settings(type, level);
    }

    PINx()                        = default;
    PINx(PINx const &)            = default;
    PINx(PINx &&)                 = default;
    PINx &operator=(PINx const &) = default;
    PINx &operator=(PINx &&)      = default;
    ~PINx() { _Reset(_GPIOx, _number); }


    bool Settings(TYPE type, LVL level = LVL::LOW);

    bool SetPinLevel(LVL level);
    bool GetPinLevel();
    void ToglePinLevel();

  private:
    bool _Reset(GPIO_TypeDef *port, uint8_t number);
};

#endif
