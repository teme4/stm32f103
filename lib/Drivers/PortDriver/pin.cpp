/**************************************************************************************************
 * @file pin.cpp
 * @author Yakovlev Vladislav (y.yakovlev@quanttelecom.ru)
 * @brief
 * @version 1.2
 * @date 04.11.2022
 *
 * @copyright Copyright (c) 2022
 *
 *************************************************************************************************/

#include "pin.hpp"

/**
 * The function sets the settings for a specific pin based on the given type and level.
 *
 * @param type represents the type of pin setting. It can have the following values:
 * @param level The "level" parameter is of type "LVL" which is likely an enumeration representing the
 * level of the pin. It could have values such as "LOW" and "HIGH" to indicate the desired logic level
 * of the pin.
 *
 * @return a boolean value.
 */
bool PINx::Settings(TYPE type, LVL level)
{
    if (!_GPIOx) return false;
    if (type == pinType) return true;

    pinType          = type;
    uint8_t settings = static_cast<uint8_t>(type);

    if (_number < 8)
    {
        _GPIOx->CRL &= ~(0x0F << (_number * 4));
        _GPIOx->CRL |= (settings << (_number * 4));
    }
    else
    {
        _GPIOx->CRH &= ~(0x0F << ((_number - 8) * 4));
        _GPIOx->CRH |= (settings << ((_number - 8) * 4));
    }

    _GPIOx->ODR &= ~(GPIO_ODR_ODR0 << _number);
    _GPIOx->ODR |= (static_cast<uint8_t>(level) << _number);

    if ((type == TYPE::OUTPUT_AFO_Open_Drain) || (type == TYPE::OUTPUT_AFO_Push_Pull) || (type == TYPE::INPUT_Analog) ||
        (type == TYPE::INPUT_Floating))
        return true;

    return true;
}

/**************************************************************************************************
 * @brief Проверка входного/выходного уровня
 * @return true - уровень логической единицы
 * @return false - уровень логического нуля
 *************************************************************************************************/
bool PINx::GetPinLevel()
{
    return ((_GPIOx->IDR & (GPIO_IDR_IDR0 << _number)) || (_GPIOx->ODR & (GPIO_ODR_ODR0 << _number)));
}

/**************************************************************************************************
 * @brief Переключение выходного уровня
 *************************************************************************************************/
void PINx::ToglePinLevel() { _GPIOx->ODR ^= (GPIO_ODR_ODR0 << _number); }

/**************************************************************************************************
 * @brief Сброс регистров управления портами ввода/вывода
 *
 * @param port адрес порта ввода/вывода
 * @param number номер пина в порте port
 * @return true Состояние HiZ вход
 * @return false imposible
 *************************************************************************************************/
bool PINx::_Reset(GPIO_TypeDef *port, uint8_t number)
{
    return true;
    static const uint8_t reset{0b1111};
    if (_number < 8)
        _GPIOx->CRL &= ~(reset << (_number * 4));
    else
        _GPIOx->CRH &= ~(reset << ((_number - 8) * 4));
    return true;
}

/**************************************************************************************************
 * @brief
 *
 * @param level
 * @return true operation was successful
 * @return false operation was failed
 *************************************************************************************************/
bool PINx::SetPinLevel(LVL level)
{
    if (!_verify) return false;

    _GPIOx->BSRR |= (((level == LVL::HIGH) ? GPIO_BSRR_BS0 : GPIO_BSRR_BR0) << _number);
    return true;
}
