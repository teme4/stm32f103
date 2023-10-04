/**************************************************************************************************
 * @file abstract_interface.hpp
 * @author Yakovlev Vladislav (y.yakovlev@quanttelecom.ru)
 * @brief The code is defining an abstract base class called `DigitalInterface`. This class serves
 *as an interface for any derived classes that inherit from it. The class declares a set of pure
 *virtual functions that must be implemented by the derived classes.
 * @version 1.0
 * @date 04.08.2022
 *
 * @copyright Quanttelecom Ⓒ
 *
 *************************************************************************************************/

#ifndef _ABSTRACT_INTERFACE_HPP
#define _ABSTRACT_INTERFACE_HPP

#include "stm32f1xx.h"
#include <array>
#include <vector>
#include <queue>
#include <cmath>

#include <PortDriver/pin.hpp>

/******************************************************************************************************************
 * @brief
 * TODO: Структура библиотеки
 * [ ]: Разделить интерфейсную и иные части
 * [ ]: Добавить заголовочный файл верхнего уровня
 *****************************************************************************************************************/
class DigitalInterface
{
  public:
   // The line `virtual bool CheckReadiness() = 0;` is declaring a pure virtual function called `CheckReadiness()` in the `DigitalInterface` class.
    virtual bool CheckReadiness()                                        = 0;
    virtual uint32_t GetAddressRegisterDR()                              = 0;
    virtual std::vector<uint8_t> Transmitt(std::vector<uint8_t> &packet) = 0;
    virtual std::vector<uint8_t> Recieve(std::vector<uint8_t> &packet)   = 0;

    DigitalInterface()                                    = default;
    DigitalInterface(DigitalInterface const &)            = default;
    DigitalInterface(DigitalInterface &&)                 = default;
    DigitalInterface &operator=(DigitalInterface const &) = default;
    DigitalInterface &operator=(DigitalInterface &&)      = default;

    virtual ~DigitalInterface(){};
};


#endif// _ABSTRACT_INTERFACE_HPP
