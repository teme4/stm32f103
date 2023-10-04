/**************************************************************************************************
 * @file timers.hpp
 * @author Yakovlev Vladislav (y.yakovlev@quanttelecom.ru)
 * @brief
 * @version 2.0
 * @date 08.02.2022
 *
 * @copyright Copyright (c) 2022
 *
 *************************************************************************************************/
#ifndef timers_H
#define timers_H

#include "stm32f1xx.h"
#include <cmath>
#include <memory>
#include <PortDriver/pin.hpp>
#include <logger/logger.hpp>
#include "../rcc/rcc.hpp"

class BoardPinOut;

class Timers
{
  public:
    static double SystemTime;
    static bool TimeOutFlag;
    static std::unique_ptr<PINx> GreenLed;
    static std::unique_ptr<PINx> RedLed;

    enum OnOff : bool
    {
        ACTIVE   = true,
        INACTIVE = false
    };

    enum Type
    {
        TIMEOUT,
        RESET
    };

    enum Units
    {
        SECOND      = 1,
        MILLISECOND = 1000,
        MICROSECOND = 1000000
    };
    enum Color
    {
        Green,
        Orange,
        Red,
        None
    };

    Timers()                          = default;
    Timers(Timers const &)            = default;
    Timers(Timers &&)                 = default;
    Timers &operator=(Timers const &) = default;
    Timers &operator=(Timers &&)      = default;
    ~Timers(){};


    void EnableIndication(uint32_t period,
                          Timers::Units time_unit,
                          std::unique_ptr<PINx> green,
                          std::unique_ptr<PINx> red)
    {
        GreenLed = std::move(green);
        RedLed   = std::move(red);
        Indication(OnOff::ACTIVE);
    }

    static void Indication(OnOff mode);
    static void Blink(Color color);

    static void SetColor(Color color);

    static double GetSystemTime();
    static void IntegrateSystemTime(double n);
    static void ResetSystemTime();
    static bool CheckTimeOutFlag();
    static void SetTimeOutFlag(bool n);

    static void delay(uint32_t delay, Timers::Units time_unit);
    static void delay(uint32_t delay, uint32_t time_unit);
    static void TimeStart(Timers::Type type, uint32_t time, Timers::Units time_unit);
    static void TimeStop();
};


class ProgrammTimer
{
  public:
    ProgrammTimer(double delta) : _stop_time(Timers::GetSystemTime() + delta) {}
    ProgrammTimer()                                 = delete;
    ProgrammTimer(ProgrammTimer const &)            = delete;
    ProgrammTimer(ProgrammTimer &&)                 = delete;
    ProgrammTimer &operator=(ProgrammTimer const &) = delete;
    ProgrammTimer &operator=(ProgrammTimer &&)      = delete;
    ~ProgrammTimer() { ; }

    /**************************************************************************************************
     * @brief Time remaining check
     * @return true - time is over;\
     * @return false - time left;
     *************************************************************************************************/
    bool TimeOutEvent()
    {
        return ((_stop_time < Timers::GetSystemTime()) ? (true) : (false));
    }
    double CheckRemainingTime() { return (_stop_time - Timers::GetSystemTime()); }
    //события выхода времени
  private:
    double _stop_time;
};


#endif
