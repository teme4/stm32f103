/**********************************************************************************************
 * @file logger.hpp
 * @author Yakovlev Vladislav (y.yakovlev@quanttelecom.ru)
 * @brief
 * @version X.x
 * @date 22.06.2022
 *
 * @copyright Copyright (c) 2022
 *
 **********************************************************************************************/
#ifndef LOGGER_HPP
#define LOGGER_HPP

#include "stm32f1xx.h"
#include <cmath>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <vector>
#include <array>

#define _LOGGER_EN_ 1
#if (_LOGGER_EN_)

#ifndef MESSAGE
#define MESSAGE(type, _f, ...) Logger::LogMessage(type, _f, ##__VA_ARGS__)
#endif// MESSAGE

#ifndef MESSAGE_CLEAR
#define MESSAGE_CLEAR(_f, ...) Logger::LogMessage(Logger::Type::NONE, _f, ##__VA_ARGS__)
#endif// MESSAGE

#ifndef MESSAGE_FATAL_ERROR
#define MESSAGE_FATAL_ERROR(_f, ...) Logger::LogMessage(Logger::Type::FATAL, _f, ##__VA_ARGS__)
#endif// MESSAGE_FATAL_ERROR

#ifndef MESSAGE_ERROR
#define MESSAGE_ERROR(_f, ...) Logger::LogMessage(Logger::Type::ERROR, _f, ##__VA_ARGS__)
#endif// MESSAGE_ERROR

#ifndef MESSAGE_WARNING
#define MESSAGE_WARNING(_f, ...) Logger::LogMessage(Logger::Type::WARNING, _f, ##__VA_ARGS__)
#endif// MESSAGE_WARNING

#ifndef MESSAGE_INFO
#define MESSAGE_INFO(_f, ...) Logger::LogMessage(Logger::Type::INFO, _f, ##__VA_ARGS__)
#endif// MESSAGE_INFO

#ifndef MESSAGE_DEBUG
#define MESSAGE_DEBUG(_f, ...) Logger::LogMessage(Logger::Type::DEBUG, _f, ##__VA_ARGS__)
#endif// MESSAGE_DEBUG

#ifndef MESSAGE_SUCCESS
#define MESSAGE_SUCCESS(_f, ...) Logger::LogMessage(Logger::Type::SUCCESS, _f, ##__VA_ARGS__)
#endif// MESSAGE_SUCCESS

#ifndef MESSAGE_START_PROCESS
#define MESSAGE_START_PROCESS(_f, ...) Logger::LogMessage(Logger::Type::PROCESS, _f, ##__VA_ARGS__)
#endif// MESSAGE_START_PROCESS

#ifndef MESSAGE_END_PROCESS
#define MESSAGE_END_PROCESS(_f, ...) Logger::LogMessage(Logger::Type::PROC_SUCCESS, _f, ##__VA_ARGS__)
#endif// MESSAGE_END_PROCESS

#ifndef MESSAGE_TRACE
#define MESSAGE_TRACE() Logger::Location()
#endif// MESSAGE_TRACE

#ifndef MESSAGE_PROGRESS_BAR
#define MESSAGE_PROGRESS_BAR(step, total) Logger::ProgressBar(step, total)
#endif// MESSAGE_TRACE

#ifndef MESSAGE_VECTOR
#define MESSAGE_VECTOR(T, name, vect) Logger::PrintVector<T>(std::string(name), vect);
#endif// MESSAGE_VECTOR

#ifndef MESSAGE_VECTOR1
#define MESSAGE_VECTOR1(vect) Logger::PrintVector(std::string(#vect), vect);
#endif// MESSAGE_VECTOR1

#ifndef MESSAGE_VARIABLE
#define MESSAGE_VARIABLE(name) Logger::SendPoint(std::string(#name), name)
#endif// MESSAGE_VECTOR

#ifndef PRINT_GRAPH_XY
#define PRINT_GRAPH_XY(nameX, nameY) Logger::SendGraphXY(std::string(#nameX), std::string(#nameY), nameX, nameY)
#endif// PRINT_GRAPH_XY

#define COLORIZE 1

#else//(!_LOGGER_EN_)

#ifndef MESSAGE
#define MESSAGE(type, _f, ...)
#endif// MESSAGE

#ifndef MESSAGE_CLEAR
#define MESSAGE_CLEAR(_f, ...)
#endif// MESSAGE

#ifndef MESSAGE_FATAL_ERROR
#define MESSAGE_FATAL_ERROR(_f, ...)
#endif// MESSAGE_FATAL_ERROR

#ifndef MESSAGE_ERROR
#define MESSAGE_ERROR(_f, ...)
#endif// MESSAGE_ERROR

#ifndef MESSAGE_WARNING
#define MESSAGE_WARNING(_f, ...)
#endif// MESSAGE_WARNING

#ifndef MESSAGE_INFO
#define MESSAGE_INFO(_f, ...)
#endif// MESSAGE_INFO

#ifndef MESSAGE_DEBUG
#define MESSAGE_DEBUG(_f, ...)
#endif// MESSAGE_DEBUG

#ifndef MESSAGE_SUCCESS
#define MESSAGE_SUCCESS(_f, ...)
#endif// MESSAGE_SUCCESS

#ifndef MESSAGE_START_PROCESS
#define MESSAGE_START_PROCESS(_f, ...)
#endif// MESSAGE_START_PROCESS

#ifndef MESSAGE_END_PROCESS
#define MESSAGE_END_PROCESS(_f, ...)
#endif// MESSAGE_END_PROCESS

#ifndef MESSAGE_TRACE
#define MESSAGE_TRACE()
#endif// MESSAGE_TRACE

#ifndef MESSAGE_PROGRESS_BAR
#define MESSAGE_PROGRESS_BAR(step, total)
#endif// MESSAGE_TRACE

#ifndef MESSAGE_VECTOR
#define MESSAGE_VECTOR(T, name, vector)
#endif// MESSAGE_VECTOR

#ifndef MESSAGE_VECTOR1
#define MESSAGE_VECTOR1(vector)
#endif// MESSAGE_VECTOR

#ifndef MESSAGE_VARIABLE
#define MESSAGE_VARIABLE(name) Logger::SendPoint(std::string(#name), name)
#endif// MESSAGE_VARIABLE

#ifndef PRINT_GRAPH_XY
#define PRINT_GRAPH_XY(nameX, nameY) Logger::SendGraphXY(std::string(#nameX), std::string(#nameY), nameX, nameY)
#endif// PRINT_GRAPH_XY


#define COLORIZE 0
#endif//(_LOGGER_EN_)

struct source_location
{
    static constexpr source_location current(const char* __file = __builtin_FILE(),
                                             const char* __func = __builtin_FUNCTION(),
                                             int __line         = __builtin_LINE()) noexcept
    {
        source_location __loc;
        __loc._M_file = __file;
        __loc._M_func = __func;
        __loc._M_line = __line;
        return __loc;
    }

    constexpr source_location() noexcept : _M_file("unknown"), _M_func(_M_file), _M_line(0) {}

    constexpr uint32_t line() const noexcept { return _M_line; }
    constexpr const char* file_name() const noexcept { return _M_file; }
    constexpr const char* function_name() const noexcept { return _M_func; }

  private:
    const char* _M_file;
    const char* _M_func;
    uint32_t _M_line;
};

class myColor
{
  public:
#if (COLORIZE)
    static const char* kRed() { return "\033[1;31m"; };
    static const char* kHRed() { return "\033[37;1;41m"; };
    static const char* kGreen() { return "\033[1;32m"; };
    static const char* kYellow() { return "\033[1;33m"; };
    static const char* kDarkBlue() { return "\033[1;34m"; };
    static const char* kMagnetta() { return "\033[1;35m"; };
    static const char* kCyan() { return "\033[1;36m"; };
    static const char* kReset() { return "\033[m\n"; };
#else
    static const char* kRed() { return ""; };
    static const char* kHRed() { return ""; };
    static const char* kGreen() { return ""; };
    static const char* kYellow() { return ""; };
    static const char* kDarkBlue() { return ""; };
    static const char* kMagnetta() { return ""; };
    static const char* kCyan() { return ""; };
    static const char* kReset() { return "\n"; };
#endif
};

class Logger
{
  public:
    static bool ALLOWED_LOG_OUT;
    static double Time;
    /**************************************************************************************************
     * @brief Тип сообщения
     ***************************************************************************************************/
    enum class Type
    {
        PROCESS,
        FATAL = 1,
        ERROR,
        WARNING,
        INFO,
        PROC_SUCCESS,
        SUCCESS,
        DEBUG,
        NONE
    };

    /**************************************************************************************************
     * @brief decrement
     ***************************************************************************************************/
    enum class Tab
    {
        Increment,
        Decrement,
        Reset,
        Get
    };

    template<typename T>
    static void SendPoint(std::string name, T& point)
    {
        name += ":" + std::to_string(point);
        printf(">%s\n", name.c_str());
    }

    template<typename T>
    static void SendGraphXY(std::string nameX, std::string nameY, T& pointX, T& pointY)
    {
        printf(">%s = f(%s):%s:%s|xy\n",
               nameX.c_str(),
               nameY.c_str(),
               std::to_string(pointX).c_str(),
               std::to_string(pointY).c_str());
    }

    static void TAB(Tab tab = Tab::Get);

    template<typename T>
    static void PrintVector(std::string name, std::vector<T> vector)
    {
        if (!ALLOWED_LOG_OUT) return;
        printf("%s>%s[%02d]: ", myColor::kCyan(), name.c_str(), vector.size());

        int i{0};
        int size{static_cast<int>(vector.size() * sizeof(T))};
        for (T& value : vector)
        {
            if (!(i % 64))
            {
                if (size > 64)
                    printf("\n0x");
                else
                    printf("0x");
            }
            printf("%02X", value);
            i += sizeof(T);
        }

        printf("%s", myColor::kReset());
        fflush(stdout);
    }

    static void LogMessage(Type type, const char* format, ...);
    static void ProgressBar(uint32_t step, uint32_t total);

    static void Dialog(const char* format, ...);
    static void Location(source_location loc = source_location::current())
    {
        LogMessage(Type::INFO,
                   "File: \"%s\"; Function: \"%s\"; Line: %d;\n",
                   loc.file_name(),
                   loc.function_name(),
                   loc.line());
    };

    static double GetSystemTime() { return Time / 1000; }
    static void IntegrateSystemTime() { Time++; }
};

/**************************************************************************************************
 * @brief text
 ***************************************************************************************************/


class MyCursor
{
  public:
#if (COLORIZE)

    /**************************************************************************************************
     * @brief передвинуть курсор на N строк
     * @param row down (-N)/ up (+N)
     *************************************************************************************************/
    static void MoveLines(int row)
    {
        if (!Logger::ALLOWED_LOG_OUT) return;
        if (row >= 0)
            printf("\033[%dA", row);
        else
            printf("\033[%dB", abs(row));
    }
    static void ClearLines() { printf("\033[K"); }
    /**************************************************************************************************
     * @brief передвинуть курсор на N столбцов
     * @param column left (-N)/ right(+N)
     *************************************************************************************************/
    static void MoveColumns(int column)
    {
        if (!Logger::ALLOWED_LOG_OUT) return;
        if (column >= 0)
            printf("\033[%dС", column);
        else
            printf("\033[%dD", abs(column));
    }
    /**************************************************************************************************
     * @brief передвинуть курсор на N строк и поставить в начало
     * @param row down (-N)/ up (+N)
     *************************************************************************************************/
    static void MoveLinesSTP(int row)
    {
        if (!Logger::ALLOWED_LOG_OUT) return;
        for (int i = 0; i < abs(row); i++)
        {
            if (row >= 0)
                printf("\033[1E");
            else
                printf("\033[1F");
        }
    }
    /**/
    /**************************************************************************************************
     * @brief Переместить курсор в указанный столбец текущей строки
     * @param set_column указанный столбец
     *************************************************************************************************/
    static void SetColumn(int set_column)
    {
        if (!Logger::ALLOWED_LOG_OUT) return;
        printf("\033[%dG", set_column);
    }
    /**************************************************************************************************
     * @brief Задает абсолютные координаты курсора
     * @param row строка
     * @param column столбец
     *************************************************************************************************/
    static void SetOxOy(int row, int column)
    {
        if (!Logger::ALLOWED_LOG_OUT) return;
        printf("\033[%d;%dH;", row, column);
    }
    static void SavePosition()
    {
        if (!Logger::ALLOWED_LOG_OUT) return;
        printf("\033[s");
    }
    static void BackToSavePosition()
    {
        if (!Logger::ALLOWED_LOG_OUT) return;
        printf("\033[u");
    }
    static void GetPosition()
    {
        if (!Logger::ALLOWED_LOG_OUT) return;
        printf("\033[");
    }
    static void ClearTerminal() { printf("\033[2J\n"); }
#else
    // clang-format off
    static void ClearTerminal() {}
    static void MoveLines(int row){}
    static void MoveColumns(int column){}
    static void MoveLinesSTP(int row){}
    static void SetColumn(int set_column){}
    static void SetOxOy(int row, int column){}
#endif
    // clang-format on
};

#endif// LOGGER_HPP

/*
%с	Символ типа char
%d	Десятичное число целого типа со знаком
%i	Десятичное число целого типа со знаком
%е	Научная нотация (е нижнего регистра)
%Е	Научная нотация (Е верхнего регистра)
%f	Десятичное число с плавающей точкой
%g	Использует код %е или %f — тот из них, который короче (при использовании %g
используется е нижнего регистра) %G	Использует код %Е или %f — тот из них, который
короче (при использовании %G используется Е верхнего регистра)
%о	Восьмеричное целое число без знака
%s	Строка символов
%u	Десятичное число целого типа без знака
%х	Шестнадцатиричное целое число без знака (буквы нижнего регистра)
%Х	Шестнадцатиричное целое число без знака (буквы верхнего регистра)
%р	Выводит на экран значение указателя
%n	Ассоциированный аргумент — это указатель на переменную целого типа, в которую
помещено количество символов, записанных на данный момент
%%	Выводит символ %
*/

// clang-format off
///< 1) для управления курсором:
///< \033[l;сН — позиционирует (устанавливает) курсор в с-ю позицию l - й строки. По умолчанию (←[Н) курсор устанавливается в верхний левый угол экрана;
///< \033[l;cf — то же;
///< \033[nА— перемещает курсор вверх на п строк без изменения позиции внутри строки. По умолчанию (←[А) принимается 1. Если курсор уже находится в верхней строке, то последовательность игнорируется;
///< \033[пВ — перемещает курсор вниз на п строк без изменения позиции внутри строки. По умолчанию (←[В) принимается 1. Если курсор уже находится в нижней строке, то последовательность игнорируется;
///< \033[пС — перемещает курсор по строке вправо на п позиций. По умолчанию (←[С) принимается 1. Если курсор уже находится в крайней правой позиции строки, то последовательность игнорируется;
///< \033[nD — перемещает курсор по строке влево на п позиций. По умолчанию (←[D) принимается 1. Если курсор уже находится в крайней левой позиции, то последова­тельность игнорируется;
///< \033[s — запоминает текущую позицию курсора (номера строки и позиции в строке, т.е. колонки) для обеспечения возможности последующей его установки в эту позицию. Более одной позиции запомнить нельзя;
///< \033[u — восстанавливает позицию курсора (устанавливает курсор в позицию, запомненную последней из последовательностей ←[s);
///< \033[6n— выводит на экран текущую позицию курсора в форме ←[l;sR, где l — номер строки, a s — номер столбца;
///< 2) для удаления символов с экрана дисплея:
///< \033[2J— очищает экран и устанавливает курсор в его левый верхний угол;
///< \033[К — удаляет все символы от курсора до конца строки (включая позицию курсора);
///< 3) для установки режима работы дисплейного адаптера и выбора цветов:
/*   \033[s;...sm— устанавливает атрибуты дисплея в текстовом режиме, к2оторые действуют до тех пор, пока не будут изменены такой
 же последовательностью. Однако атрибуты уже выведенной информации не изменяются, вследствие чего целесообразно
пред­варительно очищать экран. В качестве атрибутов s в любом порядке можно задать режим отображения символов, цвет
символов и цвет фона. Если какой-либо из атрибутов не указан, то продолжает действовать установленный.
*/
// clang-format on
