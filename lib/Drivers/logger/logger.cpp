/**********************************************************************************************
 * @file logger.cpp
 * @author Yakovlev Vladislav (y.yakovlev@quanttelecom.ru)
 * @brief
 * @version X.x
 * @date 22.06.2022
 *
 * @copyright Copyright (c) 2022
 *
 **********************************************************************************************/
#include "logger.hpp"
#include <errno.h>
#include <sys/unistd.h>// STDOUT_FILENO, STDERR_FILENO
#include <stdio.h>
double Logger::Time = 0;
bool Logger::ALLOWED_LOG_OUT{true};


#if (_LOGGER_EN_)
uint32_t float2uint1(double send_double)
{
    float trans         = send_double;
    uint32_t *flt_array = reinterpret_cast<uint32_t *>(&trans);
    return flt_array[0];
}


void Logger::ProgressBar(uint32_t step, uint32_t total)
{
    if (!ALLOWED_LOG_OUT) return;
    // progress width
    const int pwidth = 110;
    // minus label len
    uint32_t width           = pwidth - 10;
    uint32_t pos             = (step * width) / (total);
    static const char *label = "Progress:";
    float percent            = static_cast<float>(step * 100) / total;

    // set green text color, only on Windows
    printf("\033[1;33m %s [", label);

    // fill progress bar with =
    printf("\033[1;31m");
    for (int i = 0; i < static_cast<int>(pos); i++)
        printf("%c", '#');

    printf("\033[1;33m% *c", width - pos + 1, ']');
    printf(" \033[1;33m%-3.6f%%\r", percent);
    printf("\033[m");
    fflush(stdout);
}

void Logger::Dialog(const char *format, ...)
{
    static char buffer[100];
    printf("%s[INFO]: ", myColor::kCyan());
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
    printf(myColor::kReset());

    scanf("%s", buffer);
    MESSAGE_INFO("Operator: %s\n", buffer);
}

void Logger::TAB(Logger::Tab tab)
{
    if (!ALLOWED_LOG_OUT) return;
    static int Size{0};
    switch (tab)
    {
        case Tab::Increment: Size++; break;
        case Tab::Decrement:
            Size--;
            if (Size < 0) Size = 0;
            break;
        case Tab::Reset:
            Size = 0;
            printf("\033[К\n");
            break;
        default:
            for (int i = 0; i < Size; i++)
                printf("\033[К\t");
            // printf("<%- 10.6fs>", GetSystemTime());
            break;
    }

    return;
}

/**********************************************************************************************
 * @brief
 *
 * @param LVL INF / WRN / ERR / SCS
 * @param fmt
 * @param ...
 **********************************************************************************************/
void Logger::LogMessage(Type type, const char *format, ...)
{
    if (!ALLOWED_LOG_OUT) return;
    MyCursor::ClearLines();
    switch (type)
    {
        case Type::PROCESS:
            TAB();
            printf("\n%s[START PROCESS]: ", myColor::kMagnetta());
            TAB(Tab::Increment);
            break;
        case Type::FATAL:
            TAB(Tab::Decrement);
            TAB();
            printf("%s[FATAL]: ", myColor::kHRed());//
            break;
        case Type::ERROR:
            TAB();
            printf("%s[ERROR]: ", myColor::kRed());
            break;
        case Type::WARNING:
            TAB();
            printf("%s[WARNING]: ", myColor::kYellow());
            break;
        case Type::INFO:
            TAB();
            printf("%s[INFO]: ", myColor::kCyan());
            break;
        case Type::SUCCESS:
            TAB();
            printf("%s[SUCCESS]: ", myColor::kGreen());
            break;
        case Type::PROC_SUCCESS:
            TAB(Tab::Decrement);
            TAB();
            printf("%s[PROCESS SUCCESS]: ", myColor::kGreen());
            break;
        case Type::DEBUG:
            TAB();
            printf("%s[DEBUG]: ", myColor::kDarkBlue());
            break;
        default: break;
    }

    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);

    if (type == Type::PROC_SUCCESS) printf("\n");

    printf(myColor::kReset());
}





#endif
