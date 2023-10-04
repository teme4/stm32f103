
#ifndef _HARDWARE_DRIVER_DMA_HPP
#define _HARDWARE_DRIVER_DMA_HPP

#include "stm32f1xx.h"
#include <array>
#include <vector>

#include <memory>
#include "../abstract_interface.hpp"


struct SettingsChannelDMA;

class ChannelDMA
{
  public:
    enum class IRQ_Settings
    {
        TC_IRQ_EN    = 1 << 1,
        TC_IRQ_DIS   = 0 << 1,
        HTC_IRQ_EN   = 1 << 2,
        HTC_IRQ_DIS  = 0 << 2,
        TErr_IRQ_EN  = 1 << 3,
        TErr_IRQ_DIS = 0 << 3
    };
    enum class PRIORITY
    {
        SHORT   = 0 << 12,
        AVERAGE = 1 << 12,
        HIGH    = 2 << 12,
        MAXIMUM = 3 << 12
    };
    enum class SIZE
    {
        Byte     = 0 << 8,
        HalfWord = 1 << 8,
        Word     = 2 << 8
    };
    enum class Other
    {
        MemToMem_DIS   = 0 << 14,
        MemToMem_EN    = 1 << 14,
        PER_INCRT_EN   = 1 << 6,
        PER_INCRT_DIS  = 0 << 6,
        MEM_INCRT_EN   = 1 << 7,
        MEM_INCRT_DIS  = 0 << 7,
        CYCLE_MODE_EN  = 1 << 5,
        CYCLE_MODE_DIS = 0 << 5,
        MEM_TO_PER     = 1 << 4,
        PER_TO_MEM     = 0 << 4
    };

    struct SettingsChannelDMA
    {
        DMA_TypeDef *DMAmodule{};
        DMA_Channel_TypeDef *Channel{};
        ChannelDMA::Other MemToMem{};
        ChannelDMA::PRIORITY level{};
        ChannelDMA::SIZE source{};
        ChannelDMA::SIZE destination{};
        ChannelDMA::Other PerIncrement{};
        ChannelDMA::Other MemIncrement{};
        ChannelDMA::Other Cycle{};
        ChannelDMA::Other Dirrection{};
        ChannelDMA::IRQ_Settings TCI_IRQ{};
        ChannelDMA::IRQ_Settings HTCI_IRQ{};
        ChannelDMA::IRQ_Settings TErr_IRQ{};
    };

    ChannelDMA(SettingsChannelDMA setup) : _DMAmodule(setup.DMAmodule), _Channel(setup.Channel)
    {
        _HardwareSettings(setup.MemToMem,
                          setup.level,
                          setup.source,
                          setup.destination,
                          setup.PerIncrement,
                          setup.MemIncrement,
                          setup.Cycle,
                          setup.Dirrection,
                          setup.TCI_IRQ,
                          setup.HTCI_IRQ,
                          setup.TErr_IRQ);
    }

    ChannelDMA()                              = default;
    ChannelDMA(ChannelDMA const &)            = default;
    ChannelDMA(ChannelDMA &&)                 = default;
    ChannelDMA &operator=(ChannelDMA const &) = default;
    ChannelDMA &operator=(ChannelDMA &&)      = default;
    ~ChannelDMA() { ; }

    bool EnableChannel();
    bool DisableChannel();
    bool ReLoadChannel(uint32_t peripheral, uint32_t memory, uint32_t buffer_size);
    bool ReStartChannel(uint32_t peripheral, uint32_t memory, uint32_t buffer_size);

    void ClearFlagChannelDMA();

    bool CheckStateFlag();
    bool CheckGeneralInterruptFlag();
    bool CheckTransmittCompleteFlag();
    bool CheckHalfTransmittCompleteFlag();
    bool CheckTransmittErrorFlag();

    /**********************************************************************************************
     * @brief Get the Not Transferred object
     * @return uint32_t остаток текущей транзакции, [байт]
     **********************************************************************************************/
    uint32_t GetNotTransferred() { return _Channel->CNDTR; }

    /**********************************************************************************************
     * @brief Get the Size Receiver object
     * @return uint32_t объем заполненной памяти приемного буфера (байт)
     **********************************************************************************************/
    uint32_t GetSizeReceiver() { return (MaxBufferSize - _Channel->CNDTR); };

  private:
    bool _SettingsAccident{true};

    uint32_t MaxBufferSize{0};
    uint8_t _number{0};
    uint32_t regCR{0};

    DMA_TypeDef *_DMAmodule;
    DMA_Channel_TypeDef *_Channel;
    uint32_t _GCIFx{0};
    uint32_t _TCIFx{0};
    uint32_t _HTIFx{0};
    uint32_t _TEIFx{0};

    void _HardwareSettings(Other memtomem,
                           PRIORITY lvl,
                           SIZE source,
                           SIZE dest,
                           Other per_incr,
                           Other mem_incr,
                           Other cycle,
                           Other dir,
                           IRQ_Settings tc_irq,
                           IRQ_Settings htc_irq,
                           IRQ_Settings terror_irq);

    bool _Connection(uint32_t peripheral, uint32_t memory, uint32_t buffer_size);
};

#endif//_HARDWARE_DRIVER_DMA_HPP
