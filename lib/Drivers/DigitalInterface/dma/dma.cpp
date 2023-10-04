
#include "dma.hpp"


/******************************************************************************************************************
 * @brief Initializes the hardware settings for a DMA channel.
 *
 * @param memtomem specifies whether the DMA transfer is from memory to memory or from
 * memory to peripheral. It can take values from the enum type "Other".
 * @param lvl represents the priority level of the DMA channel. It is of type "PRIORITY"
 * @param source the source parameter represents the size of the source data transfer. It is of type
 * SIZE, representing different sizes such as BYTE, HALFWORD,
 * or WORD.
 * @param dest represents the destination address for the DMA
 * transfer. It specifies the memory location where the data will be transferred to.
 * @param per_incr represents the peripheral increment mode
 * for the DMA channel. It specifies whether the memory address should be incremented after each
 * transfer or not.
 * @param mem_incr represents the memory increment mode for
 * the DMA channel. It specifies whether the memory address should be incremented after each transfer
 * or not.
 * @param cycle represents the DMA transfer mode. It specifies
 * whether the DMA transfer should be performed in single mode or in circular mode.
 * @param dir represents the direction of the data transfer.
 * @param tc_irq specify the interrupt settings for the transfer complete (TC) event. It is of type "IRQ_Settings" which
 *an enumeration that represent different interrupt settings.
 * @param htc_irq The parameter "htc_irq" stands for "Half Transfer Complete Interrupt Request". It is used to specify
 *the interrupt settings for the half transfer complete event in the DMA channel.
 * @param terror_irq The "terror_irq" parameter is an IRQ_Settings enum that specifies the interrupt request (IRQ)
 *settings for the transfer error (TE) interrupt. It determines how the DMA controller should handle transfer errors
 *during data transfer.
 *
 * @return void, which means it is not returning any value.
 *****************************************************************************************************************/
void ChannelDMA::_HardwareSettings(Other memtomem,
                                   PRIORITY lvl,
                                   SIZE source,
                                   SIZE dest,
                                   Other per_incr,
                                   Other mem_incr,
                                   Other cycle,
                                   Other dir,
                                   IRQ_Settings tc_irq,
                                   IRQ_Settings htc_irq,
                                   IRQ_Settings terror_irq)
{
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    _number = ((reinterpret_cast<uint32_t>(_Channel)) - reinterpret_cast<uint32_t>(DMA1_Channel1)) / 20;
    _GCIFx  = (DMA_ISR_GIF1 << (_number * 4));
    _TCIFx  = (DMA_ISR_TCIF1 << (_number * 4));
    _HTIFx  = (DMA_ISR_HTIF1 << (_number * 4));
    _TEIFx  = (DMA_ISR_TEIF1 << (_number * 4));
    ClearFlagChannelDMA();

    regCR = static_cast<uint32_t>(memtomem) | static_cast<uint32_t>(lvl) | static_cast<uint32_t>(source) |
            static_cast<uint32_t>(dest) | static_cast<uint32_t>(per_incr) | static_cast<uint32_t>(mem_incr) |
            static_cast<uint32_t>(cycle) | static_cast<uint32_t>(dir) | static_cast<uint32_t>(tc_irq) |
            static_cast<uint32_t>(htc_irq) | static_cast<uint32_t>(terror_irq);

    _Channel->CCR = 0;
    _Channel->CCR = regCR;
    ClearFlagChannelDMA();

    _SettingsAccident = (_Channel->CCR != regCR) ? (true) : (false);

    return;
}

/******************************************************************************************************************
 * @brief Reloads a DMA channel with new peripheral, memory, and buffer size values and then enables the channel.
 *
 * @param peripheral address of the peripheral device that the DMA channel will be connected to.
 * @param memory address of the memory buffer where the data will be transferred to or from.
 * @param buffer_size size of the buffer that will be used for data transfer between the peripheral and memory.
 *
 * @return true operation was successful
 * @return false operation was failed
 *****************************************************************************************************************/
bool ChannelDMA::ReStartChannel(uint32_t peripheral, uint32_t memory, uint32_t buffer_size)
{
    if (ReLoadChannel(peripheral, memory, buffer_size)) return EnableChannel();
    return false;
}

/**************************************************************************************************
 * @brief
 * @param destination CMAR
 * @param source CPAR
 * @param buffer_size
 * @return true operation was successful
 * @return false operation was failed
 *************************************************************************************************/
bool ChannelDMA::ReLoadChannel(uint32_t peripheral, uint32_t memory, uint32_t buffer_size)
{
    if (_SettingsAccident) return false;

    _Connection(peripheral, memory, buffer_size);

    return (_Channel->CCR == regCR) ? (true) : (false);
}

/**************************************************************************************************
 * @brief Connecting a DMA channel to peripherals and memory
 * @param peripheral адрес переферии
 * @param memory адрес памяти
 * @param buffer_size объем памяти
 * @return true operation was successful
 * @return false operation was failed
 *************************************************************************************************/
bool ChannelDMA::_Connection(uint32_t peripheral, uint32_t memory, uint32_t buffer_size)
{
    DisableChannel();
    ClearFlagChannelDMA();
    _Channel->CPAR  = peripheral; ///<Адрес источника
    _Channel->CMAR  = memory;     ///<Адрес приемника
    _Channel->CNDTR = buffer_size;///<Размер буфера
    MaxBufferSize   = _Channel->CNDTR;

    return true;
}

/**************************************************************************************************
 * @brief Enable DMA Channel
 * @return true Enable
 * @return false Fuck
 *************************************************************************************************/
bool ChannelDMA::EnableChannel()
{
    _Channel->CCR |= DMA_CCR_EN;
    while (!(_Channel->CCR & DMA_CCR_EN)) {}
    return (_Channel->CCR & DMA_CCR_EN);
}

/**************************************************************************************************
 * @brief Disable DMA Channel
 * @return true Disable
 * @return false Fuck
 *************************************************************************************************/
bool ChannelDMA::DisableChannel()
{
    _Channel->CCR &= ~DMA_CCR_EN;
    while ((_Channel->CCR & DMA_CCR_EN)) {}
    return !(_Channel->CCR & DMA_CCR_EN);
}

/**************************************************************************************************
 * @brief Clear DMA Channel Interrupt Flag
 *************************************************************************************************/
void ChannelDMA::ClearFlagChannelDMA()
{
    _DMAmodule->IFCR |= _TEIFx;//Сброс флага ошибки передачи канала
    _DMAmodule->IFCR |= _HTIFx;//Сброс флага завершения передачи 50%
    _DMAmodule->IFCR |= _TCIFx;//Сброс флага завершения передачи 100%
    _DMAmodule->IFCR |= _GCIFx;//Сброс флага глобального прерывания
}

/**************************************************************************************************
 * @brief Check Channel State
 * @return true Enable
 * @return false Disable
 *************************************************************************************************/
bool ChannelDMA::CheckStateFlag() { return (_Channel->CCR & DMA_CCR_EN); }

/**************************************************************************************************
 * @brief Check Global Itrerrupt flag
 * @return true Enable
 * @return false Disable
 *************************************************************************************************/
bool ChannelDMA::CheckGeneralInterruptFlag() { return (_DMAmodule->ISR & _GCIFx); }

/**************************************************************************************************
 * @brief Check Transfer Complete Itrerrupt flag
 * @return true Enable
 * @return false Disable
 *************************************************************************************************/
bool ChannelDMA::CheckTransmittCompleteFlag() { return (_DMAmodule->ISR & _TCIFx); }

/**************************************************************************************************
 * @brief Check Half Transfer Complete Itrerrupt flag
 * @return true Enable
 * @return false Disable
 *************************************************************************************************/
bool ChannelDMA::CheckHalfTransmittCompleteFlag() { return (_DMAmodule->ISR & _HTIFx); }

/**************************************************************************************************
 * @brief Check Transfer Error Itrerrupt flag
 * @return true Enable
 * @return false Disable
 *************************************************************************************************/
bool ChannelDMA::CheckTransmittErrorFlag() { return (_DMAmodule->ISR & _TEIFx); }
