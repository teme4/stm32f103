/**************************************************************************************************
 * @file uart.hpp
 * @author Yakovlev Vladislav (y.yakovlev@quanttelecom.ru)
 * @brief
 * @version 1.0
 * @date 31.08.2022
 *
 * @copyright Copyright (c) 2022
 *
 *************************************************************************************************/
#ifndef __HARDWARE_UART_DRIVER_HPP__
#define __HARDWARE_UART_DRIVER_HPP__


#include "stm32f1xx.h"
#include <array>
#include <vector>

#include <memory>
#include "../abstract_interface.hpp"
#include <PortDriver/pin.hpp>
#include <Time&Sync/drivers.hpp>


struct UARTLines
{
    USART_TypeDef *UARTx;
    uint32_t BaudRate;
    PINx TxD;
    PINx RxD;
};

class UART : public DigitalInterface
{
  public:
    USART_TypeDef *UARTx;
    double BaudRate;
    bool SettingsAccident{true};

    enum class AdditionalSetting
    {
        DMA_TX_EN    = USART_CR3_DMAT,
        DMA_TX_DIS   = 0,
        DMA_RX_EN    = USART_CR3_DMAR,
        DMA_RX_DIS   = 0,
        IDLE_IRQ_EN  = USART_CR1_IDLEIE,
        IDLE_IRQ_DIS = 0,
        RXNE_IRQ_EN  = USART_CR1_RXNEIE,
        RXNE_IRQ_DIS = 0,
        TXE_IRQ_EN   = USART_CR1_TXEIE,
        TXE_IRQ_DIS  = 0
    };

    explicit UART(USART_TypeDef *uart, double baud_rate, std::unique_ptr<PINx> TX, std::unique_ptr<PINx> RX) :
            UARTx(uart),
            BaudRate(baud_rate),
            _TxD(std::move(TX)),
            _RxD(std::move(RX))
    {
        HardwareSettings();
    }

    UART()                        = delete;
    UART(UART const &)            = default;
    UART(UART &&)                 = default;
    UART &operator=(UART const &) = default;
    UART &operator=(UART &&)      = default;
    ~UART();

    static uint16_t GetBRRsettings(uint32_t ClockFreq, double baud_rate);
    bool AdditionalSettings(AdditionalSetting dma_tx,
                            AdditionalSetting dma_rx,
                            AdditionalSetting idle_irq,
                            AdditionalSetting rxne_irq,
                            AdditionalSetting txe_irq);

    virtual uint32_t GetAddressRegisterDR() { return (reinterpret_cast<uint32_t>(&(UARTx->DR))); }
    virtual bool CheckReadiness() override { return !SettingsAccident; }
    virtual std::vector<uint8_t> Transmitt(std::vector<uint8_t> &packet) override;
    virtual std::vector<uint8_t> Recieve(std::vector<uint8_t> &packet) override;

    enum class BufferMode
    {
        RESET,
        RELOAD,
        PUSH
    };
    static std::vector<uint8_t> Reciever(uint8_t byte, BufferMode mode);
    bool CheckFlagIDLE();
    bool CheckFlagRXNE();
    bool CheckFlagTXE();
    bool ReadSRandDR();

  private:
    std::unique_ptr<PINx> _TxD{};
    std::unique_ptr<PINx> _RxD{};
    IRQn_Type _vectorIRQn{};


    bool HardwareSettings();
};


#endif//__HARDWARE_UART_DRIVER_HPP__
