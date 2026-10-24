#pragma once

#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_usart.h"
#include "FreeRTOS.h"
#include "queue.h"

#define UART_MAX_LEN 100
	class UART
	{
	public:
		UART& Init(USART_TypeDef* Instance, uint32_t BaudRate);
		UART& DMATxInit(void);
		UART& DMARxInit(const uint8_t* buffer = nullptr, const uint32_t size = UART_MAX_LEN);

		void OnUARTITHandler();
		void OnDMAITHandler();
		void DMATransmit(uint8_t* const pData, const uint32_t Size);
		void UARTTransmit(uint8_t* pData, uint32_t Size);
		static void DMAClearAllFlags(DMA_HandleTypeDef* dmax)
		{
			const auto ele = reinterpret_cast<uint32_t>(dmax->Instance);

			(ele == reinterpret_cast<uint32_t>(DMA1_Stream0)) ? (DMA1->LIFCR = 0x0000003D) : \
				(ele == reinterpret_cast<uint32_t>(DMA1_Stream1)) ? (DMA1->LIFCR = 0x00000F40) : \
				(ele == reinterpret_cast<uint32_t>DMA1_Stream2) ? (DMA1->LIFCR = 0x003D0000) : \
				(ele == reinterpret_cast<uint32_t>DMA1_Stream3) ? (DMA1->LIFCR = 0x0F400000) : \
				(ele == reinterpret_cast<uint32_t>DMA1_Stream4) ? (DMA1->HIFCR = 0x0000003D) : \
				(ele == reinterpret_cast<uint32_t>DMA1_Stream5) ? (DMA1->HIFCR = 0x00000F40) : \
				(ele == reinterpret_cast<uint32_t>DMA1_Stream6) ? (DMA1->HIFCR = 0x003D0000) : \
				(ele == reinterpret_cast<uint32_t>DMA1_Stream7) ? (DMA1->HIFCR = 0x0F400000) : \
				(ele == reinterpret_cast<uint32_t>DMA2_Stream0) ? (DMA2->LIFCR = 0x0000003D) : \
				(ele == reinterpret_cast<uint32_t>DMA2_Stream1) ? (DMA2->LIFCR = 0x00000F40) : \
				(ele == reinterpret_cast<uint32_t>DMA2_Stream2) ? (DMA2->LIFCR = 0x003D0000) : \
				(ele == reinterpret_cast<uint32_t>DMA2_Stream3) ? (DMA2->LIFCR = 0x0F400000) : \
				(ele == reinterpret_cast<uint32_t>DMA2_Stream4) ? (DMA2->HIFCR = 0x0000003D) : \
				(ele == reinterpret_cast<uint32_t>DMA2_Stream5) ? (DMA2->HIFCR = 0x00000F40) : \
				(ele == reinterpret_cast<uint32_t>DMA2_Stream6) ? (DMA2->HIFCR = 0x003D0000) : \
				(DMA2->HIFCR = 0x0F400000);
		}

		QueueHandle_t UartQueueHandler = xQueueCreate(1, UART_MAX_LEN);
		bool updateFlag = false;
		BaseType_t pd_Rx = false;
		uint32_t dataDmaNum{};
		uint8_t m_uartrx[UART_MAX_LEN];
        // Opt-in, length/time-tagged FIFO. Existing UART users retain their queue.
        struct RxBlock {
            uint32_t received_ms;
            uint16_t length;
            uint8_t data[UART_MAX_LEN];
        };
        bool EnableBufferedRx();
        bool ReceiveBlock(RxBlock& block);
        uint32_t BufferedRxOverruns() const { return buffered_overruns_; }
        void OnBufferedRxDMAInterrupt();
        bool TransmitBounded(uint8_t* data, uint32_t size, uint32_t timeout_ms);
        HAL_StatusTypeDef TransmitBuffered(const uint8_t* data, uint16_t size);
    private:
        QueueHandle_t buffered_rx_ = nullptr;
        RxBlock buffered_block_{};
        uint8_t buffered_tx_[32]{};
        volatile bool buffered_tx_busy_ = false;
        volatile uint32_t buffered_overruns_ = 0;
        void CommitBufferedRx();
		UART_HandleTypeDef huart;
		

	};


#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

extern UART uart1, uart2, uart3, uart4, uart5, uart6;