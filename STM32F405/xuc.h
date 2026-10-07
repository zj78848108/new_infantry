#pragma once

#include <cstdint>

#include "stm32f4xx_hal.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "queue.h"

#pragma pack(push, 1)
struct XucCommandPacket
{
    uint8_t head[2];       // 'S', 'P'
    uint8_t control;       // 1: allow gimbal control
    uint8_t shoot;         // 1: request shooting
    float yaw;             // rad
    float pitch;           // rad
    uint16_t crc16;        // little-endian on the wire
};

struct XucFeedbackPacket
{
    uint8_t head[2];       // 'S', 'P'
    uint8_t mode;          // 0 idle, 1 auto aim, 2 small rune, 3 large rune
    uint8_t robot_id;
    float bullet_speed;
    uint16_t bullet_count;
    float imu_pitch;       // rad
    float imu_yaw;         // rad
    uint16_t crc16;        // little-endian on the wire
};
#pragma pack(pop)

static_assert(sizeof(XucCommandPacket) == 14, "XucCommandPacket must be 14 bytes");
static_assert(sizeof(XucFeedbackPacket) == 20, "XucFeedbackPacket must be 20 bytes");

class XUC
{
public:
    void Init(UART* huart, USART_TypeDef* instance, uint32_t baud_rate);
    void Decode();
    void Encode();

    bool Fresh(uint32_t timeout_ms = 100) const;

    bool control_enable = false;
    bool shoot_request = false;
    bool packet_valid = false;

    float target_yaw = 0.0f;
    float target_pitch = 0.0f;

    uint32_t last_rx_ms = 0;
    uint32_t crc_error_count = 0;

private:
    QueueHandle_t* queue_handler = nullptr;
    BaseType_t pd_Rx = pdFALSE;
    UART* m_uart = nullptr;

    uint8_t m_frame[UART_MAX_LEN]{};
    uint8_t tx_data[sizeof(XucFeedbackPacket)]{};
};

extern XUC xuc;