#include "xuc.h"

#include <cmath>
#include <cstring>

#include "CRC.h"
#include "control.h"
#include "imu.h"
#include "judgement.h"

namespace
{
constexpr uint16_t XUC_CRC16_INIT = 0xFFFFU;

bool IsFinite(float value)
{
    return std::isfinite(value);
}
}

void XUC::Init(UART* huart, USART_TypeDef* instance, uint32_t baud_rate)
{
    huart->Init(instance, baud_rate).DMARxInit(nullptr).DMATxInit();

    m_uart = huart;
    queue_handler = &huart->UartQueueHandler;

    control_enable = false;
    shoot_request = false;
    packet_valid = false;
    last_rx_ms = 0;
    crc_error_count = 0;
}

void XUC::Decode()
{
    if (queue_handler == nullptr || *queue_handler == nullptr || m_uart == nullptr)
        return;

    if (xQueueReceive(*queue_handler, m_frame, 0) != pdTRUE)
        return;

    uint16_t rx_len = UART_MAX_LEN;
    if (m_uart->dataDmaNum > 0U && m_uart->dataDmaNum <= UART_MAX_LEN)
        rx_len = static_cast<uint16_t>(m_uart->dataDmaNum);

    for (uint16_t offset = 0;
         offset + sizeof(XucCommandPacket) <= rx_len;
         ++offset)
    {
        if (m_frame[offset] != 'S' || m_frame[offset + 1U] != 'P')
            continue;

        XucCommandPacket packet{};
        std::memcpy(&packet, m_frame + offset, sizeof(packet));

        const uint16_t crc_calc = GetCRC16CheckSum(
            reinterpret_cast<uint8_t*>(&packet),
            sizeof(packet) - sizeof(packet.crc16),
            XUC_CRC16_INIT);

        if (crc_calc != packet.crc16)
        {
            ++crc_error_count;
            continue;
        }

        if (!IsFinite(packet.yaw) || !IsFinite(packet.pitch))
            continue;

        control_enable = packet.control != 0U;
        shoot_request = packet.shoot != 0U;
        target_yaw = packet.yaw;
        target_pitch = packet.pitch;
        last_rx_ms = HAL_GetTick();
        packet_valid = true;
        return;
    }
}

void XUC::Encode()
{
    if (m_uart == nullptr)
        return;

    XucFeedbackPacket packet{};
    packet.head[0] = 'S';
    packet.head[1] = 'P';

    // The upper computer uses 1 for auto-aim. The local CONTROL enum values
    // must not be sent directly because their numeric values differ.
    packet.mode = (ctrl.mode == CONTROL::AUTOAIM) ? 1U : 0U;
    packet.robot_id = judgement.data.robot_status_t.robot_id;

    // Shooter speed/count are not connected in this project yet.
    packet.bullet_speed = 0.0f;
    packet.bullet_count = 0U;

    const float pitch = imu_pantile.GetAnglePitch();
    const float yaw = imu_pantile.GetAngleYaw();
    packet.imu_pitch = IsFinite(pitch) ? pitch : 0.0f;
    packet.imu_yaw = IsFinite(yaw) ? yaw : 0.0f;

    packet.crc16 = GetCRC16CheckSum(
        reinterpret_cast<uint8_t*>(&packet),
        sizeof(packet) - sizeof(packet.crc16),
        XUC_CRC16_INIT);

    std::memcpy(tx_data, &packet, sizeof(packet));
    m_uart->UARTTransmit(tx_data, sizeof(tx_data));
}

bool XUC::Fresh(uint32_t timeout_ms) const
{
    return packet_valid &&
           static_cast<uint32_t>(HAL_GetTick() - last_rx_ms) <= timeout_ms;
}