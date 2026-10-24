#pragma once
#include "usart.h"
#include "imu_watch.h"

// Match the SENSOR's saved baud rate. No baud probing or flash writes.
// Default factory baud; change to 460800 if already configured that way.
#ifndef S102_UART_BAUD
#define S102_UART_BAUD 460800U
#endif

class BES102
{
public:
    static constexpr uint32_t OutputHz = 100;
    static constexpr uint32_t OutputMask = 0x1822;
    static constexpr uint16_t SensorId = 1;
    static constexpr uint16_t CommandAddress = 0;
    void Init(UART* uart);
    void Update();
    ImuWatchSensor watch;
private:
    UART* uart_ = nullptr;
    UART::RxBlock rx_{};
    uint8_t stream_[256]{};
    uint16_t used_ = 0;
    uint8_t step_ = 0, retries_ = 0;
    bool awaiting_ = false, have_sample_ = false;
    bool retry_tx_ = false;
    uint32_t boot_ms_ = 0, sent_ms_ = 0, last_ms_ = 0;
    uint32_t rate_ms_ = 0, rate_frames_ = 0;
    void Feed(const uint8_t* data, uint16_t length, uint32_t received_ms);
    void Frame(const uint8_t* frame, uint16_t length, uint32_t received_ms);
    void Configure();
    bool Send(uint16_t command, bool with_value, uint32_t value);
    void Fail(uint32_t state);
};

extern BES102 imu_s102;
