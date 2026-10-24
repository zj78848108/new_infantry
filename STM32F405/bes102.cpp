#include "bes102.h"
#include <string.h>
#include <cmath>

namespace
{
uint16_t LE16(const uint8_t* p) { return p[0] | (uint16_t(p[1]) << 8); }
uint32_t LE32(const uint8_t* p)
{
    return uint32_t(p[0]) | (uint32_t(p[1]) << 8) |
           (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
}
float FloatLE(const uint8_t* p)
{
    uint32_t bits = LE32(p);
    float result;
    memcpy(&result, &bits, sizeof(result));
    return result;
}
// RAM-only settings, then read back every setting used by this parser.
const uint16_t commands[] = {6, 136, 36, 30, 34, 31, 137, 37, 35, 7};
const uint32_t values[] = {0, 1, 0, BES102::OutputMask, BES102::OutputHz};
const uint8_t command_count = sizeof(commands) / sizeof(commands[0]);
}

void BES102::Init(UART* uart)
{
    uart_ = uart;
    boot_ms_ = rate_ms_ = HAL_GetTick();
    // Must allocate before enabling USART2 RX interrupts.
    if (!uart_ || !uart_->EnableBufferedRx()) { watch.config_state = 7; return; }
    uart_->Init(USART2, S102_UART_BAUD).DMARxInit().DMATxInit();
}

void BES102::Update()
{
    if (!uart_ || watch.config_state == 7) return;
    // Bounded work even if a noisy link keeps producing blocks.
    for (unsigned n = 0; n < 8 && uart_->ReceiveBlock(rx_); ++n)
    {
        ++watch.rx_blocks;
        watch.rx_bytes += rx_.length;
        Feed(rx_.data, rx_.length, rx_.received_ms);
    }
    watch.queue_overruns = uart_->BufferedRxOverruns();
    Configure();
    const uint32_t now = HAL_GetTick();
    watch.age_ms = have_sample_ ? now - last_ms_ : UINT32_MAX;
    watch.online = have_sample_ && watch.config_state == 2 && watch.age_ms <= 100;
    if (now - rate_ms_ >= 1000)
    {
        watch.update_hz = (watch.frame_count - rate_frames_) * 1000.f / (now - rate_ms_);
        rate_frames_ = watch.frame_count;
        rate_ms_ = now;
    }
    watch.error_count = watch.checksum_errors + watch.format_errors;
}

void BES102::Feed(const uint8_t* data, uint16_t length, uint32_t received_ms)
{
    // Retain incomplete frames between IDLE/full-DMA blocks; parse ALL frames.
    for (uint16_t i = 0; i < length; ++i)
    {
        if (used_ == sizeof(stream_)) { used_ = 0; ++watch.format_errors; }
        stream_[used_++] = data[i];
        while (used_)
        {
            if (stream_[0] != 0x3A)
            {
                memmove(stream_, stream_ + 1, --used_);
                continue;
            }
            if (used_ < 7) break;
            const uint16_t payload = LE16(stream_ + 5);
            // All expected command/data responses fit in 67 bytes.
            if (payload > 56)
            {
                ++watch.format_errors;
                memmove(stream_, stream_ + 1, --used_);
                continue;
            }
            const uint16_t total = payload + 11;
            if (used_ < total) break;
            uint16_t sum = 0;
            for (uint16_t j = 1; j < payload + 7; ++j) sum += stream_[j];
            const bool tail = stream_[total - 2] == 0x0D && stream_[total - 1] == 0x0A;
            const bool checksum = sum == LE16(stream_ + payload + 7);
            if (!tail || !checksum)
            {
                if (!tail) ++watch.format_errors;
                else ++watch.checksum_errors;
                memmove(stream_, stream_ + 1, --used_);
                continue;
            }
            Frame(stream_, payload, received_ms);
            used_ -= total;
            memmove(stream_, stream_ + total, used_);
        }
    }
}

void BES102::Frame(const uint8_t* frame, uint16_t length, uint32_t received_ms)
{
    const uint16_t id = LE16(frame + 1), command = LE16(frame + 3);
    if (id != SensorId && !(command != 9 && id == CommandAddress))
    { ++watch.format_errors; return; }
    if (command == 9)
    {
        if (watch.config_state != 2) return;
        if (length != 56) { ++watch.format_errors; return; }
        for (uint16_t p = 11; p < 63; p += 4)
            if (!std::isfinite(FloatLE(frame + p))) { ++watch.format_errors; return; }
        const uint32_t tick = LE32(frame + 7);
        // No assumed timestamp unit: use MCU RX time for rate/age.
        if (have_sample_ && tick == watch.sensor_tick)
        { ++watch.duplicate_samples; return; }
        watch.roll_deg = FloatLE(frame + 51);
        watch.pitch_deg = FloatLE(frame + 55);
        watch.yaw_deg = FloatLE(frame + 59);
        watch.sensor_tick = tick;
        ++watch.frame_count;
        last_ms_ = received_ms;
        have_sample_ = true;
        return;
    }
    if (watch.config_state != 1 || !awaiting_ || step_ >= command_count) return;
    if (command == 1 && length == 0) { Fail(4); return; }
    const uint16_t expected = step_ >= 5 && step_ <= 8 ? commands[step_] : 0;
    if (command != expected) return;
    if (length != (expected ? 4 : 0)) { Fail(5); return; }
    if (expected)
    {
        const uint32_t wanted = step_ == 5 ? OutputMask :
                                step_ == 6 ? 1 : step_ == 7 ? 0 : OutputHz;
        if (LE32(frame + 7) != wanted) { Fail(5); return; }
    }
    awaiting_ = false;
    retries_ = 0;
    sent_ms_ = HAL_GetTick();
    if (++step_ == command_count) watch.config_state = 2;
}

bool BES102::Send(uint16_t command, bool with_value, uint32_t value)
{
    uint8_t packet[15] = {0x3A};
    packet[1] = uint8_t(CommandAddress);
    packet[2] = uint8_t(CommandAddress >> 8);
    packet[3] = uint8_t(command);
    packet[4] = uint8_t(command >> 8);
    const uint16_t length = with_value ? 4 : 0;
    packet[5] = uint8_t(length);
    for (uint16_t i = 0; i < length; ++i) packet[7 + i] = uint8_t(value >> (8 * i));
    uint16_t sum = 0;
    for (uint16_t i = 1; i < length + 7; ++i) sum += packet[i];
    packet[length + 7] = uint8_t(sum);
    packet[length + 8] = uint8_t(sum >> 8);
    packet[length + 9] = 0x0D;
    packet[length + 10] = 0x0A;
    // UART copies this stack packet into persistent storage before starting DMA.
    // Configuration no longer times out just because higher-priority tasks run.
    watch.last_tx_status = uart_->TransmitBuffered(packet, length + 11);
    if (watch.last_tx_status != HAL_OK) ++watch.tx_failures;
    return watch.last_tx_status == HAL_OK;
}

void BES102::Fail(uint32_t state)
{
    watch.config_state = state;
    awaiting_ = false;
    // Best effort: do not leave the device in command mode after failure.
    const uint32_t failed_status = watch.last_tx_status;
    Send(7, false, 0);
    watch.last_tx_status = failed_status;
}

void BES102::Configure()
{
    const uint32_t now = HAL_GetTick();
    if (watch.config_state == 0)
    {
        if (now - boot_ms_ < 1500) return;
        watch.config_state = 1;
    }
    if (watch.config_state != 1) return;
    if (retry_tx_ && now - sent_ms_ < 20) return;
    if (awaiting_)
    {
        if (now - sent_ms_ < 300) return;
        if (retries_ >= 2) { Fail(3); return; }
        ++retries_;
        ++watch.config_retries;
    }
    else if (step_ && now - sent_ms_ < 10) return;
    watch.last_command = commands[step_];
    sent_ms_ = now;
    awaiting_ = Send(commands[step_], step_ >= 1 && step_ <= 4,
                     step_ < 5 ? values[step_] : 0);
    retry_tx_ = !awaiting_;
    if (retry_tx_)
    {
        if (retries_ >= 2) { Fail(6); return; }
        ++retries_;
        ++watch.config_retries;
    }
}
