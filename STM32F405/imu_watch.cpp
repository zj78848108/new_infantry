#include "imu_watch.h"
#include "imu.h"
#include "bes102.h"
#include "task.h"

volatile ImuWatch imu_watch;
BES102 imu_s102;

namespace
{
void Publish(volatile ImuWatchSensor& dest, const ImuWatchSensor& src)
{
#define COPY_FIELD(name) dest.name = src.name
    COPY_FIELD(roll_deg); COPY_FIELD(pitch_deg); COPY_FIELD(yaw_deg);
    COPY_FIELD(online); COPY_FIELD(age_ms); COPY_FIELD(frame_count);
    COPY_FIELD(update_hz); COPY_FIELD(error_count); COPY_FIELD(rx_blocks);
    COPY_FIELD(rx_bytes); COPY_FIELD(queue_overruns); COPY_FIELD(checksum_errors);
    COPY_FIELD(format_errors); COPY_FIELD(duplicate_samples); COPY_FIELD(sensor_tick);
    COPY_FIELD(config_state); COPY_FIELD(last_command); COPY_FIELD(config_retries);
    COPY_FIELD(last_tx_status); COPY_FIELD(tx_failures);
#undef COPY_FIELD
}
}

void ImuWatchTask(void*)
{
    imu_s102.Init(&uart2);
    ImuWatchSensor ch;
    uint32_t rate_ms = HAL_GetTick(), rate_frames = 0;
    TickType_t wake = xTaskGetTickCount();
    for (;;)
    {
        imu_s102.Update();
        const uint32_t now = HAL_GetTick();
        taskENTER_CRITICAL();
        const IMU::Angle a = imu_pantile.LatestAngle();
        ch.frame_count = imu_pantile.FrameCount();
        ch.error_count = imu_pantile.ErrCount();
        ch.age_ms = ch.frame_count ? HAL_GetTick() - imu_pantile.LastRxMs() : UINT32_MAX;
        taskEXIT_CRITICAL();
        ch.roll_deg = a.roll; ch.pitch_deg = a.pitch; ch.yaw_deg = a.yaw;
        ch.online = ch.frame_count && ch.age_ms <= 100;
        if (now - rate_ms >= 1000)
        {
            ch.update_hz = (ch.frame_count - rate_frames) * 1000.f / (now - rate_ms);
            rate_ms = now; rate_frames = ch.frame_count;
        }
        Publish(imu_watch.ch010, ch);
        Publish(imu_watch.s102, imu_s102.watch);
        ++imu_watch.task_running;
        imu_watch.task_stack_free_words = uxTaskGetStackHighWaterMark(nullptr);
        imu_watch.heap_free_bytes = xPortGetFreeHeapSize();
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(2));
    }
}

extern "C" void vApplicationStackOverflowHook(TaskHandle_t, char* name)
{
    taskDISABLE_INTERRUPTS();
    imu_watch.fault_kind = 1;
    imu_watch.fault_ipsr = __get_IPSR();
    for (unsigned i = 0; i < sizeof(imu_watch.fault_task) - 1 && name && name[i]; ++i)
        imu_watch.fault_task[i] = name[i];
    for (;;) {}
}

extern "C" void ImuWatchAssert(const char* file, unsigned line)
{
    taskDISABLE_INTERRUPTS();
    imu_watch.fault_kind = 2;
    imu_watch.fault_file = file;
    imu_watch.fault_line = line;
    imu_watch.fault_ipsr = __get_IPSR();
    for (;;) {}
}
