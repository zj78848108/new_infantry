#pragma once
#include <stdint.h>

// Plain, volatile RAM fields: expand imu_watch in VisualGDB Live Watch.
// Angles are each sensor's native frame, in degrees (no zero/axis alignment).
struct ImuWatchSensor
{
    float roll_deg = 0, pitch_deg = 0, yaw_deg = 0;
    uint32_t online = 0, age_ms = UINT32_MAX, frame_count = 0;
    float update_hz = 0;
    uint32_t error_count = 0;
    uint32_t rx_blocks = 0, rx_bytes = 0, queue_overruns = 0;
    uint32_t checksum_errors = 0, format_errors = 0;
    uint32_t duplicate_samples = 0, sensor_tick = 0;
    // S102: 0=boot wait, 1=configuring, 2=ready, 3=timeout,
    // 4=NACK, 5=readback mismatch, 6=TX error, 7=RX allocation failure.
    uint32_t config_state = 0, last_command = 0, config_retries = 0;
    // HAL status: 0=OK, 1=ERROR, 2=BUSY, 3=TIMEOUT.
    uint32_t last_tx_status = 0, tx_failures = 0;
};

struct ImuWatch
{
    ImuWatchSensor ch010, s102;
    uint32_t task_running = 0;
    int32_t task_create_result = 0;
    uint32_t task_stack_free_words = 0;
    uint32_t arm_cycles = 0, decode_cycles = 0;
    uint32_t motor_cycles = 0, can_tx_cycles = 0, control_cycles = 0;
    // Bits 0..4 = Arm, Decode, Motor, CAN TX, Control; bit 5 = ImuWatch.
    uint32_t task_create_ok_mask = 0, task_create_failed_mask = 0;
    uint32_t heap_free_bytes = 0;
    // 0=none, 1=stack overflow, 2=FreeRTOS assertion.
    uint32_t fault_kind = 0, fault_line = 0, fault_ipsr = 0;
    const char* fault_file = nullptr;
    char fault_task[16] = {};
};

extern volatile ImuWatch imu_watch;
void ImuWatchTask(void* argument);
