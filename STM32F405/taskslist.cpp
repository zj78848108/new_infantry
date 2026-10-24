#include "label.h"
#include "taskslist.h"
#include "can.h"
#include "motor.h"
#include "imu.h"
#include "imu_watch.h"
#include "RC.h"
#include "tim.h"
#include "control.h"
#include "led.h"
#include "delay.h"
#include "HTmotor.h"
#include "Power_read.h"
#include "xuc.h"
extern float Kp = 10;
extern float Kd = 0.6;
extern int start_flag;
extern void Chassis_UpdateGimbalYaw(float yaw_rad, bool fresh);
void TASK::Init()
{
	//创建开始任务
	xTaskCreate((TaskFunction_t)start_task,            //任务函数
		(const char*)"start_task",          //任务名称
		(uint16_t)START_STK_SIZE,        //任务堆栈大小
		(void*)NULL,                  //传递给任务函数的参数
		(UBaseType_t)START_TASK_PRIO,       //任务优先级
		(TaskHandle_t*)&StartTask_Handler);   //任务句柄              
	vTaskStartScheduler();          //开启任务调度
}

/*
开始任务任务函数
*/
namespace {
void TrackCreate(unsigned bit, BaseType_t result)
{
    if (result == pdPASS) imu_watch.task_create_ok_mask |= (1U << bit);
    else imu_watch.task_create_failed_mask |= (1U << bit);
}
}
void start_task(void*)
{
    taskENTER_CRITICAL();
    TrackCreate(0, xTaskCreate(ArmTask, "ArmTask", LED_STK_SIZE, nullptr,
                             LED_TASK_PRIO, &LedTask_Handler));
    TrackCreate(1, xTaskCreate(DecodeTask, "DecodeTask", DECODE_STK_SIZE, nullptr,
                             DECODE_TASK_PRIO, &DecodeTask_Handler));
    TrackCreate(2, xTaskCreate(MotorUpdateTask, "MotorUpdateTask", MOTOR_STK_SIZE, nullptr,
                             MOTOR_TASK_PRIO, &MotorTask_Handler));
    TrackCreate(3, xTaskCreate(CanTransimtTask, "CanTransimtTask", CANTX_STK_SIZE, nullptr,
                             CANTX_TASK_PRIO, &CanTxTask_Handler));
    TrackCreate(4, xTaskCreate(ControlTask, "ControlTask", CONTROL_STK_SIZE, nullptr,
                             CONTROL_TASK_PRIO, &ControlTask_Handler));
    // Diagnostic task is lower priority and created after all control tasks.
    imu_watch.task_create_result = xTaskCreate(ImuWatchTask, "ImuWatch", 384,
                                              nullptr, 1, nullptr);
    TrackCreate(5, imu_watch.task_create_result);
    imu_watch.heap_free_bytes = xPortGetFreeHeapSize();
    taskEXIT_CRITICAL();
    vTaskDelete(nullptr);
}
int CNT = 0;
// —— 调试观测：两个摩擦轮转速绝对值 ——
volatile int32_t rub_spd_a   = 0;   // can2_motor[0] |curspeed|
volatile int32_t rub_spd_b   = 0;   // can2_motor[1] |curspeed|
volatile int32_t rub_spd_gap = 0;   // a - b（正常应接近 0）
volatile int32_t rub_spd_1= 0;
volatile int32_t rub_spd_2 = 0;
void MotorUpdateTask(void* pvParameters)
{
	
	while (1)
	{
	TickType_t xlastWakeTime = xTaskGetTickCount();
	
		++imu_watch.motor_cycles;
		for (auto& motor : can1_motor)motor.Ontimer(can1.data, can1.temp_data);

		for (auto& motor : can2_motor)motor.Ontimer(can2.data, can2.temp_data);

		rub_spd_a = (can2_motor[0].curspeed < 0) ? -can2_motor[0].curspeed : can2_motor[0].curspeed;
		rub_spd_b = (can2_motor[1].curspeed < 0) ? -can2_motor[1].curspeed : can2_motor[1].curspeed;
		rub_spd_gap = rub_spd_a - rub_spd_b;
		rub_spd_1 = can2_motor[0].curspeed - 8000;
		rub_spd_2 = can2_motor[1].curspeed + 8000;

		for (uint8_t i = 0; i < DMMOTOR_NUM; i++)
		{

			DMmotor[i].State_Decode(can1, can1.jointidata, i);

			DMmotor[i].UpdateTargetPos(0.01f);

			DMmotor[i].DMmotor_Ontimer(
				can1,
				DMmotor[i].Kp,
				DMmotor[i].Kd,
				can1.jointpdata[i]
			);
		}


	vTaskDelayUntil(&xlastWakeTime, pdMS_TO_TICKS(2));//开始执行该任务之后1ms再执行该任务
}
}

void CanTransimtTask(void* pvParameters)
{
	while (true)
	{

		TickType_t xlastWakeTime1 = xTaskGetTickCount();

		++imu_watch.can_tx_cycles;
		switch ((timer.counter++) % 3)
		{
		case 0:
			for (uint8_t i = 0; i < DMMOTOR_NUM; i++)
			{
				DMmotor[i].DMmotor_transmit(i);
			}
			break;
		case 1:
			can1.Transmit(0x1ff, can1.temp_data + 8);
			can2.Transmit(0x1ff, can2.temp_data + 8);
			break;
		case 2:
			can1.Transmit(0x200, can1.temp_data);
			can2.Transmit(0x200, can2.temp_data);
		default:
			break;
		}
		
		vTaskDelayUntil(&xlastWakeTime1, pdMS_TO_TICKS(1));//开始执行该任务之后1ms再执行该任务

	}
}

void ControlTask(void* pvParameters)
{
	while (true)
	{
		++imu_watch.control_cycles;
		rc.Update();
		//ctrl.pantile.Update();
		ctrl.chassis.Update();
		xuc.Encode();
		ctrl.shooter.Update();
		vTaskDelay(5);
	}
}


void DecodeTask(void* pvParameters)
{
	while (true)
	{
		TickType_t xlastWakeTime = xTaskGetTickCount();

		++imu_watch.decode_cycles;
		rc.Decode();
		xuc.Decode();

		imu_pantile.Decode();
		
		vTaskDelayUntil(&xlastWakeTime, pdMS_TO_TICKS(2));
	}
}

void ArmTask(void* pvParameters)
{
	while (true)
	{
		//初始化达妙电机
		for (uint8_t i = 0; i < DMMOTOR_NUM; i++)
		{
			DMmotor[i].DMmotorinit();
		}
		++imu_watch.arm_cycles;
		power.Send();
		vTaskDelay(100);
	}
}





