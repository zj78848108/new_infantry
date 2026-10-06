#pragma once
#include "stm32f4xx.h"
#include "usart.h"
#include <string.h>
enum IMU_TYPE { IMU601 = 0, CH010, HI226 };

class IMU
{
public:
	float ACC_FSR = 4.f, GYRO_FSR = 2000.f;
	typedef struct
	{
		float roll, yaw, pitch;
	}Angle, AngularVelocity;
	typedef struct
	{
		float x{}, y{}, z{};
	}Acceleration;

	void Init(UART* huart, USART_TypeDef* Instance, const uint32_t BaudRate, IMU_TYPE type);
	void Decode();
	bool Check(uint8_t* pdata, uint16_t len, uint32_t com);
	float GetAngleYaw();
	float GetAnglePitch();
	float GetAngleRoll();
	float getangularvelocitypitch();
	void GetAcceleration(float out[3]) const;

	bool  Fresh(uint32_t timeout_ms = 100) const;
	uint32_t FrameCount() const { return m_frame_count; }
	uint32_t ErrCount()  const { return m_err_count; }
	Angle LatestAngle()  const { return m_latest; }

	void  SetYawZero(float rad) { m_yaw_zero = rad; }
	void  SetYawZero() { m_yaw_zero = m_latest.yaw; }
	float GetYawRad()           const { return m_latest.yaw - m_yaw_zero; }

	int16_t getword(uint8_t HighBit, uint8_t LowBits);

	BaseType_t pd_Rx = false;
	QueueHandle_t* queueHandler = NULL;
private:
	Angle angle;
	AngularVelocity angularvelocity;
	Acceleration acceleration;
	uint16_t crc = 0, len;

	volatile uint16_t crc_calc = 0;
	volatile uint16_t crc_recv = 0;

	IMU_TYPE type;

	volatile uint32_t dataDmaNum;

	Angle    m_latest{};
	float    m_yaw_zero = 0.f;
	volatile uint32_t m_frame_count = 0;
	volatile uint32_t m_err_count = 0;
	volatile uint32_t m_last_ms = 0;

	uint8_t rxData[UART_MAX_LEN];
	UART* m_uart;

};

static uint16_t U2(uint8_t* p) { uint16_t u; memcpy(&u, p, 2); return u; }
static uint32_t U4(uint8_t* p) { uint32_t u; memcpy(&u, p, 4); return u; }
static float    R4(uint8_t* p) { float    r; memcpy(&r, p, 4); return r; }

extern IMU imu_chassis, imu_pantile;