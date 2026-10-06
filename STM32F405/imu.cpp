#include "imu.h"
#include "label.h"
namespace
{
	constexpr uint8_t  CH_OFF_TYPE = 6;    // 数据类型 0x91
	constexpr uint8_t  CH_TYPE_91 = 0x91;
	constexpr uint8_t  CH_OFF_ACC = 8;    // float x / y / z
	constexpr uint8_t  CH_OFF_GYRO = 20;   // float pitch / roll / yaw
	constexpr uint8_t  CH_OFF_ANGLE = 44;   // float pitch / roll / yaw
	constexpr uint32_t CH_TIMEOUT_MS = 2;
}

static inline bool ReadFloatLE(const uint8_t* frame, uint16_t limit,
	uint16_t p, float* out)
{
	if ((uint32_t)p + 4u > limit) return false;
	memcpy(out, frame + p, 4);
	if (!(*out > -1e6f && *out < 1e6f)) return false;   // 滤掉 NaN / Inf
	return true;
}

void IMU::Init(UART* huart, USART_TypeDef* Instance, const uint32_t BaudRate, IMU_TYPE type)
{
	huart->Init(Instance, BaudRate).DMARxInit();
	m_uart = huart;
	this->type = type;
	queueHandler = &huart->UartQueueHandler;
}

void IMU::Decode()
{
	if (queueHandler == NULL || *queueHandler == NULL) return;

	pd_Rx = xQueueReceive(*queueHandler, rxData, pdMS_TO_TICKS(CH_TIMEOUT_MS));
	if (pd_Rx != pdTRUE) return;

	if (type == IMU601)
	{
		if (rxData[0] == 0x55 && rxData[1] == 0x55)
		{
			uint8_t* data = rxData + 4;
			if (rxData[2] == 0x01 && Check(rxData, rxData[3] + 4, rxData[rxData[3] + 4]))
			{
				angle.roll = (float)getword(data[1], data[0]) * 180.f / 32768.f;
				angle.pitch = (float)getword(data[3], data[2]) * 180.f / 32768.f;
				angle.yaw = (float)getword(data[5], data[4]) * 180.f / 32768.f;
			}
			else if (rxData[2] == 0x03 && Check(rxData, rxData[3] + 4, rxData[rxData[3] + 4]))
			{
				uint8_t Ax, Ay, Az, Gx, Gy, Gz;
				Ax = getword(data[1], data[0]);
				Ay = getword(data[3], data[2]);
				Az = getword(data[5], data[4]);
				Gx = getword(data[7], data[6]);
				Gy = getword(data[9], data[8]);
				Gz = getword(data[11], data[10]);
				acceleration.x = (float)Ax / 32768 * ACC_FSR;
				acceleration.y = (float)Ay / 32768 * ACC_FSR;
				acceleration.z = (float)Az / 32768 * ACC_FSR;
				angularvelocity.roll = (float)Gx / 32768 * GYRO_FSR;
				angularvelocity.pitch = (float)Gy / 32768 * GYRO_FSR;
				angularvelocity.yaw = (float)Gz / 32768 * GYRO_FSR;

			}
		}
	}
	else if (type == CH010 || type == HI226)
	{
		uint16_t rx_len = UART_MAX_LEN;
		bool header_found = false;
		if (m_uart != nullptr && m_uart->dataDmaNum > 0U && m_uart->dataDmaNum <= UART_MAX_LEN)
			rx_len = static_cast<uint16_t>(m_uart->dataDmaNum);

		for (uint16_t offset = 0; offset + 6U <= rx_len; offset++)
		{
			if (rxData[offset] != 0x5A || rxData[offset + 1U] != 0xA5) continue;

			header_found = true;
			uint8_t* frame = rxData + offset;
			const uint16_t data_len = ((uint16_t)frame[3] << 8) + frame[2];   // 长度在 [2..3]
			const uint16_t frame_crc = ((uint16_t)frame[5] << 8) + frame[4];   // CRC 在 [4..5]
			const uint32_t total_len = (uint32_t)data_len + 6U;                // 总长 = data + 6

			if (data_len < 60U || total_len > UART_MAX_LEN || offset + total_len > rx_len)
				continue;

			crc = 0;
			const bool crc_ok = Check(frame + 6, data_len, frame_crc);
			crc = 0;
			if (!crc_ok)
			{
				++m_err_count;
				if (frame[6] != 0x91) continue;      // 不是姿态帧才跳过
			}

			if (frame[6] == 0x91)
			{
				const int o = 6;
				acceleration.x = R4(frame + o + 12);
				acceleration.y = R4(frame + o + 16);
				acceleration.z = R4(frame + o + 20);
				angularvelocity.pitch = R4(frame + o + 24);
				angularvelocity.roll = R4(frame + o + 28);
				angularvelocity.yaw = R4(frame + o + 32);
				angle.pitch = R4(frame + o + 48);
				angle.roll = R4(frame + o + 52);
				angle.yaw = R4(frame + o + 56);

				m_latest = angle;
				m_last_ms = HAL_GetTick();
				++m_frame_count;
				return;
			}
		}
		if (!header_found) ++m_err_count;
	}
}

float IMU::GetAngleYaw()
{
	return angle.yaw;
}

float IMU::getangularvelocitypitch()
{
	return angularvelocity.pitch;
}

float IMU::GetAnglePitch()
{
	return angle.pitch;
}

float IMU::GetAngleRoll()
{
	return angle.roll;
}

void IMU::GetAcceleration(float out[3]) const
{
	out[0] = acceleration.x;
	out[1] = acceleration.y;
	out[2] = acceleration.z;
}

bool IMU::Fresh(uint32_t timeout_ms) const
{
	return m_frame_count != 0 && (HAL_GetTick() - m_last_ms) <= timeout_ms;
}

bool IMU::Check(uint8_t* pdata, uint16_t len, uint32_t com)
{
	if (type == IMU601)
	{
		uint8_t t = 0;
		for (uint16_t i = 0; i < len; i++)
		{
			t += pdata[i];
		}
		return t == com;
	}
	else if (type == CH010 || type == HI226)
	{
		for (uint16_t j = 0; j < len; ++j)
		{
			uint32_t i;
			uint32_t byte = pdata[j];
			crc ^= byte << 8;
			for (i = 0; i < 8; ++i)
			{
				uint32_t temp = crc << 1;
				if (crc & 0x8000)
				{
					temp ^= 0x1021;
				}
				crc = temp;
			}
		}

		// ↓↓↓ 第 4 项的核心：兼容 CRC 高低字节相反 ↓↓↓
		crc_calc = (uint16_t)(crc & 0xFFFFU);                     // 本地算出的 CRC
		crc_recv = (uint16_t)(com & 0xFFFFU);                     // 帧内读到的 CRC
		const uint16_t crc_recv_swap = (uint16_t)((crc_recv >> 8) | (crc_recv << 8));
		return (crc_calc == crc_recv) || (crc_calc == crc_recv_swap);
		// ↑↑↑ 正常匹配 或 字节交换后匹配，都算通过 ↑↑↑
	}
	return false;
}


int16_t IMU::getword(uint8_t HighBit, uint8_t LowBits)
{
	return HighBit << 8 | LowBits;
}