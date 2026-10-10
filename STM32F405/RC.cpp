#include "label.h"
#include "RC.h"
#include "control.h"
#include "HTmotor.h"
#include "imu.h"
#include "xuc.h"

namespace
{
	enum class AUTO_UP_PHASE
	{
		GO_ZERO,
		GO_TARGET,
		RETURN_ZERO,
		HOLD_ZERO
	};

	AUTO_UP_PHASE auto_up_phase = AUTO_UP_PHASE::GO_ZERO;

	constexpr uint32_t AUTO_UP_HOLD_MS = 1500;

	uint32_t target_reached_time = 0;

	constexpr float AUTO_UP_EPS = 0.1f;

	float follow_yaw_zero = 0.0f;

	bool follow_yaw_zero_ready = false;

	float offset_deg = 0.0f;

	float offset_rad = 0.0f;
}

void RC::Init(UART* huart, USART_TypeDef* Instance, const uint32_t BaudRate)
{
	huart->Init(Instance, BaudRate).DMARxInit(nullptr);
	m_uart = huart;
	queueHandler = &huart->UartQueueHandler;
}

void RC::OnRC()
{
	RC_CheckState();
	RC_Control();

	if (Shift_mode())
	{

	}

}

void RC::OnPC()
{
	;
}

void RC::Update()
{
	OnRC();
	OnPC();
}

void RC::RC_CheckState() {

	switch (RC_STATE(rc.s[0], rc.s[1]))
	{
	case RC_STATE(UP, UP):
		ctrl.mode = CONTROL::AUTO_UP;
		break;

	case RC_STATE(UP, MID):
		ctrl.mode = CONTROL::CHASSIS_MOVE;
		break;

	case RC_STATE(UP, DOWN):
		ctrl.mode = CONTROL::SPINNING;
		break;

	case RC_STATE(MID, UP):
		ctrl.mode = CONTROL::FOLLOW;
		break;

	case RC_STATE(MID, MID):
		ctrl.mode = CONTROL::RESET;
		break;

	case RC_STATE(MID, DOWN):
		ctrl.mode = CONTROL::SEPARATE;
		break;

	case RC_STATE(DOWN, UP):
		ctrl.mode = CONTROL::FIRE;
		break;

	case RC_STATE(DOWN, MID):
		ctrl.mode = CONTROL::UP_STAIRS;
		break;

	case RC_STATE(DOWN, DOWN):
		ctrl.mode = CONTROL::AUTOAIM;
		break;

	default:
		break;
	}

}

void RC::RC_Control() {
	if (ctrl.mode != CONTROL::AUTO_UP)
	{
		auto_up_phase = AUTO_UP_PHASE::GO_ZERO;

		target_reached_time = 0;
	}

	if (ctrl.mode != CONTROL::RESET)
	{
		switch (ctrl.mode)
		{
		case CONTROL::ROTATION:

			break;

		case CONTROL::FOLLOW:
		{
			for (uint8_t i = 0; i < 4; i++)
			{
				DMmotor[i].setSpeed = 5.0f;
				DMmotor[i].SetTargetPos(0.0f);
			}

			ctrl.chassis.speedz = 0.0f;


			if (Shift_mode())
			{
				follow_yaw_zero_ready = false;

				DMmotor[YAW].setSpeed = 0.5f;
				DMmotor[YAW].SetTargetPos(0.0f);

				DMmotor[PITCH].SetTargetPos(DMmotor[PITCH].pos);
				DMmotor[PITCH].setPos = DMmotor[PITCH].pos;
			}

			

			if (follow_yaw_zero_ready)
			{
				offset_deg =  follow_yaw_zero - imu_pantile.GetAngleYaw();

				while (offset_deg > 180.0f)
					offset_deg -= 360.0f;

				while (offset_deg < -180.0f)
					offset_deg += 360.0f;
				
				offset_rad = offset_deg * 3.14f / 180.f;

				ctrl.chassis.speedx = rc.ch[3] * cosf(offset_rad) * 1500.f / 660.f 
									  - rc.ch[2] * sinf(offset_rad) * 1500.f / 660.f;
				ctrl.chassis.speedy = rc.ch[3] * sinf(offset_rad) * 1500.f / 660.f 
									  + rc.ch[2] * cosf(offset_rad) * 1500.f / 660.f;

				DMmotor[YAW].setSpeed = 5.0f;

				DMmotor[YAW].SetTargetPos(DMmotor[YAW].targetPos + rc.ch[0] / 660.0f * 0.01f);
			}
			else
			{
				if (fabsf(DMmotor[YAW].pos) < 0.05f && imu_pantile.Fresh())
				{
					follow_yaw_zero = imu_pantile.GetAngleYaw();
					follow_yaw_zero_ready = true;
				}
				else
				{
					DMmotor[YAW].setSpeed = 0.5f;
					DMmotor[YAW].SetTargetPos(0.0f);
					follow_yaw_zero_ready = false;
				}
				
				ctrl.chassis.speedx = rc.ch[3] * 1500.f / 660.f;
				ctrl.chassis.speedy = rc.ch[2] * 1500.f / 660.f;
			}

			

			DMmotor[PITCH].setSpeed = 3.0f;
			DMmotor[PITCH].SetTargetPos(DMmotor[PITCH].targetPos + rc.ch[1] / 660.0f * 0.01f);




			break;
		}

		case CONTROL::SEPARATE:
		{

			for (uint8_t i = 0; i < 4; i++)
			{
				DMmotor[i].setSpeed = 5.0f;
				DMmotor[i].SetTargetPos(0.0f);
			}

			ctrl.chassis.speedx = rc.ch[3] * 1500.f / 660.f;
			ctrl.chassis.speedy = rc.ch[2] * 1500.f / 660.f;
			ctrl.chassis.speedz = 0.0f;


			if (Shift_mode())
			{
				DMmotor[YAW].SetTargetPos(DMmotor[YAW].pos);
				DMmotor[YAW].setPos = DMmotor[YAW].pos;

				DMmotor[PITCH].SetTargetPos(DMmotor[PITCH].pos);
				DMmotor[PITCH].setPos = DMmotor[PITCH].pos;
			}//只在切入模式时执行一次，防止电机突转

			DMmotor[YAW].setSpeed = 5.0f;

			DMmotor[YAW].SetTargetPos(DMmotor[YAW].targetPos + rc.ch[0] / 660.0f * 0.01f);

			DMmotor[PITCH].setSpeed = 3.0f;
			DMmotor[PITCH].SetTargetPos(DMmotor[PITCH].targetPos + rc.ch[1] / 660.0f * 0.01f);
		

			break;
		}

		case CONTROL::AUTOAIM:
		{
			if (xuc.control_enable && xuc.Fresh())
				{
					DMmotor[YAW].setSpeed = 5.0f;
					DMmotor[YAW].SetTargetPos(xuc.target_yaw);

					DMmotor[PITCH].setSpeed = 3.0f;
					DMmotor[PITCH].SetTargetPos(xuc.target_pitch);
				}
		}

			break;

		case CONTROL::FIRE:
		{
			ctrl.shooter.openRub = true;                    // 进 fire 就开摩擦轮
			ctrl.shooter.flag = true;                       // 进入fire为单发
			ctrl.shooter.supply_bullet = (rc.ch[1] > 400);  // 右摇杆上推=拨弹

			ctrl.chassis.speedx = 0;
			ctrl.chassis.speedy = 0;
			ctrl.chassis.speedz = 0;

			if (Shift_mode())
			{
				DMmotor[YAW].SetTargetPos(DMmotor[YAW].pos);
				DMmotor[YAW].setPos = DMmotor[YAW].pos;

				DMmotor[PITCH].SetTargetPos(DMmotor[PITCH].pos);
				DMmotor[PITCH].setPos = DMmotor[PITCH].pos;
			}//只在切入模式时执行一次，防止电机突转

			DMmotor[YAW].setSpeed = 5.0f;

			DMmotor[YAW].SetTargetPos(DMmotor[YAW].targetPos + rc.ch[0] / 660.0f * 0.01f);

			DMmotor[PITCH].setSpeed = 3.0f;
			DMmotor[PITCH].SetTargetPos(DMmotor[PITCH].targetPos + rc.ch[1] / 660.0f * 0.01f);

			break;
		}

		case CONTROL::STOP:


			break;

		case CONTROL::SPINNING:
		{
			ctrl.shooter.openRub = true;                    // 进 fire 就开摩擦轮
			ctrl.shooter.flag = false;                       // 进入spinning为连发
			ctrl.shooter.supply_bullet = (rc.ch[3] > 400);  // 右摇杆上推=拨弹

			ctrl.chassis.speedx = 0;
			ctrl.chassis.speedy = 0;
			ctrl.chassis.speedz = 0;

			if (Shift_mode())
			{
				DMmotor[YAW].SetTargetPos(DMmotor[YAW].pos);
				DMmotor[YAW].setPos = DMmotor[YAW].pos;

				DMmotor[PITCH].SetTargetPos(DMmotor[PITCH].pos);
				DMmotor[PITCH].setPos = DMmotor[PITCH].pos;
			}//只在切入模式时执行一次，防止电机突转

			DMmotor[YAW].setSpeed = 5.0f;

			DMmotor[YAW].SetTargetPos(DMmotor[YAW].targetPos + rc.ch[0] / 660.0f * 0.01f);

			DMmotor[PITCH].setSpeed = 3.0f;
			DMmotor[PITCH].SetTargetPos(DMmotor[PITCH].targetPos + rc.ch[1] / 660.0f * 0.01f);

			break;
		}

		case CONTROL::AUTO_UP:
		{
			for (uint8_t i = 0; i < 4; i++)
			{
				DMmotor[i].setSpeed =
					(i == 1 || i == 3) ? 4.0f : 2.5f;
			}

			switch (auto_up_phase)
			{
			case AUTO_UP_PHASE::GO_ZERO:
			{
				// 先确保从0开始
				for (uint8_t i = 0; i < 4; i++)
				{
					DMmotor[i].SetTargetPos(0.0f);
				}

				bool reached_zero = true;

				for (uint8_t i = 0; i < 4; i++)
				{
					if (fabsf(DMmotor[i].pos) > AUTO_UP_EPS)
					{
						reached_zero = false;
						break;
					}
				}

				if (reached_zero)
				{
					auto_up_phase = AUTO_UP_PHASE::GO_TARGET;
				}

				break;
			}

			case AUTO_UP_PHASE::GO_TARGET:
			{
				const float target[4] =
				{
					2.5f,
				   -6.0f,
				   -2.5f,
					6.0f
				};

				ctrl.chassis.speedx = 700.0f;

				for (uint8_t i = 0; i < 4; i++)
				{
					DMmotor[i].SetTargetPos(target[i]);
				}

				bool reached_target = true;

				for (uint8_t i = 0; i < 3; i++)
				{
					if (fabsf(DMmotor[i].pos - target[i]) > AUTO_UP_EPS)
					{
						reached_target = false;
						break;
					}
				}

				if (reached_target)
				{
					// 第一次检测到达目标，记录时间
					if (target_reached_time == 0)
					{
						target_reached_time = HAL_GetTick();
					}

					// 持续到位达到指定时间后进入第三阶段
					if (HAL_GetTick() - target_reached_time >= AUTO_UP_HOLD_MS)
					{
						target_reached_time = 0;
						auto_up_phase = AUTO_UP_PHASE::RETURN_ZERO;
					}
				}
				else
				{
					// 中途离开误差范围，重新计时
					target_reached_time = 0;
				}

				break;
			}

			case AUTO_UP_PHASE::RETURN_ZERO:
			{
				DMmotor[0].setSpeed = 3.5f;
				DMmotor[1].setSpeed = 5.0f;
				DMmotor[2].setSpeed = 3.5f;
				DMmotor[3].setSpeed = 5.0f;

				ctrl.chassis.speedx = 1100.0f;

				for (uint8_t i = 0; i < 4; i++)
				{
					DMmotor[i].SetTargetPos(0.0f);
				}

				bool reached_zero = true;

				for (uint8_t i = 0; i < 4; i++)
				{
					if (fabsf(DMmotor[i].pos) > AUTO_UP_EPS)
					{
						reached_zero = false;
						break;
					}
				}

				if (reached_zero)
				{
					auto_up_phase = AUTO_UP_PHASE::HOLD_ZERO;
				}

				break;
			}

			case AUTO_UP_PHASE::HOLD_ZERO:
			{
				ctrl.chassis.speedx = 0.0f;

				for (uint8_t i = 0; i < 4; i++)
				{
					DMmotor[i].SetTargetPos(0.0f);
					DMmotor[i].setSpeed = 0.0f;
				}

				break;
			}
			}

			break;
		}

		case CONTROL::CHASSIS_MOVE:
		{
			DMmotor[0].setSpeed = 2.5f;
			DMmotor[1].setSpeed = 2.5f;
			DMmotor[2].setSpeed = 2.5f;
			DMmotor[3].setSpeed = 2.5f;
			ctrl.chassis.speedx = rc.ch[3] * 1500.f / 660.f;
			ctrl.chassis.speedy = rc.ch[2] * 1500.f / 660.f;
			ctrl.chassis.speedz = rc.ch[0] * 700.f / 660.f;
			break;
		}

		case CONTROL::UP_STAIRS:
		{
			DMmotor[0].setSpeed = 5.0f;
			DMmotor[1].setSpeed = 5.0f;
			DMmotor[2].setSpeed = 5.0f;
			DMmotor[3].setSpeed = 5.0f;
			float delta1 =
				rc.ch[3] / 660.0f *
				1.0f *
				0.005f;
			float delta2 = 
				rc.ch[1] / 660.0f *
				1.0f *
				0.005f;

			DMmotor[0].SetTargetPos(
				DMmotor[0].targetPos + delta1
			);
			DMmotor[1].SetTargetPos(
				DMmotor[1].targetPos - delta2
			);
			DMmotor[2].SetTargetPos(
				DMmotor[2].targetPos - delta1
			);
			DMmotor[3].SetTargetPos(
				DMmotor[3].targetPos + delta2
			);
			ctrl.chassis.speedx = rc.ch[2] * 1500.f / 660.f;

			ctrl.chassis.speedz = rc.ch[0] * 700.0f / 660.f;

			break;
		}
		default:
			ctrl.chassis.speedx = 0;
			ctrl.chassis.speedy = 0;
			ctrl.chassis.speedz = 0;
			break;
		}
	}
	else 
	{
		for (uint8_t i = 0; i < 4; i++)
		{
			DMmotor[i].setSpeed = 2.5f;
			DMmotor[i].SetTargetPos(0.0f);
		}

		DMmotor[YAW].setSpeed = 0.5f;
		DMmotor[YAW].SetTargetPos(0.0f);

		DMmotor[PITCH].setSpeed = 0.3f;
		DMmotor[PITCH].SetTargetPos(0.0f);


	}
}

void RC::Decode()
{
	if (queueHandler == NULL || *queueHandler == NULL) {
		return;  // 或者报错
	}
	else {
		pd_Rx = xQueueReceive(*queueHandler, m_frame, NULL);
	}

	if (sizeof(m_frame) < 18) return;
	if ((m_frame[0] | m_frame[1] | m_frame[2] | m_frame[3] | m_frame[4] | m_frame[5]) == 0)return;

	rc.ch[0] = ((m_frame[0] | m_frame[1] << 8) & 0x07FF) - 1024;
	rc.ch[1] = ((m_frame[1] >> 3 | m_frame[2] << 5) & 0x07FF) - 1024;
	rc.ch[2] = ((m_frame[2] >> 6 | m_frame[3] << 2 | m_frame[4] << 10) & 0x07FF) - 1024;
	rc.ch[3] = ((m_frame[4] >> 1 | m_frame[5] << 7) & 0x07FF) - 1024;
	if (rc.ch[0] <= 50 && rc.ch[0] >= -50)rc.ch[0] = 0;
	if (rc.ch[1] <= 50 && rc.ch[1] >= -50)rc.ch[1] = 0;
	if (rc.ch[2] <= 50 && rc.ch[2] >= -50)rc.ch[2] = 0;
	if (rc.ch[3] <= 50 && rc.ch[3] >= -50)rc.ch[3] = 0;

	pre_rc.s[0] = rc.s[0];
	pre_rc.s[1] = rc.s[1];

	rc.s[0] = ((m_frame[5] >> 4) & 0x0C) >> 2;
	rc.s[1] = ((m_frame[5] >> 4) & 0x03);

	pc.x = m_frame[6] | (m_frame[7] << 8);
	pc.y = m_frame[8] | (m_frame[9] << 8);
	pc.z = m_frame[10] | (m_frame[11] << 8);
	pc.press_l = m_frame[12];
	pc.press_r = m_frame[13];

	pc.key_h = m_frame[15];//按键的高位部分R F G Z X C 
	pc.key_l = m_frame[14];//按键的低8位 W S A D SHIFT CTRL Q E

}

bool RC::Shift_mode()
{
	if (rc.s[0] != pre_rc.s[0] || rc.s[1] != pre_rc.s[1])
	{
		return true;
	}
	return false;
}