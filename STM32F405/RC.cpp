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

	constexpr uint32_t AUTO_UP_HOLD_MS = 500;

	uint32_t target_reached_time = 0;

	uint32_t return_zero_time = 0;

	bool return_zero_flag = false;

	constexpr float AUTO_UP_EPS = 0.1f;

	float follow_yaw_zero = 0.0f;

	bool follow_yaw_zero_ready = false;

	float offset_deg = 0.0f;

	float offset_rad = 0.0f;

	enum class DOWN_STAIRS_PHASE
	{
		GO_ZERO,
		GO_TARGET,
		HOLD_READY
	};

	DOWN_STAIRS_PHASE down_stairs_phase = DOWN_STAIRS_PHASE::GO_ZERO;

	uint32_t down_stairs_hold_time = 0;

	bool down_stairs_ready = false;      // 置位 = 姿态已摆好、底盘已交给摇杆

	constexpr float    DOWN_STAIRS_EPS = 0.1f;   // 到位判据(rad)，≈5.7°
	constexpr uint32_t DOWN_STAIRS_HOLD_MS = 300;   // 到位后确认稳定时间(ms)
	constexpr float    DOWN_STAIRS_MARGIN = 0.1f;   // 距行程端点余量(rad)

	constexpr float    DOWN_STAIRS_V_FRONT = 2.5f;  // 摆臂限速：前臂(0/2)
	constexpr float    DOWN_STAIRS_V_BACK = 4.0f;  // 摆臂限速：后臂(1/3)

	constexpr float    DOWN_STAIRS_SPD_X = 1500.f; // 底盘前后 满杆速度
	constexpr float    DOWN_STAIRS_SPD_Z = 700.f;  // 底盘自转 满杆速度
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
		ctrl.mode = CONTROL::DOWN_STAIRS;
		break;

	case RC_STATE(MID, UP):
		ctrl.mode = CONTROL::FOLLOW;
		break;

	case RC_STATE(MID, MID):
		ctrl.mode = CONTROL::RESET;
		break;

	case RC_STATE(MID, DOWN):
		ctrl.mode = CONTROL::SPINNING;
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

		return_zero_time = 0;

		return_zero_flag = false;
	}

	if (ctrl.mode != CONTROL::DOWN_STAIRS)
	{
		down_stairs_phase = DOWN_STAIRS_PHASE::GO_ZERO;

		down_stairs_hold_time = 0;

		down_stairs_ready = false;
	}

	if (ctrl.mode != CONTROL::RESET)
	{
		switch (ctrl.mode)
		{
		case CONTROL::ROTATION:

			break;

		case CONTROL::FOLLOW:
		{
			if (Shift_mode())
			{
				for (uint8_t i = 0; i < 4; i++)
				{
					DMmotor[i].setSpeed = 5.0f;
					DMmotor[i].SetTargetPos(DMmotor[i].pos);   // 只设一次
				}
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
				
				ctrl.chassis.speedx = rc.ch[3] * 2500.f / 660.f;
				ctrl.chassis.speedy = rc.ch[2] * 2500.f / 660.f;
			}

			

			DMmotor[PITCH].setSpeed = 3.0f;
			DMmotor[PITCH].SetTargetPos(DMmotor[PITCH].targetPos + rc.ch[1] / 660.0f * 0.01f);




			break;
		}

		case CONTROL::DOWN_STAIRS:
		{
			// ① 限速：每周期无条件重设，绝不出现 0
			for (uint8_t i = 0; i < 4; i++)
			{
				DMmotor[i].setSpeed = (i == 1 || i == 3) ? DOWN_STAIRS_V_BACK : DOWN_STAIRS_V_FRONT;
			}

			// ② 目标：从各自行程推出，反向到底并留余量（不可能被静默钳位）
			const float target[4] =
			{
				6.0f,
			  - 2.5f,
			  - 6.0f,
			    2.5f
			};

			// ③ 底盘先用 0 兜底；只有 HOLD_READY 才让摇杆接管
			ctrl.chassis.speedx = 0;
			ctrl.chassis.speedy = 0;
			ctrl.chassis.speedz = 0;

			switch (down_stairs_phase)
			{
			case DOWN_STAIRS_PHASE::GO_ZERO:
			{
				for (uint8_t i = 0; i < 4; i++)
				{
					DMmotor[i].SetTargetPos(0.0f);
				}

				bool arrived = true;

				for (uint8_t i = 0; i < 4; i++)
				{
					if (fabsf(DMmotor[i].pos) > DOWN_STAIRS_EPS)
					{
						arrived = false;
						break;
					}
				}

				if (arrived)
				{
					down_stairs_phase = DOWN_STAIRS_PHASE::GO_TARGET;
				}

				break;
			}

			case DOWN_STAIRS_PHASE::GO_TARGET:          // ← 摆臂过程，底盘锁 0
			{
				for (uint8_t i = 0; i < 4; i++)
				{
					DMmotor[i].SetTargetPos(target[i]);
				}

				bool arrived = true;

				for (uint8_t i = 0; i < 3; i++)
				{
					if (fabsf(DMmotor[i].pos - target[i]) > DOWN_STAIRS_EPS)
					{
						arrived = false;
						break;
					}
				}

				if (arrived)
				{
					if (down_stairs_hold_time == 0)
					{
						down_stairs_hold_time = HAL_GetTick();
					}

					if (HAL_GetTick() - down_stairs_hold_time >= DOWN_STAIRS_HOLD_MS)
					{
						down_stairs_hold_time = 0;
						down_stairs_ready = true;
						down_stairs_phase = DOWN_STAIRS_PHASE::HOLD_READY;
					}
				}
				else
				{
					down_stairs_hold_time = 0;
				}

				break;
			}

			case DOWN_STAIRS_PHASE::HOLD_READY:         // ← 姿态锁死 + 摇杆开底盘
			{
				for (uint8_t i = 0; i < 4; i++)
				{
					DMmotor[i].SetTargetPos(target[i]);
				}

				ctrl.chassis.speedx = rc.ch[2] * DOWN_STAIRS_SPD_X / 660.f;   // 左摇杆上下：前进/后退
				ctrl.chassis.speedz = rc.ch[0] * DOWN_STAIRS_SPD_Z / 660.f;   // 右摇杆左右：自转
				ctrl.chassis.speedy = 0;                                       // 下台阶不用横移

				break;
			}
			}

			break;
		}
			
		case CONTROL::SEPARATE:
		{
			break;
		}

		case CONTROL::AUTOAIM:
		{
			if (Shift_mode())
			{
				for (uint8_t i = 0; i < 4; i++)
				{
					DMmotor[i].setSpeed = 5.0f;
					DMmotor[i].SetTargetPos(DMmotor[i].pos);   // 只设一次
				}
			}
			ctrl.shooter.openRub = true;

			if (xuc.control_enable && xuc.Fresh() && imu_pantile.Fresh())
			{
				float yaw_err_deg = xuc.target_yaw - imu_pantile.GetAngleYaw();

				float pitch_err_deg = xuc.target_pitch - imu_pantile.GetAnglePitch();

				if (fabsf(yaw_err_deg) < 0.5f)
					yaw_err_deg = 0.0f;

				if (fabsf(pitch_err_deg) < 0.5f)
					pitch_err_deg = 0.0f;

				while (yaw_err_deg > 180.0f)
					yaw_err_deg -= 360.0f;

				while (yaw_err_deg < -180.0f)
					yaw_err_deg += 360.0f;

				float yaw_delta = - yaw_err_deg * 3.14f / 180.0f;
				float pitch_delta = - pitch_err_deg * 3.14f / 180.0f;

				DMmotor[YAW].setSpeed = 3.0f;
				DMmotor[PITCH].setSpeed = 2.0f;

				DMmotor[YAW].SetTargetPos(DMmotor[YAW].pos + yaw_delta);
				DMmotor[PITCH].SetTargetPos(DMmotor[PITCH].pos + pitch_delta);

				ctrl.shooter.supply_bullet = xuc.shoot_request;

			}
			else
			{
				DMmotor[YAW].setSpeed = 0.0f;
				DMmotor[PITCH].setSpeed = 0.0f;
				DMmotor[YAW].SetTargetPos(DMmotor[YAW].pos);
				DMmotor[YAW].setPos = DMmotor[YAW].pos;
				DMmotor[PITCH].SetTargetPos(DMmotor[PITCH].pos);
				DMmotor[PITCH].setPos = DMmotor[PITCH].pos;
				
				ctrl.shooter.supply_bullet = false;
			}

			// XUC shoot is a single-shot trigger in auto aim: 0 -> 1 queues one round.
			ctrl.shooter.flag = true;

				break;
		}
		case CONTROL::FIRE:
		{
			if (Shift_mode())
			{
				for (uint8_t i = 0; i < 4; i++)
				{
					DMmotor[i].setSpeed = 5.0f;
					DMmotor[i].SetTargetPos(DMmotor[i].pos);   // 只设一次
				}
			}

			ctrl.shooter.openRub = true;                    // 进 fire 就开摩擦轮
			ctrl.shooter.flag = true;                       // 进入fire为单发
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

		case CONTROL::STOP:


			break;

		case CONTROL::SPINNING:
		{
			if (Shift_mode())
			{
				for (uint8_t i = 0; i < 4; i++)
				{
					DMmotor[i].setSpeed = 5.0f;
					DMmotor[i].SetTargetPos(DMmotor[i].pos);   // 只设一次
				}
			}

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

				ctrl.chassis.speedx = 1200.0f;

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

				if(return_zero_flag == false)
				{
					if (return_zero_time == 0)
					{
						return_zero_time = HAL_GetTick();
					}

					if (HAL_GetTick() - return_zero_time >= 1000)
					{
						ctrl.chassis.speedx = 0.0f;
						return_zero_flag = true;
					}
					else
					{
						ctrl.chassis.speedx = 1300.0f;
					}
				}
				
				DMmotor[0].setSpeed = 25.0f;
				DMmotor[1].setSpeed = 30.0f;
				DMmotor[2].setSpeed = 25.0f;
				DMmotor[3].setSpeed = 30.0f;

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
			if (Shift_mode())
			{
				for (uint8_t i = 0; i < 4; i++)
				{
					DMmotor[i].setSpeed = 5.0f;
					DMmotor[i].SetTargetPos(DMmotor[i].pos);   // 只设一次
				}
			}

			ctrl.chassis.speedx = rc.ch[3] * 2500.f / 660.f;
			ctrl.chassis.speedy = rc.ch[2] * 2500.f / 660.f;
			ctrl.chassis.speedz = rc.ch[0] * 1500.f / 660.f;
			break;
		}

		case CONTROL::UP_STAIRS:
		{
			DMmotor[0].setSpeed = 30.0f;
			DMmotor[1].setSpeed = 30.0f;
			DMmotor[2].setSpeed = 30.0f;
			DMmotor[3].setSpeed = 30.0f;
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
		{
			ctrl.chassis.speedx = 0;
			ctrl.chassis.speedy = 0;
			ctrl.chassis.speedz = 0;

			if (Shift_mode())
			{
				for (uint8_t i = 0; i < 4; i++)
				{
					DMmotor[i].setSpeed = 5.0f;
					DMmotor[i].SetTargetPos(DMmotor[i].pos);
				}
			}

			break;
		}
		}
	}
	else 
	{
		for (uint8_t i = 0; i < 4; i++)
		{
			DMmotor[i].setSpeed = 5.0f;
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