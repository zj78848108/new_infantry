#include "control.h"
#include "tim.h"
#include "judgement.h"
#include "HTmotor.h"
#include "label.h"       // para
#include <cmath>         // sinf / cosf / fabsf
#include <algorithm>	// std::min / std::max（Ramp 在用）
#include "rc.h"
#include "xuc.h"

//namespace
//{
//	// 单位 rad，含义 = gimbal_yaw_sign * (yaw_raw - yaw0)，已归一到 [-pi, pi)
//	volatile float g_gimbal_yaw_rad = 0.f;
//	volatile bool  g_gimbal_yaw_fresh = false;
//
//	float g_yaw_zero = 0.f;     // 上电后第一帧 = "云台正对车头"
//	bool  g_yaw_zero_ready = false;
//
//	// 只有这些模式允许云台朝向跟随；AUTO_UP / RESET / STOP / FIRE 不参与
//	bool ModeAllowsKeepDir()
//	{
//		switch (ctrl.mode)
//		{
//		case CONTROL::FOLLOW:
//			return true;
//		default:
//			return false;
//		}
//	}
//}
//
//// 由 DecodeTask 调用，喂入最新 yaw
//void Chassis_UpdateGimbalYaw(float yaw_rad, bool fresh)
//{
//	g_gimbal_yaw_fresh = fresh;
//	if (!fresh) return;
//
//	if (!g_yaw_zero_ready)
//	{
//		g_yaw_zero = yaw_rad;
//		g_yaw_zero_ready = true;
//	}
//
//	float d = yaw_rad - g_yaw_zero;
//	while (d > 3.14159265f) d -= 6.28318531f;
//	while (d < -3.14159265f) d += 6.28318531f;
//
//	g_gimbal_yaw_rad = ctrl.chassis.gimbal_yaw_sign * d;
//}

void CONTROL::Init(std::vector<Motor*> motor)
{
	int num1{}, num2{}, num3{}, num4{};
	for (int i = 0; i < motor.size(); i++)
	{
		switch (motor[i]->function)
		{
		case(function_type::chassis):
			chassis_motor[num1++] = motor[i];
			break;
		case(function_type::pantile):
			pantile_motor[num2++] = motor[i];
			break;
		case(function_type::shooter):
			shooter_motor[num3++] = motor[i];
			break;
		case(function_type::supply):
			supply_motor[num4] = motor[i];
			supply_motor[num4]->spinning = false;
			supply_motor[num4]->need_curcircle = false;
			num4++;
			break;
		default:
			break;
		}
	}
	pantile_motor[PANTILE::TYPE::PITCH]->setangle = para.initial_pitch;
	pantile_motor[PANTILE::TYPE::YAW]->setangle = para.initial_yaw;
}


void CONTROL::Control_Pantile(int32_t ch_yaw, int32_t ch_pitch)
{
	
}

void CONTROL::PANTILE::Keep_Pantile(float angleKeep, PANTILE::TYPE type,IMU frameOfReference)
{
	
}

void CONTROL::CHASSIS::Keep_Direction()
{
	
}



void CONTROL::CHASSIS::Update()
{
	ctrl.chassis_motor[0]->setspeed = +speedx + speedy + speedz;
	ctrl.chassis_motor[1]->setspeed = -speedx + speedy + speedz;
	ctrl.chassis_motor[2]->setspeed = -speedx - speedy + speedz;
	ctrl.chassis_motor[3]->setspeed = +speedx - speedy + speedz;

}

void CONTROL::PANTILE::Update()
{
	/*if (ctrl.mode == CONTROL::FIRE
		|| ctrl.mode == CONTROL::SPINNING
		|| ctrl.mode == CONTROL::SEPARATE
		|| ctrl.mode == CONTROL::FOLLOW)
	{*/

		//if (rc.Shift_mode())
		//{
		//	DMmotor[YAW].SetTargetPos(DMmotor[YAW].pos);
		//	DMmotor[YAW].setPos = DMmotor[YAW].pos;

		//	DMmotor[PITCH].SetTargetPos(DMmotor[PITCH].pos);
		//	DMmotor[PITCH].setPos = DMmotor[PITCH].pos;
		//}//只在切入模式时执行一次，防止电机突转

		//DMmotor[YAW].setSpeed = 5.0f;

		//DMmotor[YAW].SetTargetPos(DMmotor[YAW].targetPos + rc.rc.ch[0] / 660.0f * 0.01f);

		//DMmotor[PITCH].setSpeed = 3.0f;
		//DMmotor[PITCH].SetTargetPos(DMmotor[PITCH].targetPos + rc.rc.ch[1] / 660.0f * 0.01f);

		/*ctrl.chassis.speedx = rc.rc.ch[0] * 1500.f / 660.f;*/
	//}

	//if (ctrl.mode == CONTROL::RESET)
	//{
	//	DMmotor[YAW].setSpeed = 2.0f;
	//	DMmotor[YAW].SetTargetPos(0.0f);
	//	DMmotor[PITCH].setSpeed = 1.0f;
	//	DMmotor[PITCH].SetTargetPos(0.0f);
	//}

	//if (ctrl.mode == CONTROL::AUTOAIM)
	//{
	//	// 只接受：上位机允许控制 + 数据未超时 的目标
	//	if (xuc.control_enable && xuc.Fresh())
	//	{
	//		DMmotor[YAW].setSpeed = 5.0f;
	//		DMmotor[YAW].SetTargetPos(xuc.target_yaw);

	//		DMmotor[PITCH].setSpeed = 3.0f;
	//		DMmotor[PITCH].SetTargetPos(xuc.target_pitch);
	//	}
	//	else
	//	{
	//		// 无有效上位机指令：保持进入 AUTOAIM 时的位置，防止突然转动
	//		if (rc.Shift_mode())
	//		{
	//			DMmotor[YAW].SetTargetPos(DMmotor[YAW].pos);
	//			DMmotor[YAW].setPos = DMmotor[YAW].pos;

	//			DMmotor[PITCH].SetTargetPos(DMmotor[PITCH].pos);
	//			DMmotor[PITCH].setPos = DMmotor[PITCH].pos;
	//		}
	//	}
	//}
}

void CONTROL::SHOOTER::Update()
{
	const bool shooting = (ctrl.mode == CONTROL::FIRE || ctrl.mode == CONTROL::SPINNING);

	// 两个摩擦轮：进 fire 模式且 openRub 才转，离开立刻停
	const int16_t sp = (shooting) ? rub_speed : 0;
	for (uint8_t i = 0; i < SHOOTER_MOTOR_NUM; i++)
		if (ctrl.shooter_motor[i] != nullptr)
			ctrl.shooter_motor[i]->setspeed = (i == 1) ? -sp : sp;  // 对装反向时取负

	if (ctrl.shooter.flag) {
		Motor* const sm = ctrl.supply_motor[0];
		if (sm != nullptr)
		{
			// 每颗弹的转子编码器步长 = 8192(转子一圈) × 减速比 ÷ 一圈弹数
			const int32_t step = static_cast<int32_t>(
				8192.f * dial_gear_ratio / bullets_per_rev + 0.5f);

			const bool trig = shooting && supply_bullet;

			if (!shooting)
			{
				feeding = false;
				feed_last_trig = false;
				sm->setspeed = 0;
			}
			else
			{
				if (trig && !feed_last_trig)      // 扳机上升沿 → 只排一发
					feed_target = sm->sum_angle + step, feeding = true;

				feed_last_trig = trig;

				if (feeding)
				{
					const int32_t err = feed_target - sm->sum_angle;
					if (err <= feed_tol && err >= -feed_tol)   // 到位
					{
						feeding = false;
						sm->setspeed = 0;
					}
					else
						sm->setspeed = (err > 0) ? feed_speed : -feed_speed;
				}
				else
					sm->setspeed = 0;
			}
		}
	}
	else {
		// 拨弹轮：进 fire 模式且 supply_bullet 才转，离开立刻停
		if (ctrl.supply_motor[0] != nullptr)
			ctrl.supply_motor[0]->setspeed = (shooting && supply_bullet) ? supply_speed : 0;
	}
}

float CONTROL::CHASSIS::Ramp(float setval, float curval, uint32_t RampSlope)
{

	if ((setval - curval) >= 0)
	{
		curval += RampSlope;
		curval = std::min(curval, setval);
	}
	else
	{
		curval -= RampSlope;
		curval = std::max(curval, setval);
	}

	return curval;
}

float CONTROL::GetDelta(float delta)
{
	if (delta <= -180.f)
	{
		delta += 360.f;
	}

	if (delta > 180.f)
	{
		delta -= 360.f;
	}
	return delta;
}

int16_t CONTROL::Setrange(const int16_t original, const int16_t range)
{
	return fmaxf(fminf(range, original), -range);
}

