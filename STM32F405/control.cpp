#include "control.h"
#include "tim.h"
#include "judgement.h"
#include "HTmotor.h"
#include "label.h"       // para
#include <cmath>         // sinf / cosf / fabsf
#include <algorithm>     // std::min / std::max（Ramp 在用）

namespace
{
	// 单位 rad，含义 = gimbal_yaw_sign * (yaw_raw - yaw0)，已归一到 [-pi, pi)
	volatile float g_gimbal_yaw_rad = 0.f;
	volatile bool  g_gimbal_yaw_fresh = false;

	float g_yaw_zero = 0.f;     // 上电后第一帧 = "云台正对车头"
	bool  g_yaw_zero_ready = false;

	// 只有这些模式允许云台朝向跟随；AUTO_UP / RESET / STOP / FIRE 不参与
	bool ModeAllowsKeepDir()
	{
		switch (ctrl.mode)
		{
		case CONTROL::FOLLOW:
			return true;
		default:
			return false;
		}
	}
}

// 由 DecodeTask 调用，喂入最新 yaw
void Chassis_UpdateGimbalYaw(float yaw_rad, bool fresh)
{
	g_gimbal_yaw_fresh = fresh;
	if (!fresh) return;

	if (!g_yaw_zero_ready)
	{
		g_yaw_zero = yaw_rad;
		g_yaw_zero_ready = true;
	}

	float d = yaw_rad - g_yaw_zero;
	while (d > 3.14159265f) d -= 6.28318531f;
	while (d < -3.14159265f) d += 6.28318531f;

	g_gimbal_yaw_rad = ctrl.chassis.gimbal_yaw_sign * d;
}

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
	Keep_Direction(0.f, 0.f);
}

void CONTROL::CHASSIS::Keep_Direction(float vx_in, float vy_in)
{
	speedx = (int32_t)vx_in;
	speedy = (int32_t)vy_in;
	speedz = 0;

	if (!keep_dir_enable) return;
	if (!ModeAllowsKeepDir()) return;
	if (!g_gimbal_yaw_fresh) return;   // IMU 掉线：退化成车体坐标，别乱转

	const float mount = mount_offset_deg * 3.14159265f / 180.f;
	const float yaw = g_gimbal_yaw_rad + mount;
	if (fabsf(yaw) > 3.0f) return;

	const float c = cosf(yaw);
	const float s = sinf(yaw);

	// IMU 的 x/y 轴与底盘(前/左)同轴同向 → 标准旋转 R(yaw)
	// (vx_in, vy_in) 是云台坐标系下的 (前, 左) 分量
	speedx = (int32_t)(keep_dir_gain * (c * vx_in - s * vy_in));
	speedy = (int32_t)(keep_dir_gain * (s * vx_in + c * vy_in));
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

