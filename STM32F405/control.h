#pragma once
#include <vector>
#include <cmath>
#include "stm32f4xx.h"
#include "motor.h"
#include "imu.h"

class CONTROL final
{
public:
	uint8_t init_DM = 0;
	Motor* chassis_motor[CHASSIS_MOTOR_NUM]{};
	Motor* pantile_motor[PANTILE_MOTOR_NUM]{};
	Motor* shooter_motor[SHOOTER_MOTOR_NUM]{};
	Motor* supply_motor[SUPPLY_MOTOR_NUM]{};
	
	enum MODE { PC, RC, AUTOAIM, RESET, ROTATION, SPINNING, FOLLOW, SEPARATE, FIRE, STOP,LOCK,CHASSIS_MOVE,UP_STAIRS,AUTO_UP} mode;
	struct CHASSIS
	{


		PID chassis_reset{};
		int32_t speedx{}, speedy{}, speedz{};

		//   → IMU 的 x/y 轴与底盘本体系（前/左）完全对齐
		bool  keep_dir_enable = true;
		float gimbal_yaw_sign = +1.f;  // 逆时针转 yaw_raw 增大 → +1；减小 → -1
		float mount_offset_deg = 0.f;   // IMU z 轴装偏补偿，本装法为 0
		float keep_dir_gain = 1.f;

		void Keep_Direction();
		void Keep_Direction(float vx_in, float vy_in);

		void Update();
		float Ramp(float setval, float curval, uint32_t RampSlope);
	};

	struct PANTILE
	{
		enum TYPE { YAW, PITCH };
		float mark_pitch{}, mark_yaw{};
		PID pantile_PID[3] = { {0.04f,0.f,0.f},{0.05f,0.f,0.f}, {0.f,0.f,0.f} };
		const float sensitivity = 2.5f;
		bool aim = false;
		void Keep_Pantile(float angleKeep, PANTILE::TYPE type, IMU frameOfReference);
		void Update();
	};

	struct SHOOTER
	{

		float now_bullet_speed = 0.f;

		int16_t rub_speed = 3000;  // 摩擦轮转子目标转速(rpm)，按实测调
		int16_t supply_speed = 100;  // 拨弹轮目标转速(rpm)
		uint8_t fire_rate = 8;     // 连发射频 Hz（配合 motor.spinning）

		bool auto_shoot = false;
		bool openRub = false, supply_bullet = false;
		bool fraction = false;
		bool fullheat_shoot = false;
		bool heat_ulimit = false;
		int16_t shoot_speed = 6000;

		bool    flag = false;

		float   bullets_per_rev = 9.f;   // 拨弹盘转一圈出弹数
		float   dial_gear_ratio = 36.f;  // 拨弹盘转一圈 = 转子转多少圈（M2006自带36:1，拨弹盘1:1接输出轴时=36）
		int16_t feed_speed = 500;        // 补一发时的转子转速(rpm)
		int32_t feed_tol = 300;        // 到位判据(counts)，取步长的 1/100 左右

		bool    feed_last_trig = false;  // 上一周期扳机状态（做上升沿）
		bool    feeding = false;  // 正在补一发
		int32_t feed_target = 0;      // 目标 sum_angle

		void Update();
	};

	CHASSIS chassis;
	PANTILE pantile;
	SHOOTER shooter;
	
	static int16_t Setrange(const int16_t original, const int16_t range);
	void Control_Pantile(int32_t ch_yaw, int32_t ch_pitch);
	float GetDelta(float delta);
	void Init(std::vector<Motor*> motor);
	void init_dm();
private:

};

extern CONTROL ctrl;

void Chassis_UpdateGimbalYaw(float yaw_rad, bool fresh);