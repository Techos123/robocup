#include "crt_fw.h"
/**
 * @brief 轮组编号
 * 1[0] 4[3]
 * 2[1] 3[2]
 * 前x左y上z
 */

/**
 * @brief 底盘初始化
 *
 * @param __Chassis_Control_Type 四轮底盘控制方法
 * @param __Speed 底盘速度限制最大值
 */
void Class_Chassis_FW::Init()
{
  Attitude_Feedback_Valid = false;
  Attitude_Feedback_Initialized = false;
  Angle_Yaw = 0.0f;
  Yaw_Rate_Rad_S = 0.0f;
  Target_Angle = 0.0f;

  // PID初始化

  // 底盘速度xPID, 输出摩擦力
  PID_Velocity_X.Init(7.0f, 0.0f, 0.0f, 1.3f, 2.0f, 1.5f, 0.001f);

  // 底盘角速度PID, 输出扭矩
  PID_Omega.Init(4.0f, 0.0f, 0.0f, 2.3f, 2.0f, 1.5f, 0.001f);

  // 底盘yaw角速度PID, 输出扭矩
  PID_YawSpeed.Init(4.9f, 0.0f, 0.0f, 0.0f, 0.5f, 1.5f, 0.001f);

  // 底盘yaw角度PID, 输出角速度
  PID_YawAngle.Init(7.4f, 2.4f, 0.0f, 0.2f, 1.5f, 0.5f, 0.002f, 0.001f);

  // 电机初始化
  Motor_Wheel[0].Init(&hfdcan3, Motor_DJI_ID_0x201, Motor_DJI_Control_Method_OMEGA, 3591.0f / 187.0f);
  Motor_Wheel[0].PID_Omega.Init(0.4f, 0.0f, 0.0f, 0.0f, 2.0f, 1.5f, 0.001f);
  // 电机初始化
  Motor_Wheel[1].Init(&hfdcan3, Motor_DJI_ID_0x202, Motor_DJI_Control_Method_OMEGA, 3591.0f / 187.0f);
  Motor_Wheel[1].PID_Omega.Init(0.4f, 0.0f, 0.0f, 0.0f, 2.0f, 1.5f, 0.001f);

  // 电机初始化
  Motor_Wheel[2].Init(&hfdcan3, Motor_DJI_ID_0x203, Motor_DJI_Control_Method_OMEGA, 3591.0f / 187.0f);
  Motor_Wheel[2].PID_Omega.Init(0.4f, 0.0f, 0.0f, 0.0f, 2.0f, 1.5f, 0.001f);

  // 电机初始化
  Motor_Wheel[3].Init(&hfdcan3, Motor_DJI_ID_0x204, Motor_DJI_Control_Method_OMEGA, 3591.0f / 187.0f);
  Motor_Wheel[3].PID_Omega.Init(0.4f, 0.0f, 0.0f, 0.0f, 2.0f, 1.5f, 0.001f);
}

void Class_Chassis_FW::Set_Attitude_Feedback(const Struct_Chassis_Attitude_Feedback &__Feedback)
{
  if (!__Feedback.Valid ||
      Basic_Math_Is_Invalid_Float(__Feedback.Yaw_Deg) ||
      Basic_Math_Is_Invalid_Float(__Feedback.Yaw_Rate_Rad_S))
  {
    Attitude_Feedback_Valid = false;
    return;
  }

  if (!Attitude_Feedback_Initialized)
  {
    Target_Angle = __Feedback.Yaw_Deg;
    Attitude_Feedback_Initialized = true;
  }

  Angle_Yaw = __Feedback.Yaw_Deg;
  Yaw_Rate_Rad_S = __Feedback.Yaw_Rate_Rad_S;
  Attitude_Feedback_Valid = true;
}

float Class_Chassis_FW::Get_Shortest_Yaw_Error(float target_yaw, float current_yaw)
{
  float error = target_yaw - current_yaw;

  while (error > 180.0f)
  {
    error -= 360.0f;
  }
  while (error < -180.0f)
  {
    error += 360.0f;
  }

  return error;
}

/**
 * @brief TIM定时器中断定期检测电机是否存活
 *
 */
void Class_Chassis_FW::TIM_100ms_Alive_PeriodElapsedCallback()
{
  for (int i = 0; i < 4; i++)
  {
    Motor_Wheel[i].TIM_100ms_Alive_PeriodElapsedCallback();
  }
}

/**
 * @brief TIM定时器中断解算回调函数
 *
 */
void Class_Chassis_FW::TIM_2ms_Resolution_PeriodElapsedCallback()
{
  Self_Resolution();
}

/**
 * @brief TIM定时器中断控制回调函数
 *
 */
void Class_Chassis_FW::TIM_2ms_Control_PeriodElapsedCallback()
{
  Output_To_Dynamics();

  // Kinematics_Inverse_Resolution();

  Dynamics_Inverse_Resolution();

  Output_To_Motor();
}

/**
 * @brief 自身解算
 *
 */
void Class_Chassis_FW::Self_Resolution()
{

  // 解算底盘速度与角速度
  Now_Velocity_X = (Motor_Wheel[0].Get_Now_Omega() + Motor_Wheel[1].Get_Now_Omega() - Motor_Wheel[2].Get_Now_Omega() - Motor_Wheel[3].Get_Now_Omega()) * Wheel_Radius / 4.0f;

  Now_Omega = -(Motor_Wheel[0].Get_Now_Omega() + Motor_Wheel[1].Get_Now_Omega() + Motor_Wheel[2].Get_Now_Omega() + Motor_Wheel[3].Get_Now_Omega()) * Wheel_Radius / Wheel_Point_Y / 4.0f;
}

/**
 * @brief 输出到动力学状态
 *
 */
void Class_Chassis_FW::Output_To_Dynamics()
{
  switch (Chassis_Control_Type)
  {
    case (Chassis_Control_Type_DISABLE):
    {

      PID_Velocity_X.Set_Integral_Error(0.0f);
      PID_Omega.Set_Integral_Error(0.0f);
      PID_YawSpeed.Set_Integral_Error(0.0f);
      PID_YawAngle.Set_Integral_Error(0.0f);
      break;
    }
    case (Chassis_Control_Type_NORMAL):
    {

      PID_Velocity_X.Set_Target(Target_Velocity_X);
      PID_Velocity_X.Set_Now(Now_Velocity_X);
      PID_Velocity_X.TIM_Calculate_PeriodElapsedCallback();

      PID_Omega.Set_Target(Target_Omega);
      PID_Omega.Set_Now(Now_Omega);
      PID_Omega.TIM_Calculate_PeriodElapsedCallback();

      if (Target_Angle == 0.0f)
      {
        if (Target_Omega == 0.0f)
        {
          PID_YawSpeed.Set_Target(0.0f);
          PID_YawSpeed.Set_Now(Yaw_Rate_Rad_S);
          PID_YawSpeed.TIM_Calculate_PeriodElapsedCallback();
          Target_Angle = Angle_Yaw;
        }
        else
        {
          PID_YawSpeed.Set_Integral_Error(0.0f);
          PID_YawSpeed.TIM_Calculate_PeriodElapsedCallback();
          Target_Angle = Angle_Yaw;
        }
      }
      else
      {
        PID_YawAngle.Set_Integral_Error(Get_Shortest_Yaw_Error(Target_Angle, Angle_Yaw) * 0.1f);
        PID_YawAngle.TIM_Calculate_PeriodElapsedCallback();
        PID_YawSpeed.Set_Target(PID_YawAngle.Get_Out());
        PID_YawSpeed.Set_Now(Yaw_Rate_Rad_S);
        PID_YawSpeed.TIM_Calculate_PeriodElapsedCallback();
      }
      break;
    }
  }
}

/**
 * @brief 动力学逆解算
 *
 */
void Class_Chassis_FW::Dynamics_Inverse_Resolution()
{
  float omega_x, omega_z;

  omega_x = PID_Velocity_X.Get_Out();
  omega_z = PID_Omega.Get_Out() + PID_YawSpeed.Get_Out();

  // 每个轮的扭力
  float tmp_omega[4];
  // 解算到每个轮组的具体摩擦力
  tmp_omega[0] = (+omega_x - omega_z * Wheel_Point_Y) / Wheel_Radius;
  tmp_omega[1] = (+omega_x - omega_z * Wheel_Point_Y) / Wheel_Radius;
  tmp_omega[2] = (-omega_x - omega_z * Wheel_Point_Y) / Wheel_Radius;
  tmp_omega[3] = (-omega_x - omega_z * Wheel_Point_Y) / Wheel_Radius;
  for (int i = 0; i < 4; i++)
  {
    // 摩擦力转换至扭矩
    Target_Wheel_Omega[i] = tmp_omega[i];
  }
}

/**
 * @brief 输出到电机
 *
 */
void Class_Chassis_FW::Output_To_Motor()
{
  switch (Chassis_Control_Type)
  {
    case (Chassis_Control_Type_DISABLE):
    {
      // 底盘失能
      for (int i = 0; i < 4; i++)
      {

        Motor_Wheel[i].Set_Target_Omega(0.0f);
      }

      break;
    }
    case (Chassis_Control_Type_NORMAL):
    {

      for (int i = 0; i < 4; i++)
      {
        Motor_Wheel[i].Set_Target_Omega(Target_Wheel_Omega[i]);
      }

      break;
    }
  }

  for (int i = 0; i < 4; i++)
  {
    Motor_Wheel[i].TIM_Calculate_PeriodElapsedCallback();
  }
}
