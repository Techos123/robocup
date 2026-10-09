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
  Target_Angle_Initialized = false;
  Angle_Yaw = 0.0f;
  Yaw_Rate_Rad_S = 0.0f;
  Target_Angle = 0.0f;
  Resolved_Target_Omega = 0.0f;
  Chassis_Yaw_Control_Type = Chassis_Yaw_Control_Type_OMEGA;

  // 底盘yaw角度PID, 输出角速度
  PID_YawAngle.Init(0.35f, 0.0f, 0.0011f, 0.0f, 0.5f, 2.0f, 0.002f, 0.01f);
  PID_YawAngle.Reset();

  // 电机初始化
  Motor_Wheel[0].Init(&hfdcan3, Motor_DJI_ID_0x201, Motor_DJI_Control_Method_OMEGA, 3591.0f / 187.0f);
  Motor_Wheel[1].Init(&hfdcan3, Motor_DJI_ID_0x202, Motor_DJI_Control_Method_OMEGA, 3591.0f / 187.0f);
  Motor_Wheel[2].Init(&hfdcan3, Motor_DJI_ID_0x203, Motor_DJI_Control_Method_OMEGA, 3591.0f / 187.0f);
  Motor_Wheel[3].Init(&hfdcan3, Motor_DJI_ID_0x204, Motor_DJI_Control_Method_OMEGA, 3591.0f / 187.0f);

  for (int i = 0; i < 4; i++)
  {
    Target_Wheel_Omega[i] = 0.0f;
    Motor_Wheel[i].PID_Omega.Init(1.0f, 0.0f, 0.0f, 0.0f,
                                  Wheel_Torque_Max, Wheel_Torque_Max, 0.002f);
    Motor_Wheel[i].PID_Omega.Reset();
  }
}

void Class_Chassis_FW::Set_Wheel_Omega_PID(float __K_P, float __K_I, float __K_D)
{
  for (int i = 0; i < 4; i++)
  {
    Motor_Wheel[i].PID_Omega.Set_K_P(__K_P);
    Motor_Wheel[i].PID_Omega.Set_K_I(__K_I);
    Motor_Wheel[i].PID_Omega.Set_K_D(__K_D);
  }
}

void Class_Chassis_FW::Set_Wheel_Omega_PID_K_P(float __K_P)
{
  for (int i = 0; i < 4; i++)
  {
    Motor_Wheel[i].PID_Omega.Set_K_P(__K_P);
  }
}

void Class_Chassis_FW::Set_Wheel_Omega_PID_K_I(float __K_I)
{
  for (int i = 0; i < 4; i++)
  {
    Motor_Wheel[i].PID_Omega.Set_K_I(__K_I);
  }
}

void Class_Chassis_FW::Set_Wheel_Omega_PID_K_D(float __K_D)
{
  for (int i = 0; i < 4; i++)
  {
    Motor_Wheel[i].PID_Omega.Set_K_D(__K_D);
  }
}

void Class_Chassis_FW::Reset_Wheel_Omega_PID()
{
  for (int i = 0; i < 4; i++)
  {
    Motor_Wheel[i].PID_Omega.Reset(0.0f, Motor_Wheel[i].Get_Now_Omega());
  }
}

void Class_Chassis_FW::Set_Attitude_Feedback(const Struct_Chassis_Attitude_Feedback &__Feedback)
{
  if (!__Feedback.Valid ||
      Basic_Math_Is_Invalid_Float(__Feedback.Yaw_Deg) ||
      Basic_Math_Is_Invalid_Float(__Feedback.Yaw_Rate_Rad_S))
  {
    Attitude_Feedback_Valid = false;
    if (Chassis_Yaw_Control_Type == Chassis_Yaw_Control_Type_ANGLE)
    {
      Target_Angle_Initialized = false;
      Resolved_Target_Omega = 0.0f;
    }
    return;
  }

  Angle_Yaw = __Feedback.Yaw_Deg;
  Yaw_Rate_Rad_S = __Feedback.Yaw_Rate_Rad_S;
  Attitude_Feedback_Valid = true;

  if (Chassis_Yaw_Control_Type == Chassis_Yaw_Control_Type_ANGLE &&
      !Target_Angle_Initialized)
  {
    Target_Angle = Angle_Yaw;
    Target_Angle_Initialized = true;
    PID_YawAngle.Reset();
  }
}

void Class_Chassis_FW::Set_Chassis_Yaw_Control_Type(Enum_Chassis_Yaw_Control_Type __Chassis_Yaw_Control_Type)
{
  if (Chassis_Yaw_Control_Type == __Chassis_Yaw_Control_Type)
  {
    return;
  }

  Chassis_Yaw_Control_Type = __Chassis_Yaw_Control_Type;
  Resolved_Target_Omega = 0.0f;

  PID_YawAngle.Reset();

  if (Chassis_Yaw_Control_Type == Chassis_Yaw_Control_Type_ANGLE)
  {
    if (Attitude_Feedback_Valid)
    {
      Target_Angle = Angle_Yaw;
      Target_Angle_Initialized = true;
      PID_YawAngle.Reset();
    }
    else
    {
      Target_Angle_Initialized = false;
    }
  }
  else
  {
    Target_Angle_Initialized = false;
  }
}

float Class_Chassis_FW::Get_Shortest_Yaw_Error(float target_yaw, float current_yaw)
{
  float error = current_yaw - target_yaw;

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
  Resolve_Target_Omega();

  Kinematics_Inverse_Resolution();

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
 * @brief 根据航向控制模式解算目标角速度
 *
 */
void Class_Chassis_FW::Resolve_Target_Omega()
{
  switch (Chassis_Control_Type)
  {
    case (Chassis_Control_Type_DISABLE):
    {
      PID_YawAngle.Set_Integral_Error(0.0f);
      Resolved_Target_Omega = 0.0f;
      break;
    }
    case (Chassis_Control_Type_NORMAL):
    {
      switch (Chassis_Yaw_Control_Type)
      {
        case (Chassis_Yaw_Control_Type_OMEGA):
        {
          Resolved_Target_Omega = Target_Omega;
          break;
        }
        case (Chassis_Yaw_Control_Type_ANGLE):
        {
          if (!Attitude_Feedback_Valid || !Target_Angle_Initialized)
          {
            PID_YawAngle.Set_Integral_Error(0.0f);
            Resolved_Target_Omega = 0.0f;
            break;
          }
          float yaw_error = Get_Shortest_Yaw_Error(Target_Angle, Angle_Yaw);
          PID_YawAngle.Set_Target(yaw_error);
          PID_YawAngle.Set_Now(0.0f);
          PID_YawAngle.TIM_Calculate_PeriodElapsedCallback();

          Resolved_Target_Omega = PID_YawAngle.Get_Out();
          break;
        }
      }
      break;
    }
  }
}

/**
 * @brief 运动学逆解算
 *
 */
void Class_Chassis_FW::Kinematics_Inverse_Resolution()
{
  if (Chassis_Control_Type == Chassis_Control_Type_DISABLE)
  {
    for (int i = 0; i < 4; i++)
    {
      Target_Wheel_Omega[i] = 0.0f;
    }
    return;
  }

  const float omega_from_x = Target_Velocity_X / Wheel_Radius;
  const float omega_from_yaw = Resolved_Target_Omega * Wheel_Point_Y / Wheel_Radius;

  Target_Wheel_Omega[0] = +omega_from_x - omega_from_yaw;
  Target_Wheel_Omega[1] = +omega_from_x - omega_from_yaw;
  Target_Wheel_Omega[2] = -omega_from_x - omega_from_yaw;
  Target_Wheel_Omega[3] = -omega_from_x - omega_from_yaw;
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
        Motor_Wheel[i].Set_Control_Method(Motor_DJI_Control_Method_TORQUE);
        Motor_Wheel[i].Set_Target_Omega(0.0f);
        Motor_Wheel[i].Set_Target_Torque(0.0f);
        Motor_Wheel[i].PID_Omega.Reset(0.0f, Motor_Wheel[i].Get_Now_Omega());
      }

      break;
    }
    case (Chassis_Control_Type_NORMAL):
    {

      for (int i = 0; i < 4; i++)
      {
        Motor_Wheel[i].Set_Control_Method(Motor_DJI_Control_Method_OMEGA);
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
