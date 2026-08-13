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
  Output_Moment_Z = 0.0f;
  Chassis_Yaw_Control_Type = Chassis_Yaw_Control_Type_ANGLE;

  // 底盘速度xPID, 输出摩擦力
  PID_Velocity_X.Init(1.0f, 0.0f, 0.0f, 1.3f, 2.0f, 20.0f, 0.002f);

  // 底盘yaw角速度PID, 输出扭矩
  PID_YawSpeed.Init(1.0f, 0.0f, 0.0f, 0.0f, 0.5f, 5.0f, 0.002f);

  // 底盘yaw角度PID, 输出角速度
  PID_YawAngle.Init(0.1f, 0.0f, 0.0f, 0.2f, 1.5f, 5.0f, 0.002f, 0.01f);

  PID_Velocity_X.Reset();

  PID_YawSpeed.Reset();
  PID_YawAngle.Reset();

  for (int i = 0; i < 4; i++)
  {
    Target_Wheel_Torque[i] = 0.0f;
  }

  // 电机初始化
  Motor_Wheel[0].Init(&hfdcan3, Motor_DJI_ID_0x201, Motor_DJI_Control_Method_TORQUE, 3591.0f / 187.0f);
  // 电机初始化
  Motor_Wheel[1].Init(&hfdcan3, Motor_DJI_ID_0x202, Motor_DJI_Control_Method_TORQUE, 3591.0f / 187.0f);

  // 电机初始化
  Motor_Wheel[2].Init(&hfdcan3, Motor_DJI_ID_0x203, Motor_DJI_Control_Method_TORQUE, 3591.0f / 187.0f);

  // 电机初始化
  Motor_Wheel[3].Init(&hfdcan3, Motor_DJI_ID_0x204, Motor_DJI_Control_Method_TORQUE, 3591.0f / 187.0f);
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
      Output_Moment_Z = 0.0f;
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
    PID_YawSpeed.Reset(0.0f, Yaw_Rate_Rad_S);
  }
}

void Class_Chassis_FW::Set_Chassis_Yaw_Control_Type(Enum_Chassis_Yaw_Control_Type __Chassis_Yaw_Control_Type)
{
  if (Chassis_Yaw_Control_Type == __Chassis_Yaw_Control_Type)
  {
    return;
  }

  Chassis_Yaw_Control_Type = __Chassis_Yaw_Control_Type;
  Output_Moment_Z = 0.0f;

  PID_YawSpeed.Reset(0.0f, Yaw_Rate_Rad_S);
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

      PID_YawSpeed.Set_Integral_Error(0.0f);
      PID_YawAngle.Set_Integral_Error(0.0f);
      Output_Moment_Z = 0.0f;
      break;
    }
    case (Chassis_Control_Type_NORMAL):
    {

      PID_Velocity_X.Set_Target(Target_Velocity_X);
      PID_Velocity_X.Set_Now(Now_Velocity_X);
      PID_Velocity_X.TIM_Calculate_PeriodElapsedCallback();

      switch (Chassis_Yaw_Control_Type)
      {
        case (Chassis_Yaw_Control_Type_OMEGA):
        {

          PID_YawSpeed.Set_Target(Target_Omega);
          PID_YawSpeed.Set_Now(0.4f * Now_Omega + 0.6f * Yaw_Rate_Rad_S);
          PID_YawSpeed.TIM_Calculate_PeriodElapsedCallback();
          Output_Moment_Z = PID_YawSpeed.Get_Out();
          break;
        }
        case (Chassis_Yaw_Control_Type_ANGLE):
        {
          if (!Attitude_Feedback_Valid || !Target_Angle_Initialized)
          {
            PID_YawAngle.Set_Integral_Error(0.0f);
            PID_YawSpeed.Set_Integral_Error(0.0f);
            Output_Moment_Z = 0.0f;
            break;
          }
          float yaw_error = Get_Shortest_Yaw_Error(Target_Angle, Angle_Yaw);
          PID_YawAngle.Set_Target(yaw_error);
          PID_YawAngle.Set_Now(0.0f);
          PID_YawAngle.TIM_Calculate_PeriodElapsedCallback();

          PID_YawSpeed.Set_Target(PID_YawAngle.Get_Out());
          PID_YawSpeed.Set_Now(0.4f * Now_Omega + 0.6f * Yaw_Rate_Rad_S);
          PID_YawSpeed.TIM_Calculate_PeriodElapsedCallback();
          Output_Moment_Z = PID_YawSpeed.Get_Out();
          break;
        }
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

  float force_x, moment_z;
  float torque_from_x, torque_from_yaw;

  force_x = PID_Velocity_X.Get_Out();
  moment_z = Output_Moment_Z;

  torque_from_x = force_x * Wheel_Radius / 4.0f;
  torque_from_yaw = moment_z * Wheel_Radius / (4.0f * Wheel_Point_Y);

  // 每个轮的扭力
  float tmp_torque[4];
  // 解算到每个轮组的具体摩擦力
  tmp_torque[0] = +torque_from_x - torque_from_yaw;
  tmp_torque[1] = +torque_from_x - torque_from_yaw;
  tmp_torque[2] = -torque_from_x - torque_from_yaw;
  tmp_torque[3] = -torque_from_x - torque_from_yaw;

  for (int i = 0; i < 4; i++)
  {
    // 摩擦力转换至扭矩
    Target_Wheel_Torque[i] = tmp_torque[i];
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

        Motor_Wheel[i].Set_Target_Torque(0.0f);
      }

      break;
    }
    case (Chassis_Control_Type_NORMAL):
    {

      for (int i = 0; i < 4; i++)
      {
        Motor_Wheel[i].Set_Target_Torque(Target_Wheel_Torque[i]);
      }

      break;
    }
  }

  for (int i = 0; i < 4; i++)
  {
    Motor_Wheel[i].TIM_Calculate_PeriodElapsedCallback();
  }
}
