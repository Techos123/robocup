#ifndef CRT_FW_H
#define CRT_FW_H

#include "alg_basic.h"
#include "dvc_motor_dji.h"

/**
 * @brief 底盘控制类型
 *
 */
enum Enum_Chassis_Control_Type
{
  Chassis_Control_Type_DISABLE = 0,
  Chassis_Control_Type_NORMAL,
};

/**
 * @brief 底盘航向控制类型
 *
 */
enum Enum_Chassis_Yaw_Control_Type
{
  Chassis_Yaw_Control_Type_OMEGA = 0,
  Chassis_Yaw_Control_Type_ANGLE,
};

struct Struct_Chassis_Attitude_Feedback
{
  float Yaw_Deg = 0.0f;
  float Yaw_Rate_Rad_S = 0.0f;
  bool Valid = false;
};

class Class_Chassis_FW
{
public:
  // 底盘yaw角度PID（外层角度环）
  Class_PID PID_YawAngle;

  // 底盘电机
  Class_Motor_DJI_C620 Motor_Wheel[4];

  inline Enum_Chassis_Control_Type Get_Chassis_Control_Type();
  inline Enum_Chassis_Yaw_Control_Type Get_Chassis_Yaw_Control_Type();

  inline float Get_Now_Velocity_X();
  inline float Get_Now_Omega();
  inline float Get_Now_Current_Sum() const;

  inline float Get_Target_Velocity_X();
  inline float Get_Target_Omega();

  inline void Set_Target_Velocity_X(float __Target_Velocity_X);
  inline void Set_Target_Omega(float __Target_Omega);
  inline void Set_Target_Angle(float __Target_Angle);

  inline void Set_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type);
  void Set_Chassis_Yaw_Control_Type(Enum_Chassis_Yaw_Control_Type __Chassis_Yaw_Control_Type);

  void Init();
  void Set_Wheel_Omega_PID(float __K_P, float __K_I, float __K_D);
  void Set_Wheel_Omega_PID_K_P(float __K_P);
  void Set_Wheel_Omega_PID_K_I(float __K_I);
  void Set_Wheel_Omega_PID_K_D(float __K_D);
  void Reset_Wheel_Omega_PID();
  void Set_Attitude_Feedback(const Struct_Chassis_Attitude_Feedback &__Feedback);
  void TIM_100ms_Alive_PeriodElapsedCallback();
  void TIM_2ms_Resolution_PeriodElapsedCallback();
  void TIM_2ms_Control_PeriodElapsedCallback();

protected:
  // 初始化相关常量

  // 轮组半径
  const float Wheel_Radius = 0.0525f;
  // 轮组投影点坐标x
  const float Wheel_Point_X = 0.100f;
  // 轮组投影点坐标y
  const float Wheel_Point_Y = 0.150f;
  // C620最大轮端目标扭矩，对应4A电流指令
  const float Wheel_Torque_Max = 1.2f;

  // 当前速度X
  float Now_Velocity_X = 0.0f;
  // 当前速度Y
  float Now_Velocity_Y = 0.0f;
  // 当前角速度
  float Now_Omega = 0.0f;

  // 目标速度X
  float Target_Velocity_X = 0.0f;
  // 目标速度Y
  float Target_Velocity_Y = 0.0f;
  // 目标角速度
  float Target_Omega = 0.0f;
  // 目标航向角（平动时用于锁定航向）
  float Target_Angle = 0.0f;

  // 底盘相对Odom角度
  float Angle_Pitch = 0.0f;
  float Angle_Roll = 0.0f;
  float Angle_Yaw = 0.0f;
  float Yaw_Rate_Rad_S = 0.0f;
  bool Attitude_Feedback_Valid = false;
  bool Target_Angle_Initialized = false;

  // 航向模式解算后的目标角速度
  float Resolved_Target_Omega = 0.0f;

  // 轮向电机角速度目标值
  float Target_Wheel_Omega[4] = {};

  // 底盘控制方法
  Enum_Chassis_Control_Type Chassis_Control_Type = Chassis_Control_Type_DISABLE;
  // 底盘航向控制方法
  Enum_Chassis_Yaw_Control_Type Chassis_Yaw_Control_Type = Chassis_Yaw_Control_Type_OMEGA;

  void Self_Resolution();

  void Resolve_Target_Omega();

  void Kinematics_Inverse_Resolution();

  void Output_To_Motor();

  float Get_Shortest_Yaw_Error(float target_yaw, float current_yaw);
};

/**
 * @brief 获取当前速度X
 *
 * @return float 当前速度X
 */
inline float
Class_Chassis_FW::Get_Now_Velocity_X()
{
  return (Now_Velocity_X);
}

/**
 * @brief 获取当前角速度
 *
 * @return float 当前角速度
 */
inline float Class_Chassis_FW::Get_Now_Omega()
{
  return (Now_Omega);
}

/**
 * @brief 获取四个电机当前电流绝对值之和
 *
 * @return float 四个电机当前电流绝对值之和, A
 */
inline float Class_Chassis_FW::Get_Now_Current_Sum() const
{
  float Current_Sum = 0.0f;

  for (int i = 0; i < 4; i++)
  {
    const float Current = Motor_Wheel[i].Get_Now_Current();
    Current_Sum += (Current < 0) ? -Current : Current;
  }

  return (Current_Sum);
}

/**
 * @brief 获取底盘控制方法
 *
 * @return Enum_Chassis_Control_Type 底盘控制方法
 */
inline Enum_Chassis_Control_Type Class_Chassis_FW::Get_Chassis_Control_Type()
{
  return (Chassis_Control_Type);
}

/**
 * @brief 获取底盘航向控制方法
 *
 * @return Enum_Chassis_Yaw_Control_Type 底盘航向控制方法
 */
inline Enum_Chassis_Yaw_Control_Type Class_Chassis_FW::Get_Chassis_Yaw_Control_Type()
{
  return (Chassis_Yaw_Control_Type);
}

/**
 * @brief 获取目标速度X
 *
 * @return float 目标速度X
 */
inline float Class_Chassis_FW::Get_Target_Velocity_X()
{
  return (Target_Velocity_X);
}

/**
 * @brief 获取目标速度方向
 *
 * @return float 目标速度方向
 */
inline float Class_Chassis_FW::Get_Target_Omega()
{
  return (Target_Omega);
}

/**
 * @brief 设定底盘控制方法
 *
 * @param __Chassis_Control_Type 底盘控制方法
 */
inline void Class_Chassis_FW::Set_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type)
{
  Chassis_Control_Type = __Chassis_Control_Type;
}

/**
 * @brief 设定目标速度X
 *
 * @param __Target_Velocity_X 目标速度X
 */
inline void Class_Chassis_FW::Set_Target_Velocity_X(float __Target_Velocity_X)
{
  Target_Velocity_X = __Target_Velocity_X;
}

/**
 * @brief 设定目标角速度
 *
 * @param __Target_Omega 目标角速度
 */
inline void Class_Chassis_FW::Set_Target_Omega(float __Target_Omega)
{
  Target_Omega = __Target_Omega;
}

inline void Class_Chassis_FW::Set_Target_Angle(float __Target_Angle)
{
  Target_Angle = __Target_Angle;
  Target_Angle_Initialized = true;
}

#endif
