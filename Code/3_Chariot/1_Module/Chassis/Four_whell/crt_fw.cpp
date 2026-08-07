#include "../User_Code/3_Chariot/1_Module/Chassis/Mecanum_whell/crt_mw.h"
/**
 * @brief 轮组编号
 * 3[0] 2[3]
 * 4[1] 1[2]
 * 前x左y上z
 */

#define EPSILON 0.1f

/**
 * @brief 底盘初始化
 *
 * @param __Chassis_Control_Type 麦克纳姆轮底盘控制方法
 * @param __Speed 底盘速度限制最大值
 */
void Class_Chassis_MW::Init(Class_AHRS_WHEELTEC *__AHRS)
{
    // 绑定AHRS并捕获初始航向
    AHRS_Chassis = __AHRS;
    Target_Angle = __AHRS->Get_Angle_Yaw();

    // PID初始化

    // 底盘速度xPID, 输出摩擦力
    PID_Velocity_X.Init(7.0f, 0.0f, 0.0f, 1.3f, 2.0f, 8.0f, 0.002f);

    // 底盘速度yPID, 输出摩擦力
    PID_Velocity_Y.Init(15.0f, 0.0f, 0.0f, 1.3f, 2.0f, 8.0f, 0.002f);

    // 底盘角速度PID, 输出扭矩
    PID_Omega.Init(4.0f, 0.0f, 0.0f, 2.3f, 2.0f, 8.0f, 0.002f);

    // 底盘yaw角速度PID, 输出扭矩
    PID_YawSpeed.Init(4.9f, 0.0f, 0.0f, 0.0f, 0.5f, 2.0f, 0.002f);

    // 底盘yaw角度PID, 输出角速度
    PID_YawAngle.Init(7.4f, 2.4f, 0.0f, 0.2f, 0.5f, 2.0f, 0.002f, 0.01f);

    // 电机初始化
    Motor_Wheel[0].Init(&hcan1, Motor_DJI_ID_0x203, Motor_DJI_Control_Method_CURRENT, 3591.0f / 187.0f /* , Motor_DJI_Power_Limit_Status_ENABLE */);

    // 电机初始化
    Motor_Wheel[1].Init(&hcan1, Motor_DJI_ID_0x204, Motor_DJI_Control_Method_CURRENT, 3591.0f / 187.0f /* , Motor_DJI_Power_Limit_Status_ENABLE */);

    // 电机初始化
    Motor_Wheel[2].Init(&hcan1, Motor_DJI_ID_0x201, Motor_DJI_Control_Method_CURRENT, 3591.0f / 187.0f /* , Motor_DJI_Power_Limit_Status_ENABLE */);

    // 电机初始化
    Motor_Wheel[3].Init(&hcan1, Motor_DJI_ID_0x202, Motor_DJI_Control_Method_CURRENT, 3591.0f / 187.0f /* , Motor_DJI_Power_Limit_Status_ENABLE */);
}

/**
 * @brief TIM定时器中断定期检测电机是否存活
 *
 */
void Class_Chassis_MW::TIM_100ms_Alive_PeriodElapsedCallback()
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
void Class_Chassis_MW::TIM_2ms_Resolution_PeriodElapsedCallback()
{
    Self_Resolution();
}

/**
 * @brief TIM定时器中断控制回调函数
 *
 */
void Class_Chassis_MW::TIM_2ms_Control_PeriodElapsedCallback()
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
void Class_Chassis_MW::Self_Resolution()
{

    // 解算底盘速度与角速度
    Now_Velocity_X = (-Motor_Wheel[0].Get_Now_Omega() - Motor_Wheel[1].Get_Now_Omega() + Motor_Wheel[2].Get_Now_Omega() + Motor_Wheel[3].Get_Now_Omega()) * Wheel_Radius / 4.0f;

    Now_Velocity_Y = (Motor_Wheel[0].Get_Now_Omega() - Motor_Wheel[1].Get_Now_Omega() - Motor_Wheel[2].Get_Now_Omega() + Motor_Wheel[3].Get_Now_Omega()) * Wheel_Radius / 4.0f;

    Now_Omega = (Motor_Wheel[0].Get_Now_Omega() + Motor_Wheel[1].Get_Now_Omega() + Motor_Wheel[2].Get_Now_Omega() + Motor_Wheel[3].Get_Now_Omega()) / (Wheel_Point_X + Wheel_Point_Y) * Wheel_Radius / 4.0f;

    // 角度解算
    float pitch = -AHRS_Chassis->Get_Angle_Pitch();
    float roll = AHRS_Chassis->Get_Angle_Roll();
    float yaw = AHRS_Chassis->Get_Angle_Yaw();

    if (isnan(pitch) == true || isnan(roll) == true || isnan(yaw) == true)
    {
        Angle_Pitch = 0.0f;
        Angle_Roll = 0.0f;
        Angle_Yaw = 0.0f;
    }
    else
    {
        Angle_Pitch = pitch;
        Angle_Roll = roll;
        Angle_Yaw = yaw;
        float zx = arm_sin_f32(Angle_Pitch);
        float zy = -arm_sin_f32(Angle_Roll) * arm_cos_f32(Angle_Pitch);
        float zz = arm_cos_f32(Angle_Roll) * arm_cos_f32(Angle_Pitch);
        float horizontal;
        arm_sqrt_f32(zy * zy + zx * zx, &horizontal);
        Slope_Pitch = atan2f(-horizontal, zz);
    }
}

/**
 * @brief 输出到动力学状态
 *
 */
void Class_Chassis_MW::Output_To_Dynamics()
{
    switch (Chassis_Control_Type)
    {
    case (Chassis_Control_Type_DISABLE):
    {

        PID_Velocity_X.Set_Integral_Error(0.0f);
        PID_Velocity_Y.Set_Integral_Error(0.0f);
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

        PID_Velocity_Y.Set_Target(Target_Velocity_Y);
        PID_Velocity_Y.Set_Now(Now_Velocity_Y);
        PID_Velocity_Y.TIM_Calculate_PeriodElapsedCallback();

        PID_Omega.Set_Target(Target_Omega);
        PID_Omega.Set_Now(Now_Omega);
        PID_Omega.TIM_Calculate_PeriodElapsedCallback();

        // ========== IMU航向纠偏补偿 ==========
        // 角度数据来自IMU(Angle_Yaw, 已在Self_Resolution中赋值为AHRS角度)
        // 角速度反馈使用轮式里程计Now_Omega(与底盘坐标系一致，避免IMU角速度符号问题)
        if (fabsf(Target_Omega) < EPSILON)
        {
            // 【平动模式】启用航向锁定
            // 角度误差环绕处理：确保PID走最短路径，防止Target_Angle和Angle_Yaw
            // 分别位于±PI边界两侧时，原始误差~2π导致机器人旋转近一圈
            float angle_error = Target_Angle - Angle_Yaw;
            angle_error = Math_Modulus_Normalization(angle_error, 2.0f * PI);
            PID_YawAngle.Set_Target(Angle_Yaw + angle_error);
            PID_YawAngle.Set_Now(Angle_Yaw);
            PID_YawAngle.TIM_Calculate_PeriodElapsedCallback();

            float inner_omega = PID_YawAngle.Get_Out() + Target_Omega;

            // 使用轮式里程计角速度(Now_Omega)作为反馈，与PID_Omega坐标系一致
            PID_YawSpeed.Set_Target(inner_omega);
            PID_YawSpeed.Set_Now(Now_Omega);
            PID_YawSpeed.TIM_Calculate_PeriodElapsedCallback();
        }
        else
        {
            // 【转动模式】角度环完全旁路，航向角跟随实际值
            Target_Angle = Angle_Yaw;
            PID_YawAngle.Set_Target(Angle_Yaw);
            PID_YawAngle.Set_Now(Angle_Yaw);
            PID_YawAngle.Set_Integral_Error(0.0f);
            PID_YawAngle.TIM_Calculate_PeriodElapsedCallback();

            // 使用轮式里程计角速度作为反馈
            PID_YawSpeed.Set_Target(Target_Omega);
            PID_YawSpeed.Set_Now(Now_Omega);
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
void Class_Chassis_MW::Dynamics_Inverse_Resolution()
{
    float force_x, force_y, torque_omega;

    force_x = PID_Velocity_X.Get_Out();
    force_y = PID_Velocity_Y.Get_Out();
    torque_omega = PID_Omega.Get_Out() + PID_YawSpeed.Get_Out();

    // 每个轮的扭力
    float tmp_force[4];
    // 解算到每个轮组的具体摩擦力
    tmp_force[0] = (-force_x + force_y + torque_omega * (Wheel_Point_X + Wheel_Point_Y)) / Wheel_Radius / 4.0f;
    tmp_force[1] = (-force_x - force_y + torque_omega * (Wheel_Point_X + Wheel_Point_Y)) / Wheel_Radius / 4.0f;
    tmp_force[2] = (+force_x - force_y + torque_omega * (Wheel_Point_X + Wheel_Point_Y)) / Wheel_Radius / 4.0f;
    tmp_force[3] = (+force_x + force_y + torque_omega * (Wheel_Point_X + Wheel_Point_Y)) / Wheel_Radius / 4.0f;
    for (int i = 0; i < 4; i++)
    {
        // 摩擦力转换至扭矩
        Target_Wheel_Current[i] = tmp_force[i];
    }
}

/**
 * @brief 输出到电机
 *
 */
void Class_Chassis_MW::Output_To_Motor()
{
    switch (Chassis_Control_Type)
    {
    case (Chassis_Control_Type_DISABLE):
    {
        // 底盘失能
        for (int i = 0; i < 4; i++)
        {

            Motor_Wheel[i].Set_Target_Current(0.0f);
        }

        break;
    }
    case (Chassis_Control_Type_NORMAL):
    {

        for (int i = 0; i < 4; i++)
        {
            Motor_Wheel[i].Set_Target_Current(Target_Wheel_Current[i]);
        }

        break;
    }
    }

    for (int i = 0; i < 4; i++)
    {
        Motor_Wheel[i].TIM_Calculate_PeriodElapsedCallback();
    }

    // _Power_Limit_Control();

    // for (int i = 0; i < 4; i++)
    // {
    //     Motor_Wheel[i].TIM_Power_Limit_After_Calculate_PeriodElapsedCallback();
    // }
}
