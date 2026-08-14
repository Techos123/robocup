/**
 * @file bsp_bmi088.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief BMI088组件之加速度计, 内含加热电阻
 * @version 0.1
 * @date 2025-08-26 0.1 新建文档
 *
 * @copyright USTC-RoboWalker (c) 2025
 *
 */

#ifndef BSP_BMI088_H
#define BSP_BMI088_H

/* Includes ------------------------------------------------------------------*/

#include "bsp_bmi088_accel.h"
#include "bsp_bmi088_gyro.h"
#include "alg_quaternion.h"
#include "alg_filter_ekf.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 数据处理结构体
 *
 */
struct Struct_BMI088_Status
{
    // 数据传输状态
    volatile bool Ready_Flag = false;
    volatile bool Transfering_Flag = false;

    // 数据传输时间戳
    volatile uint64_t Ready_Timestamp = 0;
    volatile uint64_t Transfering_Timestamp = 0;
    volatile uint64_t Sample_Timestamp = 0;
};

struct Struct_BMI088_Sample
{
    Class_Matrix_f32<3, 1> Value;
    uint64_t Timestamp = 0;
    volatile uint32_t Sequence = 0;
    bool Valid = false;
};

struct Struct_BMI088_Attitude_Snapshot
{
    Class_Matrix_f32<3, 1> Euler_Angle;
    Class_Matrix_f32<3, 1> Gyro_Body;
    Class_Quaternion_f32 Quaternion;
    bool Valid = false;
};

struct Struct_BMI088_Diagnostics
{
    Class_Matrix_f32<3, 1> Raw_Gyro;
    Class_Matrix_f32<3, 1> Corrected_Gyro;
    Class_Matrix_f32<3, 1> Gyro_Bias;
    float Last_D_T = 0.0f;
    float Accel_Norm = 0.0f;
    float Accel_Chi_Square_Loss = 0.0f;
    float Temperature = 0.0f;
    uint32_t Calibration_Progress = 0;
    uint32_t Calibration_Target = 0;
    uint32_t Dropped_Gyro_Samples = 0;
    uint32_t Dropped_Accel_Samples = 0;
    uint32_t Invalid_D_T_Count = 0;
    bool Calibration_Finished = false;
    bool Accel_Update_Accepted = false;
};

/**
 * @brief Specialized, 板载AHRS
 *
 */
class Class_BMI088
{
public:
    Class_BMI088_Accel BMI088_Accel;
    Class_BMI088_Gyro BMI088_Gyro;

    Class_Filter_EKF<4, 3, 3> EKF_Quaternion;

    bool Init();

    inline Class_Matrix_f32<3, 1> Get_Original_Accel() const;

    inline Class_Matrix_f32<3, 1> Get_Original_Gyro() const;

    inline Class_Matrix_f32<3, 1> Get_Euler_Angle() const;

    bool Get_Attitude_Valid() const;

    inline Class_Matrix_f32<3, 3> Get_Rotation_Matrix() const;

    inline Class_Matrix_f32<4, 1> Get_Axis_Angle() const;

    inline Class_Quaternion_f32 Get_Quaternion() const;

    inline Class_Matrix_f32<3, 1> Get_Accel_Body();

    inline Class_Matrix_f32<3, 1> Get_Gyro_Body();

    inline Class_Matrix_f32<3, 1> Get_Accel_Odom();

    inline Class_Matrix_f32<3, 1> Get_Gyro_Odom();

    inline float Get_Accel_Chi_Square_Loss() const;

    inline uint64_t Get_Calculating_Time() const;

    Struct_BMI088_Attitude_Snapshot Get_Attitude_Snapshot() const;

    Struct_BMI088_Diagnostics Get_Diagnostics() const;

    void SPI_RxCpltCallback();

    void EXTI_Flag_Callback(uint16_t GPIO_Pin);

    void TIM_128ms_Calculate_PeriodElapsedCallback();

    void TIM_125us_Calculate_PeriodElapsedCallback();

    void TIM_10us_Calculate_PeriodElapsedCallback();

protected:
    // 初始化相关常量

    // 绑定的SPI
    Struct_SPI_Manage_Object *SPI_Manage_Object;

    // 常量

    // 数据传输超时时间, 单位us
    uint64_t TRANSFERING_TIMEOUT = 20;

    // D_T超时时间阈值
    float D_T_TIMEOUT_THRESHOLD = 0.1f;

    // 加速度计卡方检验残差阈值
    float ACCEL_CHI_SQUARE_TEST_THRESHOLD = 0.5f;

    // 启动静止标定参数
    static constexpr uint32_t GYRO_CALIBRATION_SAMPLE_COUNT = 4096;
    static constexpr float GYRO_CALIBRATION_OMEGA_THRESHOLD = 0.0872664626f;
    static constexpr float ACCEL_NORMALIZATION_MIN_NORM = 0.001f;
    static constexpr float CALIBRATION_ACCEL_MIN_RATIO = 0.85f;
    static constexpr float CALIBRATION_ACCEL_MAX_RATIO = 1.15f;

    // 校正数据, 与温控有关, 温控在50℃
    // 加速度计仿射矩阵源数据
    const float ACCEL_AFFINE_DATA[9] = {0.9813826498493404f, 0.17232440504057203f, 0.027984325323801115f, -0.1690535919907899f, 0.9747302115792275f, -0.10336863799715153f, -0.046825636945091266f, 0.09953521655990044f, 0.986897809387138f};
    // 加速度计偏置
    const float ACCEL_BIAS_DATA[3] = {0.0038458286072392397, 0.00647039594993548f, 0.014968990490337293f};
    // 陀螺仪偏置
    const float GYRO_ZERO_OFFSET[3] = {-0.005450708363333335f, -0.0008284202383333334f, -0.0006914497383333334f};

    // EKF相关参数
    // 过程噪声协方差矩阵
    const float EKF_Q[9] = {0.865f, 0.0f, 0.0f, 0.0f, 0.975f, 0.0f, 0.0f, 0.0f, 1.077f};
    // 测量噪声协方差矩阵
    const float EKF_R[9] = {0.446f, 0.0f, 0.0f, 0.0f, 0.476f, 0.0f, 0.0f, 0.0f, 0.537f};

    // 内部变量

    // BMI088硬件初始化完成标志
    bool Hardware_Init_Finished_Flag = false;
    // EKF算法初始化状态完成标志
    bool EKF_Init_Finished_Flag = false;
    // 启动陀螺仪零偏标定完成标志
    bool Gyro_Calibration_Finished_Flag = false;

    // 加速度数据状态
    Struct_BMI088_Status Accel_Status;
    // 陀螺仪数据状态
    Struct_BMI088_Status Gyro_Status;
    // 温度计数据状态
    Struct_BMI088_Status Temperature_Status;

    // SPI完成后一次性发布的传感器样本
    Struct_BMI088_Sample Accel_Sample;
    Struct_BMI088_Sample Gyro_Sample;
    uint32_t Consumed_Accel_Sequence = 0;
    uint32_t Consumed_Gyro_Sequence = 0;

    // 加速度计归一化数据
    Class_Matrix_f32<3, 1> Vector_Normalized_Accel;
    // 最近一次原始陀螺仪数据
    Class_Matrix_f32<3, 1> Vector_Raw_Gyro;
    // 上一次陀螺仪有效数据
    Class_Matrix_f32<3, 1> Vector_Pre_Valid_Gyro;
    // 启动标定得到的陀螺仪零偏
    Class_Matrix_f32<3, 1> Vector_Gyro_Bias;
    Class_Matrix_f32<3, 1> Vector_Gyro_Calibration_Sum;
    uint32_t Gyro_Calibration_Progress = 0;
    bool Accel_Calibration_Valid = false;

    // 上次有效历史数据时间戳
    uint64_t EKF_Pre_Valid_Timestamp = 0;

    // 时间差
    float Valid_D_T = 0.000125f;
    float Last_Valid_D_T = 0.0f;
    float Accel_Norm = 0.0f;
    bool Accel_Update_Accepted = false;
    uint32_t Dropped_Gyro_Samples = 0;
    uint32_t Dropped_Accel_Samples = 0;
    uint32_t Invalid_D_T_Count = 0;

    // 读变量

    // 加速度计校正后的原始数据
    Class_Matrix_f32<3, 1> Vector_Original_Accel;
    // 陀螺仪校正后的原始数据
    Class_Matrix_f32<3, 1> Vector_Original_Gyro;

    // 欧拉角, Yaw-Pitch-Roll顺序
    Class_Matrix_f32<3, 1> Vector_Euler_Angle;
    // 旋转矩阵
    Class_Matrix_f32<3, 3> Matrix_Rotation;
    // 轴角式
    Class_Matrix_f32<4, 1> Vector_Axis_Angle;
    // 四元数
    Class_Quaternion_f32 Quarternion;

    // 机体坐标系下的加速度
    Class_Matrix_f32<3, 1> Vector_Accel_Body;
    // 机体坐标系下的角速度
    Class_Matrix_f32<3, 1> Vector_Gyro_Body;
    // 大地坐标系下的加速度
    Class_Matrix_f32<3, 1> Vector_Accel_Odom;
    // 大地坐标系下的角速度
    Class_Matrix_f32<3, 1> Vector_Gyro_Odom;

    // 加速度计卡方检验值
    float Accel_Chi_Square_Loss = 0.0f;
    // 处理时间
    uint64_t Calculating_Time = 0;

    // 写变量

    // 读写变量

    // 内部函数

    // EKF的相关函数
    // 四元数状态转移函数
    static Class_Matrix_f32<4, 1> EKF_Function_F(const Class_Matrix_f32<4, 1> &Vector_X, const Class_Matrix_f32<3, 1> &Vector_U, const float &D_T);

    // 四元数状态转移函数对状态的雅可比矩阵
    static Class_Matrix_f32<4, 4> EKF_Function_Jacobian_F_X(const Class_Matrix_f32<4, 1> &Vector_X, const Class_Matrix_f32<3, 1> &Vector_U, const float &D_T);

    // 四元数状态转移函数对过程噪声的雅可比矩阵
    static Class_Matrix_f32<4, 3> EKF_Function_Jacobian_F_W(const Class_Matrix_f32<4, 1> &Vector_X, const Class_Matrix_f32<3, 1> &Vector_U, const float &D_T);

    // 四元数测量函数
    static Class_Matrix_f32<3, 1> EKF_Function_H(const Class_Matrix_f32<4, 1> &Vector_X, const float &D_T);

    // 四元数测量函数对状态的雅可比矩阵
    static Class_Matrix_f32<3, 4> EKF_Function_Jacobian_H_X(const Class_Matrix_f32<4, 1> &Vector_X, const float &D_T);

    // 四元数测量函数对测量噪声的雅可比矩阵
    static Class_Matrix_f32<3, 3> EKF_Function_Jacobian_H_V(const Class_Matrix_f32<4, 1> &Vector_X, const float &D_T);

    void Accel_Chi_Square_Calculate();

    bool Validate_Vector(const Class_Matrix_f32<3, 1> &Vector) const;

    bool Prepare_Accel_Sample(const Struct_BMI088_Sample &Sample);

    void Process_Gyro_Calibration(const Struct_BMI088_Sample &Sample);

    void Initialize_EKF(uint64_t Timestamp);

    bool Predict_To_Timestamp(uint64_t Timestamp, const Class_Matrix_f32<3, 1> &Gyro);

    void Apply_Gyro_Sample(uint64_t Timestamp, const Class_Matrix_f32<3, 1> &Gyro);

    void Update_With_Accel(uint64_t Timestamp);

    void Normalize_EKF_State(bool Include_Prior);

    void Publish_Output(uint64_t Now_Timestamp);
};

/* Exported variables --------------------------------------------------------*/

extern Class_BMI088 BSP_BMI088;

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取加速度计校正后的原始数据
 *
 * @return 加速度计原始数据, 单位m/s²
 */
inline Class_Matrix_f32<3, 1> Class_BMI088::Get_Original_Accel() const
{
    return (Vector_Original_Accel);
}

/**
 * @brief 获取陀螺仪校正后的原始数据
 *
 * @return 陀螺仪原始数据, 单位rad/s
 */
inline Class_Matrix_f32<3, 1> Class_BMI088::Get_Original_Gyro() const
{
    return (Vector_Original_Gyro);
}

/**
 * @brief 获取Euler角
 *
 * @return Euler角, 单位rad
 */
inline Class_Matrix_f32<3, 1> Class_BMI088::Get_Euler_Angle() const
{
    return (Vector_Euler_Angle);
}

/**
 * @brief 获取旋转矩阵
 *
 * @return 旋转矩阵
 */
inline Class_Matrix_f32<3, 3> Class_BMI088::Get_Rotation_Matrix() const
{
    return (Matrix_Rotation);
}

/**
 * @brief 获取轴角式的轴
 *
 * @return 轴角式
 */
inline Class_Matrix_f32<4, 1> Class_BMI088::Get_Axis_Angle() const
{
    return (Vector_Axis_Angle);
}

/**
 * @brief 获取四元数
 *
 */
inline Class_Quaternion_f32 Class_BMI088::Get_Quaternion() const
{
    return (Quarternion);
}

/**
 * @brief 获取机体坐标系下的加速度
 *
 */
inline Class_Matrix_f32<3, 1> Class_BMI088::Get_Accel_Body()
{
    return (Vector_Accel_Body);
}

/**
 * @brief 获取机体坐标系下的角速度
 *
 */
inline Class_Matrix_f32<3, 1> Class_BMI088::Get_Gyro_Body()
{
    return (Vector_Gyro_Body);
}

/**
 * @brief 获取大地坐标系下的加速度
 *
 */
inline Class_Matrix_f32<3, 1> Class_BMI088::Get_Accel_Odom()
{
    return (Vector_Accel_Odom);
}

/**
 * @brief 获取大地坐标系下的角速度
 *
 */
inline Class_Matrix_f32<3, 1> Class_BMI088::Get_Gyro_Odom()
{
    return (Vector_Gyro_Odom);
}

/**
 * @brief 获取加速度计卡方检验残差
 *
 * @return 加速度计卡方检验残差
 */
inline float Class_BMI088::Get_Accel_Chi_Square_Loss() const
{
    return (Accel_Chi_Square_Loss);
}

/**
 * @brief 获取计算时间
 *
 * @return 计算时间, 单位s
 */
inline uint64_t Class_BMI088::Get_Calculating_Time() const
{
    return (Calculating_Time);
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
