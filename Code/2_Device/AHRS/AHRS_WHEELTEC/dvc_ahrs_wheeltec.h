/**
 * @file dvc_ahrs_wheeltec.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief WHEELTEC姿态传感器
 * @version 0.1
 * @date 2024-01-01 0.1 新增
 *
 * @copyright USTC-RoboWalker (c) 2024
 *
 */

#ifndef DVC_AHRS_WHEELTEC_H
#define DVC_AHRS_WHEELTEC_H

/* Includes ------------------------------------------------------------------*/

#include "alg_basic.h"
#include "drv_uart.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief WHEELTEC姿态传感器状态
 *
 */
enum Enum_AHRS_WHEELTEC_Status
{
    AHRS_WHEELTEC_Status_DISABLE = 0,
    AHRS_WHEELTEC_Status_ENABLE,
};

/**
 * @brief WHEELTEC姿态传感器接收数据类型
 *
 */
enum Enum_AHRS_WHEELTEC_Data_Type : uint8_t
{
    AHRS_WHEELTEC_Data_Type_IMU = 0x40,
    AHRS_WHEELTEC_Data_Type_AHRS,
};

/**
 * @brief WHEELTEC姿态传感器源数据
 *
 */
struct Struct_AHRS_WHEELTEC_UART_Data
{
    uint8_t Frame_Header;
    Enum_AHRS_WHEELTEC_Data_Type Data_Type;
    uint8_t Data_Length;
    uint8_t Sequence;
    uint8_t CRC_8;
    uint16_t CRC_16;
    uint8_t Data[128];
} __attribute__((packed));

/**
 * @brief WHEELTEC姿态传感器经过处理的数据, 惯导数据
 *
 */
struct Struct_AHRS_WHEELTEC_Data_IMU
{
    float Omega_X;
    float Omega_Y;
    float Omega_Z;
    float Accelerate_X;
    float Accelerate_Y;
    float Accelerate_Z;
    float Magnetic_X;
    float Magnetic_Y;
    float Magnetic_Z;
    float Temperature;
    float Pressure;
    float Pressure_Temperature;
    int64_t Timestamp;
    uint8_t Frame_Rear;
} __attribute__((packed));

/**
 * @brief WHEELTEC姿态传感器经过处理的数据, 姿态数据
 *
 */
struct Struct_AHRS_WHEELTEC_Data_AHRS
{
    float Omega_Roll;
    float Omega_Pitch;
    float Omega_Yaw;
    float Angle_Roll;
    float Angle_Pitch;
    float Angle_Yaw;
    float Q_0;
    float Q_1;
    float Q_2;
    float Q_3;
    uint64_t Timestamp;
    uint8_t Frame_Rear;
} __attribute__((packed));

/* Exported variables --------------------------------------------------------*/

/**
 * @brief Generic, WHEELTEC姿态传感器
 *
 */
class Class_AHRS_WHEELTEC
{
public:
    void Init(UART_HandleTypeDef *huart, uint8_t __Frame_Header = 0xfc, uint8_t __Frame_Rear = 0xfd);

    inline Enum_AHRS_WHEELTEC_Status Get_Status();

    inline float Get_Omega_X();

    inline float Get_Omega_Y();

    inline float Get_Omega_Z();

    inline float Get_Accelerate_X();

    inline float Get_Accelerate_Y();

    inline float Get_Accelerate_Z();

    inline float Get_Magnetic_X();

    inline float Get_Magnetic_Y();

    inline float Get_Magnetic_Z();

    inline float Get_Temperature();

    inline float Get_Pressure();

    inline float Get_Pressure_Temperature();

    inline uint64_t Get_Timestamp();

    inline float Get_Omega_Roll();

    inline float Get_Omega_Pitch();

    inline float Get_Omega_Yaw();

    inline float Get_Angle_Roll();

    inline float Get_Angle_Pitch();

    inline float Get_Angle_Yaw();

    inline float Get_Q_0();

    inline float Get_Q_1();

    inline float Get_Q_2();

    inline float Get_Q_3();

    void UART_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length);

    void TIM_100ms_Alive_PeriodElapsedCallback();

protected:
    // 初始化相关常量

    // 绑定的UART
    Struct_UART_Manage_Object *UART_Manage_Object;
    // 数据包头标
    uint8_t Frame_Header;
    // 数据包尾标
    uint8_t Frame_Rear;

    // 常量

    static constexpr uint16_t FRAME_DATA_OFFSET = 7U;
    static constexpr uint16_t FRAME_REAR_LENGTH = 1U;
    static constexpr uint16_t FRAME_BUFFER_SIZE = sizeof(Struct_AHRS_WHEELTEC_UART_Data);

    // 内部变量

    // 跨UART回调保存尚未组成完整帧的数据
    uint8_t Rx_Stream_Buffer[FRAME_BUFFER_SIZE] = {};
    uint16_t Rx_Stream_Length = 0;

    // 当前时刻的WHEELTEC有效AHRS帧计数
    volatile uint32_t Flag = 0;
    // 前一时刻的WHEELTEC有效AHRS帧计数
    uint32_t Pre_Flag = 0;

    // 读变量

    // WIT姿态传感器状态
    Enum_AHRS_WHEELTEC_Status WHEELTEC_AHRS_Status = AHRS_WHEELTEC_Status_DISABLE;

    // WHEELTEC姿态传感器对外接口信息
    Struct_AHRS_WHEELTEC_Data_IMU Data_IMU;
    Struct_AHRS_WHEELTEC_Data_AHRS Data_AHRS;

    // 写变量

    // 读写变量
    // 角速度滤波
    static constexpr float OMEGA_EMA_ALPHA = 0.2f; // 角速度EMA滤波系数

    float Omega_Roll_Filtered = 0.0f;  // Roll角速度EMA滤波值
    float Omega_Pitch_Filtered = 0.0f; // Pitch角速度EMA滤波值
    float Omega_Yaw_Filtered = 0.0f;   // Yaw角速度EMA滤波值

    // 内部函数

    uint8_t Verify_CRC_8(uint8_t *Message, uint32_t Length);

    uint16_t Verify_CRC_16(uint8_t *Message, uint32_t Length);

    void Data_Process(const uint8_t *Rx_Data, uint16_t Length);

    bool Try_Process_Frame();

    void Discard_Stream_Prefix(uint16_t Length);
};

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取WHEELTEC姿态传感器状态
 *
 * @return Enum_WIT_Status WHEELTEC姿态传感器状态
 */
inline Enum_AHRS_WHEELTEC_Status Class_AHRS_WHEELTEC::Get_Status()
{
    return (WHEELTEC_AHRS_Status);
}

/**
 * @brief 获取角速度x
 *
 * @return float 角速度x
 */
inline float Class_AHRS_WHEELTEC::Get_Omega_X()
{
    return (Data_IMU.Omega_X);
}

/**
 * @brief 获取角速度y
 *
 * @return float 角速度y
 */
inline float Class_AHRS_WHEELTEC::Get_Omega_Y()
{
    return (Data_IMU.Omega_Y);
}

/**
 * @brief 获取角速度z
 *
 * @return float 角速度z
 */
inline float Class_AHRS_WHEELTEC::Get_Omega_Z()
{
    return (Data_IMU.Omega_Z);
}

/**
 * @brief 获取加速度x
 *
 * @return float 加速度x
 */
inline float Class_AHRS_WHEELTEC::Get_Accelerate_X()
{
    return (Data_IMU.Accelerate_X);
}

/**
 * @brief 获取加速度y
 *
 * @return float 加速度y
 */
inline float Class_AHRS_WHEELTEC::Get_Accelerate_Y()
{
    return (Data_IMU.Accelerate_Y);
}

/**
 * @brief 获取加速度z
 *
 * @return float 加速度z
 */
inline float Class_AHRS_WHEELTEC::Get_Accelerate_Z()
{
    return (Data_IMU.Accelerate_Z);
}

/**
 * @brief 获取磁场强度x
 *
 * @return float 磁场强度x
 */
inline float Class_AHRS_WHEELTEC::Get_Magnetic_X()
{
    return (Data_IMU.Magnetic_X);
}

/**
 * @brief 获取磁场强度y
 *
 * @return float 磁场强度y
 */
inline float Class_AHRS_WHEELTEC::Get_Magnetic_Y()
{
    return (Data_IMU.Magnetic_Y);
}

/**
 * @brief 获取磁场强度z
 *
 * @return float 磁场强度z
 */
inline float Class_AHRS_WHEELTEC::Get_Magnetic_Z()
{
    return (Data_IMU.Magnetic_Z);
}

/**
 * @brief 获取温度
 *
 * @return float 温度
 */
inline float Class_AHRS_WHEELTEC::Get_Temperature()
{
    return (Data_IMU.Temperature);
}

/**
 * @brief 获取大气压强
 *
 * @return float 大气压强
 */
inline float Class_AHRS_WHEELTEC::Get_Pressure()
{
    return (Data_IMU.Pressure);
}

/**
 * @brief 获取气压计的温度
 *
 * @return float 气压计的温度
 */
inline float Class_AHRS_WHEELTEC::Get_Pressure_Temperature()
{
    return (Data_IMU.Pressure_Temperature);
}

/**
 * @brief 获取系统时间戳
 *
 * @return float 系统时间戳
 */
inline uint64_t Class_AHRS_WHEELTEC::Get_Timestamp()
{
    return (Data_IMU.Timestamp);
}

/**
 * @brief 获取滚转角速度
 *
 * @return float 滚转角速度
 */
inline float Class_AHRS_WHEELTEC::Get_Omega_Roll()
{
    float raw = Data_AHRS.Omega_Roll;
    Omega_Roll_Filtered = OMEGA_EMA_ALPHA * raw + (1.0f - OMEGA_EMA_ALPHA) * Omega_Roll_Filtered;
    return roundf(Omega_Roll_Filtered * 1000.0f) / 1000.0f;
}

/**
 * @brief 获取俯仰角速度
 *
 * @return float 俯仰角速度
 */
inline float Class_AHRS_WHEELTEC::Get_Omega_Pitch()
{
    float raw = Data_AHRS.Omega_Pitch;
    Omega_Pitch_Filtered = OMEGA_EMA_ALPHA * raw + (1.0f - OMEGA_EMA_ALPHA) * Omega_Pitch_Filtered;
    return roundf(Omega_Pitch_Filtered * 1000.0f) / 1000.0f;
}

/**
 * @brief 获取航向角速度
 *
 * @return float 航向角速度
 */
inline float Class_AHRS_WHEELTEC::Get_Omega_Yaw()
{
    float raw = Data_AHRS.Omega_Yaw;
    Omega_Yaw_Filtered = OMEGA_EMA_ALPHA * raw + (1.0f - OMEGA_EMA_ALPHA) * Omega_Yaw_Filtered;
    return roundf(Omega_Yaw_Filtered * 1000.0f) / 1000.0f;
}

/**
 * @brief 获取滚转角
 *
 * @return float 滚转角, 单位为度
 */
inline float Class_AHRS_WHEELTEC::Get_Angle_Roll()
{
    // N100原始欧拉角单位为弧度，先转换为度。
    float angle_deg = Data_AHRS.Angle_Roll / BASIC_MATH_DEG_TO_RAD;
    angle_deg = Basic_Math_Modulus_Normalization(angle_deg, 360.0f);
    return roundf(angle_deg * 1000.0f) / 1000.0f;
}

/**
 * @brief 获取俯仰角
 *
 * @return float 俯仰角, 单位为度
 */
inline float Class_AHRS_WHEELTEC::Get_Angle_Pitch()
{
    // N100原始欧拉角单位为弧度，先转换为度。
    const float angle_deg = Data_AHRS.Angle_Pitch / BASIC_MATH_DEG_TO_RAD;
    return roundf(angle_deg * 1000.0f) / 1000.0f;
}

/**
 * @brief 获取航向角
 *
 * @return float 航向角, 单位为度, 范围为[0, 360)
 */
inline float Class_AHRS_WHEELTEC::Get_Angle_Yaw()
{
    // N100原始欧拉角单位为弧度，先转换为度，再归一化到[0, 360)。
    float angle_deg = Data_AHRS.Angle_Yaw / BASIC_MATH_DEG_TO_RAD;
    angle_deg = fmodf(angle_deg, 360.0f);
    if (angle_deg < 0.0f)
    {
        angle_deg += 360.0f;
    }

    // 修正浮点边界产生的360度，并统一正负零。
    return (angle_deg >= 360.0f || angle_deg == 0.0f) ? 0.0f : angle_deg;
}

/**
 * @brief 获取四元数0
 *
 * @return float 四元数0
 */
inline float Class_AHRS_WHEELTEC::Get_Q_0()
{
    return (Data_AHRS.Q_0);
}

/**
 * @brief 获取四元数1
 *
 * @return float 四元数1
 */
inline float Class_AHRS_WHEELTEC::Get_Q_1()
{
    return (Data_AHRS.Q_1);
}

/**
 * @brief 获取四元数2
 *
 * @return float 四元数2
 */
inline float Class_AHRS_WHEELTEC::Get_Q_2()
{
    return (Data_AHRS.Q_2);
}

/**
 * @brief 获取四元数3
 *
 * @return float 四元数3
 */
inline float Class_AHRS_WHEELTEC::Get_Q_3()
{
    return (Data_AHRS.Q_3);
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
