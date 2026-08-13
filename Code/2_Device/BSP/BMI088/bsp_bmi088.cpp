/**
 * @file bsp_bmi088.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief BMI088组件之加速度计, 内含加热电阻
 * @version 0.1
 * @date 2025-08-26 0.1 新建文档
 *
 * @copyright USTC-RoboWalker (c) 2025
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "bsp_bmi088.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

Class_BMI088 BSP_BMI088;

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化BMI088
 *
 */
bool Class_BMI088::Init()
{
    SPI_Manage_Object = &SPI2_Manage_Object;
    Hardware_Init_Finished_Flag = false;
    EKF_Init_Finished_Flag = false;
    Gyro_Calibration_Finished_Flag = false;

    Accel_Status = Struct_BMI088_Status();
    Gyro_Status = Struct_BMI088_Status();
    Temperature_Status = Struct_BMI088_Status();
    Accel_Sample = Struct_BMI088_Sample();
    Gyro_Sample = Struct_BMI088_Sample();
    Consumed_Accel_Sequence = 0;
    Consumed_Gyro_Sequence = 0;
    Gyro_Calibration_Progress = 0;
    Dropped_Gyro_Samples = 0;
    Dropped_Accel_Samples = 0;
    Invalid_D_T_Count = 0;
    Last_Valid_D_T = 0.0f;
    Accel_Norm = 0.0f;
    Accel_Chi_Square_Loss = 0.0f;
    Accel_Update_Accepted = false;
    Accel_Calibration_Valid = false;
    Vector_Gyro_Bias = Class_Matrix_f32<3, 1>();
    Vector_Gyro_Calibration_Sum = Class_Matrix_f32<3, 1>();
    Vector_Pre_Valid_Gyro = Class_Matrix_f32<3, 1>();

    if (!BMI088_Accel.Init(false))
    {
        return false;
    }
    if (!BMI088_Gyro.Init())
    {
        return false;
    }

    // 欧拉角需要辅助初始化EKF, 第一次初始化默认Yaw是0
    Vector_Euler_Angle = Class_Matrix_f32<3, 1>();

    Hardware_Init_Finished_Flag = true;
    return true;
}

/**
 * @brief SPI接收完成回调函数
 *
 */
void Class_BMI088::SPI_RxCpltCallback()
{
    if (SPI_Manage_Object->Activate_GPIOx == BMI088_ACCEL__SPI_CS_GPIO_Port && SPI_Manage_Object->Activate_GPIO_Pin == BMI088_ACCEL__SPI_CS_Pin)
    {
        BMI088_Accel.SPI_RxCpltCallback();

        if (Hardware_Init_Finished_Flag)
        {
            if (SPI_Manage_Object->Rx_Buffer_Length == 6)
            {
                Accel_Status.Transfering_Flag = false;
                Accel_Sample.Value = BMI088_Accel.Get_Raw_Accel();
                Accel_Sample.Timestamp = Accel_Status.Sample_Timestamp;
                Accel_Sample.Valid = BMI088_Accel.Get_Valid_Flag() && Validate_Vector(Accel_Sample.Value);
                __DMB();
                Accel_Sample.Sequence++;
            }
            else if (SPI_Manage_Object->Rx_Buffer_Length == 2)
            {
                Temperature_Status.Transfering_Flag = false;
            }
        }
    }
    else if (SPI_Manage_Object->Activate_GPIOx == BMI088_GYRO__SPI_CS_GPIO_Port && SPI_Manage_Object->Activate_GPIO_Pin == BMI088_GYRO__SPI_CS_Pin)
    {
        BMI088_Gyro.SPI_RxCallback();

        if (Hardware_Init_Finished_Flag)
        {
            Gyro_Status.Transfering_Flag = false;
            Gyro_Sample.Value = BMI088_Gyro.Get_Raw_Gyro();
            Gyro_Sample.Timestamp = Gyro_Status.Sample_Timestamp;
            Gyro_Sample.Valid = BMI088_Gyro.Get_Valid_Flag() && Validate_Vector(Gyro_Sample.Value);
            __DMB();
            Gyro_Sample.Sequence++;
        }
    }
}

/**
 * @brief EXTI中断回调函数
 *
 * @param GPIO_Pin 中断引脚
 */
void Class_BMI088::EXTI_Flag_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == BMI088_ACCEL__INTERRUPT_Pin)
    {
        if (Accel_Status.Ready_Flag)
        {
            Dropped_Accel_Samples++;
        }
        Accel_Status.Ready_Flag = true;
        Accel_Status.Ready_Timestamp = SYS_Timestamp.Get_Now_Microsecond();
    }
    else if (GPIO_Pin == BMI088_GYRO__INTERRUPT_Pin)
    {
        if (Gyro_Status.Ready_Flag)
        {
            Dropped_Gyro_Samples++;
        }
        Gyro_Status.Ready_Flag = true;
        Gyro_Status.Ready_Timestamp = SYS_Timestamp.Get_Now_Microsecond();
    }
}

/**
 * @brief 定时器周期中断回调函数
 *
 */
void Class_BMI088::TIM_128ms_Calculate_PeriodElapsedCallback()
{
    Temperature_Status.Ready_Flag = true;
    if (BMI088_Accel.Get_Heater_Enable())
    {
        BMI088_Accel.TIM_128ms_Heater_PID_PeriodElapsedCallback();
    }
}

/**
 * @brief 定时器周期中断回调函数
 *
 */
void Class_BMI088::TIM_125us_Calculate_PeriodElapsedCallback()
{
    const uint64_t now_timestamp = SYS_Timestamp.Get_Now_Microsecond();
    Struct_BMI088_Sample accel_sample;
    Struct_BMI088_Sample gyro_sample;

    bool accel_new;
    bool gyro_new;
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    accel_sample = Accel_Sample;
    gyro_sample = Gyro_Sample;
    accel_new = accel_sample.Sequence != Consumed_Accel_Sequence;
    gyro_new = gyro_sample.Sequence != Consumed_Gyro_Sequence;

    if (accel_new)
    {
        const uint32_t received = accel_sample.Sequence - Consumed_Accel_Sequence;
        if (received > 1U)
        {
            Dropped_Accel_Samples += received - 1U;
        }
        Consumed_Accel_Sequence = accel_sample.Sequence;
    }
    if (gyro_new)
    {
        const uint32_t received = gyro_sample.Sequence - Consumed_Gyro_Sequence;
        if (received > 1U)
        {
            Dropped_Gyro_Samples += received - 1U;
        }
        Consumed_Gyro_Sequence = gyro_sample.Sequence;
    }
    if (primask == 0U)
    {
        __enable_irq();
    }

    if (accel_new)
    {
        Accel_Update_Accepted = false;
    }
    const bool accel_prepared = accel_new && Prepare_Accel_Sample(accel_sample);
    if (accel_new)
    {
        Accel_Calibration_Valid = accel_prepared;
    }

    if (!Gyro_Calibration_Finished_Flag)
    {
        if (gyro_new)
        {
            Process_Gyro_Calibration(gyro_sample);
        }
        if (Gyro_Calibration_Finished_Flag && accel_prepared)
        {
            Initialize_EKF(accel_sample.Timestamp);
        }
        Publish_Output(now_timestamp);
        Calculating_Time = SYS_Timestamp.Get_Now_Microsecond() - now_timestamp;
        return;
    }

    bool gyro_prepared = false;
    Class_Matrix_f32<3, 1> corrected_gyro;
    if (gyro_new && gyro_sample.Valid && Validate_Vector(gyro_sample.Value))
    {
        Vector_Raw_Gyro = gyro_sample.Value;
        corrected_gyro = Vector_Raw_Gyro - Vector_Gyro_Bias;
        gyro_prepared = true;
    }

    if (!EKF_Init_Finished_Flag)
    {
        if (accel_prepared)
        {
            Initialize_EKF(accel_sample.Timestamp);
        }
        if (gyro_prepared)
        {
            if (EKF_Init_Finished_Flag && gyro_sample.Timestamp > EKF_Pre_Valid_Timestamp)
            {
                Apply_Gyro_Sample(gyro_sample.Timestamp, corrected_gyro);
            }
            else
            {
                Vector_Pre_Valid_Gyro = corrected_gyro;
                Vector_Original_Gyro = corrected_gyro;
            }
        }
        Publish_Output(now_timestamp);
        Calculating_Time = SYS_Timestamp.Get_Now_Microsecond() - now_timestamp;
        return;
    }

    if (accel_prepared && gyro_prepared)
    {
        if (gyro_sample.Timestamp <= accel_sample.Timestamp)
        {
            Apply_Gyro_Sample(gyro_sample.Timestamp, corrected_gyro);
            if (EKF_Init_Finished_Flag &&
                Predict_To_Timestamp(accel_sample.Timestamp, Vector_Pre_Valid_Gyro))
            {
                Update_With_Accel(accel_sample.Timestamp);
            }
        }
        else
        {
            if (Predict_To_Timestamp(accel_sample.Timestamp, Vector_Pre_Valid_Gyro))
            {
                Update_With_Accel(accel_sample.Timestamp);
            }
            Apply_Gyro_Sample(gyro_sample.Timestamp, corrected_gyro);
        }
    }
    else if (gyro_prepared)
    {
        Apply_Gyro_Sample(gyro_sample.Timestamp, corrected_gyro);
    }
    else if (accel_prepared && Predict_To_Timestamp(accel_sample.Timestamp, Vector_Pre_Valid_Gyro))
    {
        Update_With_Accel(accel_sample.Timestamp);
    }

    Publish_Output(now_timestamp);
    Calculating_Time = SYS_Timestamp.Get_Now_Microsecond() - now_timestamp;
}

/**
 * @brief 定时器周期中断回调函数
 *
 */
void Class_BMI088::TIM_10us_Calculate_PeriodElapsedCallback()
{
    if (Hardware_Init_Finished_Flag && Accel_Status.Ready_Flag && !Accel_Status.Transfering_Flag && !Gyro_Status.Transfering_Flag && !Temperature_Status.Transfering_Flag)
    {
        // 数据准备好, 读取加速度计
        Accel_Status.Transfering_Flag = true;
        Accel_Status.Transfering_Timestamp = SYS_Timestamp.Get_Now_Microsecond();
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        Accel_Status.Sample_Timestamp = Accel_Status.Ready_Timestamp;
        Accel_Status.Ready_Flag = false;
        if (primask == 0U)
        {
            __enable_irq();
        }
        BMI088_Accel.SPI_Request_Accel();
        return;
    }

    if (Hardware_Init_Finished_Flag && Gyro_Status.Ready_Flag && !Accel_Status.Transfering_Flag && !Gyro_Status.Transfering_Flag && !Temperature_Status.Transfering_Flag)
    {
        // 数据准备好, 读取陀螺仪
        Gyro_Status.Transfering_Flag = true;
        Gyro_Status.Transfering_Timestamp = SYS_Timestamp.Get_Now_Microsecond();
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        Gyro_Status.Sample_Timestamp = Gyro_Status.Ready_Timestamp;
        Gyro_Status.Ready_Flag = false;
        if (primask == 0U)
        {
            __enable_irq();
        }
        BMI088_Gyro.SPI_Request_Gyro();
        return;
    }

    if (Hardware_Init_Finished_Flag && Temperature_Status.Ready_Flag && !Accel_Status.Transfering_Flag && !Gyro_Status.Transfering_Flag && !Temperature_Status.Transfering_Flag)
    {
        // 温度准备好, 读取温度
        Temperature_Status.Transfering_Flag = true;
        Temperature_Status.Transfering_Timestamp = SYS_Timestamp.Get_Now_Microsecond();
        BMI088_Accel.SPI_Request_Temperature();
        Temperature_Status.Ready_Flag = false;
        return;
    }

    if (Hardware_Init_Finished_Flag && Accel_Status.Transfering_Flag && (SYS_Timestamp.Get_Now_Microsecond() - Accel_Status.Transfering_Timestamp) >= TRANSFERING_TIMEOUT)
    {
        // 加速度计传输超时
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        if (HAL_SPI_GetState(SPI_Manage_Object->SPI_Handler) != HAL_SPI_STATE_READY ||
            !Accel_Status.Transfering_Flag)
        {
            if (primask == 0U)
            {
                __enable_irq();
            }
            return;
        }
        Accel_Status.Transfering_Timestamp = SYS_Timestamp.Get_Now_Microsecond();
        BMI088_Accel.SPI_Request_Accel();
        if (primask == 0U)
        {
            __enable_irq();
        }
        return;
    }

    if (Hardware_Init_Finished_Flag && Gyro_Status.Transfering_Flag && (SYS_Timestamp.Get_Now_Microsecond() - Gyro_Status.Transfering_Timestamp) >= TRANSFERING_TIMEOUT)
    {
        // 陀螺仪传输超时
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        if (HAL_SPI_GetState(SPI_Manage_Object->SPI_Handler) != HAL_SPI_STATE_READY ||
            !Gyro_Status.Transfering_Flag)
        {
            if (primask == 0U)
            {
                __enable_irq();
            }
            return;
        }
        Gyro_Status.Transfering_Timestamp = SYS_Timestamp.Get_Now_Microsecond();
        BMI088_Gyro.SPI_Request_Gyro();
        if (primask == 0U)
        {
            __enable_irq();
        }
        return;
    }

    if (Hardware_Init_Finished_Flag && Temperature_Status.Transfering_Flag && (SYS_Timestamp.Get_Now_Microsecond() - Temperature_Status.Transfering_Timestamp) >= TRANSFERING_TIMEOUT)
    {
        // 温度传输超时
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        if (HAL_SPI_GetState(SPI_Manage_Object->SPI_Handler) != HAL_SPI_STATE_READY ||
            !Temperature_Status.Transfering_Flag)
        {
            if (primask == 0U)
            {
                __enable_irq();
            }
            return;
        }
        Temperature_Status.Transfering_Timestamp = SYS_Timestamp.Get_Now_Microsecond();
        BMI088_Accel.SPI_Request_Temperature();
        if (primask == 0U)
        {
            __enable_irq();
        }
        return;
    }
}

bool Class_BMI088::Validate_Vector(const Class_Matrix_f32<3, 1> &Vector) const
{
    return !Basic_Math_Is_Invalid_Float(Vector[0][0]) &&
           !Basic_Math_Is_Invalid_Float(Vector[1][0]) &&
           !Basic_Math_Is_Invalid_Float(Vector[2][0]);
}

bool Class_BMI088::Prepare_Accel_Sample(const Struct_BMI088_Sample &Sample)
{
    if (!Sample.Valid || !Validate_Vector(Sample.Value))
    {
        return false;
    }

    Vector_Original_Accel = Sample.Value;
    if (BMI088_Accel.Get_Heater_Enable())
    {
        Vector_Original_Accel = Class_Matrix_f32<3, 3>(ACCEL_AFFINE_DATA) * Vector_Original_Accel +
                                Class_Matrix_f32<3, 1>(ACCEL_BIAS_DATA) * GRAVITY_ACCELERATION;
    }

    Accel_Norm = Vector_Original_Accel.Get_Modulus<3, 1>();
    if (Basic_Math_Is_Invalid_Float(Accel_Norm) || Accel_Norm <= ACCEL_NORMALIZATION_MIN_NORM)
    {
        return false;
    }

    Vector_Normalized_Accel = (1.0f / Accel_Norm) * Vector_Original_Accel;
    return Validate_Vector(Vector_Normalized_Accel);
}

void Class_BMI088::Process_Gyro_Calibration(const Struct_BMI088_Sample &Sample)
{
    if (!Sample.Valid || !Validate_Vector(Sample.Value))
    {
        Gyro_Calibration_Progress = 0;
        Vector_Gyro_Calibration_Sum = Class_Matrix_f32<3, 1>();
        return;
    }

    Vector_Raw_Gyro = Sample.Value;
    const float gyro_norm = Vector_Raw_Gyro.Get_Modulus<3, 1>();
    const bool accel_stationary = Accel_Calibration_Valid &&
                                  Accel_Norm >= CALIBRATION_ACCEL_MIN_RATIO * GRAVITY_ACCELERATION &&
                                  Accel_Norm <= CALIBRATION_ACCEL_MAX_RATIO * GRAVITY_ACCELERATION;
    const bool gyro_stationary = !Basic_Math_Is_Invalid_Float(gyro_norm) &&
                                 gyro_norm <= GYRO_CALIBRATION_OMEGA_THRESHOLD;

    if (!accel_stationary || !gyro_stationary)
    {
        Gyro_Calibration_Progress = 0;
        Vector_Gyro_Calibration_Sum = Class_Matrix_f32<3, 1>();
        return;
    }

    Vector_Gyro_Calibration_Sum = Vector_Gyro_Calibration_Sum + Vector_Raw_Gyro;
    Gyro_Calibration_Progress++;
    if (Gyro_Calibration_Progress >= GYRO_CALIBRATION_SAMPLE_COUNT)
    {
        Vector_Gyro_Bias = (1.0f / static_cast<float>(GYRO_CALIBRATION_SAMPLE_COUNT)) * Vector_Gyro_Calibration_Sum;
        Vector_Original_Gyro = Vector_Raw_Gyro - Vector_Gyro_Bias;
        Vector_Pre_Valid_Gyro = Vector_Original_Gyro;
        Gyro_Calibration_Finished_Flag = true;
    }
}

void Class_BMI088::Initialize_EKF(uint64_t Timestamp)
{
    Class_Matrix_f32<3, 3> matrix_q(EKF_Q);
    Class_Matrix_f32<3, 3> matrix_r(EKF_R);
    Class_Matrix_f32<4, 4> matrix_p = Namespace_ALG_Matrix::Identity<4, 4>();
    const float init_euler[3] = {
        0.0f,
        asinf(Basic_Math_Constrain(-Vector_Normalized_Accel[0][0], -1.0f, 1.0f)),
        atan2f(Vector_Normalized_Accel[1][0], Vector_Normalized_Accel[2][0])};
    const Class_Matrix_f32<4, 1> vector_x =
        Namespace_ALG_Quaternion::From_Euler_Angle(Class_Matrix_f32<3, 1>(init_euler)).Get_Normalization();

    EKF_Quaternion.Init(matrix_q, matrix_r, matrix_p, vector_x);
    EKF_Quaternion.Config_Nonlinear_State_Model(EKF_Function_F, EKF_Function_Jacobian_F_X, EKF_Function_Jacobian_F_W);
    EKF_Quaternion.Config_Nonlinear_Measurement_Model(EKF_Function_H, EKF_Function_Jacobian_H_X, EKF_Function_Jacobian_H_V);
    EKF_Quaternion.Vector_X_Prior = vector_x;
    EKF_Quaternion.Matrix_P_Prior = matrix_p;
    EKF_Pre_Valid_Timestamp = Timestamp;
    EKF_Init_Finished_Flag = true;
}

bool Class_BMI088::Predict_To_Timestamp(uint64_t Timestamp, const Class_Matrix_f32<3, 1> &Gyro)
{
    if (!EKF_Init_Finished_Flag)
    {
        return false;
    }
    if (Timestamp < EKF_Pre_Valid_Timestamp)
    {
        Invalid_D_T_Count++;
        return false;
    }
    if (Timestamp == EKF_Pre_Valid_Timestamp)
    {
        EKF_Quaternion.Vector_X_Prior = EKF_Quaternion.Vector_X;
        EKF_Quaternion.Matrix_P_Prior = EKF_Quaternion.Matrix_P;
        return true;
    }

    Valid_D_T = static_cast<float>(Timestamp - EKF_Pre_Valid_Timestamp) / 1000000.0f;
    if (Basic_Math_Is_Invalid_Float(Valid_D_T) || Valid_D_T <= 0.0f || Valid_D_T > D_T_TIMEOUT_THRESHOLD)
    {
        Invalid_D_T_Count++;
        EKF_Init_Finished_Flag = false;
        return false;
    }

    Last_Valid_D_T = Valid_D_T;
    EKF_Quaternion.Set_D_T(Valid_D_T);
    EKF_Quaternion.Vector_U = Gyro;
    EKF_Quaternion.TIM_Predict_PeriodElapsedCallback();
    Normalize_EKF_State(true);
    if (!EKF_Init_Finished_Flag)
    {
        return false;
    }
    EKF_Pre_Valid_Timestamp = Timestamp;
    return true;
}

void Class_BMI088::Apply_Gyro_Sample(uint64_t Timestamp, const Class_Matrix_f32<3, 1> &Gyro)
{
    if (EKF_Init_Finished_Flag)
    {
        Predict_To_Timestamp(Timestamp, Vector_Pre_Valid_Gyro);
    }

    // 迟到样本不能回滚EKF，但仍是后续区间可用的最新角速度。
    Vector_Pre_Valid_Gyro = Gyro;
    Vector_Original_Gyro = Gyro;
}

void Class_BMI088::Update_With_Accel(uint64_t Timestamp)
{
    (void)Timestamp;
    Accel_Chi_Square_Calculate();
    if (!Basic_Math_Is_Invalid_Float(Accel_Chi_Square_Loss) &&
        Accel_Chi_Square_Loss <= ACCEL_CHI_SQUARE_TEST_THRESHOLD)
    {
        EKF_Quaternion.Vector_Z = Vector_Normalized_Accel;
        EKF_Quaternion.TIM_Update_PeriodElapsedCallback();
        Normalize_EKF_State(false);
        Accel_Update_Accepted = EKF_Init_Finished_Flag;
    }
}

void Class_BMI088::Normalize_EKF_State(bool Include_Prior)
{
    const float norm = EKF_Quaternion.Vector_X.Get_Modulus<4, 1>();
    if (Basic_Math_Is_Invalid_Float(norm) || norm <= Matrix_Compare_Epsilon)
    {
        EKF_Init_Finished_Flag = false;
        return;
    }

    const Class_Matrix_f32<4, 1> state_before = EKF_Quaternion.Vector_X;
    const float inverse_norm = 1.0f / norm;
    Class_Matrix_f32<4, 4> normalization_jacobian =
        inverse_norm * (Namespace_ALG_Matrix::Identity<4, 4>() -
                        inverse_norm * inverse_norm * state_before * state_before.Get_Transpose());
    EKF_Quaternion.Vector_X = inverse_norm * state_before;
    EKF_Quaternion.Matrix_P = normalization_jacobian * EKF_Quaternion.Matrix_P * normalization_jacobian.Get_Transpose();

    if (Include_Prior)
    {
        EKF_Quaternion.Vector_X_Prior = EKF_Quaternion.Vector_X;
        EKF_Quaternion.Matrix_P_Prior = EKF_Quaternion.Matrix_P;
    }
}

void Class_BMI088::Publish_Output(uint64_t Now_Timestamp)
{
    if (!EKF_Init_Finished_Flag)
    {
        return;
    }

    float output_dt = 0.0f;
    if (Now_Timestamp >= EKF_Pre_Valid_Timestamp)
    {
        output_dt = static_cast<float>(Now_Timestamp - EKF_Pre_Valid_Timestamp) / 1000000.0f;
    }
    if (output_dt > D_T_TIMEOUT_THRESHOLD)
    {
        output_dt = 0.0f;
    }

    Quarternion = EKF_Function_F(EKF_Quaternion.Vector_X, Vector_Original_Gyro, output_dt).Get_Normalization();
    Vector_Euler_Angle = Quarternion.Get_Euler_Angle();
    Matrix_Rotation = Quarternion.Get_Rotation_Matrix();
    Vector_Axis_Angle = Quarternion.Get_Axis_Angle();
    const Class_Matrix_f32<3, 1> vector_gravity_body =
        Matrix_Rotation.Get_Transpose() * (-Namespace_ALG_Matrix::Axis_Z_3d() * GRAVITY_ACCELERATION);
    Vector_Accel_Body = Vector_Original_Accel + vector_gravity_body;
    Vector_Accel_Odom = Matrix_Rotation * Vector_Accel_Body;
    Vector_Gyro_Body = Vector_Original_Gyro;
    Vector_Gyro_Odom = Matrix_Rotation * Vector_Gyro_Body;
}

Struct_BMI088_Attitude_Snapshot Class_BMI088::Get_Attitude_Snapshot() const
{
    Struct_BMI088_Attitude_Snapshot result;
    const uint64_t now_timestamp = SYS_Timestamp.Get_Now_Microsecond();
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    result.Euler_Angle = Vector_Euler_Angle;
    result.Gyro_Body = Vector_Gyro_Body;
    result.Quaternion = Quarternion;
    result.Valid = Hardware_Init_Finished_Flag &&
                   Gyro_Calibration_Finished_Flag &&
                   EKF_Init_Finished_Flag &&
                   BMI088_Gyro.Get_Valid_Flag() &&
                   Gyro_Sample.Valid &&
                   now_timestamp >= Gyro_Sample.Timestamp &&
                   static_cast<float>(now_timestamp - Gyro_Sample.Timestamp) / 1000000.0f <= D_T_TIMEOUT_THRESHOLD;
    if (primask == 0U)
    {
        __enable_irq();
    }
    return result;
}

bool Class_BMI088::Get_Attitude_Valid() const
{
    return Get_Attitude_Snapshot().Valid;
}

Struct_BMI088_Diagnostics Class_BMI088::Get_Diagnostics() const
{
    Struct_BMI088_Diagnostics result;
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    result.Raw_Gyro = Vector_Raw_Gyro;
    result.Corrected_Gyro = Vector_Original_Gyro;
    result.Gyro_Bias = Vector_Gyro_Bias;
    result.Last_D_T = Last_Valid_D_T;
    result.Accel_Norm = Accel_Norm;
    result.Accel_Chi_Square_Loss = Accel_Chi_Square_Loss;
    result.Temperature = BMI088_Accel.Get_Now_Temperature();
    result.Calibration_Progress = Gyro_Calibration_Progress;
    result.Calibration_Target = GYRO_CALIBRATION_SAMPLE_COUNT;
    result.Dropped_Gyro_Samples = Dropped_Gyro_Samples;
    result.Dropped_Accel_Samples = Dropped_Accel_Samples;
    result.Invalid_D_T_Count = Invalid_D_T_Count;
    result.Calibration_Finished = Gyro_Calibration_Finished_Flag;
    result.Accel_Update_Accepted = Accel_Update_Accepted;
    if (primask == 0U)
    {
        __enable_irq();
    }
    return result;
}

/**
 * @brief 四元数状态转移函数
 *
 * @param Vector_X 状态向量
 * @param Vector_U 输入向量
 */
Class_Matrix_f32<4, 1> Class_BMI088::EKF_Function_F(const Class_Matrix_f32<4, 1> &Vector_X, const Class_Matrix_f32<3, 1> &Vector_U, const float &D_T)
{
    Class_Matrix_f32<4, 1> matrix_result;

    // 角速度矩阵
    Class_Matrix_f32<4, 4> matrix_omega;
    matrix_omega[0][0] = 0.0f;
    matrix_omega[0][1] = -Vector_U[0][0];
    matrix_omega[0][2] = -Vector_U[1][0];
    matrix_omega[0][3] = -Vector_U[2][0];
    matrix_omega[1][0] = Vector_U[0][0];
    matrix_omega[1][1] = 0.0f;
    matrix_omega[1][2] = Vector_U[2][0];
    matrix_omega[1][3] = -Vector_U[1][0];
    matrix_omega[2][0] = Vector_U[1][0];
    matrix_omega[2][1] = -Vector_U[2][0];
    matrix_omega[2][2] = 0.0f;
    matrix_omega[2][3] = Vector_U[0][0];
    matrix_omega[3][0] = Vector_U[2][0];
    matrix_omega[3][1] = Vector_U[1][0];
    matrix_omega[3][2] = -Vector_U[0][0];
    matrix_omega[3][3] = 0.0f;

    const Class_Matrix_f32<4, 1> quaternion_tmp = Vector_X + 0.5f * D_T * matrix_omega * Vector_X;
    matrix_result = quaternion_tmp.Get_Normalization<4, 1>();

    return matrix_result;
}

/**
 * @brief 四元数状态转移函数对状态的雅可比矩阵
 *
 * @param Vector_X 状态向量
 * @param Vector_U 输入向量
 */
Class_Matrix_f32<4, 4> Class_BMI088::EKF_Function_Jacobian_F_X(const Class_Matrix_f32<4, 1> &Vector_X, const Class_Matrix_f32<3, 1> &Vector_U, const float &D_T)
{
    Class_Matrix_f32<4, 4> matrix_result;

    // 角速度矩阵
    Class_Matrix_f32<4, 4> matrix_omega;
    matrix_omega[0][0] = 0.0f;
    matrix_omega[0][1] = -Vector_U[0][0];
    matrix_omega[0][2] = -Vector_U[1][0];
    matrix_omega[0][3] = -Vector_U[2][0];
    matrix_omega[1][0] = Vector_U[0][0];
    matrix_omega[1][1] = 0.0f;
    matrix_omega[1][2] = Vector_U[2][0];
    matrix_omega[1][3] = -Vector_U[1][0];
    matrix_omega[2][0] = Vector_U[1][0];
    matrix_omega[2][1] = -Vector_U[2][0];
    matrix_omega[2][2] = 0.0f;
    matrix_omega[2][3] = Vector_U[0][0];
    matrix_omega[3][0] = Vector_U[2][0];
    matrix_omega[3][1] = Vector_U[1][0];
    matrix_omega[3][2] = -Vector_U[0][0];

    Class_Matrix_f32<4, 4> transition =
        Namespace_ALG_Matrix::Identity<4, 4>() + 0.5f * D_T * matrix_omega;
    const Class_Matrix_f32<4, 1> quaternion_tmp = transition * Vector_X;
    const float modulus = quaternion_tmp.Get_Modulus<4, 1>();
    if (Basic_Math_Is_Invalid_Float(modulus) || modulus <= Matrix_Compare_Epsilon)
    {
        return transition;
    }
    const float inverse_modulus = 1.0f / modulus;
    Class_Matrix_f32<4, 4> normalization_jacobian =
        inverse_modulus * (Namespace_ALG_Matrix::Identity<4, 4>() -
                           inverse_modulus * inverse_modulus * quaternion_tmp * quaternion_tmp.Get_Transpose());
    matrix_result = normalization_jacobian * transition;

    return matrix_result;
}

/**
 * @brief 四元数状态转移函数对过程噪声的雅可比矩阵
 *
 * @param Vector_X 状态向量
 * @param Vector_U 输入向量
 * @param D_T 时间间隔
 */
Class_Matrix_f32<4, 3> Class_BMI088::EKF_Function_Jacobian_F_W(const Class_Matrix_f32<4, 1> &Vector_X, const Class_Matrix_f32<3, 1> &Vector_U, const float &D_T)
{
    Class_Matrix_f32<4, 3> matrix_result;

    // 四元数矩阵
    Class_Matrix_f32<4, 3> matrix_q;
    matrix_q[0][0] = -Vector_X[1][0];
    matrix_q[0][1] = -Vector_X[2][0];
    matrix_q[0][2] = -Vector_X[3][0];
    matrix_q[1][0] = Vector_X[0][0];
    matrix_q[1][1] = -Vector_X[3][0];
    matrix_q[1][2] = Vector_X[2][0];
    matrix_q[2][0] = Vector_X[3][0];
    matrix_q[2][1] = Vector_X[0][0];
    matrix_q[2][2] = -Vector_X[1][0];
    matrix_q[3][0] = -Vector_X[2][0];
    matrix_q[3][1] = Vector_X[1][0];
    matrix_q[3][2] = Vector_X[0][0];

    Class_Matrix_f32<4, 4> matrix_omega;
    matrix_omega[0][0] = 0.0f;
    matrix_omega[0][1] = -Vector_U[0][0];
    matrix_omega[0][2] = -Vector_U[1][0];
    matrix_omega[0][3] = -Vector_U[2][0];
    matrix_omega[1][0] = Vector_U[0][0];
    matrix_omega[1][1] = 0.0f;
    matrix_omega[1][2] = Vector_U[2][0];
    matrix_omega[1][3] = -Vector_U[1][0];
    matrix_omega[2][0] = Vector_U[1][0];
    matrix_omega[2][1] = -Vector_U[2][0];
    matrix_omega[2][2] = 0.0f;
    matrix_omega[2][3] = Vector_U[0][0];
    matrix_omega[3][0] = Vector_U[2][0];
    matrix_omega[3][1] = Vector_U[1][0];
    matrix_omega[3][2] = -Vector_U[0][0];
    matrix_omega[3][3] = 0.0f;

    const Class_Matrix_f32<4, 1> quaternion_tmp =
        (Namespace_ALG_Matrix::Identity<4, 4>() + 0.5f * D_T * matrix_omega) * Vector_X;
    const float modulus = quaternion_tmp.Get_Modulus<4, 1>();
    const Class_Matrix_f32<4, 3> input_jacobian = 0.5f * D_T * matrix_q;
    if (Basic_Math_Is_Invalid_Float(modulus) || modulus <= Matrix_Compare_Epsilon)
    {
        return input_jacobian;
    }
    const float inverse_modulus = 1.0f / modulus;
    Class_Matrix_f32<4, 4> normalization_jacobian =
        inverse_modulus * (Namespace_ALG_Matrix::Identity<4, 4>() -
                           inverse_modulus * inverse_modulus * quaternion_tmp * quaternion_tmp.Get_Transpose());
    matrix_result = normalization_jacobian * input_jacobian;

    return matrix_result;
}

/**
 * @brief 四元数测量函数
 *
 * @param Vector_X 状态向量
 * @param D_T 时间间隔
 */
Class_Matrix_f32<3, 1> Class_BMI088::EKF_Function_H(const Class_Matrix_f32<4, 1> &Vector_X, const float &D_T)
{
    Class_Matrix_f32<3, 1> matrix_result;

    matrix_result[0][0] = 2.0f * (Vector_X[1][0] * Vector_X[3][0] - Vector_X[0][0] * Vector_X[2][0]);
    matrix_result[1][0] = 2.0f * (Vector_X[2][0] * Vector_X[3][0] + Vector_X[0][0] * Vector_X[1][0]);
    matrix_result[2][0] = Vector_X[0][0] * Vector_X[0][0] - Vector_X[1][0] * Vector_X[1][0] - Vector_X[2][0] * Vector_X[2][0] + Vector_X[3][0] * Vector_X[3][0];

    return matrix_result;
}

/**
 * @brief 四元数测量函数对状态的雅可比矩阵
 *
 * @param Vector_X 状态向量
 * @param D_T 时间间隔
 */
Class_Matrix_f32<3, 4> Class_BMI088::EKF_Function_Jacobian_H_X(const Class_Matrix_f32<4, 1> &Vector_X, const float &D_T)
{
    Class_Matrix_f32<3, 4> matrix_result;

    matrix_result[0][0] = -2.0f * Vector_X[2][0];
    matrix_result[0][1] = 2.0f * Vector_X[3][0];
    matrix_result[0][2] = -2.0f * Vector_X[0][0];
    matrix_result[0][3] = 2.0f * Vector_X[1][0];

    matrix_result[1][0] = 2.0f * Vector_X[1][0];
    matrix_result[1][1] = 2.0f * Vector_X[0][0];
    matrix_result[1][2] = 2.0f * Vector_X[3][0];
    matrix_result[1][3] = 2.0f * Vector_X[2][0];

    matrix_result[2][0] = 2.0f * Vector_X[0][0];
    matrix_result[2][1] = -2.0f * Vector_X[1][0];
    matrix_result[2][2] = -2.0f * Vector_X[2][0];
    matrix_result[2][3] = 2.0f * Vector_X[3][0];

    return matrix_result;
}

/**
 * @brief 四元数测量函数对测量噪声的雅可比矩阵
 *
 * @param Vector_X 状态向量
 * @param D_T 时间间隔
 */
Class_Matrix_f32<3, 3> Class_BMI088::EKF_Function_Jacobian_H_V(const Class_Matrix_f32<4, 1> &Vector_X, const float &D_T)
{
    return Namespace_ALG_Matrix::Identity<3, 3>();
}

/**
 * @brief 卡方检验
 *
 */
void Class_BMI088::Accel_Chi_Square_Calculate()
{
    Class_Matrix_f32<3, 1> vector_error;
    Class_Matrix_f32<3, 4> matrix_h_x;
    Class_Matrix_f32<3, 3> matrix_d;

    vector_error = Vector_Normalized_Accel - EKF_Function_H(EKF_Quaternion.Vector_X_Prior, Valid_D_T);

    matrix_h_x = EKF_Function_Jacobian_H_X(EKF_Quaternion.Vector_X_Prior, Valid_D_T);

    matrix_d = matrix_h_x * EKF_Quaternion.Matrix_P_Prior * matrix_h_x.Get_Transpose() + EKF_Quaternion.Matrix_R;

    Accel_Chi_Square_Loss = (vector_error.Get_Transpose() * matrix_d.Get_Inverse() * vector_error)[0][0];
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
