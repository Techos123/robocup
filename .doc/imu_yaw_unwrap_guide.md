# IMU Yaw 回绕与连续角处理指南

## 1. 现象与原因

本项目机器人坐标系规定：

- `x` 轴向前为正。
- `y` 轴向左为正。
- `z` 轴向上为正。
- Yaw 逆时针方向为正。

当 Yaw 使用 `[-180 deg, 180 deg]` 表示时，逆时针连续旋转一周的数值变化为：

```text
0 -> 180 -> -180 -> 0
```

因此，`180 deg -> -180 deg` 是同一方向的连续姿态在有限角度范围内发生回绕，不是 IMU 数据故障。若显示、多圈计数或控制算法需要连续角度，应在应用层进行 Yaw 展开。

## 2. 连续 Yaw 展开

保存上一帧回绕角度，并将相邻两帧的差值约束到 `[-180 deg, 180 deg]`：

```cpp
class Class_Yaw_Unwrapper
{
public:
    float Update(float yaw_wrapped)
    {
        if (!Initialized)
        {
            Previous_Yaw = yaw_wrapped;
            Continuous_Yaw = yaw_wrapped;
            Initialized = true;
            return Continuous_Yaw;
        }

        float delta = yaw_wrapped - Previous_Yaw;

        while (delta > 180.0f)
        {
            delta -= 360.0f;
        }
        while (delta < -180.0f)
        {
            delta += 360.0f;
        }

        Continuous_Yaw += delta;
        Previous_Yaw = yaw_wrapped;
        return Continuous_Yaw;
    }

    void Reset(float yaw_wrapped = 0.0f)
    {
        Previous_Yaw = yaw_wrapped;
        Continuous_Yaw = yaw_wrapped;
        Initialized = false;
    }

private:
    float Previous_Yaw = 0.0f;
    float Continuous_Yaw = 0.0f;
    bool Initialized = false;
};
```

示例：

```text
回绕角度: 178, 179, -179, -178
连续角度: 178, 179,  181,  182
```

连续旋转多圈后，输出可以继续增长到 `360 deg`、`720 deg`，也可以向负方向减小。

## 3. 最短角度误差

若闭环控制只需要目标方向与当前方向之间的最短转动距离，不需要多圈角度，应计算回绕后的角度误差：

```cpp
float Get_Shortest_Yaw_Error(float target_yaw, float current_yaw)
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
```

例如，目标角为 `-179 deg`、当前角为 `179 deg` 时，最短误差为 `2 deg`，而不是 `-358 deg`。

## 4. 接入建议

Yaw 展开应在姿态解算更新后立即执行，优先放在 TIM8 的 125 us 姿态更新链路中，而不是只处理 100 Hz USB 输出。这样控制器、记录器和 USB 输出可以共享同一份连续角度。

建议同时保留两种数据：

- `Yaw_Wrapped`：范围为 `[-180 deg, 180 deg]`，用于常规姿态显示和最短角度控制。
- `Yaw_Continuous`：无固定范围，用于多圈计数、累计转角和连续曲线显示。

只有在 BMI088 姿态有效后才能初始化展开器。EKF 重置、IMU重新初始化或时间戳异常后，应重新初始化展开器，避免把无效数据累计到连续角度中。

## 5. 滤波注意事项

不能直接对回绕后的 Yaw 做普通低通滤波。例如 `179 deg` 与 `-179 deg` 的普通平均值接近 `0 deg`，但实际方向接近 `180 deg`。

可采用以下方式之一：

- 先展开为连续 Yaw，再进行低通滤波。
- 对 `sin(yaw)` 和 `cos(yaw)` 分别滤波，再通过 `atan2` 恢复角度。
- 内部姿态估计和坐标变换保持使用四元数，只在输出层转换为欧拉角。

## 6. 使用限制

- 相邻两次更新之间的真实转角必须小于 `180 deg`，否则无法仅根据两帧回绕角判断实际转动方向。应在 IMU 姿态更新频率下执行展开，而不是使用低频显示数据。
- Yaw 展开只能消除 `180 deg -> -180 deg` 的表示突变，不能消除陀螺仪噪声、零偏和积分误差。
- 当前 BMI088 融合没有磁力计或其他绝对航向观测，连续 Yaw 仍会随时间漂移。若需要长期稳定的绝对航向，必须增加磁力计、视觉、里程计或其他航向观测源。
- Pitch 接近 `+/-90 deg` 时，欧拉角存在万向节锁问题。内部控制和三维姿态运算应优先使用四元数或旋转矩阵。
