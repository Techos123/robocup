# BMI088 采样时序与四元数 EKF 修改合并指南

## 1. 文档目的

本文记录 `串口调参` 分支中针对 BMI088 姿态解算完成的修改，供实机测试通过后选择性并入 `main` 分支。

本文对应的工作区基线如下：

- 功能开发分支：`串口调参`
- 功能分支基点：`a7bdf3a`（新增串口调参）
- 编写本文时的 `main`：`84c1eb6`
- `main` 已比功能分支基点多两个提交：`4441c7f`、`84c1eb6`
- 当前功能修改尚未提交，不应直接执行整分支 merge

由于 `main` 已继续开发，而且工作区还存在与 IMU 无关的底盘 PID 调参，推荐拆分提交后选择性 cherry-pick，并人工处理 `main.c`。

## 2. 修改目标

本次修改解决以下问题：

1. IMU 数据值和时间戳不是同一完整样本，可能重复消费或使用错配数据。
2. EKF 使用固定任务周期代替真实采样间隔，采样抖动会转化为姿态积分误差。
3. 静止陀螺零偏未经启动标定，Yaw 会持续单向漂移。
4. 四元数预测和更新后缺少一致的归一化及协方差投影。
5. 动态加速度可能被当作重力观测修正姿态。
6. 主循环分别读取欧拉角和角速度，存在跨中断读取不一致。
7. UART DMA 忙时仍可能覆盖共享发送缓冲区。
8. TIM5 溢出边界可能返回不一致的微秒时间戳。

## 3. 保持不变的设计约束

合并时不得无意修改以下约束：

- EKF 状态仍为 4 状态四元数，欧拉角只作为输出。
- 不增加磁力计、视觉或其他绝对航向观测。
- 温控保持关闭。
- `Q = diag(0.865, 0.975, 1.077)`。
- `R = diag(0.446, 0.476, 0.537)`。
- 加速度卡方门限保持 `0.5`。
- `dt` 超时门限保持 `0.1 s`。
- 机器人坐标系和欧拉角符号继续遵守项目 `AGENTS.md`。

没有绝对航向观测时，Yaw 只能依靠陀螺积分。启动零偏标定可以显著减小漂移，但不能保证长时间绝对不漂移。

## 4. 实现概要

### 4.1 完整样本发布

加速度计和陀螺仪分别维护完整样本：

- 三轴数据；
- DRDY 时间戳；
- 单调递增序号；
- 有效标志。

EXTI 到达时记录 DRDY 时间戳；开始 SPI 读取时锁定该时间戳；SPI 完成后先写入数据、时间戳和有效性，最后递增序号发布样本。

125 us 融合任务在短临界区内复制完整样本和判断序号，只消费新样本，并统计序号跳变造成的丢样。

### 4.2 真实 `dt` 和事件顺序

EKF 使用相邻传感器样本时间戳之差计算真实 `dt`，不再假设每次都严格等于 125 us。

当同一融合周期同时得到加速度和陀螺样本时：

- 按样本时间戳先后顺序处理；
- 区间积分使用上一份有效陀螺数据做零阶保持；
- 新陀螺样本只作用于其采样时刻之后的区间；
- 迟到样本不能回滚已经推进的 EKF 状态。

若 `dt <= 0`、时间戳倒退或 `dt > 0.1 s`，记录异常；超时会使 EKF 失效并等待有效加速度重新初始化。

### 4.3 启动静止零偏标定

上电后累计 4096 份满足静止条件的陀螺样本：

- 陀螺模长不超过 `5 deg/s`；
- 加速度模长位于 `0.85g` 到 `1.15g`；
- 加速度和陀螺样本均必须有效。

任一静止条件不满足时，标定进度和累加和清零。标定完成后：

```text
gyro_corrected = gyro_raw - runtime_bias
```

BMI088 陀螺配置为 2000 Hz，因此持续静止时标定约需 2.05 s。标定完成前姿态反馈无效，不进入依赖姿态的底盘控制。

### 4.4 四元数 EKF

- 使用加速度方向初始化 Pitch 和 Roll，初始 Yaw 为 0。
- 每次预测和接受观测更新后归一化四元数。
- 使用归一化雅可比将协方差投影回单位四元数切空间。
- 状态和协方差的 prior/posterior 保持同步。
- 状态雅可比和噪声雅可比不再依赖其他函数的静态临时变量或调用顺序。

### 4.5 动态加速度剔除

加速度先检查有限性和非零模长，再归一化。

动态加速度仍使用原卡方门限判断：

```text
chi_square_loss <= 0.5  -> 接受加速度更新
chi_square_loss > 0.5   -> 仅陀螺预测，拒绝本次加速度更新
```

没有增加新的加速度融合门限。

### 4.6 姿态快照和失效判断

主循环通过一次快照读取欧拉角、机体系角速度、四元数和有效性，避免分别读取时跨越中断更新。

姿态有效需要同时满足：

- BMI088 硬件初始化完成；
- 启动零偏标定完成；
- EKF 有效；
- 最近陀螺样本有效；
- 最近有效陀螺样本不超过原 `0.1 s` 超时门限。

陀螺停止上报超过 0.1 s 后，底盘不会继续使用旧姿态。

### 4.7 温度采样

TIM8 每 125 us 进入融合回调，并以 1024 分频触发一次 128 ms 回调，用于读取 BMI088 温度。

温控仍然关闭；只有 `Get_Heater_Enable()` 为真时才调用加热 PID 回调。

### 4.8 UART 输出

当前串口输出周期为 20 ms，即 50 Hz，只保留以下 9 路：

| 通道 | 变量 | 含义 |
| ---: | --- | --- |
| 1 | `V` | 上位机设定线速度 |
| 2 | `now_velocity_x` | 当前 x 方向速度 |
| 3 | `W` | 上位机设定角速度 |
| 4 | `omega` | 当前角速度 |
| 5 | `A` | 当前调参逻辑中的辅助设定量 |
| 6 | `Yaw` | Yaw，单位 deg |
| 7 | `Pitch` | Pitch，单位 deg |
| 8 | `Roll` | Roll，单位 deg |
| 9 | `current_sum` | 当前电流和 |

JustFloat 单帧为 `9 * 4 + 4 = 40` 字节。在 115200 baud 下发送时间约 3.47 ms，小于 20 ms 周期。

串口通用类仍保留最多 32 通道的能力，便于临时恢复 IMU 诊断通道。UART DMA 忙检查位于写共享缓冲区之前，忙时跳过该帧并累计计数。

## 5. 涉及文件

### 5.1 建议作为 IMU 核心提交

| 文件 | 修改内容 |
| --- | --- |
| `Code/1_Middleware/3_System/Timestamp/sys_timestamp.cpp` | TIM5 溢出边界一致性读取 |
| `Code/1_Middleware/3_System/Timestamp/sys_timestamp.h` | 溢出计数声明为 `volatile` |
| `Code/2_Device/BSP/BMI088/bsp_bmi088.cpp` | 完整样本、真实 dt、零偏标定、四元数 EKF、门控和快照 |
| `Code/2_Device/BSP/BMI088/bsp_bmi088.h` | 新样本、快照、诊断结构和状态成员 |

这 4 个文件构成 IMU 核心，建议单独提交。

### 5.2 建议作为串口和应用接入提交

| 文件 | 修改内容 |
| --- | --- |
| `Code/2_Device/Synex/dvc_synex.cpp` | DMA 忙检查前移和跳帧计数 |
| `Code/2_Device/Synex/dvc_synex.h` | 通道上限 32、编译期通道检查、跳帧计数读取 |
| `Core/Src/main.c` | TIM8 分频、姿态快照、底盘反馈和 9 路 50 Hz 输出 |

`main.c` 与串口调参、底盘控制模式高度耦合，合入 `main` 时必须按片段审查，不建议无条件覆盖整个文件。

### 5.3 不属于本次 IMU 修改

当前工作区中的以下文件不得随 IMU 提交一起加入暂存区：

- `Code/3_Chariot/1_Module/Chassis/Four_whell/crt_fw.cpp`

该文件包含 PID 前馈、积分上限和输出上限调参，与 IMU 时序/EKF 修改无关，应独立测试和独立提交。

当前分支相对 `main` 还存在历史提交中的 `.doc/MOTION_PROTOCOL_USAGE.md` 和 `CMakeLists.txt` 差异。它们也不应通过整分支 merge 被顺带合入。

## 6. `main.c` 合并注意事项

合并 `main.c` 时优先保留 `main` 分支已经存在的 ROS、运动协议和底盘控制逻辑，只移植下列 IMU 接入片段：

1. `BMI088_Temperature_Divider` 变量。
2. TIM8 回调中的 1024 分频和 128 ms BMI088 回调。
3. 主循环中的 `Get_Attitude_Snapshot()` 一致性读取。
4. 快照有效时生成 `Struct_Chassis_Attitude_Feedback`。
5. 20 ms 串口节流和 9 路 `Set_Data()`。

当前 `main.c` 还包含串口调参分支自己的行为，合并前需要明确是否保留：

- 新增上位机变量 `W` 和命令字符串 `"w"`；
- `A` 和 `W` 当前都调用 `car.Set_Target_Omega()`；
- 底盘 Yaw 控制模式被设置为 `Chassis_Yaw_Control_Type_OMEGA`；
- 串口接收 switch 的索引因新增 `w` 整体后移。

这些属于控制/调参策略，不是 IMU 核心算法。若 `main` 仍使用角度环或 ROS 指令，应以 `main` 的控制模式为准。

当前代码还有一个合并前必须复核的问题：命令列表已有 13 项（索引 0 到 12），但 `synex.Init()` 传入的接收变量数量仍是 12。若保留新增的 `w` 和索引 12 的 `stop`，应在独立修改计划批准后把数量改为 13，否则最后一项不能被正确匹配。

## 7. 推荐的提交拆分

实机测试通过后，先在 `串口调参` 分支拆成至少两个提交。不要使用 `git add .`。

### 7.1 IMU 核心提交

```powershell
git add -- `
  Code/1_Middleware/3_System/Timestamp/sys_timestamp.cpp `
  Code/1_Middleware/3_System/Timestamp/sys_timestamp.h `
  Code/2_Device/BSP/BMI088/bsp_bmi088.cpp `
  Code/2_Device/BSP/BMI088/bsp_bmi088.h

git diff --cached --check
git diff --cached --stat
git commit -m "完善BMI088采样时序和四元数EKF"
```

### 7.2 串口底层提交

```powershell
git add -- `
  Code/2_Device/Synex/dvc_synex.cpp `
  Code/2_Device/Synex/dvc_synex.h

git diff --cached --check
git diff --cached --stat
git commit -m "完善串口DMA忙保护"
```

### 7.3 应用接入提交

`main.c` 建议使用交互式分块暂存，逐块确认没有带入控制策略改动：

```powershell
git add -p Core/Src/main.c
git diff --cached --check
git diff --cached
git commit -m "接入BMI088姿态快照和串口输出"
```

文档可独立提交：

```powershell
git add -- .doc/IMU_TIMING_EKF_MAIN_MERGE_GUIDE.md
git commit -m "补充BMI088修改合并指南"
```

提交前确认 `crt_fw.cpp` 不在暂存区：

```powershell
git diff --cached --name-only
```

## 8. 合入 `main` 的推荐流程

以下命令仅应在实机测试完成、工作区已妥善提交或暂存后执行。

```powershell
git switch main
git pull --ff-only
git switch -c integrate/bmi088-timing-ekf
```

按顺序 cherry-pick：

```powershell
git cherry-pick <IMU核心提交>
git cherry-pick <串口底层提交>
git cherry-pick <应用接入提交>
git cherry-pick <文档提交>
```

若 `main.c` 冲突，不要选择整文件的 ours/theirs。应保留 `main` 的初始化、ROS 和控制逻辑，再手工移植第 6 节列出的 IMU 接入片段。

冲突处理完成后：

```powershell
git add Core/Src/main.c
git cherry-pick --continue
```

最终通过代码审查和实机验证后，再把 `integrate/bmi088-timing-ekf` 合入 `main`。

## 9. 编译验证

本功能曾使用项目 `CMakePresets.json` 中的 `Debug` preset 和 STM32Cube 自带工具链成功编译链接：

- ELF：`build/Debug/robocup.elf`
- FLASH：84,460 B / 1 MB，8.05%
- DTCMRAM：20,552 B / 128 KB，15.68%
- RAM_D2：9,504 B / 32 KB，29.00%
- 唯一已知编译警告来自 CMSIS 的 `optimize` 属性，与本次修改无关

每次 cherry-pick 或解决冲突后都应重新编译。使用项目 STM32 构建技能或 STM32Cube 自带工具链，不要改用系统 CMake/GCC。

## 10. 实机验收清单

### 10.1 启动和标定

- 上电后保持底盘完全静止至少 3 s。
- 确认标定期间姿态反馈无效，底盘不会进入依赖姿态的控制。
- 确认移动底盘会使 4096 点标定重新开始。
- 确认标定完成后校正 gyro 三轴静止值接近 0。

如需观察标定进度，可临时恢复 `Get_Diagnostics()` 对应通道，但不要改变 9 路正式协议顺序。

### 10.2 坐标系和欧拉角

- Yaw 逆时针为正，范围表现为 `0 -> 180 -> -180 -> 0`。
- Pitch 按项目定义检查正负方向和约 `+/-80 deg` 范围。
- Roll 按项目定义检查正负方向和 `+/-180 deg` 跳变。
- 静止 5 min 记录 Yaw 漂移速率；允许缓慢漂移，但不应再快速单向增长。

### 10.3 动态测试

- 快速平移但不旋转，Pitch/Roll 不应被线性加速度明显拉偏。
- 原地旋转，Yaw 方向和底盘反馈方向应一致。
- 振动环境中检查姿态是否出现跳变或 NaN。
- 暂停或断开陀螺数据后，0.1 s 内姿态反馈应失效。

### 10.4 时序和通信

- 正常运行时真实 `dt` 应接近陀螺 2000 Hz 的约 0.5 ms，并允许小幅抖动。
- 丢样计数和异常 `dt` 计数通常应保持 0。
- 9 路串口通道顺序必须与第 4.8 节一致。
- 50 Hz 连续输出时不应出现 DMA 缓冲区破坏或帧尾错位。

## 11. 回滚边界

若合入后出现问题，按提交粒度回滚：

1. 先回滚应用接入提交，恢复 `main` 原有控制和串口路径。
2. 再回滚串口底层提交。
3. 最后才回滚 IMU 核心提交。

不要只回滚 `bsp_bmi088.cpp` 而保留新的头文件结构，也不要只回滚时间戳实现，否则可能导致接口或行为不一致。
