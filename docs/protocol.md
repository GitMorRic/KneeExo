# 串口协议与运行参数

[返回 README](../README.md) · [PC 工具](../tools/README.md)

当前协议为换行结束的 ASCII 文本，与 ESP-IDF 日志共用主控制台；PC 默认 115200 波特率。没有二进制长度、序号或 CRC 封装。不要将概念图当成另一种已实现的帧格式。

## 遥测

normal 默认输出 `$EXO` 约50 Hz、`$IMU2` 约25 Hz，文本控制日志每0.5秒一条；控制循环默认100 Hz。读取的是驱动缓存，不表示 IMU 物理采样率等于输出率。

```text
$EXO,t_us,state,shank_deg,shank_rate_dps,acc_g,knee_rad,knee_vel,tau_g,tau_imp,tau_cmd,tau_fb,k,b,theta_eq,landing,flex_peak,shank_cross,bat_v,motor_age_ms,motor_ok,impact_g
$IMU2,t_us,ax,ay,az,gx,gy,gz,roll,pitch,yaw,T
```

上面是字段说明；normal不额外发送这个表头。`$EXO` 当前有21个负载字段（不含 `$EXO`）；旧固件有17/18/20字段。`imu_only` 会输出以`#`开头的表头及有效通道的`$IMU1/$IMU2`，流速最高25 Hz。

| 字段 | 含义/单位 |
|---|---|
| `t_us` | MCU 启动后的微秒时间；不能当 UTC 日期 |
| `state` | `SAFE / IDS / SINGLE / ISWING / MSWING / TSWING` |
| `shank_deg`、`shank_rate_dps` | 校正后小腿角度（°）和滤波角速度（°/s） |
| `acc_g`、`impact_g` | 加速度模长与高通冲击分量，单位 g |
| `knee_rad`、`knee_vel` | 用户零位后的膝角（rad）、屈膝为正的角速度（rad/s） |
| `tau_g` | 补偿力矩（Nm） |
| `tau_imp` | 状态阻抗分量；手动模式时复用为手动力矩/位置阻抗分量（Nm） |
| `tau_cmd` | 经软限位目标裁剪及斜率限制后的最终关节命令力矩（Nm） |
| `tau_fb` | 电机驱动器估计的关节坐标力矩（Nm），反馈过期时 NaN |
| `k / b / theta_eq` | 所选状态的刚度 Nm/rad、阻尼 Nm·s/rad、平衡角 rad；手动模式未改成手动参数显示 |
| `landing / flex_peak / shank_cross` | 本周期事件标志0或1 |
| `bat_v` | ADC估算电池电压；分压和校准未核实时不能视为准确值 |
| `motor_age_ms / motor_ok` | 电机反馈年龄及是否小于120 ms；不是电机使能确认 |

IMU 字段：加速度 g、角速度 °/s、欧拉角 °、温度 °C。normal 的 `t_us` 是控制任务打印时间，不能据此判断底层 IMU 样本新鲜度。

## 固件身份（本次新增）

所有 profile 启动时输出：

```text
$INFO,app_version=<IDF项目版本>,profile=NORMAL,wear_mode=TRANSPARENT,build_date=<日期>,build_time=<时间>,idf=<IDF版本>
```

normal 支持 `$CMD,GET_INFO`，返回同样的 `$INFO`，不改变电机状态。非normal的 `wear_mode=NA`。`app_version` 来自 ESP-IDF 镜像描述，通常为 Git describe；`dirty` 只代表构建时工作区有改动。构建日期/时间无时区声明。其他 profile 没有命令任务，须看启动输出。

## 命令与响应

命令发往**主控制台**，以 `\n` 结尾。响应为 `$ACK,...` 或 `$ERR,...`，`GET_INFO` 特例返回 `$INFO,...`。字段区分大小写。以下是协议参考，有动作的命令用于已验证的离体台架。

| 命令（均以 `$CMD,` 开头） | 功能/边界 |
|---|---|
| `GET_INFO` | 只读固件版本与编译模式 |
| `GET_PARAMS` | 只读当前零位、G/phi/bias、grav_en、state_ctrl、state_lock、impact、swing、force_safe、manual_mode |
| `ZERO_IMU` | 请求以当前 IMU2 姿态为零；先返回请求 ACK，成功执行再返回实际零位 |
| `ZERO_MOTOR` | 请求以当前关节角为用户零；不写电机机械零位，也不改变机械限位 |
| `SET_GRAV,G,phi,bias` | 设置有符号补偿模型参数：Nm、rad、Nm；当前缺少完整有限数值/范围校验 |
| `GRAV_ENABLE,0` / `GRAV_ENABLE,1` | 关闭/开启重力项 |
| `STATE_CTRL,0` / `STATE_CTRL,1` | 关闭/开启状态阻抗项；FSM仍更新，其余力矩项仍可能存在 |
| `STATE_LOCK,OFF` / `STATE_LOCK,AUTO` | 恢复自动状态选择 |
| `STATE_LOCK,SAFE` 等状态名 | 固定某状态用于调试；不是安全停机 |
| `SAFE,1` / `SAFE,0` | 强制SAFE状态/释放强制；SAFE,1清手动模式，但不停止电机，也不清所有输出 |
| `MANUAL_TORQUE,tau` | 设置手动关节力矩，输入范围±3 Nm；能产生运动 |
| `MANUAL_POS,deg,K,B,limit` | 位置目标默认单位°；范围−10…100°，K 0…30，B 0…3，limit 0…3 Nm；不是自动安全的可达机械目标 |
| `MANUAL_OFF` | 退出手动，回到自动项；不是零力矩或停止命令 |
| `SET_THRESH,impact,swing` | 冲击阈值0.02…2 g、摆动阈值5…250 °/s |
| `SAVE_PARAMS` | 保存支持的运行参数到 NVS，普通烧录后仍会保留 |

当前没有串口模式切换、真实电机 Disable/E-stop 指令，也没有自动重发/事务序号。只看到 `sent ...` 不等于设备确认；应检查响应和随后遥测。`GET_PARAMS` 的 `manual_mode` 是请求值，0=自动，1=手动力矩，2=手动位置，实际输出还受代码里的门控影响。

## 保存范围与已知限制

NVS `exo` 保存：`imu_zero`、`motor_zero`、`grav_G/grav_phi/grav_bias`、`grav_en`、`impact_th`、`swing_th`。强制SAFE、state_ctrl、状态锁和manual请求不持久化。配置 schema/范围校验、并发参数更新与零位加载优先级尚待完善，见[项目现状](project-status.md)。

Qt `--csv` 导出是带 `type` 列的连续混合遥测格式；旧 `exo_console.py --csv` 只有 EXO 字段，按住录制导出的实验CSV另有 `t_rel` 等列。离线拟合/透明度工具当前要求 `type=EXO`，应使用 Qt `--csv` 文件。
