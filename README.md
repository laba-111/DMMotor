# DMMotor 通用控制层

该模块从原 `mod_motors.hpp/.cpp` 中提取达妙电机相关逻辑，并采用与其他
xrobot 模块一致的单类、单头文件结构。类内不维护 `DM4310`、`DM4340`、
`DM8009` 等型号枚举；型号差异全部由每个实例的 `DMMotor::Param` 表达。

## 设计边界

- 本模块负责 CAN 收发、MIT/PV/V/PVT 报文打包、反馈解码和在线状态记录。
- 本模块不创建控制线程，不包含关节 PID、运动学或机械臂控制策略。
- `control_id` 和 `feedback_id` 都是完整 CAN ID，不推导 `ID + 0x10`。
- `limits` 必须与达妙上位机中的 `PMAX/VMAX/TMAX` 一致，否则命令和反馈都会缩放错误。
- 一台电机在一个实例中只使用一种 `ControlType`；切换电机内部模式后应同步修改实例配置。

## 文件

- `DMMotor.hpp`：完整控制层，header-only。
- `CMakeLists.txt`：把模块目录加入 `xr` 目标的包含路径。

## 1. 配置实例

下面的 4310 与 4340 数值沿用原工程中的实例配置。它们只是示例；如果你在
达妙上位机中修改过映射范围，应填写电机当前的真实值。

```cpp
#include "DMMotor.hpp"

DMMotor::Param dm4310_param{
    "can1",                         // CAN 硬件在 HardwareContainer 中的名称
    "roll_4310",                    // 仅用于辨识实例
    0x004,                           // 当前控制模式使用的完整控制 ID
    0x104,                           // 上位机配置的反馈 ID / MST_ID
    false,                           // 是否反向
    DMMotor::ControlType::MIT,
    {-3.14159f, 3.14159f,            // p_min, p_max (rad)
     -30.0f, 30.0f,                  // v_min, v_max (rad/s)
     0.0f, 500.0f,                   // kp_min, kp_max
     0.0f, 5.0f,                     // kd_min, kd_max
     -10.0f, 10.0f},                 // t_min, t_max (Nm)
};

DMMotor::Param dm4340_param{
    "can1",
    "pitch_4340",
    0x005,
    0x105,
    true,
    DMMotor::ControlType::MIT,
    {-3.14159f, 3.14159f,
     -10.0f, 10.0f,
     0.0f, 500.0f,
     0.0f, 5.0f,
     -28.0f, 28.0f},
};

DMMotor dm4310(hw, app, dm4310_param);
DMMotor dm4340(hw, app, dm4340_param);
```

同一个类可以创建任意数量、任意型号的实例；只要每个实例的 CAN ID 不冲突，
并且量程与对应电机一致即可。

## 2. 周期更新与控制

上层控制线程中先调用 `Update()` 消费反馈，再根据反馈状态使能、清错或控制：

```cpp
void MotorLoop(DMMotor& motor) {
  motor.Update();
  const Motor::Feedback& feedback = motor.GetFeedback();

  if (!motor.IsOnline()) {
    // 尚未收到反馈，按系统安全策略处理。
    return;
  }

  if (feedback.state == 0) {
    motor.Enable();
    return;
  }

  if (feedback.state != 1) {
    motor.ClearError();
    return;
  }

  Motor::MotorCmd command{};
  command.mode = Motor::ControlMode::MODE_MIT;
  command.position = 0.5f;
  command.velocity = 0.0f;
  command.kp = 20.0f;
  command.kd = 0.5f;
  command.torque = 0.0f;
  motor.Control(command);
}
```

位置闭环时不要把 `kd` 设为 0，并应从较小的 `kp/kd/torque` 开始调试。

## 3. 直接调用四种协议

除了统一的 `Motor::Control()`，也可以直接调用协议接口。方法只会在实例的
`control_type` 匹配时发帧，否则返回 `LibXR::ErrorCode::STATE_ERR`。

```cpp
motor.MITControl(position, velocity, kp, kd, feedforward_torque);
motor.PositionVelocityControl(position, velocity);       // ControlType::PV
motor.VelocityControl(velocity);                          // ControlType::V
motor.PositionVelocityTorqueControl(position,
                                    velocity_limit,
                                    torque_limit);        // ControlType::PVT
```

控制 ID 应按电机当前模式填写完整值。沿用原驱动的约定时通常为：MIT 使用
`ID`、PV 使用 `0x100 + ID`、V 使用 `0x200 + ID`、PVT 使用 `0x300 + ID`。
反馈 ID 独立填写，以达妙上位机的 `MST_ID` 为准。

例如一台基础 ID 为 `0x005`、处于速度模式的电机，可以这样配置：

```cpp
DMMotor::Param velocity_motor_param{
    "can1", "velocity_motor", 0x205, 0x105, false,
    DMMotor::ControlType::V,
    {-3.14159f, 3.14159f, -10.0f, 10.0f,
     0.0f, 500.0f, 0.0f, 5.0f, -28.0f, 28.0f},
};
```

## 4. YAML 构造参数

模块头部已经提供 Manifest V2。对应的参数形状如下：

```yaml
param:
  can_bus_name: can1
  motor_name: roll_4310
  control_id: 4
  feedback_id: 260
  reverse: false
  control_type: DMMotor::ControlType::MIT
  limits:
    p_min: -3.1415926
    p_max: 3.1415926
    v_min: -30.0
    v_max: 30.0
    kp_min: 0.0
    kp_max: 500.0
    kd_min: 0.0
    kd_max: 5.0
    t_min: -10.0
    t_max: 10.0
```

## 5. 调试顺序

1. 在达妙上位机确认控制模式、控制 ID、反馈 ID、PMAX、VMAX 和 TMAX。
2. 让代码配置与这些值完全一致，并确认 CAN 使用标准帧和正确波特率。
3. 电机悬空或卸载，从失能状态开始；先确认 `Update()` 能得到合理反馈。
4. 发送 `Enable()`，使用很小的 `kp/kd/torque` 测试方向。
5. 方向相反时修改 `reverse`，不要同时交换反馈符号和命令符号。
6. 稳定后再逐步提高控制参数和负载。

`GetLastTxStatus()` 可查看最近一次发送结果，`IsOnline(timeout_us)` 可按反馈时间
判断电机是否在线；默认超时时间为 100 ms。
