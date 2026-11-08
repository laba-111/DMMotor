#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 达妙电机通用控制层，型号差异由每个实例的 ID 与映射量程配置表达
depends:
  - id: QDU-Robomaster/Motor
    ref: same-or-dev
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>

#include "Motor.hpp"
#include "can.hpp"
#include "latest_snapshot.hpp"
#include "libxr_assert.hpp"
#include "libxr_def.hpp"
#include "libxr_type.hpp"
#include "timebase.hpp"

/**
 * @class DMMotor
 * @brief 达妙电机通用控制层。
 *
 * @details
 * 类中只实现 CAN 协议、控制报文打包和反馈解码。电机型号不在类内枚举，
 * 每台电机通过 Param::limits、control_id 和 feedback_id 描述自身差异。
 */
class DMMotor : public Motor
{
 public:
  /** @brief 电机当前在达妙上位机中配置的控制协议。 */
  enum class ControlType : uint8_t
  {
    MIT = 0,
    PV = 1,
    V = 2,
    PVT = 3,
  };

  /**
   * @brief MIT 定点映射量程。
   * @warning 必须与电机参数中的 PMAX、VMAX、TMAX 保持一致。
   */
  struct LimitParam
  {
    float p_min;
    float p_max;
    float v_min;
    float v_max;
    float kp_min;
    float kp_max;
    float kd_min;
    float kd_max;
    float t_min;
    float t_max;
  };

  /** @brief 单台电机的实例配置。 */
  struct Param
  {
    const char* motor_name;
    uint32_t control_id;
    uint32_t feedback_id;
    bool reverse;
    ControlType control_type;
    LimitParam limits;
  };

  DMMotor(LibXR::CAN& can_bus,
          const Param& param = {.motor_name = "dm_motor",
                                .control_id = 1,
                                .feedback_id = 257,
                                .reverse = false,
                                .control_type = DMMotor::ControlType::MIT,
                                .limits = {.p_min = -3.1415926f,
                                           .p_max = 3.1415926f,
                                           .v_min = -30.0f,
                                           .v_max = 30.0f,
                                           .kp_min = 0.0f,
                                           .kp_max = 500.0f,
                                           .kd_min = 0.0f,
                                           .kd_max = 5.0f,
                                           .t_min = -10.0f,
                                           .t_max = 10.0f}})
      : param_(param), can_(std::addressof(can_bus))
  {
    ASSERT(IsParamValid(param_));

    auto rx_callback = LibXR::CAN::Callback::Create(
        [](bool in_isr, DMMotor* self, const LibXR::CAN::ClassicPack& pack)
        { RxCallback(in_isr, self, pack); }, this);

    can_->Register(rx_callback, LibXR::CAN::Type::STANDARD,
                   LibXR::CAN::FilterMode::ID_RANGE, param_.feedback_id,
                   param_.feedback_id);
  }

  /** @brief 发送达妙使能命令。 */
  void Enable() override { last_tx_status_ = SendSpecialCommand(0xFC); }

  /** @brief 发送达妙失能命令。 */
  void Disable() override { last_tx_status_ = SendSpecialCommand(0xFD); }

  /** @brief 控制层的 Relax 等价于失能电机。 */
  void Relax() override { Disable(); }

  /** @brief 清除电机故障。 */
  void ClearError() override { last_tx_status_ = SendSpecialCommand(0xFB); }

  /** @brief 将当前位置保存为电机零点。 */
  void SaveZeroPoint() override { last_tx_status_ = SendSpecialCommand(0xFE); }

  /**
   * @brief 读取最新 CAN 反馈快照并刷新反馈；中间帧不排队。
   * @return OK 表示没有新帧或解析成功；SIZE_ERR 表示最新帧是短帧。
   */
  LibXR::ErrorCode Update() override
  {
    LibXR::CAN::ClassicPack pack{};
    if (!feedback_frame_.LoadLatest(pack))
    {
      return LibXR::ErrorCode::OK;
    }
    const LibXR::ErrorCode result = Decode(pack);
    if (result == LibXR::ErrorCode::OK)
    {
      last_online_time_ = LibXR::Timebase::GetMicroseconds();
    }
    return result;
  }

  const Feedback& GetFeedback() override { return feedback_; }

  /**
   * @brief 通过统一 Motor 接口发送命令。
   *
   * MODE_MIT、MODE_TORQUE 对应 MIT；MODE_POSITION 对应 PV/PVT；
   * MODE_VELOCITY 对应 V。命令模式与 Param::control_type 不匹配时不发帧。
   */
  void Control(const MotorCmd& cmd) override
  {
    switch (cmd.mode)
    {
      case ControlMode::MODE_POSITION:
        if (param_.control_type == ControlType::PV)
        {
          last_tx_status_ = PositionVelocityControl(cmd.position, cmd.velocity);
        }
        else if (param_.control_type == ControlType::PVT)
        {
          last_tx_status_ =
              PositionVelocityTorqueControl(cmd.position, cmd.velocity, cmd.torque);
        }
        else
        {
          last_tx_status_ = LibXR::ErrorCode::STATE_ERR;
        }
        break;

      case ControlMode::MODE_VELOCITY:
        last_tx_status_ = VelocityControl(cmd.velocity);
        break;

      case ControlMode::MODE_TORQUE:
        last_tx_status_ = MITControl(0.0f, 0.0f, 0.0f, 0.0f, cmd.torque);
        break;

      case ControlMode::MODE_MIT:
        last_tx_status_ =
            MITControl(cmd.position, cmd.velocity, cmd.kp, cmd.kd, cmd.torque);
        break;

      case ControlMode::MODE_CURRENT:
      default:
        last_tx_status_ = LibXR::ErrorCode::NOT_SUPPORT;
        break;
    }
  }

  /** @brief 发送 MIT 位置、速度、刚度、阻尼和前馈扭矩命令。 */
  LibXR::ErrorCode MITControl(float position, float velocity, float kp, float kd,
                              float torque)
  {
    if (param_.control_type != ControlType::MIT)
    {
      last_tx_status_ = LibXR::ErrorCode::STATE_ERR;
      return last_tx_status_;
    }

    const LimitParam& limit = param_.limits;
    const float send_position = ToMotorDirection(position, limit.p_min, limit.p_max);
    const float send_velocity = ToMotorDirection(velocity, limit.v_min, limit.v_max);
    const float send_torque = ToMotorDirection(torque, limit.t_min, limit.t_max);
    kp = std::clamp(kp, limit.kp_min, limit.kp_max);
    kd = std::clamp(kd, limit.kd_min, limit.kd_max);

    const uint16_t position_raw =
        FloatToUint(send_position, limit.p_min, limit.p_max, 16);
    const uint16_t velocity_raw =
        FloatToUint(send_velocity, limit.v_min, limit.v_max, 12);
    const uint16_t kp_raw = FloatToUint(kp, limit.kp_min, limit.kp_max, 12);
    const uint16_t kd_raw = FloatToUint(kd, limit.kd_min, limit.kd_max, 12);
    const uint16_t torque_raw = FloatToUint(send_torque, limit.t_min, limit.t_max, 12);

    uint8_t data[8]{};
    data[0] = static_cast<uint8_t>(position_raw >> 8);
    data[1] = static_cast<uint8_t>(position_raw);
    data[2] = static_cast<uint8_t>(velocity_raw >> 4);
    data[3] = static_cast<uint8_t>(((velocity_raw & 0x0F) << 4) | (kp_raw >> 8));
    data[4] = static_cast<uint8_t>(kp_raw);
    data[5] = static_cast<uint8_t>(kd_raw >> 4);
    data[6] = static_cast<uint8_t>(((kd_raw & 0x0F) << 4) | (torque_raw >> 8));
    data[7] = static_cast<uint8_t>(torque_raw);

    last_tx_status_ = SendFrame(param_.control_id, data, sizeof(data));
    return last_tx_status_;
  }

  /** @brief 发送位置速度模式命令，数据为两个小端 float。 */
  LibXR::ErrorCode PositionVelocityControl(float position, float velocity)
  {
    if (param_.control_type != ControlType::PV)
    {
      last_tx_status_ = LibXR::ErrorCode::STATE_ERR;
      return last_tx_status_;
    }

    const LimitParam& limit = param_.limits;
    const float send_position = ToMotorDirection(position, limit.p_min, limit.p_max);
    const float send_velocity = ToMotorDirection(velocity, limit.v_min, limit.v_max);

    uint8_t data[8]{};
    std::memcpy(data, &send_position, sizeof(send_position));
    std::memcpy(data + sizeof(send_position), &send_velocity, sizeof(send_velocity));

    last_tx_status_ = SendFrame(param_.control_id, data, sizeof(data));
    return last_tx_status_;
  }

  /** @brief 发送速度模式命令，数据为一个小端 float。 */
  LibXR::ErrorCode VelocityControl(float velocity)
  {
    if (param_.control_type != ControlType::V)
    {
      last_tx_status_ = LibXR::ErrorCode::STATE_ERR;
      return last_tx_status_;
    }

    const LimitParam& limit = param_.limits;
    const float send_velocity = ToMotorDirection(velocity, limit.v_min, limit.v_max);

    uint8_t data[4]{};
    std::memcpy(data, &send_velocity, sizeof(send_velocity));

    last_tx_status_ = SendFrame(param_.control_id, data, sizeof(data));
    return last_tx_status_;
  }

  /**
   * @brief 发送位置速度力矩模式命令。
   * @param position 目标位置，rad。
   * @param velocity_limit 速度幅值上限，rad/s。
   * @param torque_limit 扭矩幅值上限，Nm。
   */
  LibXR::ErrorCode PositionVelocityTorqueControl(float position, float velocity_limit,
                                                 float torque_limit)
  {
    if (param_.control_type != ControlType::PVT)
    {
      last_tx_status_ = LibXR::ErrorCode::STATE_ERR;
      return last_tx_status_;
    }

    const LimitParam& limit = param_.limits;
    const float max_torque = MaxAbsTorque();
    if (max_torque <= 0.0f)
    {
      last_tx_status_ = LibXR::ErrorCode::ARG_ERR;
      return last_tx_status_;
    }

    const float send_position = ToMotorDirection(position, limit.p_min, limit.p_max);
    const uint16_t velocity_raw = static_cast<uint16_t>(
        std::clamp(std::abs(velocity_limit) * 100.0f, 0.0f, 10000.0f));
    const uint16_t torque_raw = static_cast<uint16_t>(
        std::clamp(std::abs(torque_limit) / max_torque * 10000.0f, 0.0f, 10000.0f));

    uint8_t data[8]{};
    std::memcpy(data, &send_position, sizeof(send_position));
    data[4] = static_cast<uint8_t>(velocity_raw);
    data[5] = static_cast<uint8_t>(velocity_raw >> 8);
    data[6] = static_cast<uint8_t>(torque_raw);
    data[7] = static_cast<uint8_t>(torque_raw >> 8);

    last_tx_status_ = SendFrame(param_.control_id, data, sizeof(data));
    return last_tx_status_;
  }

  [[nodiscard]] float GetAngle() const { return feedback_.position; }
  [[nodiscard]] float GetTorque() const { return feedback_.torque; }
  [[nodiscard]] float GetOmega() const { return feedback_.omega; }
  [[nodiscard]] float GetMaxTorque() const { return MaxAbsTorque(); }
  [[nodiscard]] const char* GetName() const { return param_.motor_name; }
  [[nodiscard]] uint32_t GetControlId() const { return param_.control_id; }
  [[nodiscard]] uint32_t GetFeedbackId() const { return param_.feedback_id; }
  [[nodiscard]] ControlType GetControlType() const { return param_.control_type; }
  [[nodiscard]] LibXR::ErrorCode GetLastTxStatus() const { return last_tx_status_; }
  [[nodiscard]] LibXR::MicrosecondTimestamp GetLastOnlineTime() const
  {
    return last_online_time_;
  }

  /** @brief 根据最近一次反馈时间判断电机是否在线。 */
  [[nodiscard]] bool IsOnline(uint64_t timeout_us = 100000) const
  {
    if (static_cast<uint64_t>(last_online_time_) == 0)
    {
      return false;
    }
    return (LibXR::Timebase::GetMicroseconds() - last_online_time_).ToMicrosecond() <=
           timeout_us;
  }

  /** @brief 供构造前检查实例参数。 */
  [[nodiscard]] static bool IsParamValid(const Param& param)
  {
    const LimitParam& limit = param.limits;
    return param.motor_name != nullptr && param.control_id <= 0x7FFu &&
           param.feedback_id <= 0x7FFu && limit.p_max > limit.p_min &&
           limit.v_max > limit.v_min && limit.kp_max > limit.kp_min &&
           limit.kd_max > limit.kd_min && limit.t_max > limit.t_min;
  }

  void OnMonitor() {}

 private:
  static uint16_t FloatToUint(float value, float min, float max, uint8_t bits)
  {
    value = std::clamp(value, min, max);
    const uint32_t raw_max = (1u << bits) - 1u;
    const float normalized = (value - min) / (max - min);
    return static_cast<uint16_t>(normalized * static_cast<float>(raw_max));
  }

  static float UintToFloat(uint16_t value, float min, float max, uint8_t bits)
  {
    const uint32_t raw_max = (1u << bits) - 1u;
    return static_cast<float>(value) * (max - min) / static_cast<float>(raw_max) + min;
  }

  float DirectionFactor() const { return param_.reverse ? -1.0f : 1.0f; }

  float ToMotorDirection(float value, float min, float max) const
  {
    return std::clamp(DirectionFactor() * value, min, max);
  }

  float MaxAbsTorque() const
  {
    return std::max(std::abs(param_.limits.t_min), std::abs(param_.limits.t_max));
  }

  LibXR::ErrorCode SendSpecialCommand(uint8_t command)
  {
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, command};
    return SendFrame(param_.control_id, data, sizeof(data));
  }

  LibXR::ErrorCode SendFrame(uint32_t id, const uint8_t* data, uint8_t dlc)
  {
    if (data == nullptr || dlc > 8)
    {
      return LibXR::ErrorCode::ARG_ERR;
    }

    LibXR::CAN::ClassicPack pack{};
    pack.id = id;
    pack.type = LibXR::CAN::Type::STANDARD;
    pack.dlc = dlc;
    std::memcpy(pack.data, data, dlc);
    return can_->AddMessage(pack);
  }

  static void RxCallback(bool in_isr, DMMotor* self, const LibXR::CAN::ClassicPack& pack)
  {
    UNUSED(in_isr);
    self->feedback_frame_.Store(pack);
  }

  LibXR::ErrorCode Decode(const LibXR::CAN::ClassicPack& pack)
  {
    if (pack.dlc < 8)
    {
      return LibXR::ErrorCode::SIZE_ERR;
    }

    const LimitParam& limit = param_.limits;
    const uint16_t position_raw = static_cast<uint16_t>(
        (static_cast<uint16_t>(pack.data[1]) << 8) | static_cast<uint16_t>(pack.data[2]));
    const uint16_t velocity_raw =
        static_cast<uint16_t>((static_cast<uint16_t>(pack.data[3]) << 4) |
                              (static_cast<uint16_t>(pack.data[4]) >> 4));
    const uint16_t torque_raw =
        static_cast<uint16_t>(((static_cast<uint16_t>(pack.data[4]) & 0x0Fu) << 8) |
                              static_cast<uint16_t>(pack.data[5]));

    feedback_.error_id = pack.data[0] & 0x0F;
    feedback_.state = pack.data[0] >> 4;
    feedback_.position =
        DirectionFactor() * UintToFloat(position_raw, limit.p_min, limit.p_max, 16);
    feedback_.omega =
        DirectionFactor() * UintToFloat(velocity_raw, limit.v_min, limit.v_max, 12);
    feedback_.velocity = feedback_.omega * 60.0f / static_cast<float>(LibXR::TWO_PI);
    feedback_.torque =
        DirectionFactor() * UintToFloat(torque_raw, limit.t_min, limit.t_max, 12);
    feedback_.temp = static_cast<float>(std::max(pack.data[6], pack.data[7]));
    feedback_.abs_angle = feedback_.position;

    return LibXR::ErrorCode::OK;
  }

  Param param_;
  Motor::Feedback feedback_{};
  LibXR::CAN* can_;
  LibXR::LatestSnapshot<LibXR::CAN::ClassicPack> feedback_frame_{
      LibXR::CAN::ClassicPack{}};
  LibXR::MicrosecondTimestamp last_online_time_{};
  LibXR::ErrorCode last_tx_status_ = LibXR::ErrorCode::OK;
};
