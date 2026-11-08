#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "DMMotor.hpp"

namespace
{
uint64_t clock_us = 1000;

void require(bool condition, const char* message)
{
  if (!condition)
  {
    std::fprintf(stderr, "%s\n", message);
    std::exit(1);
  }
}

class TestCAN : public LibXR::CAN
{
 public:
  LibXR::ErrorCode SetConfig(const Configuration&) override
  {
    return LibXR::ErrorCode::OK;
  }
  uint32_t GetClockFreq() const override { return 0; }
  LibXR::ErrorCode AddMessage(const ClassicPack& pack) override
  {
    last_pack = pack;
    ++sent_count;
    return LibXR::ErrorCode::OK;
  }
  void Receive(const ClassicPack& pack) { OnMessage(pack, true); }

  ClassicPack last_pack{};
  unsigned sent_count = 0;
};

DMMotor::Param make_param(DMMotor::ControlType type, bool reverse = false)
{
  return {
      "test",
      4,
      260,
      reverse,
      type,
      {-3.1415926f, 3.1415926f, -30.0f, 30.0f, 0.0f, 500.0f, 0.0f, 5.0f, -10.0f, 10.0f}};
}

void require_frame(const TestCAN& can, const std::array<uint8_t, 8>& bytes,
                   uint8_t size = 8)
{
  require(can.last_pack.id == 4 && can.last_pack.type == LibXR::CAN::Type::STANDARD,
          "Control must use the configured complete standard CAN ID");
  require(can.last_pack.dlc == size, "Unexpected control frame length");
  for (uint8_t i = 0; i < size; ++i)
  {
    require(can.last_pack.data[i] == bytes[i], "Unexpected control payload");
  }
}

void test_mit_and_special_commands()
{
  TestCAN can;
  DMMotor motor(can, make_param(DMMotor::ControlType::MIT));
  require(
      motor.MITControl(100.0f, 100.0f, 1000.0f, 100.0f, 100.0f) == LibXR::ErrorCode::OK,
      "MIT commands must clamp to the configured ranges");
  require_frame(can, {255, 255, 255, 255, 255, 255, 255, 255});
  motor.MITControl(-100.0f, -100.0f, -1.0f, -1.0f, -100.0f);
  require_frame(can, {0, 0, 0, 0, 0, 0, 0, 0});

  Motor& base = motor;
  Motor::MotorCmd command{};
  command.mode = Motor::MODE_MIT;
  base.Control(command);
  require_frame(can, {0x7F, 0xFF, 0x7F, 0xF0, 0, 0, 0x07, 0xFF});
  base.Enable();
  require_frame(can, {255, 255, 255, 255, 255, 255, 255, 0xFC});
  base.Disable();
  require_frame(can, {255, 255, 255, 255, 255, 255, 255, 0xFD});
  base.ClearError();
  require_frame(can, {255, 255, 255, 255, 255, 255, 255, 0xFB});
  base.SaveZeroPoint();
  require_frame(can, {255, 255, 255, 255, 255, 255, 255, 0xFE});
  const unsigned SENT = can.sent_count;
  require(motor.VelocityControl(1.0f) == LibXR::ErrorCode::STATE_ERR,
          "A command for a different configured protocol must be rejected");
  require(can.sent_count == SENT, "A rejected command must not transmit a frame");
}

void test_float_protocols()
{
  TestCAN pv_can;
  DMMotor pv(pv_can, make_param(DMMotor::ControlType::PV, true));
  pv.PositionVelocityControl(1.0f, 2.0f);
  require_frame(pv_can, {0, 0, 0x80, 0xBF, 0, 0, 0, 0xC0});

  TestCAN velocity_can;
  DMMotor velocity(velocity_can, make_param(DMMotor::ControlType::V));
  velocity.VelocityControl(100.0f);
  require_frame(velocity_can, {0, 0, 0xF0, 0x41, 0, 0, 0, 0}, 4);

  TestCAN pvt_can;
  DMMotor pvt(pvt_can, make_param(DMMotor::ControlType::PVT));
  pvt.PositionVelocityTorqueControl(1.0f, 30.0f, 5.0f);
  require_frame(pvt_can, {0, 0, 0x80, 0x3F, 0xB8, 0x0B, 0x88, 0x13});
}

void test_latest_feedback()
{
  TestCAN can;
  DMMotor motor(can, make_param(DMMotor::ControlType::MIT, true));
  require(!motor.IsOnline(), "A new motor must start offline");
  require(motor.Update() == LibXR::ErrorCode::OK, "An empty mailbox is valid");

  LibXR::CAN::ClassicPack pack{
      261, LibXR::CAN::Type::STANDARD, 8, {0x10, 255, 255, 255, 255, 255, 20, 30}};
  can.Receive(pack);
  motor.Update();
  require(!motor.IsOnline(), "The feedback CAN ID filter must reject other motors");

  pack.id = 260;
  for (uint8_t i = 0; i < 20; ++i)
  {
    pack.data[1] = i;
    can.Receive(pack);
  }
  pack.data[1] = 255;
  can.Receive(pack);
  require(motor.Update() == LibXR::ErrorCode::OK, "The latest feedback must decode");
  const auto& FEEDBACK = motor.GetFeedback();
  require(std::abs(FEEDBACK.position + 3.1415926f) < 0.0001f &&
              std::abs(FEEDBACK.omega + 30.0f) < 0.0001f &&
              std::abs(FEEDBACK.torque + 10.0f) < 0.0001f,
          "The newest complete feedback must be decoded in the configured direction");
  require(FEEDBACK.state == 1 && FEEDBACK.temp == 30.0f, "Feedback status must survive");
  require(motor.IsOnline(), "Valid feedback must mark the motor online");

  clock_us = 1100;
  pack.dlc = 7;
  can.Receive(pack);
  require(motor.Update() == LibXR::ErrorCode::SIZE_ERR,
          "Short feedback must be rejected");
  require(static_cast<uint64_t>(motor.GetLastOnlineTime()) == 1000,
          "Rejected feedback must not renew online status");
  clock_us = 101001;
  require(!motor.IsOnline(), "Online status must expire after the configured timeout");
}
}  // namespace

namespace LibXR
{
MicrosecondTimestamp Timebase::GetMicroseconds() { return clock_us; }
}  // namespace LibXR

int main()
{
  test_mit_and_special_commands();
  test_float_protocols();
  test_latest_feedback();
  std::puts("DMMotor protocol tests passed");
}
