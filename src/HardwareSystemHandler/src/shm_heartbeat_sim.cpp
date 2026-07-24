#include "odrive_can_interface/can_comm.hpp"

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

static void usage(const char *argv0)
{
  std::cerr
      << "Usage:\n"
      << "  " << argv0 << " [can_if] [node_id] [period_ms] [axis_state] [axis_error] [procedure_result] [trajectory_done]\n\n"
      << "Defaults:\n"
      << "  can_if=can0 node_id=0 period_ms=10 axis_state=1 axis_error=0 procedure_result=0 trajectory_done=0\n\n"
      << "ODrive Axis States (raw):\n"
      << "  0x00 = UNDEFINED\n"
      << "  0x01 = IDLE\n"
      << "  0x02 = STARTUP_SEQUENCE\n"
      << "  0x03 = FULL_CALIBRATION_SEQUENCE\n"
      << "  0x04 = MOTOR_CALIBRATION\n"
      << "  0x06 = ENCODER_INDEX_SEARCH\n"
      << "  0x07 = ENCODER_OFFSET_CALIBRATION\n"
      << "  0x08 = CLOSED_LOOP_CONTROL\n"
      << "  0x09 = LOCKIN_SPIN\n"
      << "  0x0A = ENCODER_DIR_FIND\n"
      << "  0x0B = HOMING\n";
}

int main(int argc, char **argv)
{
  std::string can_if = "can0";
  unsigned long node_id = 0;
  unsigned long period_ms = 10;
  unsigned long axis_state = 1;
  uint32_t axis_error = 0;
  unsigned long procedure_result = 0;
  unsigned long trajectory_done = 0;

  if (argc >= 2)
  {
    const std::string arg = argv[1];
    if (arg == "-h" || arg == "--help")
    {
      usage(argv[0]);
      return 0;
    }
    can_if = arg;
  }
  if (argc >= 3)
  {
    node_id = std::strtoul(argv[2], nullptr, 0);
  }
  if (argc >= 4)
  {
    period_ms = std::strtoul(argv[3], nullptr, 0);
    if (period_ms == 0)
    {
      std::cerr << "period_ms must be > 0\n";
      return 2;
    }
  }
  if (argc >= 5)
  {
    axis_state = std::strtoul(argv[4], nullptr, 0);
  }
  if (argc >= 6)
  {
    axis_error = static_cast<uint32_t>(std::strtoul(argv[5], nullptr, 0));
  }
  if (argc >= 7)
  {
    procedure_result = std::strtoul(argv[6], nullptr, 0);
  }
  if (argc >= 8)
  {
    trajectory_done = std::strtoul(argv[7], nullptr, 0);
  }

  CANInterface can;
  if (!can.openInterface(can_if))
  {
    std::cerr << "Failed to open CAN interface: " << can_if << "\n";
    return 1;
  }

  const uint32_t can_id = (static_cast<uint32_t>(node_id) << 5) | 0x01;
  const auto period = std::chrono::milliseconds(period_ms);

  std::cout << "Simulating CAN heartbeat: can_if=" << can_if
            << " node_id=" << node_id
            << " period_ms=" << period_ms
            << " axis_state=0x" << std::hex << axis_state << std::dec
            << " axis_error=0x" << std::hex << axis_error << std::dec
            << "\n";

  std::vector<uint8_t> data(8, 0);
  while (true)
  {
    data[0] = static_cast<uint8_t>(axis_error & 0xFF);
    data[1] = static_cast<uint8_t>((axis_error >> 8) & 0xFF);
    data[2] = static_cast<uint8_t>((axis_error >> 16) & 0xFF);
    data[3] = static_cast<uint8_t>((axis_error >> 24) & 0xFF);
    data[4] = static_cast<uint8_t>(axis_state & 0xFF);
    data[5] = static_cast<uint8_t>(procedure_result & 0xFF);
    data[6] = static_cast<uint8_t>(trajectory_done & 0xFF);
    data[7] = 0;

    if (!can.sendFrame(can_id, data))
    {
      std::cerr << "Failed to send heartbeat on " << can_if << "\n";
      return 1;
    }

    std::this_thread::sleep_for(period);
  }

  return 0;
}
