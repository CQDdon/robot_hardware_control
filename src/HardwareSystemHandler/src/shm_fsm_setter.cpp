#include "hardware_ipc_common/shared_memory.hpp"

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>

static void usage(const char *argv0)
{
  std::cerr
      << "Usage:\n"
      << "  " << argv0 << " <axis_state_summary> [axis_count] [axis_error]\n\n"
      << "axis_state_summary:\n"
      << "  unknown|idle|startup|calibrating|homing|running  (or 0|1|2|3|4|5)\n\n"
      << "axis_count (optional): number of axes (default 0)\n"
      << "axis_error (optional): per-axis error value, numeric (default 0)\n";
}

static bool parse_axis_state_summary(const std::string &s, AxisStateSummary &out)
{
  if (s == "unknown" || s == "0")
  {
    out = AxisStateSummary::Unknown;
    return true;
  }
  if (s == "idle" || s == "1")
  {
    out = AxisStateSummary::Idle;
    return true;
  }
  if (s == "startup" || s == "2")
  {
    out = AxisStateSummary::Startup;
    return true;
  }
  if (s == "calibrating" || s == "calib" || s == "3")
  {
    out = AxisStateSummary::Calibrating;
    return true;
  }
  if (s == "homing" || s == "4")
  {
    out = AxisStateSummary::Homing;
    return true;
  }
  if (s == "running" || s == "5")
  {
    out = AxisStateSummary::Running;
    return true;
  }
  return false;
}

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    usage(argv[0]);
    return 2;
  }

  AxisStateSummary axis_state{};
  if (!parse_axis_state_summary(argv[1], axis_state))
  {
    usage(argv[0]);
    return 2;
  }

  unsigned long axis_count = 0;
  if (argc >= 3)
  {
    axis_count = std::strtoul(argv[2], nullptr, 0);
  }

  uint32_t axis_error = 0;
  if (argc >= 4)
  {
    axis_error = static_cast<uint32_t>(std::strtoul(argv[3], nullptr, 0));
  }

  hardware_ipc_common::SharedMemoryInterface shm;
  if (!shm.open(false))
  {
    std::cerr << "Failed to open SHM (" << SHM_HWI_CMD_IF << ", "
              << SHM_HWI_STATE << ", " << SHM_HWI_STATE_DEBUG << ", "
              << SHM_HSH_CTRL << ")\n";
    return 1;
  }

  HwiStateBlock st{};
  st.axis_count = static_cast<uint8_t>(std::min<unsigned long>(axis_count, SHM_MAX_AXES));
  for (size_t i = 0; i < st.axis_count; ++i)
  {
    st.axes[i].odrive_state_summary = axis_state;
    st.axes[i].error = axis_error;
    st.axes[i].online = 1;
  }

  if (!shm.write_state(st))
  {
    std::cerr << "Failed to write SHM state\n";
    return 1;
  }

  std::cout << "Wrote SHM: axis_state_summary=" << static_cast<unsigned>(axis_state)
            << " axis_count=" << static_cast<unsigned>(st.axis_count)
            << " axis_error=" << axis_error << "\n";
  return 0;
}
