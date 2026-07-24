#include "odrive_can_interface/shared_memory.hpp"

#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <thread>

using namespace odrive_can_interface;

static const char *iface_name(CommandInterface iface)
{
    switch (iface)
    {
    case CommandInterface::None:
        return "None";
    case CommandInterface::Position:
        return "Position";
    case CommandInterface::Velocity:
        return "Velocity";
    case CommandInterface::Torque:
        return "Torque";
    default:
        return "Unknown";
    }
}

int main(int argc, char **argv)
{
    unsigned long period_ms = 100;
    if (argc >= 2)
    {
        period_ms = std::strtoul(argv[1], nullptr, 0);
        if (period_ms == 0)
        {
            std::cerr << "period_ms must be > 0\n";
            return 2;
        }
    }

    SharedMemoryInterface shmitf;
    if (!shmitf.open(false))
    {
        std::cerr << "Failed to open shared memory\n";
        return 1;
    }

    const auto period = std::chrono::milliseconds(period_ms);
    constexpr size_t kMaxDisplayAxes = 6;

    while (true)
    {
        const auto *cmd = shmitf.cmd_if();
        if (!cmd)
        {
            std::cerr << "Shared memory cmd_if is null\n";
            return 1;
        }

        const size_t count = std::min<size_t>(cmd->axis_count, kMaxDisplayAxes);
        std::cout << "\r";
        std::cout << "cmd_if raw: ";
        for (size_t i = 0; i < count; ++i)
        {
            std::cout << "A" << i << "=" << std::fixed << std::setprecision(3)
                      << cmd->axes[i].value << "," << iface_name(cmd->axes[i].interface);
            if (i + 1 < count)
            {
                std::cout << " | ";
            }
        }
        std::cout << std::flush;

        std::this_thread::sleep_for(period);
    }

    return 0;
}
