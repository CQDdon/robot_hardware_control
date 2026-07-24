#include "odrive_can_interface/shared_memory.hpp"

#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <thread>

using namespace odrive_can_interface;

static const char *action_name(ControlAction action)
{
    switch (action)
    {
    case ControlAction::None:
        return "None";
    case ControlAction::Idle:
        return "Idle";
    case ControlAction::ClosedLoop:
        return "ClosedLoop";
    case ControlAction::FullCalib:
        return "FullCalib";
    case ControlAction::Homing:
        return "Homing";
    case ControlAction::ClearErrors:
        return "ClearErrors";
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
        const auto *ctrl = shmitf.control();
        if (!ctrl)
        {
            std::cerr << "Shared memory control is null\n";
            return 1;
        }

        const size_t count = std::min<size_t>(ctrl->axis_count, kMaxDisplayAxes);
        std::cout << "\r";
        std::cout << "ctrl targets(rev): ";
        for (size_t i = 0; i < count; ++i)
        {
            std::cout << "A" << i << "=" << std::fixed << std::setprecision(3)
                      << ctrl->axes[i].target << "," << action_name(ctrl->axes[i].action);
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
