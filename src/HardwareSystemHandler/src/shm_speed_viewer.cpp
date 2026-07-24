#include "odrive_can_interface/shared_memory.hpp"

#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <thread>

using namespace odrive_can_interface;

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
        const auto *state = shmitf.state();
        if (!state)
        {
            std::cerr << "Shared memory state is null\n";
            return 1;
        }

        const size_t count = std::min<size_t>(state->axis_count, kMaxDisplayAxes);
        std::cout << "\r";
        std::cout << "vel(rad/s): ";
        for (size_t i = 0; i < count; ++i)
        {
            std::cout << "A" << i << "=" << std::fixed << std::setprecision(2)
                      << state->axes[i].velocity;
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
