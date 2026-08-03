#include "hardware_ipc_common/shared_memory.hpp"

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <sys/select.h>

using namespace hardware_ipc_common;

class TermiosGuard
{
public:
    TermiosGuard()
    {
        tcgetattr(STDIN_FILENO, &old_);
        termios raw = old_;
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    }

    ~TermiosGuard()
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &old_);
    }

private:
    termios old_{};
};

static bool read_key(char &out)
{
    fd_set set;
    FD_ZERO(&set);
    FD_SET(STDIN_FILENO, &set);
    timeval tv{};
    if (select(STDIN_FILENO + 1, &set, nullptr, nullptr, &tv) > 0)
    {
        char c = 0;
        if (read(STDIN_FILENO, &c, 1) == 1)
        {
            out = c;
            return true;
        }
    }
    return false;
}

int main(int argc, char **argv)
{
    unsigned long rate_hz = 200;
    for (int i = 1; i < argc; ++i)
    {
        char *end = nullptr;
        const unsigned long val = std::strtoul(argv[i], &end, 0);
        if (end != argv[i] && *end == '\0')
        {
            rate_hz = val;
            break;
        }
    }
    if (rate_hz == 0)
    {
        std::cerr << "rate_hz must be > 0\n";
        return 2;
    }

    SharedMemoryInterface shmitf;
    if (!shmitf.open(false))
    {
        std::cerr << "Failed to open shared memory\n";
        return 1;
    }

    TermiosGuard termios_guard;

    float speed = 1.0f;
    const float step = 0.1f;
    float vx = 0.0f;
    float vy = 0.0f;

    const auto period = std::chrono::microseconds(static_cast<int64_t>(1000000UL / rate_hz));
    auto next_time = std::chrono::steady_clock::now();
    uint32_t seq = 0;

    std::cout << "Teleop keys: qweadzxc (8 dirs), s=stop, n/m=+/- speed, Ctrl+C to exit\n";

    while (true)
    {
        next_time += period;

        char key = 0;
        if (read_key(key))
        {
            int dx = 0;
            int dy = 0;
            switch (key)
            {
            case 'q': dx = -1; dy = 1; break;
            case 'w': dx = 0; dy = 1; break;
            case 'e': dx = 1; dy = 1; break;
            case 'a': dx = -1; dy = 0; break;
            case 'd': dx = 1; dy = 0; break;
            case 'z': dx = -1; dy = -1; break;
            case 'x': dx = 0; dy = -1; break;
            case 'c': dx = 1; dy = -1; break;
            case 's': dx = 0; dy = 0; break;
            case 'n': speed += step; break;
            case 'm': speed -= step; break;
            default: break;
            }

            if (key == 's')
            {
                vx = 0.0f;
                vy = 0.0f;
            }
            else if (dx != 0 || dy != 0)
            {
                vx = speed * static_cast<float>(dx);
                vy = speed * static_cast<float>(dy);
            }
        }

        HwiCommandIfBlock out{};
        out.axis_count = 6;
        out.sequence_id = seq++;
        out.timestamp_ns = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count());

        for (size_t i = 0; i < out.axis_count; ++i)
        {
            out.axes[i].interface = CommandInterface::Velocity;
            out.axes[i].value = (i < 3) ? vx : vy;
        }

        (void)shmitf.write_cmd_if(out);

        std::cout << "\r" << "speed=" << std::fixed << std::setprecision(2)
                  << speed << " vx=" << vx << " vy=" << vy << "   " << std::flush;

        std::this_thread::sleep_until(next_time);
    }

    return 0;
}
