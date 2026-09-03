#include <iostream>
#include <iomanip>
#include <cstdint>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>
#include <algorithm>
#include <cstring>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include "hardware_ipc_common/shared_memory.hpp"

using namespace hardware_ipc_common;

// ANSI colors
#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
#define MAGENTA "\033[35m"
#define CYAN    "\033[36m"
#define BOLD    "\033[1m"
#define CLEAR_SCREEN "\033[H\033[2J"

std::atomic<bool> g_running{true};

// CAN1 Telemetry state (Battery 0x753 & Speed 0x751)
std::atomic<bool> g_can1_running{true};
std::atomic<bool> g_can1_socket_ok{false};
std::atomic<float> g_can1_battery_v{0.0f};
std::atomic<uint32_t> g_can1_battery_raw{0};
std::atomic<int64_t> g_can1_battery_last_ns{0};

std::atomic<float> g_can1_speed_kmh{0.0f};
std::atomic<float> g_can1_speed_mps{0.0f};
std::atomic<uint32_t> g_can1_speed_raw{0};
std::atomic<int64_t> g_can1_speed_last_ns{0};

void can1_rx_worker()
{
    while (g_can1_running)
    {
        int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (s < 0)
        {
            g_can1_socket_ok = false;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }

        struct ifreq ifr;
        std::memset(&ifr, 0, sizeof(ifr));
        std::strncpy(ifr.ifr_name, "can1", IFNAMSIZ - 1);
        if (ioctl(s, SIOCGIFINDEX, &ifr) < 0)
        {
            close(s);
            g_can1_socket_ok = false;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }

        struct sockaddr_can addr{};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        struct can_filter rfilter[2];
        rfilter[0].can_id = 0x751;
        rfilter[0].can_mask = CAN_SFF_MASK;
        rfilter[1].can_id = 0x753;
        rfilter[1].can_mask = CAN_SFF_MASK;
        setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

        struct timeval tv{};
        tv.tv_sec = 0;
        tv.tv_usec = 250000;
        setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        if (bind(s, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0)
        {
            close(s);
            g_can1_socket_ok = false;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }

        g_can1_socket_ok = true;

        while (g_can1_running)
        {
            struct can_frame frame{};
            int nbytes = read(s, &frame, sizeof(struct can_frame));
            if (nbytes < 0)
            {
                if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
                break;
            }

            const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();

            if (frame.can_id == 0x753 && frame.can_dlc >= 8)
            {
                uint32_t raw_bat = 0;
                std::memcpy(&raw_bat, &frame.data[4], sizeof(uint32_t));
                g_can1_battery_raw.store(raw_bat, std::memory_order_relaxed);
                g_can1_battery_v.store(static_cast<float>(raw_bat) / 100.0f, std::memory_order_relaxed);
                g_can1_battery_last_ns.store(now_ns, std::memory_order_relaxed);
            }
            else if (frame.can_id == 0x751 && frame.can_dlc >= 4)
            {
                uint32_t raw_spd = 0;
                std::memcpy(&raw_spd, &frame.data[0], sizeof(uint32_t));
                uint32_t clamped = (raw_spd > 4095) ? 4095 : raw_spd;
                float kmh = (static_cast<float>(clamped) / 4095.0f) * 25.0f;
                g_can1_speed_raw.store(raw_spd, std::memory_order_relaxed);
                g_can1_speed_kmh.store(kmh, std::memory_order_relaxed);
                g_can1_speed_mps.store(kmh / 3.6f, std::memory_order_relaxed);
                g_can1_speed_last_ns.store(now_ns, std::memory_order_relaxed);
            }
        }

        close(s);
        g_can1_socket_ok = false;
    }
}

void sigint_handler(int)
{
    g_running = false;
    g_can1_running = false;
}

const char* axis_state_to_string(AxisStateSummary state)
{
    switch (state)
    {
        case AxisStateSummary::Unknown:     return "Unknown";
        case AxisStateSummary::Idle:        return "Idle";
        case AxisStateSummary::Startup:     return "Startup";
        case AxisStateSummary::Calibrating: return "Calibrating";
        case AxisStateSummary::Homing:      return "Homing";
        case AxisStateSummary::Running:     return "Running";
        default:                            return "Invalid";
    }
}

const char* get_state_color(AxisStateSummary state)
{
    switch (state)
    {
        case AxisStateSummary::Idle:        return GREEN;
        case AxisStateSummary::Running:     return CYAN;
        case AxisStateSummary::Homing:      return YELLOW;
        case AxisStateSummary::Calibrating: return MAGENTA;
        case AxisStateSummary::Unknown:     return RED;
        default:                            return RESET;
    }
}

const char* desired_state_to_string(AxisDesiredState state)
{
    switch (state)
    {
        case AxisDesiredState::Idle:            return "Idle";
        case AxisDesiredState::ClosedLoop:      return "ClosedLoop";
        case AxisDesiredState::FullCalibration: return "FullCalibration";
        case AxisDesiredState::Homing:          return "Homing";
        default:                                return "Invalid";
    }
}

void print_dashboard(const HwiStateBlock& state, const HwiCommandIfBlock& cmd_if, const HshControlBlock& ctrl)
{
    (void)cmd_if;
    std::cout << CLEAR_SCREEN;
    std::cout << BOLD << CYAN << "==========================================================================" << RESET << "\n";
    std::cout << BOLD << CYAN << "              ROBOT HARDWARE SHARED MEMORY DASHBOARD                      " << RESET << "\n";
    std::cout << BOLD << CYAN << "==========================================================================" << RESET << "\n";
    
    // System Info
    std::cout << BOLD << " [System] " << RESET 
              << "Sequence: " << state.sequence_id 
              << " | Axes: " << static_cast<int>(state.axis_count) 
              << " | FSM Motion Enable: " << (ctrl.motion_enable ? GREEN "YES (ON)" RESET : RED "NO (LOCKED)" RESET)
              << "\n\n";

    // Vehicle CAN1 Telemetry (Battery 0x753 & Speed 0x751)
    const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

    const int64_t bat_last_ns = g_can1_battery_last_ns.load(std::memory_order_relaxed);
    const int64_t spd_last_ns = g_can1_speed_last_ns.load(std::memory_order_relaxed);
    const double bat_age_ms = (bat_last_ns > 0) ? (now_ns - bat_last_ns) / 1000000.0 : 99999.0;
    const double spd_age_ms = (spd_last_ns > 0) ? (now_ns - spd_last_ns) / 1000000.0 : 99999.0;

    const float bat_v = g_can1_battery_v.load(std::memory_order_relaxed);
    const uint32_t bat_raw = g_can1_battery_raw.load(std::memory_order_relaxed);
    const float spd_kmh = g_can1_speed_kmh.load(std::memory_order_relaxed);
    const float spd_mps = g_can1_speed_mps.load(std::memory_order_relaxed);
    const uint32_t spd_raw = g_can1_speed_raw.load(std::memory_order_relaxed);

    std::cout << BOLD << GREEN << " [Vehicle CAN1 Telemetry (Battery & Speed)] " << RESET << "\n";
    if (bat_age_ms < 1500.0) {
        const char* bat_col = (bat_v >= 46.0f) ? GREEN : ((bat_v >= 42.0f) ? YELLOW : RED);
        std::cout << "  * Battery Voltage (ID 0x753)  : " << bat_col << BOLD << std::fixed << std::setprecision(2)
                  << bat_v << " V" << RESET
                  << " (raw: " << bat_raw << ", " << std::fixed << std::setprecision(0) << bat_age_ms << " ms ago)" << "\n";
    } else {
        std::cout << "  * Battery Voltage (ID 0x753)  : " << RED << "[NO SIGNAL / TIMEOUT]" << RESET;
        if (!g_can1_socket_ok) std::cout << YELLOW << " (can1 interface offline/down)" << RESET;
        std::cout << "\n";
    }

    if (spd_age_ms < 1500.0) {
        std::cout << "  * Vehicle Speed   (ID 0x751)  : " << CYAN << BOLD << std::fixed << std::setprecision(2)
                  << spd_kmh << " km/h" << RESET << " (" << std::fixed << std::setprecision(2) << spd_mps << " m/s)"
                  << " (raw: " << spd_raw << "/4095, " << std::fixed << std::setprecision(0) << spd_age_ms << " ms ago)" << "\n";
    } else {
        std::cout << "  * Vehicle Speed   (ID 0x751)  : " << RED << "[NO SIGNAL / TIMEOUT]" << RESET;
        if (!g_can1_socket_ok) std::cout << YELLOW << " (can1 interface offline/down)" << RESET;
        std::cout << "\n";
    }
    std::cout << "\n";

    // RS485 & Relay Peripherals
    const bool is_auto = (state.relay.is_auto_mode != 0);
    const bool estop_triggered = (state.relay.is_estop_active != 0);

    std::cout << BOLD << MAGENTA << " [RS485 Modbus & CAN Relay Peripherals] " << RESET << "\n";
    std::cout << "  * Mode Switch (CAN Relay DI6): " << (is_auto ? GREEN "AUTO [CLOSED]" RESET : YELLOW "MANUAL [OPEN]" RESET) << "\n";
    std::cout << "  * E-Stop Switch (CAN Relay DI7): "
              << (estop_triggered ? RED "[STOPPED / PRESSED / OPEN]" RESET : GREEN "[NORMAL / RELEASED / CLOSED]" RESET) << "\n";
    std::cout << "  * Digital Inputs (DI1..DI8)  : " << YELLOW << "0x"
              << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(state.relay.din_mask) << std::dec << std::setfill(' ')
              << " (Bits: [";
    for (int b = 7; b >= 0; --b) {
        if ((state.relay.din_mask >> b) & 1) std::cout << YELLOW << "1" << RESET;
        else std::cout << "0";
        if (b > 0) std::cout << " ";
    }
    std::cout << "])" << RESET << "\n";
    const float pot = state.rs485.steering_pot_percent;
    const float norm_steer = std::clamp((pot - 43.1f) / 30.6f, -1.0f, 1.0f);
    const char* steer_dir = (norm_steer > 0.05f) ? " [RIGHT]" : ((norm_steer < -0.05f) ? " [LEFT]" : " [CENTER]");
    std::cout << "  * Steering Pot (RS485 AI8 / 0x0007): " << CYAN << std::fixed << std::setprecision(1) << pot << " %"
              << " (" << std::showpos << (norm_steer * 100.0f) << "%" << steer_dir << ")" << std::noshowpos << RESET << "\n";
    std::cout << "  * Throttle Output (RS485 AO8 / 0x0007): " << CYAN << std::fixed << std::setprecision(2) << ctrl.rs485.throttle_voltage << " V" << RESET << "\n";
    
    // Relay Mask in binary & hex
    std::cout << "  * ESP32 Relay Mask (ID 0x500): " << YELLOW << "0x" 
              << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(ctrl.relay.relay_mask) << std::dec << std::setfill(' ')
              << " (Bits: [";
    for (int b = 7; b >= 0; --b) {
        if ((ctrl.relay.relay_mask >> b) & 1) std::cout << GREEN << "1" << RESET;
        else std::cout << "0";
        if (b > 0) std::cout << " ";
    }
    std::cout << "])" << RESET << "\n\n";

    // ODrive Axes Table
    std::cout << BOLD << YELLOW << " [ODrive Motor Axes] " << RESET << "\n";
    std::cout << "+------+--------+--------+--------------+--------------+--------+----------+----------+\n";
    std::cout << "| Axis | CAN ID | Online | Actual State | Desired State| Motion | Position | Velocity |\n";
    std::cout << "+------+--------+--------+--------------+--------------+--------+----------+----------+\n";

    for (size_t i = 0; i < state.axis_count && i < SHM_MAX_AXES; ++i)
    {
        const auto& axis = state.axes[i];
        const auto& ctrl_axis = ctrl.axes[i];

        char online_buf[32];
        if (axis.online) snprintf(online_buf, sizeof(online_buf), "%sYES%s", GREEN, RESET);
        else snprintf(online_buf, sizeof(online_buf), "%sNO %s", RED, RESET);

        const char* st_col = get_state_color(axis.odrive_state_summary);
        const char* act_st = axis_state_to_string(axis.odrive_state_summary);
        const char* des_st = desired_state_to_string(ctrl_axis.desired_axis_state);

        printf("| %4zu | %6u |   %s   | %s%-12s%s | %-12s | %s%-6s%s | %8.2f | %8.2f |\n",
               i,
               axis.can_id,
               online_buf,
               st_col, act_st, RESET,
               des_st,
               ctrl_axis.motion_enable ? GREEN : RED,
               ctrl_axis.motion_enable ? "ON" : "OFF",
               RESET,
               axis.position,
               axis.velocity);
    }
    std::cout << "+------+--------+--------+--------------+--------------+--------+----------+----------+\n";

    // Error Reporting
    bool any_error = false;
    for (size_t i = 0; i < state.axis_count && i < SHM_MAX_AXES; ++i)
    {
        if (state.axes[i].error != 0)
        {
            if (!any_error) {
                std::cout << "\n" << BOLD << RED << " [Fault Details] " << RESET << "\n";
                any_error = true;
            }
            uint32_t err = state.axes[i].error;
            printf("  Axis[%zu]: error_code=0x%08X (axis=0x%02X, motor=0x%02X, enc=0x%02X, ctrl=0x%02X)\n",
                   i, err, (err & 0xFF), ((err >> 8) & 0xFF), ((err >> 16) & 0xFF), ((err >> 24) & 0xFF));
        }
    }

    std::cout << "\n" << BOLD << BLUE << "Press Ctrl+C to exit dashboard." << RESET << std::endl;
}

int main(int argc, char** argv)
{
    bool run_once = false;
    if (argc > 1 && (std::string(argv[1]) == "-1" || std::string(argv[1]) == "--once")) {
        run_once = true;
    }

    signal(SIGINT, sigint_handler);

    std::thread can1_thread(can1_rx_worker);

    SharedMemoryInterface shmitf;
    if (!shmitf.open(false))
    {
        std::cerr << RED << "✗ Failed to open shared memory!" << RESET << std::endl;
        std::cerr << "  Make sure ros2_control_node (vehicle_hardware_bringup) is running." << std::endl;
        g_can1_running = false;
        if (can1_thread.joinable()) can1_thread.join();
        return 1;
    }

    while (g_running)
    {
        HwiStateBlock state{};
        HwiCommandIfBlock cmd_if{};
        HshControlBlock ctrl{};

        if (shmitf.read_state(state) && shmitf.read_control(ctrl))
        {
            (void)shmitf.read_cmd_if(cmd_if);
            print_dashboard(state, cmd_if, ctrl);
        }
        else
        {
            std::cout << YELLOW << "Waiting for valid data in Shared Memory..." << RESET << std::endl;
        }

        if (run_once) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 10Hz live update
    }

    g_can1_running = false;
    if (can1_thread.joinable())
    {
        can1_thread.join();
    }

    return 0;
}
