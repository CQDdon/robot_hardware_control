#include <iostream>
#include <iomanip>
#include <cstdint>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>
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

void sigint_handler(int)
{
    g_running = false;
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

    // RS485 & Relay Peripherals
    const bool is_auto = (state.rs485.is_auto_mode != 0);
    const float estop_v = state.rs485.e_stop_voltage;
    const bool estop_triggered = (estop_v > 4.7f);

    std::cout << BOLD << MAGENTA << " [RS485 Modbus & CAN Relay Peripherals] " << RESET << "\n";
    std::cout << "  * Mode Switch (AI 0x0005)    : " << (is_auto ? GREEN "AUTO (<= 4.7V)" RESET : YELLOW "MANUAL (> 4.7V)" RESET) << "\n";
    std::cout << "  * E-Stop Switch (AI 0x0006)  : " << (estop_triggered ? RED : GREEN)
              << std::fixed << std::setprecision(2) << estop_v << " V " << RESET
              << (estop_triggered ? RED "[STOPPED / PRESSED]" RESET : GREEN "[NORMAL / RELEASED]" RESET) << "\n";
    std::cout << "  * Steering Pot (AI 0x0007)   : " << CYAN << std::fixed << std::setprecision(1) << state.rs485.steering_pot_percent << " %" << RESET << "\n";
    std::cout << "  * Throttle Output (AO8 / 0x0007): " << CYAN << std::fixed << std::setprecision(2) << ctrl.rs485.throttle_voltage << " V" << RESET << "\n";
    
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

    SharedMemoryInterface shmitf;
    if (!shmitf.open(false))
    {
        std::cerr << RED << "✗ Failed to open shared memory!" << RESET << std::endl;
        std::cerr << "  Make sure ros2_control_node (vehicle_hardware_bringup) is running." << std::endl;
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

    return 0;
}
