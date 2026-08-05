#include <iostream>
#include <iomanip>
#include <cstdint>
#include "hardware_ipc_common/shared_memory.hpp"

using namespace hardware_ipc_common;

// ANSI color codes
#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
#define MAGENTA "\033[35m"
#define CYAN    "\033[36m"
#define BOLD    "\033[1m"

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

int main(int, char**)
{
    // Open shared memory
    SharedMemoryInterface shmitf;

    if (!shmitf.open(false))
    {
        std::cerr << RED << "✗ Failed to open shared memory!" << RESET << std::endl;
        std::cerr << "  Make sure ros2_control_node is running." << std::endl;
        return 1;
    }

    HwiStateBlock state_data{};
    HwiCommandIfBlock cmd_if_data{};
    HshControlBlock ctrl_data{};
    HwiStateDebugBlock state_dbg_data{};
    if (!shmitf.read_state(state_data) || !shmitf.read_cmd_if(cmd_if_data) || !shmitf.read_control(ctrl_data))
    {
        std::cerr << RED << "✗ Failed to access shared memory blocks!" << RESET << std::endl;
        return 1;
    }
    (void)shmitf.read_state_debug(state_dbg_data);
    const auto* state = &state_data;
    const auto* cmd_if = &cmd_if_data;
    const auto* ctrl = &ctrl_data;
    const auto* state_dbg = &state_dbg_data;

    // Print header
    std::cout << BOLD << CYAN << "╔═══════════════════════════════════════════════════════════════════╗" << RESET << std::endl;
    std::cout << BOLD << CYAN << "║              ODrive Hardware Shared Memory State                  ║" << RESET << std::endl;
    std::cout << BOLD << CYAN << "╚═══════════════════════════════════════════════════════════════════╝" << RESET << std::endl;
    std::cout << std::endl;

    // System info - removed system_state as it doesn't exist in HwiStateBlock
    
    std::cout << BOLD << "Sequence ID:  " << RESET << state->sequence_id << std::endl;
    std::cout << BOLD << "Axis Count:   " << RESET << static_cast<int>(state->axis_count) << std::endl;
    std::cout << BOLD << "Timestamp:    " << RESET << state->timestamp_ns / 1000000 << " ms" << std::endl;
    std::cout << std::endl;

    // Control block info
    std::cout << BOLD << "Control Block:" << RESET << std::endl;
    std::cout << "  Motion Enable: " << (ctrl->motion_enable ? GREEN "YES" RESET : RED "NO" RESET) << std::endl;
    std::cout << "  Debug Enable:  " << (ctrl->debug_enable ? GREEN "YES" RESET : RED "NO" RESET) << std::endl;
    std::cout << "  Safety Action: " << static_cast<int>(ctrl->safety_action) << std::endl;
    std::cout << std::endl;

    // Axis table header
    std::cout << BOLD << YELLOW << "┌──────┬────────┬────────┬──────────────┬──────────────┬────────┬──────────┬──────────┬──────────────┐" << RESET << std::endl;
    std::cout << BOLD << YELLOW << "│ Axis │ CAN ID │ Online │ Actual state │ Desired state│ Motion │ Position │ Velocity │ gen/target/err│" << RESET << std::endl;
    std::cout << BOLD << YELLOW << "├──────┼────────┼────────┼──────────────┼──────────────┼────────┼──────────┼──────────┼──────────────┤" << RESET << std::endl;

    // Axis data
    for (size_t i = 0; i < state->axis_count && i < SHM_MAX_AXES; ++i)
    {
        const auto& axis = state->axes[i];
        const auto& ctrl_axis = ctrl->axes[i];

        // Online status
        const char* online_color = axis.online ? GREEN : RED;
        const char* online_str = axis.online ? " ✓ " : " ✗ ";

        // State with color
        const char* state_color = get_state_color(axis.odrive_state_summary);

        std::cout << "│ " << std::setw(4) << i << " ";
        std::cout << "│ " << std::setw(6) << axis.can_id << " ";
        std::cout << "│ " << online_color << online_str << RESET << " ";
        std::cout << "│ " << state_color << std::setw(12) << std::left 
                  << axis_state_to_string(axis.odrive_state_summary) << RESET << " ";
        std::cout << "│ " << std::setw(12) << std::left
                  << desired_state_to_string(ctrl_axis.desired_axis_state) << " ";
        std::cout << "│ " << online_color << (ctrl_axis.motion_enable ? "on" : "off") << RESET << "    ";
        std::cout << "│ " << std::setw(8) << std::right << std::fixed << std::setprecision(2) 
                  << axis.position << " ";
        std::cout << "│ " << std::setw(8) << std::right << std::fixed << std::setprecision(2) 
                  << axis.velocity << " ";
        std::cout << "│ " << ctrl_axis.desired_state_generation << "/"
                  << ctrl_axis.target_sequence << "/" << ctrl_axis.clear_error_req_id << " ";
        std::cout << "│" << std::endl;
    }

    std::cout << BOLD << YELLOW << "└──────┴────────┴────────┴──────────────┴──────────────┴────────┴──────────┴──────────┴──────────────┘" << RESET << std::endl;
    std::cout << std::endl;

    // Error details
    bool any_errors = false;
    for (size_t i = 0; i < state->axis_count && i < SHM_MAX_AXES; ++i)
    {
        if (state->axes[i].error != 0)
        {
            if (!any_errors)
            {
                std::cout << BOLD << RED << "Error Details:" << RESET << std::endl;
                any_errors = true;
            }

            uint32_t err = state->axes[i].error;
            uint8_t axis_error       = (err >> 0)  & 0xFF;
            uint8_t motor_error      = (err >> 8)  & 0xFF;
            uint8_t encoder_error    = (err >> 16) & 0xFF;
            uint8_t controller_error = (err >> 24) & 0xFF;

            std::cout << "  Axis[" << i << "]: ";
            std::cout << "axis=0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(axis_error) << " ";
            std::cout << "motor=0x" << std::setw(2) << std::setfill('0') << static_cast<int>(motor_error) << " ";
            std::cout << "encoder=0x" << std::setw(2) << std::setfill('0') << static_cast<int>(encoder_error) << " ";
            std::cout << "controller=0x" << std::setw(2) << std::setfill('0') << static_cast<int>(controller_error) << std::dec;
            std::cout << std::endl;
        }
    }

    if (any_errors)
        std::cout << std::endl;

    // Debug info (if available and enabled)
    if (ctrl->debug_enable && state_dbg)
    {
        std::cout << BOLD << MAGENTA << "Debug Info:" << RESET << std::endl;
        for (size_t i = 0; i < state->axis_count && i < SHM_MAX_AXES; ++i)
        {
            const auto& dbg = state_dbg->axes[i];
            if (dbg.last_hb_timestamp_ns > 0)
            {
                std::cout << "  Axis[" << i << "] Last Heartbeat: " 
                          << dbg.last_hb_timestamp_ns / 1000000 << " ms ago" << std::endl;
            }
        }
        std::cout << std::endl;
    }

    // Footer
    std::cout << BOLD << BLUE << "Legend: " << RESET;
    std::cout << GREEN << "✓=Online " << RESET;
    std::cout << RED << "✗=Offline " << RESET;
    std::cout << "| States: " 
              << GREEN << "Idle " << RESET
              << CYAN << "Running " << RESET
              << YELLOW << "Homing " << RESET
              << MAGENTA << "Calibrating" << RESET << std::endl;

    return 0;
}
