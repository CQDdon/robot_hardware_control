#include <chrono>
#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>
#include <cassert>
#include "rclcpp/rclcpp.hpp"
#include "hardware_ipc_common/shared_memory.hpp"
#include "hardware_system_handler/hsh_node.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::cout << "====================================================\n";
    std::cout << " RUNNING BEHAVIOR TEST SUITE FOR ROBOT HARDWARE WS  \n";
    std::cout << "====================================================\n";

    hardware_ipc_common::SharedMemoryInterface shm;
    if (!shm.open(true))
    {
        std::cerr << "[FAIL] Could not open shared memory!\n";
        return 1;
    }

    std::mutex data_mutex;
    HwiStateBlock sim_state{};
    sim_state.axis_count = 2;
    sim_state.axes[0].online = 1;
    sim_state.axes[0].odrive_state_summary = AxisStateSummary::Idle;
    sim_state.axes[1].online = 1;
    sim_state.axes[1].odrive_state_summary = AxisStateSummary::Idle;
    sim_state.relay.is_auto_mode = 1;
    sim_state.relay.is_estop_active = 0; // Normal
    sim_state.rs485.steering_pot_percent = 50.0f;

    HwiCommandIfBlock sim_cmd_if{};
    sim_cmd_if.axis_count = 2;
    sim_cmd_if.axes[0].interface = CommandInterface::Position;
    sim_cmd_if.axes[0].value = 0.0f;
    sim_cmd_if.axes[0].action = ControlAction::ClosedLoop;
    sim_cmd_if.axes[1].interface = CommandInterface::Velocity;
    sim_cmd_if.axes[1].value = 10.0f;
    sim_cmd_if.axes[1].action = ControlAction::ClosedLoop;
    sim_cmd_if.rs485.throttle_voltage = 3.5f;
    sim_cmd_if.relay.relay_mask = 0xAA;

    std::atomic<bool> ticker_running{true};
    std::thread ticker([&]() {
        uint32_t seq = 0;
        while (ticker_running.load())
        {
            const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                    std::chrono::system_clock::now().time_since_epoch()).count();
            {
                std::lock_guard<std::mutex> lock(data_mutex);
                sim_state.timestamp_ns = now_ns;
                sim_state.sequence_id = ++seq;
                shm.write_state(sim_state);

                sim_cmd_if.timestamp_ns = now_ns;
                sim_cmd_if.sequence_id = seq;
                shm.write_cmd_if(sim_cmd_if);
            }
            std::this_thread::sleep_for(20ms);
        }
    });

    // Khởi chạy HSH Node trong luồng riêng với option bypass homing cho unit test
    rclcpp::NodeOptions options;
    options.append_parameter_override("bypass_position_homing", true);
    auto node = std::make_shared<hardware_system_handler::HshNode>(options);
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    std::thread spin_thread([&exec]() {
        exec.spin();
    });

    std::cout << "[TEST 1] Testing normal bringup & transition to RUNNING...\n";
    
    HshControlBlock ctrl{};
    for (int i = 0; i < 50; ++i)
    {
        if (shm.read_control(ctrl) && ctrl.motion_enable == 1 && std::abs(ctrl.rs485.throttle_voltage - 3.5f) < 0.1f)
        {
            break;
        }
        std::this_thread::sleep_for(100ms);
    }

    std::cout << "  -> Current motion_enable: " << (int)ctrl.motion_enable 
              << ", throttle_voltage: " << ctrl.rs485.throttle_voltage << " V\n";
    assert(ctrl.motion_enable == 1 && "FAILED: Expected motion_enable == 1 in AUTO RUNNING mode");
    assert(std::abs(ctrl.rs485.throttle_voltage - 3.5f) < 0.1f && "FAILED: Throttle should pass through in AUTO mode");
    std::cout << "  [PASS] Auto Mode Bringup & Motion Permitted OK!\n\n";

    std::cout << "[TEST 2] Testing Auto -> Manual mode switch (DI6)...\n";
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        sim_state.relay.is_auto_mode = 0; // Chuyển sang MANUAL (DI6 Open)
    }
    std::this_thread::sleep_for(300ms);

    shm.read_control(ctrl);
    std::cout << "  -> Manual Mode - motion_enable: " << (int)ctrl.motion_enable 
              << ", throttle_voltage: " << ctrl.rs485.throttle_voltage << " V\n";
    assert(ctrl.motion_enable == 0 && "FAILED: motion_enable must be 0 in MANUAL mode");
    assert(ctrl.rs485.throttle_voltage == 0.0f && "FAILED: throttle_voltage must be clamped to 0V in MANUAL mode");
    std::cout << "  [PASS] Manual Mode Safety Interlock OK!\n\n";

    std::cout << "[TEST 3] Testing Manual -> Auto restoration (DI6)...\n";
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        sim_state.relay.is_auto_mode = 1; // Chuyển lại AUTO (DI6 Closed)
    }
    std::this_thread::sleep_for(300ms);

    shm.read_control(ctrl);
    std::cout << "  -> Restored Auto - motion_enable: " << (int)ctrl.motion_enable 
              << ", throttle_voltage: " << ctrl.rs485.throttle_voltage << " V\n";
    assert(ctrl.motion_enable == 1 && "FAILED: motion_enable should be restored in AUTO mode");
    assert(std::abs(ctrl.rs485.throttle_voltage - 3.5f) < 0.1f && "FAILED: throttle_voltage should be restored");
    std::cout << "  [PASS] Auto Restoration OK!\n\n";

    std::cout << "[TEST 4] Testing E-Stop active on CAN Relay DI7 protection...\n";
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        sim_state.relay.is_estop_active = 1; // Kích hoạt E-Stop khẩn cấp (DI7 Active)
    }
    std::this_thread::sleep_for(400ms);

    shm.read_control(ctrl);
    std::cout << "  -> E-Stop triggered - motion_enable: " << (int)ctrl.motion_enable 
              << ", throttle_voltage: " << ctrl.rs485.throttle_voltage << " V\n";
    std::cout << "  -> Brake Axis Mode: " << (int)ctrl.axes[0].control_mode
              << ", Target: " << ctrl.axes[0].target
              << ", DesiredState: " << (int)ctrl.axes[0].desired_axis_state << "\n";
    std::cout << "  -> Wheel Axis DesiredState: " << (int)ctrl.axes[1].desired_axis_state << "\n";

    assert(ctrl.motion_enable == 0 && "FAILED: motion_enable must be 0 in STOP state");
    assert(ctrl.rs485.throttle_voltage == 0.0f && "FAILED: throttle_voltage must be 0V in STOP state");
    assert(ctrl.axes[0].control_mode == CommandInterface::Position && "FAILED: Brake axis must be in Position control");
    assert(ctrl.axes[0].desired_axis_state == AxisDesiredState::ClosedLoop && "FAILED: Brake axis must be ClosedLoop");
    assert(ctrl.axes[1].desired_axis_state == AxisDesiredState::Idle && "FAILED: Wheel axis must be Idle");
    std::cout << "  [PASS] E-Stop Safety Shutdown & Brake Locking OK!\n\n";

    std::cout << "[TEST 5] Testing E-Stop Latching (Cannot recover automatically without reset)...\n";
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        sim_state.relay.is_estop_active = 0; // DI7 hồi phục về bình thường nhưng FSM phải chốt ở STOP
    }
    std::this_thread::sleep_for(300ms);

    shm.read_control(ctrl);
    assert(ctrl.motion_enable == 0 && "FAILED: STOP state must remain latched until explicit system reset");
    assert(ctrl.rs485.throttle_voltage == 0.0f && "FAILED: throttle must remain locked");
    std::cout << "  [PASS] STOP State Latch OK!\n\n";

    std::cout << "====================================================\n";
    std::cout << " ALL 5 BEHAVIORAL VERIFICATION TESTS PASSED (100%)! \n";
    std::cout << "====================================================\n";

    ticker_running.store(false);
    if (ticker.joinable()) ticker.join();
    exec.cancel();
    if (spin_thread.joinable()) spin_thread.join();
    rclcpp::shutdown();
    return 0;
}
