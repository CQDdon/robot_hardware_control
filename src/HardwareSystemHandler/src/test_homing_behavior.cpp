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
    std::cout << "========================================================\n";
    std::cout << " RUNNING BRAKE ODRIVE HOMING VERIFICATION TEST SUITE    \n";
    std::cout << "========================================================\n";

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
    sim_state.rs485.steering_pot_percent = 43.1f;

    HwiCommandIfBlock sim_cmd_if{};
    sim_cmd_if.axis_count = 2;
    sim_cmd_if.axes[0].interface = CommandInterface::Position;
    sim_cmd_if.axes[0].value = 0.0f;
    sim_cmd_if.axes[0].action = ControlAction::ClosedLoop;
    sim_cmd_if.axes[1].interface = CommandInterface::Position;
    sim_cmd_if.axes[1].value = 0.0f;
    sim_cmd_if.axes[1].action = ControlAction::ClosedLoop;
    sim_cmd_if.rs485.throttle_voltage = 3.0f;
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
            std::this_thread::sleep_for(10ms);
        }
    });

    // Node với bypass_position_homing = false (BẬT HOMING THẬT CHO PHANH)
    rclcpp::NodeOptions options;
    options.append_parameter_override("bypass_position_homing", false);
    auto node = std::make_shared<hardware_system_handler::HshNode>(options);
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    std::thread spin_thread([&exec]() {
        exec.spin();
    });

    std::cout << "\n[TEST 1] Testing Homing at Program Startup...\n";
    HshControlBlock ctrl{};
    bool homing_requested = false;
    for (int i = 0; i < 40; ++i)
    {
        if (shm.read_control(ctrl))
        {
            if (ctrl.axes[0].desired_axis_state == AxisDesiredState::Homing)
            {
                homing_requested = true;
                break;
            }
        }
        std::this_thread::sleep_for(50ms);
    }
    std::cout << "  -> Brake axis desired_axis_state: " << (int)ctrl.axes[0].desired_axis_state << "\n";
    assert(homing_requested && "FAILED: Brake motor must be commanded AxisDesiredState::Homing at startup");
    std::cout << "  [PASS] StartHoming emitted Homing request to brake motor!\n";

    // Mô phỏng ODrive nhận lệnh Homing và phản hồi state Homing
    std::cout << "  -> Simulating ODrive entering Homing state...\n";
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        sim_state.axes[0].odrive_state_summary = AxisStateSummary::Homing;
    }
    std::this_thread::sleep_for(100ms);

    // Mô phỏng ODrive chạm cảm biến tiệm cận và hoàn tất homing (chuyển về Idle)
    std::cout << "  -> Simulating ODrive reaching proximity sensor & returning to Idle (waiting 2.2s delay)...\n";
    std::this_thread::sleep_for(2200ms);
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        sim_state.axes[0].odrive_state_summary = AxisStateSummary::Idle;
    }

    // Đợi HSH xác nhận homing done và chuyển sang READY -> RUNNING
    bool resumed_running = false;
    for (int i = 0; i < 30; ++i)
    {
        if (shm.read_control(ctrl) && ctrl.motion_enable == 1)
        {
            resumed_running = true;
            break;
        }
        std::this_thread::sleep_for(100ms);
    }
    assert(resumed_running && "FAILED: FSM must transition from WAIT_HOMING -> READY -> RUNNING after startup homing");
    std::cout << "  [PASS] Startup Homing completed -> Vehicle RUNNING (motion_enable=1)!\n";

    std::cout << "\n[TEST 2] Testing E-Stop trigger...\n";
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        sim_state.relay.is_estop_active = 1; // Nhấn E-Stop DI7
    }
    std::this_thread::sleep_for(300ms);
    shm.read_control(ctrl);
    assert(ctrl.motion_enable == 0 && "FAILED: motion_enable must be 0 in STOP state");
    assert(ctrl.axes[0].desired_axis_state == AxisDesiredState::ClosedLoop && "FAILED: Brake must be locked in STOP");
    std::cout << "  [PASS] E-Stop active, vehicle in STOP state!\n";

    std::cout << "\n[TEST 3] Testing E-Stop release & Homing on recovery...\n";
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        sim_state.relay.is_estop_active = 0; // Nhả E-Stop
        sim_state.relay.is_auto_mode = 0;   // Gạt qua MANUAL
    }
    std::this_thread::sleep_for(300ms);

    // Gạt lại AUTO để thỏa mãn chu trình liên động an toàn
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        sim_state.relay.is_auto_mode = 1; // Gạt lại AUTO
    }

    // FSM phải chuyển từ STOP -> WAIT_HOMING và phát lệnh Homing cho động cơ phanh
    bool homing_after_estop = false;
    for (int i = 0; i < 40; ++i)
    {
        if (shm.read_control(ctrl))
        {
            if (ctrl.axes[0].desired_axis_state == AxisDesiredState::Homing)
            {
                homing_after_estop = true;
                break;
            }
        }
        std::this_thread::sleep_for(50ms);
    }
    assert(homing_after_estop && "FAILED: Brake motor MUST enter Homing state every time recovering after E-Stop!");
    std::cout << "  [PASS] Transitioned STOP -> WAIT_HOMING -> Brake Homing commanded!\n";

    // Mô phỏng ODrive thực hiện homing sau E-Stop
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        sim_state.axes[0].odrive_state_summary = AxisStateSummary::Homing;
    }
    std::this_thread::sleep_for(100ms);
    std::cout << "  -> Simulating ODrive homing completed at proximity sensor after E-Stop...\n";
    std::this_thread::sleep_for(2200ms);
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        sim_state.axes[0].odrive_state_summary = AxisStateSummary::Idle;
    }

    // Đợi FSM chuyển về RUNNING
    bool running_again = false;
    for (int i = 0; i < 30; ++i)
    {
        if (shm.read_control(ctrl) && ctrl.motion_enable == 1)
        {
            running_again = true;
            break;
        }
        std::this_thread::sleep_for(100ms);
    }
    assert(running_again && "FAILED: FSM must resume to RUNNING after homing completes post-E-Stop");
    std::cout << "  [PASS] Post-E-Stop Homing completed -> Vehicle RUNNING again!\n";

    std::cout << "\n========================================================\n";
    std::cout << " ALL HOMING VERIFICATION TESTS PASSED (100%)!           \n";
    std::cout << "========================================================\n";

    ticker_running.store(false);
    if (ticker.joinable()) ticker.join();
    exec.cancel();
    if (spin_thread.joinable()) spin_thread.join();
    rclcpp::shutdown();
    return 0;
}
