#include "hardware_system_handler/hsh_node.hpp"
#include "hardware_system_handler/state_init.hpp"
#include "hardware_system_handler/state_wait_sensors.hpp"
#include "hardware_system_handler/state_wait_homing.hpp"
#include "hardware_system_handler/state_ready.hpp"
#include "hardware_system_handler/state_running.hpp"
#include "hardware_system_handler/state_fault.hpp"
#include "hardware_system_handler/state_offline.hpp"
#include "hardware_system_handler/state_stop.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <stdexcept>
#include <sstream>
#include <cstdlib>

namespace
{
    constexpr size_t kActiveAxisLimit = 8;
    constexpr int64_t kHomingTimeoutNs = 30000000000LL;
    constexpr int64_t kHomingDoneDelayNs = 2000000000LL;

    size_t active_axis_count(uint8_t axis_count)
    {
        return std::min<size_t>({static_cast<size_t>(axis_count), SHM_MAX_AXES, kActiveAxisLimit});
    }

    struct ShmDerivedState
    {
        bool has_error{false};
        bool all_idle{false};
    };

    ShmDerivedState derive_shm_state(const HwiStateBlock &state)
    {
        ShmDerivedState out{};
        const size_t count = active_axis_count(state.axis_count);
        out.all_idle = true;
        for (size_t i = 0; i < count; ++i)
        {
            const auto &axis = state.axes[i];
            if (axis.error != 0)
            {
                out.has_error = true;
            }
            if (!is_axis_online(axis) || axis.odrive_state_summary != AxisStateSummary::Idle)
            {
                out.all_idle = false;
            }
        }
        return out;
    }

    enum class FsmState
    {
        Unknown = -1,
        Init = 0,
        WaitSensors = 1,
        WaitHoming = 2,
        Ready = 3,
        Running = 4,
        Fault = 5,
        Offline = 6,
        Stop = 7
    };

    FsmState get_fsm_state()
    {
        if (HardwareSystemHandler::is_in_state<INIT>())
            return FsmState::Init;
        if (HardwareSystemHandler::is_in_state<WAIT_SENSORS>())
            return FsmState::WaitSensors;
        if (HardwareSystemHandler::is_in_state<WAIT_HOMING>())
            return FsmState::WaitHoming;
        if (HardwareSystemHandler::is_in_state<READY>())
            return FsmState::Ready;
        if (HardwareSystemHandler::is_in_state<RUNNING>())
            return FsmState::Running;
        if (HardwareSystemHandler::is_in_state<FAULT>())
            return FsmState::Fault;
        if (HardwareSystemHandler::is_in_state<OFFLINE>())
            return FsmState::Offline;
        if (HardwareSystemHandler::is_in_state<STOP>())
            return FsmState::Stop;
        return FsmState::Unknown;
    }

} // namespace

namespace hardware_system_handler
{

    using namespace std::chrono_literals;

    namespace
    {
        constexpr auto kStateStaleTimeout = 500ms;
        constexpr auto kCommandStaleTimeout = 100ms;
        constexpr auto kSensorStaleTimeout = 500ms;
    }

    bool HshNode::refresh_state()
    {
        HwiStateBlock snapshot{};
        if (shmitf_.read_state(snapshot))
        {
            last_state_ = snapshot;
            have_state_snapshot_ = true;
            last_state_snapshot_time_ = get_clock()->now();
        }
        return have_state_snapshot_ && get_clock()->now() - last_state_snapshot_time_ <= kStateStaleTimeout;
    }

    bool HshNode::refresh_cmd_if()
    {
        HwiCommandIfBlock snapshot{};
        if (shmitf_.read_cmd_if(snapshot))
        {
            last_cmd_if_ = snapshot;
            have_cmd_snapshot_ = true;
            last_cmd_snapshot_time_ = get_clock()->now();
        }
        return have_cmd_snapshot_ && get_clock()->now() - last_cmd_snapshot_time_ <= kCommandStaleTimeout;
    }

    bool HshNode::refresh_sensor_state()
    {
        HwiSensorStateBlock snapshot{};
        if (shmitf_.read_sensor_state(snapshot))
        {
            last_sensor_state_ = snapshot;
            have_sensor_snapshot_ = true;
            last_sensor_snapshot_time_ = get_clock()->now();
        }
        return have_sensor_snapshot_ && get_clock()->now() - last_sensor_snapshot_time_ <= kSensorStaleTimeout;
    }

    void HshNode::request_axis_state(size_t axis, AxisDesiredState state)
    {
        if (axis >= SHM_MAX_AXES) return;
        auto &control = desired_control_.axes[axis];
        if (control.desired_axis_state != state)
        {
            control.desired_axis_state = state;
            ++control.desired_state_generation;
        }
        control.action = state == AxisDesiredState::Idle ? ControlAction::Idle :
                         state == AxisDesiredState::ClosedLoop ? ControlAction::ClosedLoop :
                         state == AxisDesiredState::Homing ? ControlAction::Homing :
                         ControlAction::FullCalib;
    }

    void HshNode::request_clear_errors(size_t axis)
    {
        if (axis >= SHM_MAX_AXES || clear_error_requested_[axis]) return;
        ++desired_control_.axes[axis].clear_error_req_id;
        clear_error_requested_[axis] = true;
    }

    HshNode::HshNode() : rclcpp::Node("hsh_node")
    {
        bypass_position_homing_ = this->declare_parameter<bool>("bypass_position_homing", false);
        homing_required_ = !bypass_position_homing_;
        if (bypass_position_homing_)
        {
            RCLCPP_WARN(logger_, "Position homing is bypassed by parameter; only use this for a known-safe mechanism.");
        }
        if (!open_shared_memory())
        {
            throw std::runtime_error("Failed to open shared memory segments");
        }

        poll_timer_ = this->create_wall_timer(
            100ms, std::bind(&HshNode::poll_shared_memory, this));
        watch_dog_timer_ = this->create_wall_timer(
            100ms, std::bind(&HshNode::EventHandler, this));
        transmit_timer_ = this->create_wall_timer(
            5ms, std::bind(&HshNode::ControlCommand, this));

        RCLCPP_INFO(logger_, "HSH node initialized; polling shared memory every 100ms");

        // Start FSM
        RCLCPP_INFO(logger_, "Starting FSM...");
        HardwareSystemHandler::start();

        axis_error_pub_ = this->create_publisher<std_msgs::msg::String>("/hsh/axis_errors", 5000);
        fsm_state_pub_ = this->create_publisher<std_msgs::msg::String>("/hsh/fsm_state", 10);
    }

    bool HshNode::open_shared_memory()
    {
        if (!shmitf_.open(false))
        {
            RCLCPP_FATAL(logger_, "Unable to open shared memory segments (%s, %s, %s, %s)",
                         SHM_HWI_CMD_IF, SHM_HWI_STATE, SHM_HWI_STATE_DEBUG, SHM_HSH_CTRL);
            return false;
        }

        RCLCPP_INFO(logger_, "Connected to shared memory segments (%s, %s, %s, %s)",
                    SHM_HWI_CMD_IF, SHM_HWI_STATE, SHM_HWI_STATE_DEBUG, SHM_HSH_CTRL);
        return true;
    }

    void HshNode::poll_shared_memory()
    {
        if (!shmitf_.ready())
        {
            RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 5000,
                                 "Shared memory not ready yet");
            return;
        }

        if (!refresh_state())
        {
            RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 5000,
                                 "Shared memory state pointer is null");
            return;
        }

        const auto *state = &last_state_;
        const auto derived = derive_shm_state(*state);
        const size_t count = active_axis_count(state->axis_count);
        desired_control_.debug_enable = debug_mode_ ? 1 : 0;

        const bool state_looks_initialized =
            (state->axis_count > 0) || (state->sequence_id != 0) || (state->timestamp_ns != 0);
        const bool axis_count_valid = state->axis_count <= SHM_MAX_AXES;
        const bool init_ready = state_looks_initialized && axis_count_valid;

        if (state_looks_initialized && !axis_count_valid)
        {
            RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 5000,
                                 "Shared memory axis_count out of range: %u", state->axis_count);
        }

        if (init_ready)
        {
            if (error_checked_ && !derived.has_error)
            {
                RCLCPP_INFO(logger_, "System recovered (no errors detected)");
                error_checked_ = false;
            }

            if (HardwareSystemHandler::is_in_state<INIT>())
            {
                HardwareSystemHandler::dispatch(EvInitDone{});
            }
            else if (!error_checked_ && derived.has_error)
            {
                RCLCPP_ERROR(logger_, "Error detected in system!");
                error_checked_ = true;

                // Log detailed errors
                for (size_t i = 0; i < count; ++i)
                {
                    uint32_t err = state->axes[i].error;
                    if (err != 0)
                    {
                        uint8_t axis_error = (err >> 0) & 0xFF;
                        uint8_t motor_error = (err >> 8) & 0xFF;
                        uint8_t encoder_error = (err >> 16) & 0xFF;
                        uint8_t controller_error = (err >> 24) & 0xFF;

                        RCLCPP_ERROR(logger_, "Axis[%zu] error: axis=%u motor=%u encoder=%u controller=%u",
                                     i, axis_error, motor_error, encoder_error, controller_error);
                    }
                }

                HardwareSystemHandler::dispatch(EvAxisError{});
            }
            else if (error_checked_ && derived.has_error)
            {
                // Already handled, waiting for auto recovery
            }
        }
    }

    bool HshNode::SensorActive()
    {
        /*======================TESTING====================*/
        if (!using_sensor)                                                      
        {
            return true;
        }
        /*======================TESTING====================*/
        if (!refresh_sensor_state())
        {
            return false;
        }
        const auto *sensor_state = &last_sensor_state_;
        sensor_count_ = sensor_state->sensor_count;
                // RCLCPP_INFO(logger_,"Sensor count via SHM:%u", sensor_count_);
        if (sensor_count_ == 0)
        {
            return false;
        }

        for (size_t i = 0; i < sensor_count_ && i < SHM_MAX_SENSORS; ++i)
        {
            const auto &sensor = sensor_state->sensors[i];
            if (sensor.online == 0)
            {
                return false;
            }
            if (sensor.status != SensorStatus::Active)
            {
                return false;
            }
        }

        return true;
    }

    bool HshNode::StartHoming()
    {
        if (!homing_required_)
            return false;
        if (homing_in_progress_)
            return false;

        if (!SensorActive())
            return false;

        if (!refresh_state() || !refresh_cmd_if())
            return false;
        auto *ctrl = &desired_control_;
        const auto *state = &last_state_;
        const auto *cmd_if = &last_cmd_if_;

        const auto now = get_clock()->now();
        const size_t count = active_axis_count(state->axis_count);
        ctrl->axis_count = static_cast<uint8_t>(count);
        for (size_t i = count; i < SHM_MAX_AXES; ++i)
        {
            request_axis_state(i, AxisDesiredState::Idle);
            ctrl->axes[i].motion_enable = false;
            ctrl->axes[i].target = 0.0f;
        }
        bool sent = false;
        for (size_t i = 0; i < count; ++i)
        {
            if (cmd_if->axes[i].interface != CommandInterface::Position)
                continue;
            if (homing_command_sent_[i] || homing_done_[i])
                continue;
            request_axis_state(i, AxisDesiredState::Homing);
            ctrl->axes[i].motion_enable = false;
            ctrl->axes[i].target = 0.0f;
            homing_command_sent_[i] = true;
            homing_command_time_ns_[i] = now.nanoseconds();
            sent = true;
        }

        if (sent)
        {
            homing_in_progress_ = true;
            RCLCPP_INFO(logger_, "StartHoming: sent homing command to %zu axes", count);
        }
        // EventHandler only updates desired_control_; ControlCommand publishes it.
        return sent;
    }

    float HshNode::CommandErrorRate()
    {
        float max_rel_error = 0.0f;

        if (!refresh_cmd_if() || !refresh_state()) return max_rel_error;
        const auto *cmd_if = &last_cmd_if_;
        const auto *state = &last_state_;

        const size_t count = active_axis_count(state->axis_count);
        for (size_t i = 0; i < count; ++i)
        {
            float cmd = 0.0f;
            float fb = 0.0f;

            if (cmd_if->axes[i].interface == CommandInterface::Velocity)
            {
                cmd = static_cast<float>(cmd_if->axes[i].value);
                fb = static_cast<float>(state->axes[i].velocity);
            }
            else
            {
                cmd = static_cast<float>(cmd_if->axes[i].value);
                fb = static_cast<float>(state->axes[i].position);
            }

            // Tránh chia cho 0
            if (std::abs(cmd) < 1e-6f)
                continue;

            float rel_error = std::abs(cmd - fb) / std::abs(cmd);

            if (rel_error > max_rel_error)
                max_rel_error = rel_error;
        }

        return max_rel_error;
    }

    bool HshNode::CommandVerify()
    {
        if (!refresh_cmd_if() || !refresh_state()) return false;
        const auto *cmd_if = &last_cmd_if_;
        const auto *state = &last_state_;
        const size_t count = active_axis_count(state->axis_count);
        for (size_t i = 0; i < count; ++i)
        {
            if (cmd_if->axes[i].interface == CommandInterface::Velocity)
            {
                if (std::abs(cmd_if->axes[i].value) > MAX_AXIS_VEL)
                {
                    RCLCPP_ERROR(logger_, "Invalid command for Odrive, user value: %f (RPS), max value: %f (RPS)", cmd_if->axes[i].value, MAX_AXIS_VEL);
                    return false;
                }
            }
            else if (cmd_if->axes[i].interface == CommandInterface::Position)
            {
                if (std::abs(cmd_if->axes[i].value) > MAX_AXIS_POS)
                {
                    RCLCPP_ERROR(logger_, "Invalid command for Odrive, user value: %f (Rad), max value: %f (Rad)", cmd_if->axes[i].value, MAX_AXIS_POS);
                    return false;
                }
            }
        }
        return true;
    }

    void HshNode::EventHandler()
    {
        if (!refresh_state() || !refresh_cmd_if())
        {
            desired_control_.motion_enable = false;
            desired_control_.timestamp_ns = static_cast<uint64_t>(get_clock()->now().nanoseconds());
            desired_control_.sequence_id += 1;
            for (size_t i = 0; i < SHM_MAX_AXES; ++i)
            {
                request_axis_state(i, AxisDesiredState::Idle);
                desired_control_.axes[i].motion_enable = false;
                desired_control_.axes[i].target = 0.0f;
            }
            return;
        }
        auto *ctrl = &desired_control_;
        const auto *state = &last_state_;
        const auto *cmd_if = &last_cmd_if_;

        const bool sensors_active = SensorActive();

        auto log_event = [&](const char *name) {
            RCLCPP_INFO_THROTTLE(logger_, *get_clock(), 1000, "Dispatch event: %s", name);
        };

        const auto now = get_clock()->now();
        const size_t count = active_axis_count(state->axis_count);
        const bool state_looks_initialized =
            (state->axis_count > 0) || (state->sequence_id != 0) || (state->timestamp_ns != 0);

        const auto current_state = get_fsm_state();
        if (static_cast<int>(current_state) != last_fsm_state_)
        {
            const int prev_fsm_state = last_fsm_state_;
            last_fsm_state_ = static_cast<int>(current_state);
            if (current_state == FsmState::WaitSensors)
            {
                wait_sensors_start_ = now;
                sensors_ok_sent_ = false;
            }
            if (current_state == FsmState::WaitHoming)
            {
                wait_homing_start_ = now;
                homing_timeout_sent_ = false;
            }
            if (current_state == FsmState::Fault)
            {
                fault_enter_time_ = now;
                clear_error_time_ = now;
                clear_error_pending_ = true;
                clear_error_requested_.fill(false);
            }
            if (current_state == FsmState::Ready)
            {
                clear_error_pending_ = false;
                debug_mode_ = false;
                error_events_.clear();
                clear_error_requested_.fill(false);
            }
            if (current_state == FsmState::Offline)
            {
                offline_from_fault_ = (prev_fsm_state == static_cast<int>(FsmState::Fault));
                axis_online_sent_ = false;
            }
            else
            {
                offline_from_fault_ = false;
            }
            
            // Publish FSM state
            if (fsm_state_pub_) {
                std_msgs::msg::String msg;
                const char* state_names[] = {
                    "Init", "WaitSensors", "WaitHoming", "Ready", 
                    "Running", "Fault", "Offline", "Stop"
                };
                int state_idx = static_cast<int>(current_state);
                if (state_idx >= 0 && state_idx < 8) {
                    msg.data = state_names[state_idx];
                } else {
                    msg.data = "Unknown";
                }
                fsm_state_pub_->publish(msg);
            }
        }

        // Publish the current state on every watchdog cycle (10 Hz), not only
        // on a transition.  A diagnostic subscriber started after bringup can
        // therefore still observe RUNNING/FAULT/OFFLINE.
        if (fsm_state_pub_)
        {
            std_msgs::msg::String msg;
            switch (current_state)
            {
            case FsmState::Init: msg.data = "Init"; break;
            case FsmState::WaitSensors: msg.data = "WaitSensors"; break;
            case FsmState::WaitHoming: msg.data = "WaitHoming"; break;
            case FsmState::Ready: msg.data = "Ready"; break;
            case FsmState::Running: msg.data = "Running"; break;
            case FsmState::Fault: msg.data = "Fault"; break;
            case FsmState::Offline: msg.data = "Offline"; break;
            case FsmState::Stop: msg.data = "Stop"; break;
            default: msg.data = "Unknown"; break;
            }
            fsm_state_pub_->publish(msg);
        }

        if (!has_timestamp_ || state->timestamp_ns != last_timestamp_ns_)
        {
            last_timestamp_ns_ = state->timestamp_ns;
            last_timestamp_change_time_ = now;
            has_timestamp_ = true;
            state_live_ = true;
        }

        bool any_offline = false;
        bool any_error = false;
        bool any_homing = false;
        bool homing_needed = false;
        bool homing_done = false;
        bool all_required_homed = true;
        [[maybe_unused]] bool all_offline = (count > 0);
        [[maybe_unused]] bool any_running = false;

        for (size_t i = 0; i < count; ++i)
        {
            const auto &axis = state->axes[i];
            const bool online = is_axis_online(axis);
            if (!online)
            {
                any_offline = true;
                homing_done = false;
                if (cmd_if != nullptr &&
                    cmd_if->axes[i].interface == CommandInterface::Position)
                {
                    homing_done_[i] = false;
                }
            }
            else
            {
                all_offline = false;
            }

            if (axis.error != 0)
            {
                any_error = true;
            }

            if (axis.odrive_state_summary == AxisStateSummary::Homing)
            {
                any_homing = true;
                if (!homing_started_[i])
                {
                    homing_started_[i] = true;
                    homing_command_time_ns_[i] = now.nanoseconds();
                }
                homing_done_[i] = false;
                homing_done = false;
            }

            if (axis.odrive_state_summary == AxisStateSummary::Running)
            {
                any_running = true;
            }

            const bool requires_homing = !bypass_position_homing_ &&
                (cmd_if != nullptr) && (cmd_if->axes[i].interface == CommandInterface::Position);
            if (requires_homing)
            {
                homing_needed = true;
                const int64_t elapsed_ns = now.nanoseconds() - homing_command_time_ns_[i];
                const bool idle_after_delay =
                    homing_command_sent_[i] &&
                    homing_started_[i] &&
                    homing_command_time_ns_[i] > 0 &&
                    elapsed_ns > kHomingDoneDelayNs &&
                    axis.odrive_state_summary == AxisStateSummary::Idle;
                if (idle_after_delay)
                {
                    homing_done_[i] = true;
                }
                if (!homing_done_[i])
                {
                    all_required_homed = false;
                }
            }
        }

        if (homing_needed)
        {
            homing_done = all_required_homed;
        }


        if (any_error)
        {
            error_events_.push_back(now);
        }
        while (!error_events_.empty() &&
               (now - error_events_.front()).nanoseconds() > 10000000000LL)
        {
            error_events_.pop_front();
        }
        const bool error_rate_exceeded = error_events_.size() >= 3;

        const bool in_fault = (current_state == FsmState::Fault);
        const bool safe_state =
            (current_state == FsmState::Init) ||
            (current_state == FsmState::WaitSensors) ||
            (current_state == FsmState::WaitHoming) ||
            (current_state == FsmState::Ready) ||
            (current_state == FsmState::Running);

        if (safe_state && any_offline)
        {
            log_event("EvAxisOffline");
            HardwareSystemHandler::dispatch(EvAxisOffline{});
        }

        if (safe_state && any_error && !error_checked_)
        {
            log_event("EvAxisError");
            HardwareSystemHandler::dispatch(EvAxisError{});
        }

        if (any_offline)
        {
            std::ostringstream oss;
            oss << "Axis offline detected; timestamp_ns=" << state->timestamp_ns << " offline_axes=[";
            bool first = true;
            for (size_t i = 0; i < count; ++i)
            {
                const auto &axis = state->axes[i];
                if (is_axis_online(axis))
                    continue;
                if (!first)
                    oss << ",";
                oss << i;
                first = false;
            }
            oss << "]";
            RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 1000, "%s", oss.str().c_str());
        }

        if (any_offline)
        {
            if (!offline_seen_)
            {
                offline_seen_ = true;
            }
            axis_online_sent_ = false;
        }
        else
        {
            offline_seen_ = false;
        }

        if (current_state == FsmState::WaitSensors)
        {
            if (!sensors_ok_sent_ && sensors_active)
            {
                log_event("EvSensorsOk");
                HardwareSystemHandler::dispatch(EvSensorsOk{});
                sensors_ok_sent_ = true;
            }

            if (!sensors_ok_sent_ &&
                (now - wait_sensors_start_).nanoseconds() > 1000000000LL)
            {
                log_event("EvSensorsTimeout");
                HardwareSystemHandler::dispatch(EvSensorsTimeout{});
            }
        }

        if (current_state == FsmState::Offline && !any_offline && !axis_online_sent_)
        {
            if (offline_from_fault_)
            {
                log_event("EvAxisOnlineFault");
                HardwareSystemHandler::dispatch(EvAxisOnlineFault{});
            }
            else
            {
                log_event("EvAxisOnline");
                HardwareSystemHandler::dispatch(EvAxisOnline{});
            }
            axis_online_sent_ = true;
        }

        if (in_fault)
        {
            for (size_t i = 0; i < count; ++i)
            {
                if (state->axes[i].error != 0)
                {
                    request_axis_state(i, AxisDesiredState::Idle);
                    ctrl->axes[i].motion_enable = false;
                    ctrl->axes[i].target = 0.0f;
                    request_clear_errors(i);
                }
            }

            if (!any_error && clear_error_pending_ && sensors_active)
            {
                log_event("EvClearError");
                HardwareSystemHandler::dispatch(EvClearError{});
                clear_error_pending_ = false;
            }

            const bool clear_timeout_with_error =
                clear_error_pending_ &&
                (now - clear_error_time_).nanoseconds() > 500000000LL && any_error;
            if (error_rate_exceeded || clear_timeout_with_error)
            {
                if (error_rate_exceeded)
                {
                    RCLCPP_ERROR(logger_, "FAULT too frequent; entering STOP");
                }
                if (clear_timeout_with_error)
                {
                    RCLCPP_ERROR(logger_, "Clear error timeout while error persists; entering STOP");
                }
                log_event("EvCriticalError");
                HardwareSystemHandler::dispatch(EvCriticalError{});
                debug_mode_ = true;
                clear_error_pending_ = false;
            }
        }

        // WAIT_SENSORS always transitions through WAIT_HOMING in the legacy
        // FSM.  For a configured spring-return brake this is only a transit
        // state: leave it immediately without emitting a homing command.
        if (current_state == FsmState::WaitHoming && bypass_position_homing_)
        {
            ctrl->motion_enable = false;
            homing_required_ = false;
            homing_in_progress_ = false;
            homing_command_sent_.fill(false);
            homing_started_.fill(false);
            homing_done_.fill(false);
            RCLCPP_WARN(logger_, "Bypassing WAIT_HOMING; dispatching EvHomingDone without CAN homing.");
            HardwareSystemHandler::dispatch(EvHomingDone{});
            return;
        }

        if (current_state == FsmState::WaitHoming && homing_required_)
        {
            ctrl->motion_enable = false;
            if (any_offline)
            {
                return;
            }
            if (!homing_timeout_sent_ &&
                (now - wait_homing_start_).nanoseconds() > kHomingTimeoutNs)
            {
                log_event("EvHomingTimeout");
                HardwareSystemHandler::dispatch(EvHomingTimeout{});
                homing_timeout_sent_ = true;
                return;
            }
            StartHoming();
        }

        if (current_state == FsmState::WaitHoming && homing_required_ &&
            homing_done && !any_homing && !any_offline)
        {
            for (size_t i = 0; i < count; ++i)
            {
                if (cmd_if->axes[i].interface == CommandInterface::Position)
                {
                    // Complete the procedural generation before permitting a
                    // later, distinct homing generation.
                    request_axis_state(i, AxisDesiredState::Idle);
                    ctrl->axes[i].motion_enable = false;
                    ctrl->axes[i].target = 0.0f;
                }
            }
            log_event("EvHomingDone");
            HardwareSystemHandler::dispatch(EvHomingDone{});
            ctrl->motion_enable = true;
            homing_required_ = false;
            homing_in_progress_ = false;
            homing_command_sent_.fill(false);
            homing_started_.fill(false);
            homing_done_.fill(false);
        }

        if (current_state == FsmState::Ready)
        {
            if (homing_required_)
            {
                ctrl->motion_enable = false;
                log_event("EvHomingRequired");
                HardwareSystemHandler::dispatch(EvHomingRequired{});
                return;
            }

            if (CommandVerify())
            {
                log_event("EvEnableCmd");
                HardwareSystemHandler::dispatch(EvEnableCmd{});
                ctrl->motion_enable = true;
            }
        }

        if (current_state == FsmState::Running)
        {
            if (!sensors_active)
            {
                log_event("EvSensorsTimeout");
                HardwareSystemHandler::dispatch(EvSensorsTimeout{});
            }
        }

        if (current_state == FsmState::Stop)
        {
            motion_allowed_.store(false);
            ctrl->motion_enable = false;
            for (size_t i = 0; i < count; ++i)
            {
                request_axis_state(i, AxisDesiredState::Idle);
                ctrl->axes[i].motion_enable = false;
                ctrl->axes[i].target = 0.0f;
            }
        }

        if (debug_mode_ && axis_error_pub_ && any_error)
        {
            std_msgs::msg::String msg;
            std::ostringstream oss;
            oss << "axis_errors: ";
            bool first = true;
            for (size_t i = 0; i < count; ++i)
            {
                if (state->axes[i].error == 0)
                    continue;
                if (!first)
                    oss << ", ";
                oss << "axis[" << i << "]=0x" << std::hex << state->axes[i].error << std::dec;
                first = false;
            }
            msg.data = oss.str();
            axis_error_pub_->publish(msg);
        }
    }

    void HshNode::ControlCommand()
    {
        if (!refresh_state() || !refresh_cmd_if())
        {
            desired_control_.motion_enable = false;
            desired_control_.timestamp_ns = static_cast<uint64_t>(get_clock()->now().nanoseconds());
            desired_control_.sequence_id += 1;
            for (size_t i = 0; i < SHM_MAX_AXES; ++i)
            {
                request_axis_state(i, AxisDesiredState::Idle);
                desired_control_.axes[i].motion_enable = false;
                desired_control_.axes[i].target = 0.0f;
            }
            (void)shmitf_.write_control(desired_control_);
            return;
        }
        auto *ctrl = &desired_control_;
        const auto *cmd_if = &last_cmd_if_;
        const auto *state = &last_state_;
        if (!HshNode::SensorActive())
        {
            return;
        }

        const size_t count = active_axis_count(state->axis_count);
        ctrl->axis_count = static_cast<uint8_t>(count);
        ctrl->timestamp_ns = static_cast<uint64_t>(get_clock()->now().nanoseconds());
        ctrl->sequence_id += 1;

        if (!HshNode::CommandVerify())
        {
            return;
        }

        for (size_t i = 0; i < count; ++i)
        {
            const auto interface = cmd_if->axes[i].interface;
            const bool is_brake_axis = interface == CommandInterface::Position;

            ctrl->axes[i].control_mode = interface;
            if (cmd_if->axes[i].action == ControlAction::Idle)
            {
                ctrl->axes[i].motion_enable = false;
                request_axis_state(i, AxisDesiredState::Idle);
                ctrl->axes[i].target = 0.0f;
                continue;
            }

            // Velocity axes are wheel drives and remain gated by the HSH
            // motion permission. Never let a disabled wheel retain a stale
            // closed-loop target.
            if (!is_brake_axis && !ctrl->motion_enable)
            {
                ctrl->axes[i].motion_enable = false;
                request_axis_state(i, AxisDesiredState::Idle);
                ctrl->axes[i].target = 0.0f;
                continue;
            }

            float target = cmd_if->axes[i].value;
            if (interface == CommandInterface::Position ||
                interface == CommandInterface::Velocity)
            {
                target = target / (2.0f * 3.141592f);
            }
            ctrl->axes[i].motion_enable = true;
            request_axis_state(i, AxisDesiredState::ClosedLoop);
            if (ctrl->axes[i].target != target) ++ctrl->axes[i].target_sequence;
            ctrl->axes[i].target = target;
            RCLCPP_DEBUG_THROTTLE(logger_, *get_clock(), 500,
                                  "ControlCommand: axis=%zu action=ClosedLoop target=%.3f",
                                  i, static_cast<double>(ctrl->axes[i].target));
        }
        (void)shmitf_.write_control(desired_control_);
    }
} // namespace hardware_system_handler

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<hardware_system_handler::HshNode>());
    rclcpp::shutdown();
    return 0;
}
