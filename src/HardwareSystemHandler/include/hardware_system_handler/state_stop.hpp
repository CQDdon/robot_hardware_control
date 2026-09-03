#pragma once

#include "hardware_system_handler/fsm.hpp"

// Forward declaration
class READY;

class STOP : public HshFsm
{
public:
    void entry() override
    {
        motion_allowed_ = false;
        RCLCPP_WARN(logger_, "STOP State engaged: Locking brakes, disabling wheels and throttle");
    }

    void exit() override {}

    void react(EvEStopReset const &) override
    {
        RCLCPP_INFO(logger_, "STOP State: E-Stop reset conditions met -> Transitioning to READY");
        transit<READY>();
    }

    template <typename Event>
    void react(Event const &)
    {
        // Terminal state: other events ignored unless reset
    }
};
