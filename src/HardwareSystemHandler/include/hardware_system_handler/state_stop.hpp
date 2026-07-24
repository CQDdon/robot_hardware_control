#pragma once

#include "hardware_system_handler/fsm.hpp"


class STOP : public HshFsm
{
public:
    void entry()
    {
        motion_allowed_ = false;
        RCLCPP_WARN(logger_, "STOP State engaged");
    }

    void exit() {}

    template <typename Event>
    void react(Event const &)
    {
        // Terminal state: no transitions by default
    }
};
