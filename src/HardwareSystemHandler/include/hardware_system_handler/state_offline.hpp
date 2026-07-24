#pragma once

#include "hardware_system_handler/fsm.hpp"

class READY;
class FAULT;

class OFFLINE : public HshFsm {
public:
    void entry() override {
        RCLCPP_WARN(logger_, "OFFLINE State");
    }
    void exit() override {}

    void react(EvAxisOnline const&) override {
        transit<READY>();
    }

    void react(EvAxisOnlineFault const&) override {
        transit<FAULT>();
    }

    template <typename Event>
    void react(Event const&) { }
};
