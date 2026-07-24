#pragma once

#include "hardware_system_handler/fsm.hpp"

class FAULT;
class OFFLINE;

class RUNNING : public HshFsm {
public:
    void entry() override {
        motion_allowed_ = true;
        RCLCPP_INFO(logger_, "RUNNING State");
    }

    void exit() override {}

    void react(EvSensorsTimeout const&) override {
        transit<FAULT>();
    }

    void react(EvAxisOffline const&) override {
        transit<OFFLINE>();
    }

    void react(EvAxisError const&) override {
        transit<FAULT>();
    }

    template <typename Event>
    void react(Event const&) { }
};
