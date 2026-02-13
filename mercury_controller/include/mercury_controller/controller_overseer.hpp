#pragma once

#include "monitored_service_client.hpp"
#include "simulink_model.hpp"
#include <rclcpp/rclcpp.hpp>

#include <filesystem>


class ControllerOverseer : public rclcpp::Node {

    bool waitingOnInit;

    int escPowerStopsLow, escPowerStopsHigh;

    YAML::Node configTree;

    using fs = std::filesystem;

    string autoffConfigPath;

    SimulinkModelClass completeController;



    ControllerOverseer() : Node("controller_overseer") {
        
    }



}