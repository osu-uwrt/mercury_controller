#pragma once

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "simulink_model.hpp"


class ControllerOverseer : public rclcpp::Node {

    using string = std::string

    bool waitingOnInit;

    int escPowerStopsLow, escPowerStopsHigh;

    YAML::Node configTree;

    using fs = std::filesystem;

    string autoffConfigPath;

    SimulinkModelClass completeController;



    ControllerOverseer() : Node("controller_overseer") {
        
    }



};