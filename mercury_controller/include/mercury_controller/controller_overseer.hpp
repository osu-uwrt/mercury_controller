#pragma once

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <filesystem>

#include "simulink_model.hpp"


class ControllerOverseer : public rclcpp::Node {

    using string = std::string;

    bool waitingOnInit;

    int escPowerStopsLow, escPowerStopsHigh;

    YAML::Node configTree;

    string autoffConfigPath;

    std::shared_ptr<SimulinkModelClass> completeController;



    ControllerOverseer() : Node("controller_overseer") {
        
    }

    readConfig();

    


};