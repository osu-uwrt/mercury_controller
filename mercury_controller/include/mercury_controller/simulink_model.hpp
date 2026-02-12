#pragma once

#include <rclcpp/rclcpp.hpp>

#include <rcl_interfaces/srv/list_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>


#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_value.hpp>
#include "monitored_service_client.hpp"
#include "controller_overseer.hpp"
#include <std_srvs/srv/trigger.hpp>

#include <vector>
#include <yaml-cpp/yaml.h>
#include <string>
#include <unordered_set>
#include <memory>

#define PARAMETER_SCALE 1000000
#define RELOAD_TIME 2

class SimulinkModelClass{

    using string = std::string;
    bool modelActive;
    string fullNodeName;

    using ListParams = rcl_interfaces::srv::ListParameters;
    using SetParams = rcl_interfaces::srv::SetParameters;
    using SetParamsResult = rcl_interfaces::msg::SetParametersResult;
    using Parameter = rcl_interfaces::msg::Parameter;
    using ParameterType = rclcpp::ParameterType;
    using ParameterValue = rcl_interfaces::msg::ParameterValue;


    //list and set param client
    std::shared_ptr<MonitoredServiceClient<ListParams>> listParamClient;
    std::shared_ptr<MonitoredServiceClient<SetParams>> setParamClient;

    //overseer node and node name
    std::shared_ptr<ControllerOverseer> overseer;
    string nodeName;

    //set of known parameters
    std::unordered_set<string> knownParams;

    //reload parameter service and ROS Time of last reload time
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reloadParamService;
    rclcpp::Time lastReloadTime;

    //have params been loaded
    bool paramsLoaded;

    public:

    std::vector<std::pair<std::string,int64_t>> intV;
    std::vector<std::pair<std::string,bool>> boolV;
    std::vector<std::pair<std::string,std::vector<int64_t>>> arrayV;

    //constructor
    SimulinkModelClass(std::shared_ptr<ControllerOverseer> overseerNode, string nodeName);

    /*
        Checks if model is active, handles when model comes up or down.
    */
    void checkIfActive(std::vector<string>> activeNodes);

    bool listAndSetModelParameters();

    void setModelParametersFromListCallback(ListParams::Response::SharedPtr response);


    void setModelParameters(std::vector<string> paramsToSet);


    void setParametersDoneCallback(SetParams::Response::SharedPtr res, SetParams::Request::SharedPtr req);


    bool reloadParams();


    std_srvs::srv::Trigger::Response reloadParametersCallback(std_srvs::srv::Trigger::Response res);








    





    




};