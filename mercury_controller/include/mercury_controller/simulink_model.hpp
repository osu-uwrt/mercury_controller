#pragma once

#include <rclcpp/rclcpp.hpp>

#include <rcl_interfaces/srv/list_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>

#include <std_srvs/srv/trigger.hpp>


#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>


#include "monitored_service_client.hpp"

class ControllerOverseer;

using string = std::string;

using ListParams = rcl_interfaces::srv::ListParameters;
using SetParams = rcl_interfaces::srv::SetParameters;
using SetParamsResult = rcl_interfaces::msg::SetParametersResult;
using Parameter = rcl_interfaces::msg::Parameter;
using ParameterType = rclcpp::ParameterType;
using ParameterValue = rcl_interfaces::msg::ParameterValue;

#define PARAMETER_SCALE 1000000
#define RELOAD_TIME 2

class SimulinkModelClass{


    bool modelActive;
    string fullNodeName;




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

    //list and set param client
    std::shared_ptr<MonitoredServiceClient<ListParams>> listParamClient;
    std::shared_ptr<MonitoredServiceClient<SetParams>> setParamClient;

    
    std::vector<std::pair<std::string,int64_t>> intV;
    std::vector<std::pair<std::string,bool>> boolV;
    std::vector<std::pair<std::string,std::vector<int64_t>>> arrayV;

    //constructor
    SimulinkModelClass(std::shared_ptr<ControllerOverseer> overseerNode, string nodeName);

    /*
        Checks if model is active, handles when model comes up or down.
    */
    void checkIfActive(const std::vector<string>& activeNodes);

    /*
        returns true is the parameters are successfully set and listed 
     */
    bool listAndSetModelParameters();


    /*
        Callback that does the setting of the parameters for the listAndSetModelParameters function
    */
    void setModelParametersFromListCallback(ListParams::Response::SharedPtr response);


    /*
        Sets the models parameters from a vector of strings
    */
    void setModelParameters(std::vector<string> paramsToSet);

    /*
        Callback for when the set parameters client is called
    */
    void setParametersDoneCallback(SetParams::Response::SharedPtr res, SetParams::Request::SharedPtr req);


    /*
        reloads the parameters from the .yaml file
    */
    bool reloadParams();

    /*
        Callback that makes the response for the triggered client for reloading the parameters
    */
    void reloadParametersCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                                                std::shared_ptr<std_srvs::srv::Trigger::Response> res);



};