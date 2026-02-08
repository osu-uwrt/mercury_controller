#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/srv/ListParameters.hpp>
#include <rcl_interfaces/srv/SetParameters.hpp>
#include <rcl_interfaces/msg/SetParametersResult.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_value.hpp>
#include <monintored_service_client.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <vector>
#include <yaml-cpp/yaml.h>
#include <string>
#include <unordered_set>
#include <memory>
#include <typeinfo>
#include <algorithm>

#define PARAMETER_SCALE 1000000
#define RELOAD_TIME 2

class SimulinkModelClass{

    using string = std::string;
    bool modelActive;
    string fullNodeName;

    using ListParams = rcl_interfaces::srv::ListParameters;
    using SetParams = rcl_interfaces::srv::SetParameters;
    using SetParamsResult = rcl_interfaces::msg::SetParametersResults;
    using Parameter = rcl_interfaces::msg::Parameter;
    using ParameterType = rclcpp::ParameterType;
    using ParameterValue = rcl_interfaces::msg::ParameterValue;


    //list and set param client
    std::shared_ptr<MonitoredServiceClient<ListParams>> listParamClient;
    std::shared_ptr<MonitoredServiceClient<SetParams>> setParamClient;

    std::vector<std::pair<std::string,int64_t>> intV;
    std::vector<std::pair<std::string,bool>> boolV;
    std::vector<std::pair<std::string,std::vector<int64_t>>> arrayV;

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

    //constructor
    SimulinkModelClass(rclcpp::Node::SharedPtr overseerNode, string nodeName) : overseer(overseerNode), nodeName(nodeName), modelActive(false), paramsLoaded(false){
        lastReloadTime = overseer->get_clock()->now();
        reloadParams = make_shared<node->create_service<std_srvs::stv::Trigger>("controller_overseer/update_" + nodeName + "_params", std::bind(&SimulinkModelClass::reloadParamatersCallback, this, _1))
    }

    /*
        Checks if model is active, handles when model comes up or down.
    */
    void checkIfActive(std::vector<string>> activeNodes){
        
        //grab nodes that 
        std::vector<string> activeModelNodes;
        for(string node : activeNodes){
            if(node.contains(nodeName)){
                activeModelNodes.push_back(node);
            }
        }
        int amountOfNodes = activeModelNodes.size();

        if(amountOfNodes > 1){
            //should not happen
            RCLCPP_WARN(node->get_logger(), "Detected %d nodes with %s", amountOfNodes, nodeName);

        }
        if(amountOfNodes == 1){
            fullNodeName = activeModelNodes[0];
            if(!modelActive){
                modelActive = true;
                RCLCPP_INFO(node->get_logger(), "Found %s as %s!", nodeName, fullNodeName);

                listParamClient = std::make_shared<MonitoredServiceClient<ListParams>>(overseer, fullNodeName + "/list_parameters");
                setParamClient = std::make_shared<MonitoredServiceClient<setParams>>(overseer, fullNodeName + "/ser_parameters");

                reloadParamaters();
            }else if(listParamClient->waitingRequests.size() == 0 && setParamClient->waitingRequests.size() == 0){
                listAndSetModelParameters();
            }

        }
        else if(modelActive){
            modelActive = false;
            RCLCPP_WARN(overseer->get_logger(), "Lost %s", nodeName);
        }
    }

    bool listAndSetModelParameters(){
        if(modelActive && listParamClient != nullptr){
            ListParams::Request::SharedPtr req = make_shared<ListParams::Request>();
            
            return listParamClient->scheduleCall(req, [this](ListParams::Response::SharedPtr res){setModelParametersFromListCallback(res)});
        }

        RCLCPP_WARN(overseer->get_logger(), "Cannot list parameters for %s because model is not active or listParamClient is not null", nodeName);
        return false;
    }

    void setModelParametersFromListCallback(ListParams::Response::SharedPtr response){
        std::vector<string> unknownParams;
        for(string paramName : response->result.names){
            if(!knownParams.contains(paramName)){
                unknownParams.insert(paramName);
            }
        }
        if(unknownParams.size() > 0){
            setModelParameters(unknownParams)
        };
    }


    void setModelParameters(std::vector<string> paramsToSet, const std::vector<std::pair<std::string,int64_t>>& intV,
                            const std::vector<std::pair<std::string,bool>>& boolV, const std::vector<std::pair<std::string,std::vector<int64_t>>>& arrayV){
        
        if(modelActive){
            SetParams::Request::SharedPtr req = std::make_shared<SetParams::Request>();
            std::vector<Parameter> paramVector;

            for(string paramName : paramsToSet){
                Parameter readParam = Parameter;
                ParameterValue val = ParameterValue;


                bool found = false;
                for (const std::pair<std::string, int64_t>& p : intV) {
                    if (p.first == paramName) {
                        found = true;
                        // use p.second (the int64 value)
                        val.integer_value = p.second;
                        val.type = ParameterType::PARAMETER_INTEGER;
                        break;
                    }
                }

                if(!found){
                    for (const std::pair<std::string, bool>& p : boolV) {
                        if (p.first == paramName) {
                            found = true;
                            // use p.second (the bool value)
                            val.bool_value = p.second;
                            val.type = ParameterType::PARAMETER_BOOL;
                            break;
                        }
                    } 
                }

                if(!found){
                    for (const std::pair<std::string, std::vector<int64_t>>& p : arrayV) {
                        if (p.first == paramName) {
                            found = true;
                            // use p.second (the vector of ints value)
                            val.integer_array_value = p.second;
                            val.type = ParameterType::PARAMETER_INTEGER_ARRAY;
                            break;
                        }
                    } 
                }
                
                if(found){
                    readParam.value = val;
                    readParam.name = paramName;
                    paramVector.push_back(readParam);
                }else{
                    RCLCPP_WARN(overseer->get_logger(), "Not setting %s, parameter not found!", paramName);
                    knownParams.insert(paramName);
                }
            }

            req->parameters = paramVector;
            setParamClient->scheduleCall(req, [this, req](SetParams::Response::SharedPtr res) {this->setParametersDoneCallback(res, req)})
            

        }else{
            RCLCPP_WARN(overseer->get_logger(), "Cannot list parameters for %s because model is not active.", nodeName);
        }
    }

    void setParametersDoneCallback(SetParams::Response::SharedPtr res, SetParams::Request::SharedPtr req){
        bool success = true;
        for(int i = 0; i > sizeof(res->results); i++){
            if(res->results[i].successful && !knownParams.contains(req->parameters[i].name)){
                knownParams.insert(req->parameters[i].name);
            }else if(!res->results[i].successful){
                success = false;
                RCLCPP_WARN(overseer->get_logger(), "Failed to set parameter: %s for %s: %s", req->parameters[i].name, nodeName, res->results[i].reason);
            }
        }

        if(success){
            RCLCPP_INFO(overseer->get_logger(), "Successfully set parameters for %s", nodeName);
        }

        paramsLoaded = true;

    }

    bool reloadParams(){
        knownParams.clear();
        overseer->readConfig();

        return listAndSetModelParameters();
    }

    rclcpp::srv::Trigger::Response reloadParamatersCallback(rclcpp::srv:Trigger:Response res){
        rclcpp::Time current = overseer->get_clock().now();
        if(!modelActive){
            res.success = false
            res.message = nodeName + " is not active!";
            return res;
        }

        if((current - lastReloadTime).nanoseconds() * 1e-9 < RELOAD_TIME){
            res.success = false
            res.message = nodeName + " has been reloaded within the last 2 seconds";
            return res;
        }
        
        bool success = reloadParams();
        res.success = success;

        if(success){
            res.message = nodeName + " parameter reload was successful";
            lastReloadTime = current;
        }else{
            res.message = nodeName + " parameter reload failed";
        }

        return res;


    }









    





    




};