#include "simulink_model.hpp"
#include "controller_overseer.hpp"

#include <algorithm>


    //constructor
    SimulinkModelClass::SimulinkModelClass(std::shared_ptr<ControllerOverseer> overseerNode, string nodeName) : overseer(overseerNode), nodeName(nodeName), modelActive(false), paramsLoaded(false){
        lastReloadTime = overseer->get_clock()->now();
        reloadParamService = overseerNode->create_service<std_srvs::srv::Trigger>("controller_overseer/update_" + nodeName + "_params", std::bind(&SimulinkModelClass::reloadParametersCallback, this,  std::placeholders::_1, std::placeholders::_2));
    }

    /*
        Checks if model is active, handles when model comes up or down.
    */
    void SimulinkModelClass::checkIfActive(const std::vector<string>& activeNodes){
        
        //grab nodes that 
        std::vector<string> activeModelNodes;
        for(string node : activeNodes){
            if(node.find(nodeName) != std::string::npos){
                activeModelNodes.push_back(node);
            }
        }
        int amountOfNodes = activeModelNodes.size();

        if(amountOfNodes > 1){
            //should not happen
            RCLCPP_WARN(overseer->get_logger(), "Detected %d nodes with %s", amountOfNodes, nodeName.c_str());

        }
        if(amountOfNodes == 1){
            fullNodeName = activeModelNodes[0];
            if(!modelActive){
                modelActive = true;
                RCLCPP_INFO(overseer->get_logger(), "Found %s as %s!", nodeName.c_str(), fullNodeName.c_str());

                listParamClient = std::make_shared<MonitoredServiceClient<ListParams>>(overseer, fullNodeName + "/list_parameters");
                setParamClient = std::make_shared<MonitoredServiceClient<SetParams>>(overseer, fullNodeName + "/set_parameters");

                reloadParams();
            }else if(listParamClient->isIdle() && setParamClient->isIdle()){
                listAndSetModelParameters();
            }

        }
        else if(modelActive){
            modelActive = false;
            RCLCPP_WARN(overseer->get_logger(), "Lost %s", nodeName.c_str());
        }
    }

    bool SimulinkModelClass::listAndSetModelParameters(){
        if(modelActive && listParamClient != nullptr){
            ListParams::Request::SharedPtr req = std::make_shared<ListParams::Request>();
            
            return listParamClient->scheduleCall(req, [this](ListParams::Response::SharedPtr res){setModelParametersFromListCallback(res);});
        }

        RCLCPP_WARN(overseer->get_logger(), "Cannot list parameters for %s because model is not active or listParamClient is not null", nodeName.c_str());
        return false;
    }

    void SimulinkModelClass::setModelParametersFromListCallback(ListParams::Response::SharedPtr response){
        std::vector<string> unknownParams;
        for(string paramName : response->result.names){
            if(knownParams.find(paramName) == knownParams.end()){
                unknownParams.push_back(paramName);
            }
        }
        if(unknownParams.size() > 0){
            setModelParameters(unknownParams);
        };
    }


    void SimulinkModelClass::setModelParameters(std::vector<string> paramsToSet){
        
        if(modelActive){
            SetParams::Request::SharedPtr req = std::make_shared<SetParams::Request>();
            std::vector<Parameter> paramVector;

            for(string paramName : paramsToSet){
                Parameter readParam;
                ParameterValue val;


                bool found = false;
                for (const std::pair<std::string, int64_t>& p : intV) {
                    if (p.first == paramName) {
                        found = true;
                        val.integer_value = p.second;
                        val.type = ParameterType::PARAMETER_INTEGER;
                        break;
                    }
                }

                if(!found){
                    for (const std::pair<std::string, bool>& p : boolV) {
                        if (p.first == paramName) {
                            found = true;
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
                    RCLCPP_WARN(overseer->get_logger(), "Not setting %s, parameter not found!", paramName.c_str());
                    knownParams.insert(paramName);
                }
            }

            req->parameters = paramVector;
            setParamClient->scheduleCall(req, [this, req](SetParams::Response::SharedPtr res) {this->setParametersDoneCallback(res, req);});
            

        }else{
            RCLCPP_WARN(overseer->get_logger(), "Cannot list parameters for %s because model is not active.", nodeName.c_str());
        }
    }

    void SimulinkModelClass::setParametersDoneCallback(SetParams::Response::SharedPtr res, SetParams::Request::SharedPtr req){
        bool success = true;
        for(int i = 0; i < res->results.size(); i++){
            if(res->results[i].successful && knownParams.find(req->parameters[i].name) == knownParams.end()){
                knownParams.insert(req->parameters[i].name);
            }else if(!res->results[i].successful){
                success = false;
                RCLCPP_WARN(overseer->get_logger(), "Failed to set parameter: %s for %s: %s", req->parameters[i].name.c_str(), nodeName.c_str(), res->results[i].reason.c_str());
            }
        }

        if(success){
            RCLCPP_INFO(overseer->get_logger(), "Successfully set parameters for %s", nodeName.c_str());
        }

        paramsLoaded = true;

    }

    bool SimulinkModelClass::reloadParams(){
        knownParams.clear();
        overseer->readConfig();

        return listAndSetModelParameters();
    }

    void SimulinkModelClass::reloadParametersCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                                                                    std::shared_ptr<std_srvs::srv::Trigger::Response> res){
        rclcpp::Time current = overseer->get_clock()->now();
        if(!modelActive){
            res->success = false;
            res->message = nodeName + " is not active!";
            return;
        }

        if((current - lastReloadTime).nanoseconds() * 1e-9 < RELOAD_TIME){
            res->success = false;
            res->message = nodeName + " has been reloaded within the last 2 seconds";
            return;
        }
        
        res->success = reloadParams();

        if(res->success){
            res->message = nodeName + " parameter reload was successful";
            lastReloadTime = current;
        }else{
            res->message = nodeName + " parameter reload failed";
        }


    }