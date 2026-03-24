#include "simulink_model.hpp"
#include "controller_overseer.hpp"

#include <algorithm>


    //constructor
    SimulinkModelClass::SimulinkModelClass(std::shared_ptr<ControllerOverseer> overseerNode, string name) : overseer(overseerNode), nodeName(name), modelActive(false), paramsLoaded(false){
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

        //if there is one node, continue
        if(amountOfNodes == 1){
            fullNodeName = activeModelNodes[0];

            //if the model is not active, set the model as active and make the list/set parameters clients
            if(!modelActive){
                modelActive = true;
                RCLCPP_INFO(overseer->get_logger(), "Found %s as %s!", nodeName.c_str(), fullNodeName.c_str());

                listParamClient = std::make_shared<MonitoredServiceClient<ListParams>>(overseer.get(), fullNodeName + "/list_parameters");
                setParamClient = std::make_shared<MonitoredServiceClient<SetParams>>(overseer.get(), fullNodeName + "/set_parameters");

                reloadParams();

            //if the model is active and neither client has any actions, list and set the parameters
            }else if(listParamClient->isIdle() && setParamClient->isIdle()){
                listAndSetModelParameters();
            }

        }else if(modelActive){
            //if the model was at some point active but the node is not found
            modelActive = false;
            RCLCPP_WARN(overseer->get_logger(), "Lost %s", nodeName.c_str());
        }
    }

    bool SimulinkModelClass::listAndSetModelParameters(){

        //if the model is active and there is a list parameters client, call and list parameters service and then set parameters
        if(modelActive && listParamClient != nullptr){
            ListParams::Request::SharedPtr req = std::make_shared<ListParams::Request>();

            return listParamClient->scheduleCall(req, [this](ListParams::Response::SharedPtr res){setModelParametersFromListCallback(res);});
        }

        RCLCPP_WARN(overseer->get_logger(), "Cannot list parameters for %s because model is not active or listParamClient is not null", nodeName.c_str());
        return false;
    }

    void SimulinkModelClass::setModelParametersFromListCallback(ListParams::Response::SharedPtr response){
        std::vector<string> unknownParams;
        //store all parameters that are not known
        for(string paramName : response->result.names){
            if(knownParams.find(paramName) == knownParams.end()){
                unknownParams.push_back(paramName);
            }
        }
        //set all unknown parameters
        if(unknownParams.size() > 0){
            setModelParameters(unknownParams);
        };
    }


    void SimulinkModelClass::setModelParameters(std::vector<string> paramsToSet){
        
        //if model is active, set parameters
        if(modelActive){
            //make a set parameters request and vector of parameters
            SetParams::Request::SharedPtr req = std::make_shared<SetParams::Request>();
            std::vector<Parameter> paramVector;

            //for every unknown parameter, serach the yaml output vectors for the parameter
            for(string paramName : paramsToSet){
                Parameter readParam;
                ParameterValue val;


                bool found = false;
                //check for integers
                for (const std::pair<std::string, int64_t>& p : intV) {
                    if (p.first == paramName) {
                        found = true;
                        val.integer_value = p.second;
                        val.type = ParameterType::PARAMETER_INTEGER;
                        break;
                    }
                }

                //if not found, search for booleans
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

                //if not found, search for arrays
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
                
                //if found, construct Parameter object and place in vector
                if(found){
                    readParam.value = val;
                    readParam.name = paramName;
                    paramVector.push_back(readParam);
                }else{
                    RCLCPP_WARN(overseer->get_logger(), "Not setting %s, parameter not found!", paramName.c_str());
                    knownParams.insert(paramName);
                }
            }

            //set parameters from vector
            req->parameters = paramVector;
            setParamClient->scheduleCall(req, [this, req](SetParams::Response::SharedPtr res) {this->setParametersDoneCallback(res, req);});
            

        }else{
            RCLCPP_WARN(overseer->get_logger(), "Cannot list parameters for %s because model is not active.", nodeName.c_str());
        }
    }

    //Callback for Set Parameters service.
    void SimulinkModelClass::setParametersDoneCallback(SetParams::Response::SharedPtr res, SetParams::Request::SharedPtr req){

        //loop through all responses
        bool success = true;
        for(int i = 0; i < sizeof(res->results)/sizeof(res->results[0]); i++){
            //if result is successful and parameter is not known, add to known parameters
            if(res->results[i].successful && knownParams.find(req->parameters[i].name) == knownParams.end()){
                knownParams.insert(req->parameters[i].name);
            //if unsuccessful, print
            }else if(!res->results[i].successful){
                success = false;
                RCLCPP_WARN(overseer->get_logger(), "Failed to set parameter: %s for %s: %s", req->parameters[i].name.c_str(), nodeName.c_str(), res->results[i].reason.c_str());
            }
        }

        //if all were successful, inform
        if(success){
            RCLCPP_INFO(overseer->get_logger(), "Successfully set parameters for %s", nodeName.c_str());
        }

        paramsLoaded = true;

    }


    //tell overseer to reload from config file
    bool SimulinkModelClass::reloadParams(){
        knownParams.clear();
        overseer->readConfig();

        return listAndSetModelParameters();
    }


    //callback for reload parameters callback
    void SimulinkModelClass::reloadParametersCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                                                                    std::shared_ptr<std_srvs::srv::Trigger::Response> res){
        //store the time
        rclcpp::Time current = overseer->get_clock()->now();

        //check if model is active
        if(!modelActive){
            res->success = false;
            res->message = nodeName + " is not active!";
            return;
        }

        //check if reload has occurred within the last 2 seconds
        if((current - lastReloadTime).nanoseconds() * 1e-9 < RELOAD_TIME){
            res->success = false;
            res->message = nodeName + " has been reloaded within the last 2 seconds";
            return;
        }
        
        //reload
        res->success = reloadParams();

        //construct message 
        if(res->success){
            res->message = nodeName + " parameter reload was successful";
            lastReloadTime = current;
        }else{
            res->message = nodeName + " parameter reload failed";
        }


    }