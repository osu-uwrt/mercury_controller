#pragma once

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <riptide_msgs2/msg/dshot_partial_telemetry.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <cmath>
#include <memory>
#include <string>
#include <fstream>
#include <sstream>
#include <getline>
#include <algorithm>
#include <filesystem>
#include <unordered_map>
#include <vector>
#include <exception>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/time.h"

#include "simulink_model.hpp"
#include "Eigen/Dense"


#define FF_PUBLISH_PARAM "disable_native_ff"
#define PARAMETERSCALE 1000000
#define AUTOFF_INIT_TOLERANCE .01


/*
    Helper function to make a yaml node into a vector, used for thrusters
*/
template<typename T>
T getYamlNodeAs(const YAML::Node& n, const std::vector<std::string>& keywords)
{
    if(keywords.empty())
    {
        throw std::runtime_error("getYamlNodeAs() requires at least one keyword.");
    }

    YAML::Node node = YAML::Clone(n);
    
    try
    {
        for(std::string s : keywords)
        {
            node = node[s];
        }

        return node.as<T>();
    } catch(YAML::Exception& e)
    {
        std::string msg = "Failed to parse value at tag " + keywords[0];
        for(size_t i = 1; i < keywords.size(); i++)
        {
            msg +=  " -> " + keywords[i];
        }
        
        msg += ": " + std::string(e.what());
        throw std::runtime_error(msg);
    }
}

/*
    Helper function to quickly go to Eigen 3d vector object
*/
Eigen::Vector3d std2v3d(std::vector<double> stdVect)
{
    return v3d(stdVect[0], stdVect[1], stdVect[2]);
}



class ControllerOverseer : public rclcpp::Node {

    using v3d = Eigen::Vector3d;
    using v4d = Eigen::Vector4d;
    using vXd = Eigen::VectorXd;
    using m3d = Eigen::Matrix3d;
    using mXd = Eigen::MatrixXd;
    using quat = Eigen::Quaterniond;  

    using ListParams = rcl_interfaces::srv::ListParameters;
    using SetParams = rcl_interfaces::srv::SetParameters;
    using SetParamsResult = rcl_interfaces::msg::SetParametersResult;
    using Parameter = rcl_interfaces::msg::Parameter;
    using ParameterType = rclcpp::ParameterType;
    using ParameterValue = rcl_interfaces::msg::ParameterValue;
    
    using fs = std::filesystem;

    using int_vector = std::vector<std::pair<std::string,double>>;
    using bool_vector = std::vector<std::pair<std::string,bool>>;
    using array_vector = std::vector<std::pair<std::string,std::vector<double>>>;

    using string = std::string;

    public:

    ControllerOverseer() : Node("controller_overseer") {

        //parameter declarations
        declare_parameter("robot", "");
        robotName = get_parameter("robot").as_string();

        declare_parameter("vehicle_config", "");
        configPath = get_parameter("vehicle_config").as_string();

        declare_parameter("thruster_solver_node_name", "");
        thrusterSolverName = get_parameter("thruster_solver_node_name").as_string();

        declare_parameter("write_ff_autotune", true);
        writeAutoFF = get_parameter("write_ff_autotune").as_bool();

        declare_parameter(FF_PUBLISH_PARAM, false);

        setConfigPath();
        readConfig();

        generateThrusterForceMatrix(thrusterInfo, com);

        for(int i = 0; i < 8; i++){
            activeThrusters[i] = true;
            submergedThrusters[i] = true;
            thrusterWeights[i] = 1;
        }


        using std::placeholders::_1,std::placeholders::_2;

        thrusterTelemetry = create_subscription<riptide_msgs2::msg::DshotPartialTelemetry>("/state/thrusters/telemetry", rclcpp::SystemDefaultsQoS(), std::bind(&ControllerOverseer::thrusterTelemetryCB, this, _1));        
        setThrusterSolverParams = create_client<rcl_interfaces::srv::SetParameters>(thrusterSolverName + "/set_parameters");

        motionEnabledPub = create_publisher<std_msgs::msg::Bool>("controller/motion_enabled", rclcpp::SystemDefaultsQoS());
        thrusterMode = create_subscription<std_msgs::msg::Int16>("thrusterSolver/thrusterState", rclcpp::SystemDefaultsQoS(), std::bind(&ControllerOverseer::setThrusterModeCB, this, _1));
        odom = create_subscription<nav_msgs::msg::Odometry>("odometry/filtered", rclcpp::SystemDefaultsQoS(), std::bind(&ControllerOverseer::odometryCB, this, _1));
        ffAutoTune = create_subscription<geometry_msgs::msg::Twist>("ff_auto_tune", rclcpp::SystemDefaultsQoS(), std::bind(&ControllerOverseer::ffAutoTuneCB, this, _1));
        weightsPub = create_publisher<std::msgs::Int32MultiArray>("controller/solver_weights", rclcpp::SystemDefaultsQoS());
        ffPub = create_publisher<geometry_msgs::msg::Twist>("controller/FF_body_force", rclcpp::SystemDefaultsQoS());
        reInitPub = create_publisher<std_msgs::msg::Empty>("controller/re_init_accumulators", rclcpp::SystemDefaultsQoS());

        setTeleop = create_service<std_srvs::srv::SetBool>("setTeleop", std::bind(&ControllerOverseer::setTeleop, this, _1, _2));

        tfBuffer = std::make_shared<tf2_ros::Buffer>(get_clock());
        tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

        tfNamespace = get_parameter("robot").as_string();

        using namespace std::chrono_literals;
        updateTimer = create_wall_timer(1s, std::bind(&ControllerOverseer::doUpdate, this));
        weightTimer = create_wall_timer(1s, std::bind(&ControllerOverseer::adjustThrusterWeights, this)); 

    }

    //construct the complete controller class using the pointer for "this" instance
    void bind_pointers(std::shared_ptr<ControllerOverseer> node){
        completeController = std::make_shared<SimulinkModelClass>(node, "complete_controller");
    }


    /*
    Yaml traversal -- get recursed
    */
    void traversal(int_vector& ints, bool_vector& bools, array_vector& arrays, const YAML::Node& tree, const std::string path){
        switch(tree.Type()){
            case YAML::NodeType::Map:
                {
                    for(YAML::const_iterator it = tree.begin(); it != tree.end(); ++it){
                        std::string child = it->first.as<std::string>();
                        std::string nextPath;
                        if(path.empty()){
                            nextPath = child;
                        }else{
                            nextPath = path + "__" + child;
                        }

                        traversal(ints, bools, arrays, it->second, nextPath);
                    }
                    break;
                }
            case YAML::NodeType::Sequence:{

                std::vector<double> array;

                for(const auto& item : tree){
                    if(item.IsScalar()){
                        try{
                             changed = item.as<double>()*PARAMETERSCALE;
                            array.push_back(changed);
                        }catch (const YAML::BadConversion& e){
                            RCLCPP_ERROR(get_logger(), "%s not read properly", path.c_str());
                        }

                    }
                }
                if(!path.empty()){
                    arrays.emplace_back(path, array);
                }
                break;
            }

            case YAML::NodeType::Scalar:{
                int num = 0;
                bool boolean;
                try{
                    num = tree.as<double>() * PARAMETERSCALE;
                } catch (const YAML::BadConversion& e){
                    try{
                    boolean = tree.as<bool>();
                    }catch(const YAML::BadConversion& e){
                        RCLCPP_ERROR(get_logger(), "%s: %s not read properly", path, tree.Tag());
                        break;
                    }
                }


                if(!path.empty()){
                    if(num){
                        ints.emplace_back(path, num);
                    }else{
                        bools.emplace_back(path, boolean);
                    }
                }
                break;
            }

            default:
            break;


        }

    }

    /*
    Read both autoff and regular config files and place information into the simulink class
    */
    void readConfig(){
        try{
            configTree = YAML::LoadFile(configPath);
            thrusterInfo = configTree["thrusters"];
            com = getYamlNodeAs<std::vector<double>>(configTree, {"com"});

            baseWrench = getYamlNodeAs<std::vector<double>>(configTree, {"controller", "feed_forward", "base_wrench"});
            traversal(completeController.intV, completeController.boolV, completeController.arrayV, configTree, "");

            auto thrusterSolverInfo = configTree["thruster_solver"];
            defaultWeight = getYamlNodeAs<double>(thrusterSolverInfo, {"default_weight"});
            surfaceWeight = getYamlNodeAs<double>(thrusterSolverInfo, {"surfaced_weight"});
            disabledWeight = getYamlNodeAs<double>(thrusterSolverInfo, {"disable_weight"});
            lowDowndraftWeight = getYamlNodeAs<double>(thrusterSolverInfo, {"low_downdraft_weight"});

        }catch(YAML::BadFile &e){
            RCLCPP_ERROR(get_logger(), "Cannot open config file at %s", configPath.c_str());
        }

        try{
            autoffTree = YAML::LoadFile(autoffConfigPath);
            autoffTree = autoffTree["auto_ff"];
            traversal(completeController->intV, completeController->boolV, completeController->arrayV, autoffTree, "controller__autoff__initial_ff");
        }catch(YAML::BadFile &e){
            RCLCPP_ERROR(get_logger(), "Cannot open config file at %s", autoffconfigPath.c_str());
        }



    }

    void generateThrusterForceMatrix(const YAML::Node& thrusterInfo, const YAML::Node& com){
        std::vector<double> thrusterFT;
        std::vector<double> comXYZ = com.as<std::vector<double>>();

        int i = 0;
        for(const auto& thruster : thrusterInfo){
            std::vector<double> thrusterPose = getYamlNodeAs<std::vector<double>>(thruster, {"pose"});
            m3d R = 
                    Eigen::AngleAxisd(thrusterPose[5],   Eigen::Vector3d::UnitZ()).toRotationMatrix() *
                    Eigen::AngleAxisd(thrusterPose[4], Eigen::Vector3d::UnitY()).toRotationMatrix() *
                    Eigen::AngleAxisd(thrusterPose[3],  Eigen::Vector3d::UnitX()).toRotationMatrix();

            v3d forceVector = R * Eigen::Vector3d::UnitX(); 

            std::vector<double> positionFromCom;
            for(int j = 0; j<3; j++){
                positionFromCom.push_back(thrusterPose[j] - comXYZ[j]);
            }
                
            v3d momentArm(positionFromCom[0], positionFromCom[1], positionFromCom[2]);
            v3d torque = momentArm.cross(forceVector);
                
            thrusterFT.push_back(forceVector(0));
            thrusterFT.push_back(forceVector(1));
            thrusterFT.push_back(forceVector(2));
            thrusterFT.push_back(torque(0));
            thrusterFT.push_back(torque(1));
            thrusterFT.push_back(torque(2));
        }

        completeController->arrayV.emplace_back("talos_wrenchmat", thrusterFT);
    }

    void setConfigPath(){
        configPath = get_parameter("vehicle_config").as_string();
        if(configPath == ""){
            string descriptionsShareDir = get_package_share_directory("riptide_descriptions2");
            string robotConfigSubpath = "config/" + robotName + ".yaml";
            
            configPath = descriptionsShareDir + "/" + robotConfigSubpath;
            
            std::vector<string> dirSplit;
            std::stringstream ss(configPath);
            string subPath;

            while (std::getline(ss, subPath, '/')) {
                parts.push_back(subPath);
            }
            
            bool flag = false;
            for (const string& i : dirSplit) {
                if(i == "install"){
                    flag = true;
                    break;
                }        
            }
            if(flag){
                string colconRoot = "";
                int i = 0;
                while(dirSplit[i] != "install"){
                    colconRoot += "/" + dirSplit[i];
                    i++;
                }
                colconRoot += "/src";
                
                std::vector<string> possiblePaths;
                possiblePaths.push_back(colconRoot + "/riptide_core/riptide_descriptions/" + robotConfigSubpath);
                possiblePaths.push_back(colconRoot + "/riptide_descriptions/" + robotConfigSubpath);

                for(const string& path : possiblePaths){
                    if(fs::exists(path)){
                        configPath = path;
                        RCLCPP_INFO(get_logger(), "Discovered source directory, overriding descriptions to use %s", configPath.c_str());
                    }
                }
            }
        }

        string controlShareDir = get_package_share_directory("riptide_controllers2");
        string autoFFSubpath = "config/" + robotName + "_autoff.yaml";

        if(fs::exists("/home/ros/colcon_deploy")){
            RCLCPP_INFO(get_logger(), "I think I am not running on the orin!");
            autoffConfigPath = controlShareDir + autoFFSubpath;
        }else{
            RCLCPP_INFO(get_logger(), "I think I am running on the orin!");
            autoffConfigPath = "/bin" + robotName + "_autoff.yaml";     //this will need to change it is no longer /bin
        }
    }

    void thrusterTelemetryCB(riptide_msgs2::msg::DshotPartialTelemetry::SharedPtr msg){
        bool adjustWeights = false;

        escPowerCheckTimer.reset();

        if(msg.start_thruster_num == 0){
            int i = 0;
            for(auto esc : msg.esc_telemetry){
                if(!esc.thruster_ready && activeThrusters[i] == true){
                    activeThrusters[i] = false;
                    adjustWeights = true;

                }else if(esc.thruster_ready && activeThrusters[i] == false){
                    activeThrusters[i] = true;
                    adjustWeights = true;
                }
                i++;
            }
            if(msg.disabled_flags != 0){
                escPowerStopsLow++;
            }else{
                escPowerStopsLow = 0;
            }
        }else{
            int i = 4;
            for(auto esc : msg.esc_telemetry){
                if(!esc.thruster_ready && activeThrusters[i] == true){
                    activeThrusters[i] = false;
                    adjustWeights = true;

                }else if(esc.thruster_ready && activeThrusters[i] == false){
                    activeThrusters[i] = true;
                    adjustWeights = true;
                }
                i++;
            }
            if(msg.disabled_flags != 0){
                escPowerStopsHigh++;
            }else{
                escPowerStopsHigh = 0;
            }
        }

        if(adjustWeights){
            adjustThrusterWeights();
        }
        
        std_msgs::msg::Bool motionMsg;

        if(escPowerStopsLow > ESC_POWER_STOP_TOLERANCE || escPowerStopsHigh > ESC_POWER_STOP_TOLERANCE){
            motionMsg.data = false;
            enabled =false;
        }else{
            motionMsg.data = true;
            enabled = true;           
        }

        motionEnabledPub->publish(motionMsg);

    }

    void escPowerTimeout(){
        if(!enabled){
            RCLCPP_WARN(get_logger(), "Not recieving thruster telemetry!");
        }

        std_msgs::msg::Bool motionMsg;
        motionMsg.data = false;
        enabled = false;
        motionEnabledPub->publish(motionMsg);
    }

    void setThrusterModeCB(std_msgs::msg::Int16::SharedPtr msg){
        if(msg.data != thrusterMode){
            thrusterMode = msg.data;
            adjustThrusterWeights();
        }

    }
    

    void odometryCB(nav_msgs::msg::Odometry::SharedPtr msg){
        if(!startTime){
            startTime = get_clock()->now();
        }
        
        bool submerged[8] = [false,false,false,false,false,false,false,false];
        
        double killPlane = getYamlNodeAs<double>(configTree, {"controller_overseer", "thruster_kill_plane"});

        geometry_msgs::msg::TransformStamped pos;
        try{
            for(int i = 0; i < 8; i++){

                pos = tfBuffer->lookup_transform("world", tfNamespace + "thruster_" + std::to_string(i), tf2::TimePointZero);

                if(pos.transform.translation.z < killPlane){
                    submerged[i] = true;
                }
            }
            if(submerged != submergedThrusters){
                submergedThrusters = submerged;
                adjustThrusterWeights();
            }
        }
        catch(const std::exception& ex){
            if(get_clock().now().to_msg().sec >= 1.0 + startTime.to_msg().sec){
                RCLCPP_ERROR(get_logger(), "Thruster position lookup failed with exception %s", ex.what());
            }
        }
    }

    void adjustThrusterWeights(){
        int activeThrusterCount = 0;
        int submergedThrustersCount = 0;

        for(int i = 0; i<activeThrusters.size(); i++){
            if(activeThrusters[i]){
                activeThrusterCount++;

                if(!submergedThrusters[i]){
                    thrusterWeights[i] = surfaceWeight;
                }else{
                    submergedThrustersCount++;
                    thrusterWeights[i] = defaultWeight;
                }
            }else{
                thrusterWeights[i] = disabledWeight;
            }
        }

        if(activeThrusterCount <= 6){
            if(enabled){
                RCLCPP_ERROR(get_logger(), "System is underactuated. Only: " + std::to_string(activeThrusterCount) + " thrusters are active. Killing thrusters!");
                enabled = false;
            }else{
                enabled = true;
            }
        }

        if(submergedThrusters >= 8 && thrusterMode == 2){
            thrusterWeights[4] = lowDowndraftWeight;
            thrusterWeights[5] = lowDowndraftWeight;
        } 

        if(thrusterMode == 0){
            for(int i = 0; i < thrusterWeights.size(); i++){
                thrusterWeights[i] = 0;
            }
        }

        geometry_msgs::msg::Int32MultiArray msg;
        
        std::vector<int> weights;
        for(const double& weight : thrusterWeights){
            weights.push_back(weight);
        }

        msg.data = weights;

        weightsPub->publish(msg);
    }

    void setTeleop(std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr res){
        try{
            ParameterValue pVal;
            pVal.type = ParameterType::PARAMETER_INTEGER_ARRAY;

            if(req->data){
                pVal.integer_array_value = [3000000, 3000000, 2000000, 1000000, 1000000, 3000000];
                res->message = "Successfully enabled teleop!";
            }else{
                pVal.integer_array_value = getYamlNodeAs<std::vector<int>>(configTree, {"controller", "active_force_control"});
                res->message = "Successfully enabled active control!";
            }

            Parameter param;
            param.value = pVal;
            param.name = "controller__active_force_mask";

            SetParams::Request::SharedPtr setParamsRequest;
            setParamsRequest->parameters = [param];

            completeController->setParamClient.scheduleCall(setParamsRequest, 
                                                                [this, req](SetParams::Response::SharedPtr res){
                                                                completeController->setParametersDoneCallback(res);
                                                            });

            res->success = true;
                
        }catch(std::exception &e){
            RCLCPP_WARN(get_logger(), "Failed to initialize active control, model may not be started");
            res->success = false;
            res->message = "Failed to initialize active control, model may not be started";
        }
    }

    void doUpdate(){
    std::vector<std::string> activeROSNodeNames;

    std::vector<std::pair<string, string>> activeROSNodes = get_node_graph_interface()->get_node_names_and_namespaces();

    for (const auto& node : active_rosnodes) {
        std::string node_name = node.second + "/" + node.first;

        // Remove double leading "//"
        if (node_name.rfind("//", 0) == 0) { 
            node_name = node_name.substr(1);
        }

        active_rosnode_names.push_back(node_name);
        }

        completeController->checkIfActive(activeROSNodeNames);

        if(!get_parameter(FF_PUBLISH_PARAM).as_bool()){
            publishingFF = true;

            geometry_msgs::msg::Twist msg;
            msg.linear.x = baseWrench[0];
            msg.linear.y = baseWrench[1];
            msg.linear.z = baseWrench[2];
            msg.angular.x = baseWrench[3];
            msg.angular.y = baseWrench[4];
            msg.angular.z = baseWrench[5];

            ffPub->publish(msg);
        }else if(publishingFF){

            publishingFF = false;
            geometry_msgs::msg::Twist msg;
            msg.linear.x = 0.0;
            msg.linear.y = 0.0;
            msg.linear.z = 0.0;
            msg.angular.x = 0.0;
            msg.angular.y = 0.0;
            msg.angular.z = 0.0;

            ffPub->publish(msg);
        }
    }

    //ASK ABOUT THIS LOGIC JOHN!!!!
    void ffAutoTuneCB(geometry_msgs::msg::Twist msg){
        if(!writeAutoFF || !completeController->paramsLoaded || autoffConfigPath == ""){
            return;
        }

        std::vector<double> initFF;
        if(waitingOnInit){
            bool initFound = false;
            for(const std::pair<string, std::vector<double>>& pair : array_vector completeController->arrayV){
                if(pair.first == "controller__autoff__initial_ff"){
                    initFF = pair.second;
                    initFound = true;
                    break;
                }
            }

            
            if(std::abs(initFF[0] - msg.linear.x) < AUTOFF_INIT_TOLERANCE && std::abs(initFF[1] - msg.linear.y) < AUTOFF_INIT_TOLERANCE && std::abs(initFF[2] - msg.linear.z) < AUTOFF_INIT_TOLERANCE 
                && std::abs(initFF[3] - msg.angular.x) < AUTOFF_INIT_TOLERANCE && std::abs(initFF[4] - msg.angular.y) < AUTOFF_INIT_TOLERANCE && std::abs(initFF[1] - msg.angular.z) < AUTOFF_INIT_TOLERANCE){
                    
                    waitingOnInit = false;
            } else{

                std_msgs::msg::Empty emptyMsg;
                reInitPub->publish(emptyMsg);
            }
            
            return;
        }

        if(initFF[0] != msg.linear.x || initFF[1] != msg.linear.y || initFF[2] != msg.linear.z || initFF[3] != msg.angular.x || initFF[4] != msg.angular.y || initFF[5] != msg.angular.z){
            string autoFFstr = "autoff: [";
            for(int i = 0; i<5 i++){
                autoFFstr += std::to_string(initFF[i]) + ",";
            }
            autoFFstr += std::to_string(initFF[5]) + "]";
            
            std::ofstream ffconfig(autoffConfigPath);
            if(ffconfig.is_open()){
                ffconfig << autoFFstr;
            }else{
                RCLCPP_ERROR(get_logger(), "Cannot open ff auto tune file at: %s", autoffconfigPath.c_str());
            }
            ffconfig.close();
        }

    }



    private:

    bool waitingOnInit;

    int escPowerStopsLow, escPowerStopsHigh;

    YAML::Node configTree;
    YAML::Node autoffTree;

    YAML::Node thrusterInfo;
    YAML::Node com;

    bool activeThrusters[8];
    bool submergedThrusters[8];
    double thrusterWeights[8];

    int thrusterMode;

    string autoffConfigPath = "";

    std::shared_ptr<SimulinkModelClass> completeController;

    rclcpp::Subscriber<riptide_msgs2::msg::DshotPartialTelemetry>::SharedPtr thrusterTelemetry;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motionEnabledPub;
    rclcpp::Service<rcl_interfaces::srv::SetParameters>::SharedPtr setThrusterSolverParams;

    rclcpp::Subscriber<std_msgs::msg::Int16>::SharedPtr thrusterMode;
    rclcpp::Subscriber<nav_msgs::msg::Odometry>::SharedPtr odom;
    rclcpp::Subscriber<geometry_msgs::msg::Twist>::SharedPtr ffAutoTune;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr weightsPub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr ffPub;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr reInitPub;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr setTeleop;

    tf2_ros::Buffer::SharedPtr tfBuffer;
    tf2_ros::TransformListener::SharedPtr tfListener;

    string tfNamespace;

    rclcpp::Time startTime;
    rclcpp::TimerBase::SharedPtr updateTimer;
    rclcpp::TimerBase::SharedPtr weightTimer;
    rclcpp::TimerBase::SharedPtr escPowerCheckTimer;

    bool enabled;
    bool publishingFF;

    std::vector<double> baseWrench;

    double defaultWeight, surfaceWeight, disabledWeight, lowDowndraftWeight;

    //parameters
    string robotName, configPath, thrusterSolverName;
    bool writeAutoFF;

};