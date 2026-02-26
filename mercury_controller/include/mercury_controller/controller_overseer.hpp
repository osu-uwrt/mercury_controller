#pragma once

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <riptide_msgs2/msg/dshot_partial_telemetry.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <filesystem>

#include "simulink_model.hpp"
#include "Eigen/Dense"


#define FF_PUBLISH_PARAM "disable_native_ff"

#define PARAMETER_SCALE 1000000


class ControllerOverseer : public rclcpp::Node {

    using string = std::string;

    bool waitingOnInit;

    int escPowerStopsLow, escPowerStopsHigh;

    YAML::Node configTree;
    YAML::Node autoffTree;

    YAML::Node thrusterInfo;
    YAML::Node com;

    bool activeThrusters[8];
    bool submergedThrusters[8];
    int thrusterWeights[8];

    int thrusterMode;

    string autoffConfigPath;

    std::shared_ptr<SimulinkModelClass> completeController;

    rclcpp::Subscriber</*DShotPartialTelemetry*/>::SharedPtr thrusterTelemetry;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motionEnabledPub;

    rclcpp::Service<rcl_interfaces::srv::SetParameters>::SharedPtr setThrusterSolverParams;

    rclcpp::Subscriber<std_msgs::msg::Int16>::SharedPtr thrusterMode;

    rclcpp::Subscriber<nav_msgs::msg::Odometry>::SharedPtr odom;

    rclcpp::Subscriber<geometry_msgs::msg::Twist>::SharedPtr ffAutoTune;
    
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr weightsPub;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr ffPub;

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr reInitPub;

    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr setTeleop;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr setTeleop;

    tf2_ros::Buffer::SharedPtr tfBuffer;

    tf2_ros::TransformListener::SharedPtr tfListener;

    string tfNamespace;

    rclcpp::Time startTime;

    rclcpp::TimerBase::SharedPtr updateTimer;

    rclcpp::TimerBase::SharedPtr weightTimer;

    bool enabled;
    bool publishingFF;

    rclcpp::TimerBase::SharedPtr escPowerCheckTimer;

    //parameters
    string robotName, configPath, thrusterSolverName;
    bool writeAutoFF;

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

            
        thrusterTelemetry = create_subscription</*DShotPartialTelemetry*/>("/state/thrusters/telemetry", rclcpp::SystemDefaultsQoS(), /*bind callback*/);

        motionEnabledPub = create_publisher<std_msgs::msg::Bool>("controller/motion_enabled", rclcpp::SystemDefaultsQoS());

        setThrusterSolverParams = create_client<rcl_interfaces::srv::SetParameters>(thrusterSolverName + "/set_parameters");

        thrusterMode = create_subscription<std_msgs::msg::Int16>("thrusterSolver/thrusterState", rclcpp::SystemDefaultsQoS(), /*bind callback*/);

        odom = create_subscription<nav_msgs::msg::Odometry>("odometry/filtered", rclcpp::SystemDefaultsQoS(), /*callback*/);

        ffAutoTune = create_subscription<geometry_msgs::msg::Twist>("ff_auto_tune", rclcpp::SystemDefaultsQoS(), /*callback*/);
        
        weightsPub = create_publisher<std::msgs::Int32MultiArray>("controller/solver_weights", rclcpp::SystemDefaultsQoS());

        ffPub = create_publisher<geometry_msgs::msg::Twist>("controller/FF_body_force", rclcpp::SystemDefaultsQoS());

        reInitPub = create_publisher<std_msgs::msg::Empty>("controller/re_init_accumulators", rclcpp::SystemDefaultsQoS());

        setTeleop = create_client<std_srvs::srv::SetBool>("setTeleop", /*callback*/);

        tfBuffer = std::make_shared<tf2_ros::Buffer>(get_clock());

        tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

        tfNamespace = "talos";      //get_parameter("robot").as_string()??

        updateTimer = create_wall_timer(std::chrono_literals::1s, /*callback*/);

        weightTimer = create_wall_timer(std::chrono_literals::1s, /*callback*/);
        

    }

    public:

    //construct the complete controller class using the pointer for "this" instance
    void bind_pointers(std::shared_ptr<ControllerOverseer> node){
        completeController = std::make_shared<SimulinkModelClass>(node, "complete_controller");
    }


    /*
    Yaml traversal -- get recursed
    */
    void traversal(std::vector<std::pair<std::string,int64_t>>& ints, std::vector<std::pair<std::string,bool>>& bools, std::vector<std::pair<std::string,std::vector<int64_t>>> & arrays, const YAML::Node& tree, const std::string path){
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

                std::vector<int64_t> array;

                for(const auto& item : tree){
                    if(item.IsScalar()){
                        try{
                            int64_t changed = item.as<double>()*PARAMETERSCALE;
                            array.push_back(changed);
                        }catch (const YAML::BadConversion& e){
                            RCLCPP_ERROR(get_logger(), "%s: %s not read properly", path, item.Tag());
                        }

                    }
                }
                if(!path.empty()){
                    arrays.emplace_back(path, array);
                }
                break;
            }

            case YAML::NodeType::Scalar:{
                int64_t num = 0;
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
            traversal(completeController.intV, completeController.boolV, completeController.arrayV, configTree, "");
        }catch(YAML::BadFile &e){
            RCLCPP_ERROR(get_logger(), "Cannot open config file at %s", configPath.c_str());
        }

        try{
            autoffTree = YAML::LoadFile(autoffConfigPath);
            autoffTree = autoffTree["auto_ff"];
            traversal(completeController.intV, completeController.boolV, completeController.arrayV, autoffTree, "controller__autoff__initial_ff");
        }catch(YAML::BadFile &e){
            RCLCPP_ERROR(get_logger(), "Cannot open config file at %s", autoffconfigPath.c_str());
        }

    }

    

    


};