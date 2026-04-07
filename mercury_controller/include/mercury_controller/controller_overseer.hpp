#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>

#include <yaml-cpp/yaml.h>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/empty.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <std_srvs/srv/set_bool.hpp>

#include <mercury_msgs/msg/dshot_partial_telemetry.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <cmath>
#include <memory>
#include <string>
#include <fstream>
#include <sstream>
#include <string>
#include <algorithm>
#include <filesystem>
#include <unordered_map>
#include <vector>
#include <exception>
#include <chrono>

#include <Eigen/Dense>

#include "simulink_model.hpp"
#include "overseer_util.hpp"


#define PARAMETERSCALE 1000000
#define AUTOFF_INIT_TOLERANCE .01
#define ESC_POWER_STOP_TOLERANCE 2


namespace fs = std::filesystem;

class ControllerOverseer : public rclcpp::Node {

    using v3d = Eigen::Vector3d;
    using m3d = Eigen::Matrix3d;

    using SetParams = rcl_interfaces::srv::SetParameters;
    using SetParamsResult = rcl_interfaces::msg::SetParametersResult;
    using Parameter = rcl_interfaces::msg::Parameter;
    using ParameterType = rclcpp::ParameterType;
    using ParameterValue = rcl_interfaces::msg::ParameterValue;
    
    using int_vector = std::vector<std::pair<std::string,int64_t>>;
    using bool_vector = std::vector<std::pair<std::string,bool>>;
    using array_vector = std::vector<std::pair<std::string,std::vector<int64_t>>>;

    using string = std::string;

    public:

    //constructor
    ControllerOverseer();

    //construct the complete controller class using the pointer for "this" instance
    void init(std::shared_ptr<ControllerOverseer> node);


    /*
    Yaml traversal -- get recursed
    */
    void traversal(int_vector& ints, bool_vector& bools, array_vector& arrays, const YAML::Node& tree, const std::string path);

    /*
    Read both autoff and regular config files and place information into the simulink class
    */
    void readConfig();

    /*
    From thruster and com portions of yaml file, set wrench matrix based on thruster positions.
    */
    void generateThrusterForceMatrix();

    /*
    Set yaml file config paths, right now this is just talos.yaml and talos_autoff.yaml; this will change
     */
    void setConfigPath();

    /*
    Callback from thruster telemetry subscriber, checks that thrusters are still working
    */
    void thrusterTelemetryCB(mercury_msgs::msg::DshotPartialTelemetry::SharedPtr msg);

    /*
    Timer callback that checks if esc boards are publishing
    This is ONLY called if the thruster telemetry subscriber hasn't recieved a message in 2 seconds
    */
    void escPowerTimeout();

    /*
    Callback that adjusts thruster weights if thruster mode changes (low downdraft or normal)
    */
    void setThrusterModeCB(std_msgs::msg::Int16::SharedPtr msg);
    
    /*
    Callback to odometry subscriber, adjusts thruster weights if any thrusters are no longer submerged
    */
    void odometryCB(nav_msgs::msg::Odometry::SharedPtr msg);

    /*
    Checks that enough thrusters are working and adjusts weights based on if they are surfaced or if in low downdraft mode
    */
    void adjustThrusterWeights();

    /*
    Set teleop service callback that sets controller mask based off yaml file and if teleop mode is on.
    */
    void setTeleopCB(std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr res);

    /*
    1 second timer callback that lists and sets model parameters if model is active and publishes feed forward message
    */
    void doUpdate();

    /*
    Rewrite to auto tune yaml file if it has changed    
    */
    void ffAutoTuneCB(geometry_msgs::msg::Twist::SharedPtr msg);


    //true if waiting on autoff initialization
    bool waitingOnInit;

    //thruster status information
    int escPowerStopsLow, escPowerStopsHigh;
    std::array<bool, 8> activeThrusters;
    std::array<bool, 8> submergedThrusters;
    std::array<double, 8> thrusterWeights;

    //yaml trees
    YAML::Node controllerTree;
    YAML::Node autoffTree;
    YAML::Node thrusterInfo;

    //position of center of mass
    std::vector<double> com;

    //thruster mode, 1: normal, 2: low downdraft
    int thrusterMode = 1;

    //path to config yaml files
    string autoffConfigPath = "";
    string configPath;

    //pointer to simulink model class which calls set and list params services
    std::shared_ptr<SimulinkModelClass> completeController;

    //thruster info publishers and subscribers
    rclcpp::Subscription<mercury_msgs::msg::DshotPartialTelemetry>::SharedPtr thrusterTelemetry;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motionEnabledPub;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr thrusterModeSub;
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr setThrusterSolverParams;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr weightsPub;

    //feed forward publishersand subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr ffAutoTune;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr ffPub;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr reInitPub;

    //sets active control mode
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr setTeleop;

    //tf2 buffer and transform listener for thruster matrix generation
    std::unique_ptr<tf2_ros::Buffer> tfBuffer;          //unique ptr because that's how ROS documentation has it
    std::shared_ptr<tf2_ros::TransformListener> tfListener;
    string tfNamespace;

    //time that odometry msgs began flooding in
    rclcpp::Time startTime;
    bool startTimeSet = false;

    //timers for doUpdate, adjustThrusterWeights, and escPowerTimeouts
    rclcpp::TimerBase::SharedPtr updateTimer;
    rclcpp::TimerBase::SharedPtr weightTimer;
    rclcpp::TimerBase::SharedPtr escPowerCheckTimer;

    //bool for if still fully acutated, and if feed forward is being pubilshed
    bool enabled;
    bool publishingFF;

    //base feed forward wrench and current initial feed forward
    std::vector<double> baseWrench;
    std::vector<double> currentInitFF;

   //different weight values for thrusters 
    double defaultWeight, surfaceWeight, disabledWeight, lowDowndraftWeight;

    //parameters
    string robotName, thrusterSolverName;
    bool writeAutoFF;

};