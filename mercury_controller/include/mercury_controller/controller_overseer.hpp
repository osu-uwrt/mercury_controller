
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

#include <riptide_msgs2/msg/dshot_partial_telemetry.hpp>
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


#define FF_PUBLISH_PARAM "disable_native_ff"
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

    void generateThrusterForceMatrix(const YAML::Node& thrusterInfo, const YAML::Node& com);

    void setConfigPath();

    void thrusterTelemetryCB(riptide_msgs2::msg::DshotPartialTelemetry::SharedPtr msg);

    void escPowerTimeout();

    void setThrusterModeCB(std_msgs::msg::Int16::SharedPtr msg);
    

    void odometryCB(nav_msgs::msg::Odometry::SharedPtr msg);

    void adjustThrusterWeights();

    void setTeleopCB(std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr res);

    void doUpdate();

    //ASK ABOUT THIS LOGIC JOHN!!!!
    void ffAutoTuneCB(geometry_msgs::msg::Twist::SharedPtr msg);


    bool waitingOnInit;

    int escPowerStopsLow, escPowerStopsHigh;

    YAML::Node configTree;
    YAML::Node autoffTree;

    YAML::Node thrusterInfo;
    YAML::Node com;

    std::array<bool, 8> activeThrusters;
    std::array<bool, 8> submergedThrusters;
    std::array<double, 8> thrusterWeights;


    int thrusterMode = 0;

    string autoffConfigPath = "";

    std::shared_ptr<SimulinkModelClass> completeController;

    rclcpp::Subscription<riptide_msgs2::msg::DshotPartialTelemetry>::SharedPtr thrusterTelemetry;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motionEnabledPub;
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr setThrusterSolverParams;

    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr thrusterModeSub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr ffAutoTune;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr weightsPub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr ffPub;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr reInitPub;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr setTeleop;

    std::unique_ptr<tf2_ros::Buffer> tfBuffer;          //unique ptr because that's how ROS documentation has it
    std::shared_ptr<tf2_ros::TransformListener> tfListener;

    string tfNamespace;

    rclcpp::Time startTime;
    bool startTimeSet = false;

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