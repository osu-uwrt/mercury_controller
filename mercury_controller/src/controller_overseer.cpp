#include "controller_overseer.hpp"

ControllerOverseer::ControllerOverseer() : Node("controller_overseer") {
    // parameter declarations
    declare_parameter("robot", "");
    robotName = get_parameter("robot").as_string();

    declare_parameter("vehicle_config", "");
    configPath = get_parameter("vehicle_config").as_string();

    declare_parameter("thruster_solver_node_name", "");
    thrusterSolverName = get_parameter("thruster_solver_node_name").as_string();

    declare_parameter("write_ff_autotune", true);
    writeAutoFF = get_parameter("write_ff_autotune").as_bool();

    declare_parameter("disable_native_ff", false);

    // set thruster info
    for (int i = 0; i < 8; i++) {
        activeThrusters[i] = true;
        submergedThrusters[i] = true;
        thrusterWeights[i] = 1;
    }

    using std::placeholders::_1, std::placeholders::_2;

    // Pulishers, Subscribers, Services
    // thruster info
    thrusterTelemetry = create_subscription<mercury_msgs::msg::DshotPartialTelemetry>(
        "state/thrusters/telemetry", rclcpp::SystemDefaultsQoS(),
        std::bind(&ControllerOverseer::thrusterTelemetryCB, this, _1));
    setThrusterSolverParams =
        create_client<rcl_interfaces::srv::SetParameters>(thrusterSolverName + "/set_parameters");
    thrusterModeSub = create_subscription<std_msgs::msg::Int16>(
        "thrusterSolver/thrusterState", rclcpp::SystemDefaultsQoS(),
        std::bind(&ControllerOverseer::setThrusterModeCB, this, _1));
    odom = create_subscription<nav_msgs::msg::Odometry>(
        "odometry/filtered", rclcpp::SystemDefaultsQoS(),
        std::bind(&ControllerOverseer::odometryCB, this, _1));
    motionEnabledPub = create_publisher<std_msgs::msg::Bool>(
        "controller/motion_enabled", rclcpp::SystemDefaultsQoS());
    weightsPub = create_publisher<std_msgs::msg::Int32MultiArray>(
        "controller/solver_weights", rclcpp::SystemDefaultsQoS());

    // feed forward
    ffAutoTune = create_subscription<geometry_msgs::msg::Twist>(
        "ff_auto_tune", rclcpp::SystemDefaultsQoS(),
        std::bind(&ControllerOverseer::ffAutoTuneCB, this, _1));
    ffPub = create_publisher<geometry_msgs::msg::Twist>(
        "controller/FF_body_force", rclcpp::SystemDefaultsQoS());
    reInitPub = create_publisher<std_msgs::msg::Empty>(
        "controller/re_init_accumulators", rclcpp::SystemDefaultsQoS());

    // set control mode
    setTeleop = create_service<std_srvs::srv::SetBool>(
        "setTeleop", std::bind(&ControllerOverseer::setTeleopCB, this, _1, _2));

    // tf2 info for thruster matrix
    tfBuffer = std::make_unique<tf2_ros::Buffer>(get_clock());
    tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
    tfNamespace = get_parameter("robot").as_string();

    using namespace std::chrono_literals;
    // timers to check thruster status
    updateTimer = create_wall_timer(1s, std::bind(&ControllerOverseer::doUpdate, this));
    weightTimer =
        create_wall_timer(1s, std::bind(&ControllerOverseer::adjustThrusterWeights, this));
    escPowerCheckTimer =
        create_wall_timer(2s, std::bind(&ControllerOverseer::escPowerTimeout, this));
}

// construct the complete controller class using the pointer for "this" instance, then perform
// initializtion functions
void ControllerOverseer::init(std::shared_ptr<ControllerOverseer> node) {
    completeController = std::make_shared<SimulinkModelClass>(node, "complete_controller");

    setConfigPath();
    readConfig();
    generateThrusterForceMatrix();
}

/*
 * Yaml traversal, places all info into necessary
 */
void ControllerOverseer::traversal(
    int_vector & ints, bool_vector & bools, array_vector & arrays, const YAML::Node & tree,
    const std::string path) {
    switch (tree.Type()) {
        case YAML::NodeType::Map: {
            for (YAML::const_iterator it = tree.begin(); it != tree.end(); ++it) {
                std::string child = it->first.as<std::string>();
                std::string nextPath;
                if (path.empty()) {
                    nextPath = child;
                } else {
                    nextPath = path + "__" + child;
                }

                traversal(ints, bools, arrays, it->second, nextPath);
            }
            break;
        }
        case YAML::NodeType::Sequence: {
            std::vector<int64_t> array;

            for (const auto & item : tree) {
                if (item.IsScalar()) {
                    try {
                        int64_t changed = item.as<double>() * PARAMETERSCALE;
                        array.push_back(changed);
                    } catch (const YAML::BadConversion & e) {
                        RCLCPP_ERROR(get_logger(), "%s not read properly", path.c_str());
                    }
                }
            }
            if (!path.empty()) {
                arrays.emplace_back(path, array);
            }
            break;
        }

        case YAML::NodeType::Scalar: {
            int64_t num = 0;
            bool boolean;
            try {
                num = tree.as<double>() * PARAMETERSCALE;
            } catch (const YAML::BadConversion & e) {
                try {
                    boolean = tree.as<bool>();
                } catch (const YAML::BadConversion & e) {
                    RCLCPP_ERROR(
                        get_logger(), "%s: %s not read properly", path.c_str(), tree.Tag().c_str());
                    break;
                }
            }

            if (!path.empty()) {
                if (num) {
                    ints.emplace_back(path, num);
                } else {
                    bools.emplace_back(path, boolean);
                }
            }
            break;
        }

        default:
            break;
    }
}

void ControllerOverseer::setConfigPath() {
    configPath = get_parameter("vehicle_config").as_string();

    // set configPath
    if (configPath == "") {
        string descriptionsShareDir =
            ament_index_cpp::get_package_share_directory("mercury_descriptions");
        string robotConfigSubpath =
            robotName + "/config/" + robotName; // the rest of the path is hard-coded in readConfig

        configPath = descriptionsShareDir + "/" + robotConfigSubpath;

        RCLCPP_INFO(get_logger(), "Config Path: %s", configPath.c_str());
        // split the path into all subpaths
        std::vector<string> dirSplit;
        std::stringstream ss(configPath);
        string subPath;

        while (std::getline(ss, subPath, '/')) {
            dirSplit.push_back(subPath);
        }

        // if in the install directory, change configPath to point to src
        bool flag = false;
        for (const string & i : dirSplit) {
            if (i == "install") {
                flag = true;
                break;
            }
        }
        if (flag) {
            string colconRoot = "";
            int i = 0;
            // take out install from path and add source
            while (dirSplit[i] != "install") {
                colconRoot += "/" + dirSplit[i];
                i++;
            }
            colconRoot += "/src";

            std::vector<string> possiblePaths;
            possiblePaths.push_back(
                colconRoot + "/mercury_common/mercury_descriptions/" + robotConfigSubpath);
            possiblePaths.push_back(colconRoot + "/mercury_descriptions/" + robotConfigSubpath);

            // set config path to existing src directory
            for (const string & path : possiblePaths) {
                if (fs::exists(path)) {
                    configPath = path;
                    RCLCPP_INFO(get_logger(), "NEW Config Path: %s", configPath.c_str());

                    RCLCPP_INFO(
                        get_logger(),
                        "Discovered source directory, overriding descriptions to use %s",
                        configPath.c_str());
                }
            }
        }
    }

    // find autoff config
    string controlShareDir = ament_index_cpp::get_package_share_directory("mercury_controller");
    string autoFFSubpath = "/config/" + robotName + "_autoff.yaml";

    // check if running on nano or not, set autoffConfig accordingly
    if (!fs::exists("/home/ros/colcon_deploy")) {
        RCLCPP_INFO(get_logger(), "I think I am NOT running on the nano!");
        autoffConfigPath = controlShareDir + autoFFSubpath;
    } else {
        RCLCPP_INFO(get_logger(), "I think I am running on the nano!");
        autoffConfigPath =
            "/bin/" + robotName + "_autoff.yaml"; // this will need to change it is no longer /bin
    }
}

/*
 * Read both autoff and regular config files and place information into the simulink class
 */
void ControllerOverseer::readConfig() {
    try {
        // load controller tree
        controllerTree = YAML::LoadFile(configPath + "_controller.yaml");

        // load thruster info, and com
        YAML::Node xacroTree = YAML::LoadFile(configPath + "_xacro_frames.yaml");
        thrusterInfo = xacroTree["thrusters"];
        YAML::Node configTree = YAML::LoadFile(configPath + ".yaml");
        com = configTree["com"].as<std::vector<double>>();

        // set base wrench and add all parameters from yaml file into completeController
        baseWrench = getYamlNodeAs<std::vector<double>>(
            controllerTree, {"controller", "feed_forward", "base_wrench"});
        traversal(
            completeController->intV, completeController->boolV, completeController->arrayV,
            configTree, "");
        traversal(
            completeController->intV, completeController->boolV, completeController->arrayV,
            controllerTree, "");

        // save thruster solver information
        auto thrusterSolverInfo = controllerTree["thruster_solver"];
        defaultWeight = getYamlNodeAs<double>(thrusterSolverInfo, {"default_weight"});
        surfaceWeight = getYamlNodeAs<double>(thrusterSolverInfo, {"surfaced_weight"});
        disabledWeight = getYamlNodeAs<double>(thrusterSolverInfo, {"disable_weight"});
        lowDowndraftWeight = getYamlNodeAs<double>(thrusterSolverInfo, {"low_downdraft_weight"});

    } catch (YAML::BadFile & e) {
        RCLCPP_ERROR(get_logger(), "Cannot open config file at %s", configPath.c_str());
    }

    try {
        // load autoff yaml file
        autoffTree = YAML::LoadFile(autoffConfigPath);
        currentInitFF = getYamlNodeAs<std::vector<double>>(autoffTree, {"auto_ff"});
        autoffTree = autoffTree["auto_ff"];
        traversal(
            completeController->intV, completeController->boolV, completeController->arrayV,
            autoffTree, "controller__autoff__initial_ff");
    } catch (YAML::BadFile & e) {
        RCLCPP_ERROR(get_logger(), "Cannot open config file at %s", autoffConfigPath.c_str());
    }
}

void ControllerOverseer::generateThrusterForceMatrix() {
    std::vector<int64_t> thrusterFT;

    // set thruster force torque vector as (Fx, Fy, Fz, Tx, Ty, Tz)
    for (const auto & thruster : thrusterInfo) {
        std::vector<double> thrusterPose = getYamlNodeAs<std::vector<double>>(thruster, {"pose"});
        // make 3d matrix of each axis angled
        m3d R = Eigen::AngleAxisd(thrusterPose[5], Eigen::Vector3d::UnitZ()).toRotationMatrix() *
                Eigen::AngleAxisd(thrusterPose[4], Eigen::Vector3d::UnitY()).toRotationMatrix() *
                Eigen::AngleAxisd(thrusterPose[3], Eigen::Vector3d::UnitX()).toRotationMatrix();

        // force vector is just that matrix left multiplied by the x direction
        v3d forceVector = R * Eigen::Vector3d::UnitX();

        std::vector<double> positionFromCom;
        for (int j = 0; j < 3; j++) {
            positionFromCom.push_back(thrusterPose[j] - com[j]);
        }

        // torque = momentArm x forceVector (cross multiply)
        v3d momentArm(positionFromCom[0], positionFromCom[1], positionFromCom[2]);
        v3d torque = momentArm.cross(forceVector);

        // multiply all by parameter scale so they can be set as parameters
        thrusterFT.push_back(static_cast<int64_t>(forceVector(0) * PARAMETERSCALE));
        thrusterFT.push_back(static_cast<int64_t>(forceVector(1) * PARAMETERSCALE));
        thrusterFT.push_back(static_cast<int64_t>(forceVector(2) * PARAMETERSCALE));
        thrusterFT.push_back(static_cast<int64_t>(torque(0) * PARAMETERSCALE));
        thrusterFT.push_back(static_cast<int64_t>(torque(1) * PARAMETERSCALE));
        thrusterFT.push_back(static_cast<int64_t>(torque(2) * PARAMETERSCALE));
    }

    // for(int i = 0; i<6*8; i++){
    //     RCLCPP_INFO(get_logger(), "%ld", thrusterFT[i]);
    // }

    completeController->arrayV.emplace_back(robotName + "_wrenchmat", thrusterFT);
}

void ControllerOverseer::thrusterTelemetryCB(
    mercury_msgs::msg::DshotPartialTelemetry::SharedPtr msg) {
    // reset power check timer
    escPowerCheckTimer->reset();

    // check if activeThrusters matches the thruster telemetry message
    // messages come in thruster groups of 4, either 0-3 or 4-7.
    bool adjustWeights = false;
    if (msg->start_thruster_num == 0) {
        int i = 0;
        for (auto esc : msg->esc_telemetry) {
            // if mismatch, call adjust thruster weights
            if (!esc.thruster_ready && activeThrusters[i] == true) {
                activeThrusters[i] = false;
                adjustWeights = true;

            } else if (esc.thruster_ready && activeThrusters[i] == false) {
                activeThrusters[i] = true;
                adjustWeights = true;
            }
            i++;
        }
        // if a thruster is down, add to power stops
        if (msg->disabled_flags != 0) {
            escPowerStopsLow++;
            RCLCPP_WARN(get_logger(), "Recieving disabled flag: %d", msg->disabled_flags);
        } else {
            escPowerStopsLow = 0;
        }
    } else {
        int i = 4;
        for (auto esc : msg->esc_telemetry) {
            // if mismatch, call adjust thruster weights
            if (!esc.thruster_ready && activeThrusters[i]) {
                activeThrusters[i] = false;
                adjustWeights = true;

            } else if (esc.thruster_ready && !activeThrusters[i]) {
                activeThrusters[i] = true;
                adjustWeights = true;
            }
            i++;
        }
        // if a thruster is down, add to power stops
        if (msg->disabled_flags != 0) {
            escPowerStopsHigh++;
            RCLCPP_WARN(get_logger(), "Recieving disabled flag: %d", msg->disabled_flags);
        } else {
            escPowerStopsHigh = 0;
        }
    }

    if (adjustWeights) {
        adjustThrusterWeights();
    }

    // publish if motion is enabled
    std_msgs::msg::Bool motionMsg;
    if (escPowerStopsLow > ESC_POWER_STOP_TOLERANCE ||
        escPowerStopsHigh > ESC_POWER_STOP_TOLERANCE) {
        motionMsg.data = false;
        enabled = false;
    } else {
        motionMsg.data = true;
        enabled = true;
    }

    motionEnabledPub->publish(motionMsg);

    // RCLCPP_INFO(get_logger(), "Setting motion_enabled to %d first", motionMsg.data);
}

// This will only be called if thruster telemetry hasn't received a message in 2 seconds
void ControllerOverseer::escPowerTimeout() {
    // if we are running and this timer runs out, print warn message
    if (enabled) {
        RCLCPP_WARN(get_logger(), "Not recieving thruster telemetry!");
    }

    // publish false motion message
    std_msgs::msg::Bool motionMsg;
    motionMsg.data = false;
    enabled = false;
    motionEnabledPub->publish(motionMsg);

    RCLCPP_INFO(get_logger(), "Setting motion_enabled to %d second", motionMsg.data);
}

void ControllerOverseer::setThrusterModeCB(std_msgs::msg::Int16::SharedPtr msg) {
    if (msg->data != thrusterMode) {
        thrusterMode = msg->data;
        adjustThrusterWeights();
    }
}

void ControllerOverseer::odometryCB(nav_msgs::msg::Odometry::SharedPtr msg) {
    // start time if needed
    if (!startTimeSet) {
        startTime = get_clock()->now();
        startTimeSet = true;
    }

    std::array<bool, 8> submerged = {false, false, false, false, false, false, false, false};
    double killPlane =
        getYamlNodeAs<double>(controllerTree, {"controller_overseer", "thruster_kill_plane"});

    geometry_msgs::msg::TransformStamped pos;
    try {
        for (int i = 0; i < 8; i++) {
            // if thruster above water, set submerged to true
            pos = tfBuffer->lookupTransform(
                "world", tfNamespace + "/thruster_" + std::to_string(i), tf2::TimePointZero);
            if (pos.transform.translation.z < killPlane) {
                submerged[i] = true;
            }
        }
        // if the amount of submerged thrusters has changed, adjust thruster weights
        if (submerged != submergedThrusters) {
            submergedThrusters = submerged;
            adjustThrusterWeights();
        }
    } catch (const std::exception & ex) {
        if (get_clock()->now().seconds() >= 1.0 + startTime.seconds()) {
            RCLCPP_ERROR(
                get_logger(), "Thruster position lookup failed with exception %s", ex.what());
        }
    }
}

void ControllerOverseer::adjustThrusterWeights() {
    int activeThrusterCount = 0;
    int submergedThrustersCount = 0;

    // count amount of active thrusters and submerged thrusters and set weights
    for (int i = 0; i < 8; i++) {
        if (activeThrusters[i]) {
            activeThrusterCount++;

            if (!submergedThrusters[i]) {
                thrusterWeights[i] = surfaceWeight;
            } else {
                submergedThrustersCount++;
                thrusterWeights[i] = defaultWeight;
            }
        } else {
            thrusterWeights[i] = disabledWeight;
        }
    }

    // if underactuated, disable robot
    if (activeThrusterCount <= 6) {
        if (enabled) {
            RCLCPP_ERROR(
                get_logger(),
                "System is underactuated. Only: %d thrusters are active. Killing thrusters!",
                activeThrusterCount);
            enabled = false;
        } else {
            enabled = true;
        }
    }

    // set low downdraft mode
    if (submergedThrustersCount >= 8 && thrusterMode == 2) {
        // THIS IS FOR TALOS NEEDS TO CHANGE!
        thrusterWeights[4] = lowDowndraftWeight;
        thrusterWeights[5] = lowDowndraftWeight;
    }

    // disable thrusters if no mode set
    if (thrusterMode == 0) {
        for (int i = 0; i < 8; i++) {
            thrusterWeights[i] = 0;
        }
    }

    // publish weights
    std_msgs::msg::Int32MultiArray msg;
    std::vector<int> weights;
    for (const double & weight : thrusterWeights) {
        weights.push_back(weight);
    }
    msg.data = weights;
    weightsPub->publish(msg);
}

void ControllerOverseer::setTeleopCB(
    std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res) {
    try {
        // set control mask based off yaml
        ParameterValue pVal;
        pVal.type = ParameterType::PARAMETER_INTEGER_ARRAY;
        pVal.integer_array_value = getYamlNodeAs<std::vector<int64_t>>(
            controllerTree, {"controller", "active_force_mask"});

        if (req->data) {
            // set teleop
            pVal.integer_array_value[0] = 3000000;
            pVal.integer_array_value[1] = 3000000;
            pVal.integer_array_value[5] = 3000000;

            res->message = "Successfully enabled teleop!";
        } else {
            res->message = "Successfully enabled active control!";
        }
        // create the parameter
        Parameter param;
        param.value = pVal;
        param.name = "controller__active_force_mask";

        // set the parameter using the setParamClient
        SetParams::Request::SharedPtr setParamsRequest = std::make_shared<SetParams::Request>();
        setParamsRequest->parameters = {param};
        completeController->setParamClient->scheduleCall(
            setParamsRequest, [this, setParamsRequest](SetParams::Response::SharedPtr res) {
                completeController->setParametersDoneCallback(res, setParamsRequest);
            });
        res->success = true;

    } catch (std::exception & e) {
        RCLCPP_WARN(get_logger(), "Failed to initialize active control, model may not be started");
        res->success = false;
        res->message = "Failed to initialize active control, model may not be started";
    }
}

void ControllerOverseer::doUpdate() {
    std::vector<std::string> activeROSNodeNames;

    std::vector<std::pair<string, string>> activeROSNodes =
        get_node_graph_interface()->get_node_names_and_namespaces();

    // create vector of all active ros nodes
    for (const auto & node : activeROSNodes) {
        string nodeName = node.second + "/" + node.first;
        // Remove double leading "//"
        if (nodeName.rfind("//", 0) == 0) {
            nodeName = nodeName.substr(1);
        }
        activeROSNodeNames.push_back(nodeName);
    }
    // check if complete controller node is active
    completeController->checkIfActive(activeROSNodeNames);

    // publish FF if necessary
    if (!get_parameter("disable_native_ff").as_bool()) {
        publishingFF = true;

        geometry_msgs::msg::Twist msg;
        msg.linear.x = baseWrench[0];
        msg.linear.y = baseWrench[1];
        msg.linear.z = baseWrench[2];
        msg.angular.x = baseWrench[3];
        msg.angular.y = baseWrench[4];
        msg.angular.z = baseWrench[5];

        // ffPub->publish(msg);

        // reset if needed
    } else if (publishingFF) {
        publishingFF = false;
        geometry_msgs::msg::Twist msg;
        msg.linear.x = 0.0;
        msg.linear.y = 0.0;
        msg.linear.z = 0.0;
        msg.angular.x = 0.0;
        msg.angular.y = 0.0;
        msg.angular.z = 0.0;

        // ffPub->publish(msg);
        RCLCPP_INFO(get_logger(), "Feed forward disabled");
    }
}

void ControllerOverseer::ffAutoTuneCB(geometry_msgs::msg::Twist::SharedPtr msg) {
    // if overseer not ready, break
    if (!writeAutoFF || !completeController->paramsLoaded || autoffConfigPath == "") {
        return;
    }
    // find initial ff
    if (waitingOnInit) {
        bool initFound = false;
        std::vector<double> initFF = getYamlNodeAs<std::vector<double>>(
            controllerTree, {"controller", "autoff", "initial_ff"});

        // if init is found and autoff tolerance is met, waiting on init is false
        if (initFound && std::abs(initFF[0] - msg->linear.x) < AUTOFF_INIT_TOLERANCE &&
            std::abs(initFF[1] - msg->linear.y) < AUTOFF_INIT_TOLERANCE &&
            std::abs(initFF[2] - msg->linear.z) < AUTOFF_INIT_TOLERANCE &&
            std::abs(initFF[3] - msg->angular.x) < AUTOFF_INIT_TOLERANCE &&
            std::abs(initFF[4] - msg->angular.y) < AUTOFF_INIT_TOLERANCE &&
            std::abs(initFF[1] - msg->angular.z) < AUTOFF_INIT_TOLERANCE) {
            waitingOnInit = false;
        } else {
            RCLCPP_WARN(get_logger(), "Init not found or autoff init tolerance not met");
            std_msgs::msg::Empty emptyMsg;
            reInitPub->publish(emptyMsg);
        }

        return;
    }

    // if auto tune ff has changed, rewrite to autoff config file
    if (currentInitFF[0] != msg->linear.x || currentInitFF[1] != msg->linear.y ||
        currentInitFF[2] != msg->linear.z || currentInitFF[3] != msg->angular.x ||
        currentInitFF[4] != msg->angular.y || currentInitFF[5] != msg->angular.z) {
        string autoFFstr =
            "autoff: [" + std::to_string(msg->linear.x) + "," + std::to_string(msg->linear.y) +
            "," + std::to_string(msg->linear.z) + "," + std::to_string(msg->angular.x) + "," +
            std::to_string(msg->angular.y) + "," + std::to_string(msg->angular.z) + "]\n";

        std::ofstream ffconfig(autoffConfigPath);
        if (ffconfig.is_open()) {
            ffconfig << autoFFstr;
        } else {
            RCLCPP_ERROR(
                get_logger(), "Cannot open ff auto tune file at: %s", autoffConfigPath.c_str());
        }
        ffconfig.close();
    }
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    std::shared_ptr<ControllerOverseer> node = std::make_shared<ControllerOverseer>();
    node->init(node);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
