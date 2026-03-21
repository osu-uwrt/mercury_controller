#include "controller_overseer.hpp"


    ControllerOverseer::ControllerOverseer() : Node("controller_overseer") {

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



        for(int i = 0; i < 8; i++){
            activeThrusters[i] = true;
            submergedThrusters[i] = true;
            thrusterWeights[i] = 1;
        }


        using std::placeholders::_1,std::placeholders::_2;

        thrusterTelemetry = create_subscription<riptide_msgs2::msg::DshotPartialTelemetry>("/state/thrusters/telemetry", rclcpp::SystemDefaultsQoS(), std::bind(&ControllerOverseer::thrusterTelemetryCB, this, _1));        
        setThrusterSolverParams = create_client<rcl_interfaces::srv::SetParameters>(thrusterSolverName + "/set_parameters");

        motionEnabledPub = create_publisher<std_msgs::msg::Bool>("controller/motion_enabled", rclcpp::SystemDefaultsQoS());
        thrusterModeSub = create_subscription<std_msgs::msg::Int16>("thrusterSolver/thrusterState", rclcpp::SystemDefaultsQoS(), std::bind(&ControllerOverseer::setThrusterModeCB, this, _1));
        odom = create_subscription<nav_msgs::msg::Odometry>("odometry/filtered", rclcpp::SystemDefaultsQoS(), std::bind(&ControllerOverseer::odometryCB, this, _1));
        ffAutoTune = create_subscription<geometry_msgs::msg::Twist>("ff_auto_tune", rclcpp::SystemDefaultsQoS(), std::bind(&ControllerOverseer::ffAutoTuneCB, this, _1));
        weightsPub = create_publisher<std_msgs::msg::Int32MultiArray>("controller/solver_weights", rclcpp::SystemDefaultsQoS());
        ffPub = create_publisher<geometry_msgs::msg::Twist>("controller/FF_body_force", rclcpp::SystemDefaultsQoS());
        reInitPub = create_publisher<std_msgs::msg::Empty>("controller/re_init_accumulators", rclcpp::SystemDefaultsQoS());

        setTeleop = create_service<std_srvs::srv::SetBool>("setTeleop", std::bind(&ControllerOverseer::setTeleopCB, this, _1, _2));

        tfBuffer = std::make_unique<tf2_ros::Buffer>(get_clock());
        tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

        tfNamespace = get_parameter("robot").as_string();

        using namespace std::chrono_literals;
        updateTimer = create_wall_timer(1s, std::bind(&ControllerOverseer::doUpdate, this));
        weightTimer = create_wall_timer(1s, std::bind(&ControllerOverseer::adjustThrusterWeights, this)); 
        escPowerCheckTimer = create_wall_timer(2s, std::bind(&ControllerOverseer::escPowerTimeout, this));

    }

    //construct the complete controller class using the pointer for "this" instance
    void ControllerOverseer::init(std::shared_ptr<ControllerOverseer> node){
        completeController = std::make_shared<SimulinkModelClass>(node, "complete_controller");

        setConfigPath();
        readConfig();
        generateThrusterForceMatrix(thrusterInfo, com);
    }


    /*
    Yaml traversal -- get recursed
    */
    void ControllerOverseer::traversal(int_vector& ints, bool_vector& bools, array_vector& arrays, const YAML::Node& tree, const std::string path){
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
                            int64_t changed = item.as<int64_t>()*PARAMETERSCALE;
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
                int64_t num = 0;
                bool boolean;
                try{
                    num = tree.as<int64_t>() * PARAMETERSCALE;
                } catch (const YAML::BadConversion& e){
                    try{
                    boolean = tree.as<bool>();
                    }catch(const YAML::BadConversion& e){
                        RCLCPP_ERROR(get_logger(), "%s: %s not read properly", path.c_str(), tree.Tag().c_str());
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
    void ControllerOverseer::readConfig(){
        try{
            configTree = YAML::LoadFile(configPath);
            thrusterInfo = configTree["thrusters"];
            com = configTree["com"];

            baseWrench = getYamlNodeAs<std::vector<double>>(configTree, {"controller", "feed_forward", "base_wrench"});
            traversal(completeController->intV, completeController->boolV, completeController->arrayV, configTree, "");

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
            currentInitFF = getYamlNodeAs<std::vector<double>>(autoffTree, {"auto_ff"});
            autoffTree = autoffTree["auto_ff"];
            traversal(completeController->intV, completeController->boolV, completeController->arrayV, autoffTree, "controller__autoff__initial_ff");
        }catch(YAML::BadFile &e){
            RCLCPP_ERROR(get_logger(), "Cannot open config file at %s", autoffConfigPath.c_str());
        }



    }

    void ControllerOverseer::generateThrusterForceMatrix(const YAML::Node& thrusterInfo, const YAML::Node& com){
        std::vector<int64_t> thrusterFT;
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
                
            thrusterFT.push_back(static_cast<int64_t>(forceVector(0) * PARAMETERSCALE));
            thrusterFT.push_back(static_cast<int64_t>(forceVector(1) * PARAMETERSCALE));
            thrusterFT.push_back(static_cast<int64_t>(forceVector(2) * PARAMETERSCALE));
            thrusterFT.push_back(static_cast<int64_t>(torque(0) * PARAMETERSCALE));
            thrusterFT.push_back(static_cast<int64_t>(torque(1) * PARAMETERSCALE));
            thrusterFT.push_back(static_cast<int64_t>(torque(2) * PARAMETERSCALE));
        }

        completeController->arrayV.emplace_back("talos_wrenchmat", thrusterFT);
    }

    void ControllerOverseer::setConfigPath(){
        configPath = get_parameter("vehicle_config").as_string();
        if(configPath == ""){
            string descriptionsShareDir = ament_index_cpp::get_package_share_directory("riptide_descriptions2");
            string robotConfigSubpath = "config/" + robotName + ".yaml";
            
            configPath = descriptionsShareDir + "/" + robotConfigSubpath;
            
            std::vector<string> dirSplit;
            std::stringstream ss(configPath);
            string subPath;

            while (std::getline(ss, subPath, '/')) {
                dirSplit.push_back(subPath);
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

        string controlShareDir = ament_index_cpp::get_package_share_directory("riptide_controllers2");
        string autoFFSubpath = "config/" + robotName + "_autoff.yaml";

        if(fs::exists("/home/ros/colcon_deploy")){
            RCLCPP_INFO(get_logger(), "I think I am not running on the orin!");
            autoffConfigPath = controlShareDir + autoFFSubpath;
        }else{
            RCLCPP_INFO(get_logger(), "I think I am running on the orin!");
            autoffConfigPath = "/bin" + robotName + "_autoff.yaml";     //this will need to change it is no longer /bin
        }
    }

    void ControllerOverseer::thrusterTelemetryCB(riptide_msgs2::msg::DshotPartialTelemetry::SharedPtr msg){
        bool adjustWeights = false;

        escPowerCheckTimer->reset();

        if(msg->start_thruster_num == 0){
            int i = 0;
            for(auto esc : msg->esc_telemetry){
                if(!esc.thruster_ready && activeThrusters[i] == true){
                    activeThrusters[i] = false;
                    adjustWeights = true;

                }else if(esc.thruster_ready && activeThrusters[i] == false){
                    activeThrusters[i] = true;
                    adjustWeights = true;
                }
                i++;
            }
            if(msg->disabled_flags != 0){
                escPowerStopsLow++;
            }else{
                escPowerStopsLow = 0;
            }
        }else{
            int i = 4;
            for(auto esc : msg->esc_telemetry){
                if(!esc.thruster_ready && activeThrusters[i] == true){
                    activeThrusters[i] = false;
                    adjustWeights = true;

                }else if(esc.thruster_ready && activeThrusters[i] == false){
                    activeThrusters[i] = true;
                    adjustWeights = true;
                }
                i++;
            }
            if(msg->disabled_flags != 0){
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

    void ControllerOverseer::escPowerTimeout(){
        if(!enabled){
            RCLCPP_WARN(get_logger(), "Not recieving thruster telemetry!");
        }

        std_msgs::msg::Bool motionMsg;
        motionMsg.data = false;
        enabled = false;
        motionEnabledPub->publish(motionMsg);
    }

    void ControllerOverseer::setThrusterModeCB(std_msgs::msg::Int16::SharedPtr msg){
        if(msg->data != thrusterMode){
            thrusterMode = msg->data;
            adjustThrusterWeights();
        }

    }
    

    void ControllerOverseer::odometryCB(nav_msgs::msg::Odometry::SharedPtr msg){
        if(!startTimeSet){
            startTime = get_clock()->now();
            startTimeSet = true;
        }
        
        std::array<bool, 8> submerged = {false,false,false,false,false,false,false,false};
        
        double killPlane = getYamlNodeAs<double>(configTree, {"controller_overseer", "thruster_kill_plane"});

        geometry_msgs::msg::TransformStamped pos;
        try{
            for(int i = 0; i < 8; i++){

                pos = tfBuffer->lookupTransform("world", tfNamespace + "thruster_" + std::to_string(i), tf2::TimePointZero);

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
            if(get_clock()->now().seconds() >= 1.0 + startTime.seconds()){
                RCLCPP_ERROR(get_logger(), "Thruster position lookup failed with exception %s", ex.what());
            }
        }
    }

    void ControllerOverseer::adjustThrusterWeights(){
        int activeThrusterCount = 0;
        int submergedThrustersCount = 0;

        for(int i = 0; i<8; i++){
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
                RCLCPP_ERROR(get_logger(), "System is underactuated. Only: %s thrusters are active. Killing thrusters!", std::to_string(activeThrusterCount).c_str());
                enabled = false;
            }else{
                enabled = true;
            }
        }

        if(submergedThrustersCount >= 8 && thrusterMode == 2){
            thrusterWeights[4] = lowDowndraftWeight;
            thrusterWeights[5] = lowDowndraftWeight;
        } 

        if(thrusterMode == 0){
            for(int i = 0; i < 8; i++){
                thrusterWeights[i] = 0;
            }
        }

        std_msgs::msg::Int32MultiArray msg;
        
        std::vector<int> weights;
        for(const double& weight : thrusterWeights){
            weights.push_back(weight);
        }

        msg.data = weights;

        weightsPub->publish(msg);
    }

    void ControllerOverseer::setTeleopCB(std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr res){
        try{
            
            ParameterValue pVal;
            pVal.type = ParameterType::PARAMETER_INTEGER_ARRAY;
            pVal.integer_array_value = getYamlNodeAs<std::vector<int64_t>>(configTree, {"controller", "active_force_control"});

            if(req->data){
                //set teleop
                pVal.integer_array_value[0] = 3000000;
                pVal.integer_array_value[1] = 3000000;
                pVal.integer_array_value[5] = 3000000;

                res->message = "Successfully enabled teleop!";
            }else{
                res->message = "Successfully enabled active control!";
            }

            Parameter param;
            param.value = pVal;
            param.name = "controller__active_force_mask";

            SetParams::Request::SharedPtr setParamsRequest = std::make_shared<SetParams::Request>();
            setParamsRequest->parameters = {param};

            completeController->setParamClient->scheduleCall(setParamsRequest, 
                                                                [this, setParamsRequest](SetParams::Response::SharedPtr res){
                                                                completeController->setParametersDoneCallback(res, setParamsRequest);
                                                            });

            res->success = true;
                
        }catch(std::exception &e){
            RCLCPP_WARN(get_logger(), "Failed to initialize active control, model may not be started");
            res->success = false;
            res->message = "Failed to initialize active control, model may not be started";
        }
    }

    void ControllerOverseer::doUpdate(){
        std::vector<std::string> activeROSNodeNames;

        std::vector<std::pair<string, string>> activeROSNodes = get_node_graph_interface()->get_node_names_and_namespaces();

        for (const auto& node : activeROSNodes) {
            std::string nodeName = node.second + "/" + node.first;

            // Remove double leading "//"
            if (nodeName.rfind("//", 0) == 0) { 
                nodeName = nodeName.substr(1);
            }

            activeROSNodeNames.push_back(nodeName);
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
    void ControllerOverseer::ffAutoTuneCB(geometry_msgs::msg::Twist::SharedPtr msg){
        if(!writeAutoFF || !completeController->paramsLoaded || autoffConfigPath == ""){
            return;
        }

        std::vector<int64_t> initFF;
        if(waitingOnInit){
            bool initFound = false;
            for(const std::pair<string, std::vector<int64_t>>& pair : completeController->arrayV){
                if(pair.first == "controller__autoff__initial_ff"){
                    initFF = pair.second;
                    initFound = true;
                    break;
                }
            }


            
            if(initFound && std::abs(initFF[0] - msg->linear.x) < AUTOFF_INIT_TOLERANCE && std::abs(initFF[1] - msg->linear.y) < AUTOFF_INIT_TOLERANCE && std::abs(initFF[2] - msg->linear.z) < AUTOFF_INIT_TOLERANCE 
                && std::abs(initFF[3] - msg->angular.x) < AUTOFF_INIT_TOLERANCE && std::abs(initFF[4] - msg->angular.y) < AUTOFF_INIT_TOLERANCE && std::abs(initFF[1] - msg->angular.z) < AUTOFF_INIT_TOLERANCE){
                    
                    waitingOnInit = false;
            } else{
                RCLCPP_WARN(get_logger(), "Init not found or autoff init tolerance not met");
                std_msgs::msg::Empty emptyMsg;
                reInitPub->publish(emptyMsg);
            }
            
            return;
        }

        if(currentInitFF[0] != msg->linear.x || currentInitFF[1] != msg->linear.y || currentInitFF[2] != msg->linear.z || currentInitFF[3] != msg->angular.x || currentInitFF[4] != msg->angular.y || currentInitFF[5] != msg->angular.z){
            
            string autoFFstr = "autoff: [" + std::to_string(msg->linear.x) + "," + std::to_string(msg->linear.y) + "," + std::to_string(msg->linear.z) + "," + std::to_string(msg->angular.x) + "," + std::to_string(msg->angular.y) + "," + std::to_string(msg->angular.z) + "]\n";    
            
            std::ofstream ffconfig(autoffConfigPath);
            if(ffconfig.is_open()){
                ffconfig << autoFFstr;
            }else{
                RCLCPP_ERROR(get_logger(), "Cannot open ff auto tune file at: %s", autoffConfigPath.c_str());
            }
            ffconfig.close();
        }

    }





int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);

    std::shared_ptr<ControllerOverseer> node = std::make_shared<ControllerOverseer>();
    node->init(node);
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}