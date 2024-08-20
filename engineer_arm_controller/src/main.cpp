#include "main.hpp"

// initialize a FSM
FSM fsm;
rclcpp::Node::SharedPtr node;
std::shared_ptr<rclcpp::Publisher<engineer_msg::msg::JointCommand, std::allocator<void>>> joint_command_publisher;
std::shared_ptr<rclcpp::Publisher<engineer_msg::msg::EndEffector, std::allocator<void>>> end_effector_publisher;
std::shared_ptr<rclcpp::Publisher<engineer_msg::msg::RobotArmState, std::allocator<void>>> robotarm_command_publisher;
std::shared_ptr<rclcpp::Publisher<engineer_msg::msg::SucketState, std::allocator<void>>> sucket_command_publisher;
std::shared_ptr<rclcpp::Publisher<engineer_msg::msg::LiftState, std::allocator<void>>> lift_command_publisher;

engineer_msg::msg::JointState joint_state_bag;

bool sucket_target = false;
bool lift_rise = false;
bool lift_fall = false;
bool sucket_target_from_customctrl = false;
bool last_sucket_target_from_customctrl = false;
bool lift_rise_from_customctrl = false;
bool lift_fall_from_customctrl = false;
bool little_source_land_flag = false;

double target_end_yaw = 0;
double target_end_pitch = 0;
double target_end_roll = 0;

float remain_end_roll = 0;

uint8_t lift_position_real;

/**
 * @brief ROS2 service callback function for the FSM
*/
void fsmCommandCallback(const std::shared_ptr<engineer_msg::srv::FSMCommand::Request> request,
                        std::shared_ptr<engineer_msg::srv::FSMCommand::Response> response)
{
    //TODO: 添加更多状态时需要修改这个函数
    // get the command from the request
    FSMStateName start_state = fsm.getCurrentStateName();
    std::string start_state_str = fsm.getCurrentStateNameStr();
    FSMStateName next_state;
    FSMCommand command;

    if (request->command == "passive" || request->command == "PASSIVE")
    {
        command = FSMCommand::PASSIVE;
    }
    else if (request->command == "custom_control" || request->command == "CUSTOM_CONTROL")
    {
        command = FSMCommand::CUSTOM_CONTROL;
    }
    else if (request->command == "auto_perlace_1" || request->command == "AUTO_PERLACE_1")
    {
        command = FSMCommand::AUTO_PERLACE_1;
    }
    else if (request->command == "auto_perlace_2" || request->command == "AUTO_PERLACE_2")
    {
        command = FSMCommand::AUTO_PERLACE_2;
    }
    else if (request->command == "auto_takeout_1" || request->command == "AUTO_TAKEOUT_1")
    {
        command = FSMCommand::AUTO_TAKEOUT_1;
    }
    else if (request->command == "auto_takeout_2" || request->command == "AUTO_TAKEOUT_2")
    {
        command = FSMCommand::AUTO_TAKEOUT_2;
    }
    else if (request->command == "little_source_land" || request->command == "LITTLE_SOURCE_LAND")
    {
        command = FSMCommand::LITTLE_SOURCE_LAND;
    }
    else if (request->command == "big_source_land" || request->command == "BIG_SOURCE_LAND")
    {
        command = FSMCommand::BIG_SOURCE_LAND;
    }
    else if (request->command == "defend" || request->command == "DEFEND")
    {
        command = FSMCommand::DEFEND;
    }
    else{
        response->is_successful = false;
        response->status_now = fsm.getCurrentStateNameStr();
        response->reason = "Invalid command";
        return;
    }

    //检测当前命令是否意味着状态没有切换，如果是，直接返回
    if(command == FSMCommand::PASSIVE && start_state == FSMStateName::PASSIVE){
        response->is_successful = false;
        response->status_now = fsm.getCurrentStateNameStr();
        response->reason = "Same state";
        return;
    }
    else if(command == FSMCommand::CUSTOM_CONTROL && start_state == FSMStateName::CUSTOM_CONTROL){
        response->is_successful = false;
        response->status_now = fsm.getCurrentStateNameStr();
        response->reason = "Same state";
        return;
    }
    else if(command == FSMCommand::AUTO_PERLACE_1 && start_state == FSMStateName::AUTO_PERLACE_1){
        response->is_successful = false;
        response->status_now = fsm.getCurrentStateNameStr();
        response->reason = "Same state";
        return;
    }
    else if(command == FSMCommand::AUTO_PERLACE_2 && start_state == FSMStateName::AUTO_PERLACE_2){
        response->is_successful = false;
        response->status_now = fsm.getCurrentStateNameStr();
        response->reason = "Same state";
        return;
    }
    else if(command == FSMCommand::AUTO_TAKEOUT_1 && start_state == FSMStateName::AUTO_TAKEOUT_1){
        response->is_successful = false;
        response->status_now = fsm.getCurrentStateNameStr();
        response->reason = "Same state";
        return;
    }
    else if(command == FSMCommand::AUTO_TAKEOUT_2 && start_state == FSMStateName::AUTO_TAKEOUT_2){
        response->is_successful = false;
        response->status_now = fsm.getCurrentStateNameStr();
        response->reason = "Same state";
        return;
    }
    else if(command == FSMCommand::LITTLE_SOURCE_LAND && start_state == FSMStateName::LITTLE_SOURCE_LAND){
        little_source_land_flag = true;
        response->is_successful = false;
        response->status_now = fsm.getCurrentStateNameStr();
        response->reason = "Same state";
        return;
    }
    else if(command == FSMCommand::BIG_SOURCE_LAND && start_state == FSMStateName::BIG_SOURCE_LAND){
        response->is_successful = false;
        response->status_now = fsm.getCurrentStateNameStr();
        response->reason = "Same state";
        return;
    }
    else if(command == FSMCommand::DEFEND && start_state == FSMStateName::DEFEND){
        response->is_successful = false;
        response->status_now = fsm.getCurrentStateNameStr();
        response->reason = "Same state";
        return;
    }

    //设置命令后，延时50ms重新检测状态，如果切换则返回成功，否则返回未知原因失败
    fsm.setCommand(command);
    rclcpp::sleep_for(std::chrono::milliseconds(50));
    next_state = fsm.getCurrentStateName();
    if(next_state != start_state){
        response->is_successful = true;
        response->status_now = fsm.getCurrentStateNameStr();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "FSM state changed from %s to %s",
                    start_state_str.c_str(), fsm.getCurrentStateNameStr().c_str());
        response->reason = "Success";
    }
    else{
        response->is_successful = false;
        response->status_now = fsm.getCurrentStateNameStr();
        response->reason = "Unknown reason";
    }
}

/**
 * @brief ROS2 service callback function to get the state of the FSM
*/
void fsmStateCallback(const std::shared_ptr<engineer_msg::srv::FSMState::Request> request,
                      std::shared_ptr<engineer_msg::srv::FSMState::Response> response)
{
    response->status_now = fsm.getCurrentStateNameStr();
}

/**
 * @brief ROS2 subscriber callback function for the joint angles
 * @details Set the joint angles for Kinematics_Controller
*/
void jointStateCallback(const engineer_msg::msg::JointState::SharedPtr msg)
{
    kinematics_controller.setNowJointAngles(msg->position[0], msg->position[1], msg->position[2],
                                            msg->position[3], msg->position[4], msg->position[5]);
    lift_position_real = msg->lift_position;
}

/**
 * @brief ROS2 subscriber callback for target end effector
*/
void targetEndEffectorCallback(const engineer_msg::msg::EndEffector::SharedPtr msg)
{
    kinematics_controller.setTargetEndEffector(msg->position[0], msg->position[1], msg->position[2],
                                               msg->orientation[0], msg->orientation[1], msg->orientation[2]);
    target_end_yaw = msg->orientation[2];
    target_end_pitch = msg->orientation[1];
    target_end_roll = msg->orientation[0];
}

/**
 * @brief ROS2 subscriber callback for robot arm state
*/
void RobotArmStateCallback(const engineer_msg::msg::RobotArmState::SharedPtr msg)
{
    robotarm_ready = msg->auto_action_ready;
}


int main()
{
    // initialize a ROS2 node
    rclcpp::init(0, nullptr);
    node = rclcpp::Node::make_shared("engineer_arm_controller");

    // initialize a ROS2 service to set command for the FSM
    auto fsm_command_service = node->create_service<engineer_msg::srv::FSMCommand>("fsm_command",
        fsmCommandCallback);

    // initialize a ROS2 service to get the current state of the FSM
    auto fsm_state_service = node->create_service<engineer_msg::srv::FSMState>("fsm_state",
        fsmStateCallback);

    // initialize a ROS2 subscriber to get the joint angles
    auto joint_state_subscriber = node->create_subscription<engineer_msg::msg::JointState>(
        "joint_state", 10, jointStateCallback);

    // initialize a ROS2 subscriber to get the target end effector
    auto target_end_effector_subscriber = node->create_subscription<engineer_msg::msg::EndEffector>(
        "target_end_effector", 10, targetEndEffectorCallback);

    // initialize a ROS2 subscriber to get the robot arm state
    auto robotarm_state_subscriber = node->create_subscription<engineer_msg::msg::RobotArmState>(
        "robotarm_state", 10, RobotArmStateCallback);

    // initialize a ROS2 publisher to publish the joint angles
    joint_command_publisher = node->create_publisher<engineer_msg::msg::JointCommand>(
        "joint_command", 10);
    
    // initialize a ROS2 publisher to publish the now end effector
    end_effector_publisher = node->create_publisher<engineer_msg::msg::EndEffector>(
        "now_end_effector", 10);

    // initialize a ROS2 subscriber to get the sucket state from customctrl
    auto sucket_state_subscriber = node->create_subscription<engineer_msg::msg::SucketState>(
        "customctrl_sucket_state", 10, [&](const engineer_msg::msg::SucketState::SharedPtr msg){
            sucket_target_from_customctrl = msg->state;
        });

    // initialize a ROS2 subscriber to get the lift state from customctrl
    auto lift_state_subscriber = node->create_subscription<engineer_msg::msg::LiftState>(
        "customctrl_lift_state", 10, [&](const engineer_msg::msg::LiftState::SharedPtr msg){
            lift_rise_from_customctrl = msg->rise;
            lift_fall_from_customctrl = msg->fall;
        });

    //用于发布机械臂状态
    robotarm_command_publisher = node->create_publisher<engineer_msg::msg::RobotArmState>(
        "auto_command", 10);

    // initialize a ROS2 publisher to publish the sucket command
    sucket_command_publisher = node->create_publisher<engineer_msg::msg::SucketState>(
        "sucket_command", 10);

    // initialize a ROS2 publisher to publish the lift command
    lift_command_publisher = node->create_publisher<engineer_msg::msg::LiftState>(
        "lift_command", 10);

    
    // start a new thread,initialize a timer to run the FSM, 20ms per loop
    std::thread fsm_thread([&](){
        rclcpp::Rate loop_rate(50);
        while(rclcpp::ok()){
            fsm.run();

            engineer_msg::msg::RobotArmState auto_state;
            auto_state.auto_action = robotarm_state_code;
            auto_state.auto_action_ready = robotarm_ready;
            robotarm_command_publisher->publish(auto_state);

            loop_rate.sleep();
        }
    });

    // spin the node
    rclcpp::spin(node);
    rclcpp::shutdown();
}
