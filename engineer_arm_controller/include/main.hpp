#pragma once

#include "rclcpp/rclcpp.hpp"
#include "FSM/FSM.hpp"
#include "std_msgs/msg/string.hpp"
#include "engineer_msg/srv/fsm_command.hpp"
#include "engineer_msg/srv/fsm_state.hpp"
#include "engineer_msg/msg/joint_state.hpp"
#include "engineer_msg/msg/joint_command.hpp"
#include "engineer_msg/msg/end_effector.hpp"
#include "engineer_msg/msg/sucket_state.hpp"
#include "engineer_msg/msg/lift_state.hpp"
#include "engineer_msg/msg/robot_arm_state.hpp"

extern class FSM fsm;
extern rclcpp::Node::SharedPtr node;
extern class std::shared_ptr<rclcpp::Publisher<engineer_msg::msg::JointCommand, std::allocator<void>>> joint_command_publisher;
extern class std::shared_ptr<rclcpp::Publisher<engineer_msg::msg::EndEffector, std::allocator<void>>> end_effector_publisher;
extern class std::shared_ptr<rclcpp::Publisher<engineer_msg::msg::RobotArmState, std::allocator<void>>> robotarm_command_publisher;
extern class std::shared_ptr<rclcpp::Publisher<engineer_msg::msg::SucketState, std::allocator<void>>> sucket_command_publisher;
extern class std::shared_ptr<rclcpp::Publisher<engineer_msg::msg::LiftState, std::allocator<void>>> lift_command_publisher;

extern engineer_msg::msg::JointState joint_state_bag;

extern bool sucket_target;
extern bool lift_rise;
extern bool lift_fall;
extern bool sucket_target_from_customctrl;
extern bool last_sucket_target_from_customctrl;
extern bool lift_rise_from_customctrl;
extern bool lift_fall_from_customctrl;
extern bool robotarm_ready;
extern bool little_source_land_flag;

extern double target_end_yaw;
extern double target_end_pitch;
extern double target_end_roll;

extern float remain_end_roll;

extern uint8_t lift_position_real;