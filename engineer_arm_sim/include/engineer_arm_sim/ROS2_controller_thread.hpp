#pragma once

#include "rclcpp/rclcpp.hpp"
//include std_msgs for ROS2
#include "std_msgs/msg/string.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "engineer_msg/msg/joint_state.hpp"
#include "engineer_msg/msg/joint_command.hpp"
#include <iostream>
#include "engineer_arm_sim/alg_pid.h"

void ROS2_controller_thread_func();