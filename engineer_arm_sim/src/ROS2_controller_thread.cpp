// include rclcpp for ROS2
#include "engineer_arm_sim/ROS2_controller_thread.hpp"
#include "engineer_arm_sim/sim.hpp"

//init target pos of six joints
double target_pos[6] = {0, 0, 0, 0, 0, 0};
//init target vel of six joints
double target_vel[6] = {0, 0, 0, 0, 0, 0};
//init target torque of six joints
double target_torque[6] = {10, 0, 0, 0, 0, 0};

//init position PID class (controller) for six joints
Class_PID pos_pid[6];
//init velocity PID class (controller) for six joints
Class_PID vel_pid[6];

/**
 * @brief motor PID controller
*/
void motor_PID_controller()
{
    // static int count = 0;
    // count++;
    // if (count % 1000 == 0) {
    //     target_pos[1] += 3.1416;
    // }
    
    // get target position and real position of six joint and calculate position PID
    for (int i = 0; i < 6; i++) {
        // calculate position PID
        pos_pid[i].Set_Target(target_pos[i]);
        pos_pid[i].Set_Now(d->qpos[m->jnt_qposadr[i]]);
        pos_pid[i].TIM_Adjust_PeriodElapsedCallback();
    }
    //then get target velocity and real velocity of six joint and calculate velocity PID
    double target_vel_new[6];
    for (int i = 0; i < 6; i++) {
        // get target velocity of six joint
        target_vel_new[i] = target_vel[i] + pos_pid[i].Get_Out();
        // calculate velocity PID
        vel_pid[i].Set_Target(target_vel_new[i]);
        vel_pid[i].Set_Now(d->qvel[m->jnt_dofadr[i]]);
        vel_pid[i].TIM_Adjust_PeriodElapsedCallback();
    }
    // use the output to control motors
    double target_torque_new[6];
    for (int i = 0; i < 6; i++) {
        // get target torque of six joint
        target_torque_new[i] = vel_pid[i].Get_Out() + target_torque[i];
        // set target torque of six joint
        d->qfrc_applied[m->jnt_dofadr[i]] = target_torque_new[i];
        // std::cout << "target_torque_new[" << i << "] = " << target_torque_new[i] << std::endl;
    }

}

/**
 * @brief motor MIT controller
*/
void motor_MIT_controller()
{
    double Kp[6] = {5.0, 2.0, 1.0, 0.05, 0.02, 0.02};
    double Kd[6] = {5.0, 2.0, 1.0, 0.05, 0.02, 0.02};

    for (int i = 0; i < 6; i++) {
        // get position error
        double pos_error = target_pos[i] - d->qpos[m->jnt_qposadr[i]];
        //get velocity error
        double vel_error = target_vel[i] - d->qvel[m->jnt_dofadr[i]];
        // get target torque of six joint
        double target_torque_new = Kp[i] * pos_error + Kd[i] * vel_error + target_torque[i];
        //output
        d->qfrc_applied[m->jnt_dofadr[i]] = target_torque_new;
    }


}

/**
 * @brief ROS2 JointCommand subscriber callback function
*/
void ROS2_JointCommand_callback(const engineer_msg::msg::JointCommand::SharedPtr msg)
{
    //Set target position of six joints
    for(int i = 0; i < 6; i++) {
        target_pos[i] = msg->position[i];
    }
    //Set target velocity of six joints
    for(int i = 0; i < 6; i++) {
        target_vel[i] = msg->velocity[i];
    }
    //Set target torque of six joints
    for(int i = 0; i < 6; i++) {
        target_torque[i] = msg->torque[i];
    }
}

/**
 * @brief ROS2 JointState publisher function
 * 
 * @param joint_state_pub ROS2 JointState publisher
*/
void ROS2_JointState_publisher(std::shared_ptr<rclcpp::Publisher<engineer_msg::msg::JointState, std::allocator<void>>> joint_state_pub)
{
    // std::cout << "ROS2_JointState_publisher" << std::endl;
    // ROS2 JointState message
    auto joint_state_msg = std::make_shared<engineer_msg::msg::JointState>();
    joint_state_msg->header.stamp = rclcpp::Clock().now();
    joint_state_msg->header.frame_id = "engineer_arm";
    for (int i = 0; i < m->njnt; i++) {
        joint_state_msg->position[i] = d->qpos[m->jnt_qposadr[i]];
        joint_state_msg->velocity[i] = d->qvel[m->jnt_dofadr[i]];
        joint_state_msg->torque[i] = d->qfrc_applied[m->jnt_dofadr[i]];
    }
    joint_state_pub->publish(*joint_state_msg);
}

/**
 * @brief motor position controller
*/
void pos_control(const mjModel* m, mjData* d){
    // static int count = 0;
    // count++;
    // if (count % 1000 == 0) {
    //     target_pos[4] += 1.0f;
    // }

    for (int i = 0; i < 6; i++) {
        d->qpos[m->jnt_qposadr[i]] = target_pos[i];
    }
}
//由于mujoco中使用PID控制和MIT控制的效果都不好，而且只需要做运动学仿真，所以暂时直接设置仿真中的关节角度
void (*mjcb_control)(const mjModel* m, mjData* d) = pos_control;

/**
 * @brief ROS2 controller thread function
 * 
*/
void ROS2_controller_thread_func()
{
    // //init position PID contorllers
    // pos_pid[0].Init(6.0f, 0.0f, 0.0f, 0.0f, 0.0f, 8.0f);
    // pos_pid[1].Init(6.0f, 0.0f, 0.0f, 0.0f, 0.0f, 8.0f);
    // pos_pid[2].Init(5.0f, 0.0f, 0.0f, 0.0f, 0.0f, 8.0f);
    // pos_pid[3].Init(10.0f, 0.0f, 0.0f, 0.0f, 0.0f, 8.0f);
    // pos_pid[4].Init(8.0f, 0.0f, 0.0f, 0.0f, 0.0f, 8.0f);
    // pos_pid[5].Init(8.0f, 0.0f, 0.0f, 0.0f, 0.0f, 8.0f);
    // //init velocity PID contorllers
    // vel_pid[0].Init(5.5f, 2.0f, 0.0f, 0.0f, 30.0f, 90.0f);
    // vel_pid[1].Init(5.0f, 2.0f, 0.0f, 0.0f, 30.0f, 90.0f);
    // vel_pid[2].Init(0.6f, 0.2, 0.0f, 0.0f, 12.0f, 36.0f);
    // vel_pid[3].Init(0.2f, 0.1f, 0.0f, 0.0f, 3.0f, 6.0f);
    // vel_pid[4].Init(0.1, 0.05, 0.0f, 0.0f, 3.0f, 6.0f);
    // vel_pid[5].Init(0.1, 0.05, 0.0f, 0.0f, 3.0f, 6.0f);

    // ROS2 node initialization
    rclcpp::init(0, nullptr);
    auto node = rclcpp::Node::make_shared("ROS2_controller_thread");
    
    //ROS2 JointState publisher Initialization
    auto joint_state_pub = node->create_publisher<engineer_msg::msg::JointState>("joint_state", 10);

    //ROS2 JointCommand subscriber Initialization
    auto joint_command_sub = node->create_subscription<engineer_msg::msg::JointCommand>("joint_command", 10, ROS2_JointCommand_callback);

    //ROS2 controller timer, 1ms
    
    //由于由于mujoco中使用PID控制和MIT位置控制的效果都不好，所以放弃使用基于力学的控制器，直接设置关节角度
    // auto controller_timer = node->create_wall_timer(std::chrono::milliseconds(1), motor_position_controller);

    //ROS2 JointState Publish Timer, 20ms
    auto joint_state_pub_timer = node->create_wall_timer(std::chrono::milliseconds(20), [joint_state_pub]() -> void {ROS2_JointState_publisher(joint_state_pub);});

    // ROS2 spin
    rclcpp::spin(node);
    
}