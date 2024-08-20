#include "FSM/FSMState_CustomControl.hpp"

FSMState_CustomControl::FSMState_CustomControl():
                  FSMState(FSMStateName::CUSTOM_CONTROL, "custom_control"){}

void FSMState_CustomControl::enter()
{
    //TODO: add enter code 
}

void FSMState_CustomControl::run()
{

    // //测试：(本函数的计算周期为20ms),反复运动
    // static bool is_first_time = true;
    // static double target_x = 0.2;
    // static double target_pitch = -0.6;
    // static double target_yaw = -0.5;
    // if(is_first_time){
    //     is_first_time = false;
    //     target_x = 0.2;
    //     target_pitch = -0.6;
    //     target_yaw = -0.5;
    // }
    // //每次移动0.01
    // static bool is_up_x = true;
    // if(target_x > 0.4){
    //     is_up_x = false;
    // }
    // else if(target_x < 0.2){
    //     is_up_x = true;
    // }
    // if (is_up_x)
    // {
    //     target_x += 0.01;
    // }
    // else
    // {
    //     target_x -= 0.01;
    // }
    // //pitch
    // static bool is_up_pitch = true;
    // if(target_pitch > 0.6){
    //     is_up_pitch = false;
    // }
    // else if(target_pitch < -0.6){
    //     is_up_pitch = true;
    // }
    // if (is_up_pitch)
    // {
    //     target_pitch += 0.01;
    // }
    // else
    // {
    //     target_pitch -= 0.01;
    // }
    // //yaw
    // static bool is_up_yaw = true;
    // if(target_yaw > -0.2){
    //     is_up_yaw = false;
    // }
    // else if(target_yaw < -0.5){
    //     is_up_yaw = true;
    // }
    // if (is_up_yaw)
    // {
    //     target_yaw += 0.005;
    // }
    // else
    // {
    //     target_yaw -= 0.005;
    // }
    // //设置目标
    // kinematics_controller.setTargetEndEffector(target_x, 0.2, 0.3, 0.0, target_pitch, target_yaw);
    // //结束

    kinematics_controller.forwardKinematics();

    if(kinematics_controller.groundMode){
        kinematics_controller.inverseKinematicsGround();
    }
    else{
        kinematics_controller.inverseKinematics();
    }

    //call the ROS2 joint_command_publisher to publish the joint command
    engineer_msg::msg::JointCommand joint_command;
    joint_command.position[0] = kinematics_controller.getTargetJointAngles()(0, 0);
    joint_command.position[1] = kinematics_controller.getTargetJointAngles()(1, 0);
    joint_command.position[2] = kinematics_controller.getTargetJointAngles()(2, 0);
    joint_command.position[3] = kinematics_controller.getTargetJointAngles()(3, 0);
    joint_command.position[4] = kinematics_controller.getTargetJointAngles()(4, 0);
    joint_command.position[5] = kinematics_controller.getTargetJointAngles()(5, 0);
    joint_command_publisher->publish(joint_command);

    remain_end_roll = joint_command.position[5];

    //call the ROS2 end_effector_publisher to publish the end effector
    engineer_msg::msg::EndEffector end_effector;
    end_effector.position[0] = kinematics_controller.getNowEndEffectorEuler()(0, 0);
    end_effector.position[1] = kinematics_controller.getNowEndEffectorEuler()(1, 0);
    end_effector.position[2] = kinematics_controller.getNowEndEffectorEuler()(2, 0);
    end_effector.orientation[0] = kinematics_controller.getNowEndEffectorEuler()(3, 0);
    end_effector.orientation[1] = kinematics_controller.getNowEndEffectorEuler()(4, 0);
    end_effector.orientation[2] = kinematics_controller.getNowEndEffectorEuler()(5, 0);
    end_effector_publisher->publish(end_effector);

    //call the ROS2 sucket_publisher to publish the sucket state
    // if (((sucket_target_from_customctrl) && (!last_sucket_target_from_customctrl)) ||
    //     ((!sucket_target_from_customctrl) && (last_sucket_target_from_customctrl)))
    // {
    //     if (sucket_target == true)
    //     {
    //         sucket_target = false;
    //     }
    //     else
    //     {
    //         sucket_target = true;
    //     }
    // }
    // if (sucket_target_from_customctrl != last_sucket_target_from_customctrl)
    // {
    //     sucket_target = !sucket_target;
    // }
    sucket_target = (sucket_target ^ (sucket_target_from_customctrl ^ last_sucket_target_from_customctrl));
    last_sucket_target_from_customctrl = sucket_target_from_customctrl;

    engineer_msg::msg::SucketState sucket_state;
    sucket_state.state = sucket_target;
    sucket_command_publisher->publish(sucket_state);

    //call the ROS2 sucket_publisher to publish the lift state
    lift_rise = lift_rise_from_customctrl;
    lift_fall = lift_fall_from_customctrl;
    engineer_msg::msg::LiftState lift_state;
    lift_state.rise = lift_rise;
    lift_state.fall = lift_fall;
    lift_command_publisher->publish(lift_state);
}

void FSMState_CustomControl::exit()
{
    //TODO: add exit code
}

