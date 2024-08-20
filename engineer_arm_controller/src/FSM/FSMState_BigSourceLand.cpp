#include "FSM/FSMState_BigSourceLand.hpp"



FSMState_BigSourceLand::FSMState_BigSourceLand():
                  FSMState(FSMStateName::BIG_SOURCE_LAND, "big_source_land"){}

void FSMState_BigSourceLand::enter()
{
    start_flag_ = true;
}

void FSMState_BigSourceLand::run()
{
    if(start_flag_)
    {
        //移动抬升机构
        if (lift_position_real > init_height + 0.5)
        {
            lift_rise = false;
            lift_fall = true;
        }
        else{
            lift_rise = false;
            lift_fall = false;
            start_flag_ = false;
        }
    }

    //call the ROS2 joint_command_publisher to publish the joint command
    engineer_msg::msg::JointCommand joint_command;
    joint_command.position[0] = 0.0f;
    joint_command.position[1] = 0.0f;
    joint_command.position[2] = 0.0f;
    joint_command.position[3] = -target_end_yaw;
    joint_command.position[4] = target_end_pitch;
    joint_command.position[5] = 0;
    joint_command_publisher->publish(joint_command);

    remain_end_roll = joint_command.position[5];

    //call the ROS2 sucket_publisher to publish the sucket state
    sucket_target = (sucket_target ^ (sucket_target_from_customctrl ^ last_sucket_target_from_customctrl));
    last_sucket_target_from_customctrl = sucket_target_from_customctrl;
    engineer_msg::msg::SucketState sucket_state;
    sucket_state.state = sucket_target;
    sucket_command_publisher->publish(sucket_state);

    //call the ROS2 sucket_publisher to publish the lift state
    if(!start_flag_){
        lift_rise = lift_rise_from_customctrl;
        lift_fall = lift_fall_from_customctrl;
    }

    engineer_msg::msg::LiftState lift_state;
    lift_state.rise = lift_rise;
    lift_state.fall = lift_fall;
    lift_command_publisher->publish(lift_state);
}

void FSMState_BigSourceLand::exit()
{
    //TODO: add exit code
}

