#include "FSM/FSMState_Defend.hpp"



FSMState_Defend::FSMState_Defend():
                  FSMState(FSMStateName::DEFEND, "defend"){}

void FSMState_Defend::enter()
{
    start_flag_ = true;
}

void FSMState_Defend::run()
{
    if(start_flag_)
    {
        //移动抬升机构
        if (lift_position_real > init_height + 0.5)
        {
            lift_rise = false;
            lift_fall = true;
        }
        else if (lift_position_real < init_height - 0.5)
        {
            lift_rise = true;
            lift_fall = false;
        }
        else{
            lift_rise = false;
            lift_fall = false;
            start_flag_ = false;
        }
    }

    //call the ROS2 joint_command_publisher to publish the joint command
    engineer_msg::msg::JointCommand joint_command;
    joint_command.position[0] = -0;
    joint_command.position[1] = -1.0f;
    joint_command.position[2] = M_PI / 2 + 0.5;
    joint_command.position[3] = -0.5;
    joint_command.position[4] = 0.0;
    joint_command_publisher->publish(joint_command);

    //call the ROS2 sucket_publisher to publish the sucket state
    sucket_target = (sucket_target ^ (sucket_target_from_customctrl ^ last_sucket_target_from_customctrl));
    last_sucket_target_from_customctrl = sucket_target_from_customctrl;
    engineer_msg::msg::SucketState sucket_state;
    sucket_state.state = sucket_target;
    sucket_command_publisher->publish(sucket_state);

    //call the ROS2 sucket_publisher to publish the lift state
    if(!start_flag_){
        lift_rise = false;
        lift_fall = false;
    }

    engineer_msg::msg::LiftState lift_state;
    lift_state.rise = lift_rise;
    lift_state.fall = lift_fall;
    lift_command_publisher->publish(lift_state);
}

void FSMState_Defend::exit()
{
    //TODO: add exit code
}

