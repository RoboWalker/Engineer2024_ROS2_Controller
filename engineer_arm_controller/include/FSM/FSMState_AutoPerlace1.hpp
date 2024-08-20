#pragma once

#include "FSM/FSMState.hpp"
#include "main.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "engineer_msg/msg/joint_state.hpp"
#include "engineer_msg/msg/joint_command.hpp"   

class FSMState_AutoPerlace1: public FSMState
{
    public:
        FSMState_AutoPerlace1();
        void enter();
        void run();
        void exit();

    protected:
        uint64_t BagPlay_Counter = 0;
        std::unique_ptr<rosbag2_cpp::readers::SequentialReader> reader_;
};
