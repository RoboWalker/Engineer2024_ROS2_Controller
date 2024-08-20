#pragma once

#include "FSM/FSMState.hpp"
#include "main.hpp"

class FSMState_Defend: public FSMState
{
    public:
        FSMState_Defend();
        void enter();
        void run();
        void exit();

    private:
        bool start_flag_ = true; //启动保护模式的标签。先把抬升放到指定高度，然后设置为false。
        double init_height = 2;
};
