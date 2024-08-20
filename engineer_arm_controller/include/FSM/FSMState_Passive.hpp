#pragma once

#include "FSM/FSMState.hpp"
#include "main.hpp"

class FSMState_Passive: public FSMState
{
    public:
        FSMState_Passive();
        void enter();
        void run();
        void exit();
};
