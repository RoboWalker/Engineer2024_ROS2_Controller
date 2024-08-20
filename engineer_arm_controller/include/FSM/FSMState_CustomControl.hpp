#pragma once

#include "FSM/FSMState.hpp"
#include "main.hpp"
#include "Kinematics_Controller/Kinematics_Controller.hpp"

class FSMState_CustomControl: public FSMState
{
    public:
        FSMState_CustomControl();
        void enter();
        void run();
        void exit();
};
