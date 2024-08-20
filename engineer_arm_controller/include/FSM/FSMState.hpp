#pragma once

#include "enumClass.hpp"

class FSMState
{
    public:
        FSMState(FSMStateName stateName, std::string stateNameStr);

        virtual void enter() = 0;
        virtual void run() = 0;
        virtual void exit() = 0;
        FSMStateName checkTransition(FSMCommand command);
        FSMStateName _stateName;
        std::string _stateNameStr;

    protected:
};

extern uint8_t robotarm_state_code;
extern bool robotarm_ready;
extern bool lift_ready;