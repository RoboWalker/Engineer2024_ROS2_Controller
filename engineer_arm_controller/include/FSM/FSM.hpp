#pragma once

#include <memory>
#include "FSM/FSMState.hpp"
#include "FSMState_Passive.hpp"
#include "FSMState_CustomControl.hpp"
#include "FSMState_AutoPerlace1.hpp"
#include "FSMState_AutoPerlace2.hpp"
#include "FSMState_AutoTakeout1.hpp"
#include "FSMState_AutoTakeout2.hpp"
#include "FSMState_LittleSourceLand.hpp"
#include "FSMState_BigSourceLand.hpp"
#include "FSMState_Defend.hpp"
#include "enumClass.hpp"

//TODO: add other state
struct FSMStateList{
    FSMState *invalid;
    FSMState *passive;
    FSMState *custom_control;
    FSMState *auto_perlace_1;
    FSMState *auto_perlace_2;
    FSMState *auto_takeout_1;
    FSMState *auto_takeout_2;
    FSMState *little_source_land;
    FSMState *auto_defent_on;
    FSMState *auto_defent_off;
    FSMState *big_source_land;
    FSMState *defend;
   
    void deletePtr(){
        delete invalid;
        delete passive;
        delete custom_control;
        delete auto_perlace_1;
        delete auto_perlace_2;
        delete auto_takeout_1;
        delete auto_takeout_2;
        delete little_source_land;
        delete auto_defent_on;
        delete auto_defent_off;
        delete big_source_land;
        delete defend;
    }  
};

class FSM{
    public:
        FSM();
        ~FSM();
        void initialize();
        void run();
        FSMStateName getCurrentStateName() {return _currentState->_stateName;}
        std::string getCurrentStateNameStr() {return _currentState->_stateNameStr;}
        void setCommand(FSMCommand command) {_command = command;}

    private:
        FSMState* getNextState(FSMStateName stateName);
        bool checkSafty();
        FSMCommand _command;
        FSMState *_currentState;
        FSMState *_nextState;
        FSMStateName _nextStateName;
        FSMStateList _stateList;
        FSMMode _mode;
        long long _startTime;
        int count;
};