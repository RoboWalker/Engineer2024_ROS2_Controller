#include "FSM/FSM.hpp"
#include <iostream>

FSM::FSM()
{
    _stateList.invalid = nullptr;

    //TODO: add other state
    _stateList.passive = new FSMState_Passive();
    _stateList.custom_control = new FSMState_CustomControl();
    _stateList.auto_perlace_1 = new FSMState_AutoPerlace1();
    _stateList.auto_perlace_2 = new FSMState_AutoPerlace2();
    _stateList.auto_takeout_1 = new FSMState_AutoTakeout1();
    _stateList.auto_takeout_2 = new FSMState_AutoTakeout2();
    _stateList.little_source_land = new FSMState_LittleSourceLand;
    _stateList.big_source_land = new FSMState_BigSourceLand();
    _stateList.defend = new FSMState_Defend();

    initialize();
    std::cout << "FSM initialized" << std::endl;
}

FSM::~FSM(){
    _stateList.deletePtr();
}

void FSM::initialize()
{
    count = 0;
    _command = FSMCommand::CUSTOM_CONTROL;
    _currentState = _stateList.custom_control;
    _currentState -> enter();
    _nextState = _currentState;
    _mode = FSMMode::NORMAL;
}

void FSM::run()
{
    if(_mode == FSMMode::NORMAL)
    {
        _currentState->run();
        _nextStateName = _currentState->checkTransition(this->_command);
        if(_nextStateName != _currentState->_stateName)
        {
            _mode = FSMMode::CHANGE;
            _nextState = getNextState(_nextStateName);
        }
    }

    if(_mode == FSMMode::CHANGE)
    {
        // std::cout << "change state" << std::endl;
        _currentState->exit();
        _currentState = _nextState;
        _currentState->enter();
        _mode = FSMMode::NORMAL;
        robotarm_ready = false;
        std::cout << "FSM state changed to " << _currentState->_stateNameStr << std::endl;
        _currentState->run();       
    }

    count++;
}

//TODO: add other state
FSMState* FSM::getNextState(FSMStateName nextstateName)
{
    switch(nextstateName)
    {
        case FSMStateName::INVALID:
            return _stateList.invalid;
            break;
        case FSMStateName::PASSIVE:
            return _stateList.passive;
            break;
        case FSMStateName::CUSTOM_CONTROL:
            return _stateList.custom_control;
            break;
        case FSMStateName::AUTO_PERLACE_1:
            return _stateList.auto_perlace_1;
            break;
        case FSMStateName::AUTO_PERLACE_2:
            return _stateList.auto_perlace_2;
            break;
        case FSMStateName::AUTO_TAKEOUT_1:
            return _stateList.auto_takeout_1;
            break;
        case FSMStateName::AUTO_TAKEOUT_2:
            return _stateList.auto_takeout_2;
            break;
        case FSMStateName::LITTLE_SOURCE_LAND:
            return _stateList.little_source_land;
            break;
        case FSMStateName::BIG_SOURCE_LAND:
            return _stateList.big_source_land;
            break;
        case FSMStateName::DEFEND:
            return _stateList.defend;
            break;
        default:
            return _stateList.invalid;
            break;
    }
}