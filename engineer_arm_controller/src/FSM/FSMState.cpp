#include "FSM/FSMState.hpp"

uint8_t robotarm_state_code = 0x00;
bool robotarm_ready = false;
bool lift_ready = false;

FSMState::FSMState(FSMStateName stateName, std::string stateNameStr):
            _stateName(stateName), _stateNameStr(stateNameStr){}

FSMStateName FSMState::checkTransition(FSMCommand command)
{
    //DEFEND模式可以打断任何其他模式
    if (command == FSMCommand::DEFEND)
    {
        robotarm_state_code = 0x00; //该模式不是自动放置模式，设置为0意为可以被打断
        return FSMStateName::DEFEND;
    }
    

    //在自动化操作的时候禁止被常规命令打断
    if (robotarm_state_code == 0x00)
    {
        switch (command)
        {
            case FSMCommand::CUSTOM_CONTROL:
                robotarm_state_code = 0x00;
                return FSMStateName::CUSTOM_CONTROL;
                break;
            case FSMCommand::AUTO_PERLACE_1:
                robotarm_state_code = 0x01;
                return FSMStateName::AUTO_PERLACE_1;
                break;
            case FSMCommand::AUTO_PERLACE_2:
                robotarm_state_code = 0x02;
                return FSMStateName::AUTO_PERLACE_2;
                break;
            case FSMCommand::AUTO_TAKEOUT_1:
                robotarm_state_code = 0x03;
                return FSMStateName::AUTO_TAKEOUT_1;
                break;
            case FSMCommand::AUTO_TAKEOUT_2:
                robotarm_state_code = 0x04;
                return FSMStateName::AUTO_TAKEOUT_2;
                break;
            case FSMCommand::LITTLE_SOURCE_LAND:
                robotarm_state_code = 0x05;
                return FSMStateName::LITTLE_SOURCE_LAND;
                break;
            case FSMCommand::BIG_SOURCE_LAND:
                robotarm_state_code = 0x00; //该模式不是自动放置模式，设置为0意为可以被打断
                return FSMStateName::BIG_SOURCE_LAND;
                break;
            case FSMCommand::PASSIVE:
                robotarm_state_code = 0x00;
                return FSMStateName::PASSIVE;
                break;
            default:
                robotarm_state_code = 0x00;
                return FSMStateName::INVALID;
                break;
        }
    }
    return this->_stateName;
    
}
