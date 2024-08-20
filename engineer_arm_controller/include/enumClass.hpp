#pragma once

#include <iostream>
#include <sstream>

//TODO: add other command
enum class FSMCommand{
    POWEROFF,               //断电
    PASSIVE,                //关闭控制，进入当前状态
    CUSTOM_CONTROL,         //开始使用自定义控制器控制
    AUTO_PERLACE_1,         //机械臂自动放回第一个矿石
    AUTO_PERLACE_2,         //机械臂自动放回第二个矿石
    AUTO_TAKEOUT_1,         //机械臂自动取出第一个矿石
    AUTO_TAKEOUT_2,         //机械臂自动取出第二个矿石
    LITTLE_SOURCE_LAND,
    BIG_SOURCE_LAND,           //取大资源岛模式
    DEFEND                  //保护矿石模式
};

enum class FSMMode{
    NORMAL,
    CHANGE
};

//TODO: add other state
enum class FSMStateName{
    INVALID,                //无效状态
    POWEROFF,               //机械臂处于断电状态
    PASSIVE,                //机械臂保持当前状态
    CUSTOM_CONTROL,         //机械臂使用自定义控制器来控制
    AUTO_PERLACE_1,         //机械臂自动放回第一个矿石
    AUTO_PERLACE_2,         //机械臂自动放回第二个矿石
    AUTO_TAKEOUT_1,         //机械臂自动取出第一个矿石
    AUTO_TAKEOUT_2,         //机械臂自动取出第二个矿石
    LITTLE_SOURCE_LAND,
    BIG_SOURCE_LAND,           //取大资源岛模式
    DEFEND                  //保护矿石模式
};
