#!/bin/bash
cd /home/engineer
source env_setup

cd /home/engineer/engineer_ws
ros2 launch engineer_hw hw.launch.py
