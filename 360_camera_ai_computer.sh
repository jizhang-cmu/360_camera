#!/bin/bash

export ROS_DOMAIN_ID=1

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

cd $SCRIPT_DIR
source ./install/setup.bash
ros2 launch receive_theta receive_theta_ai_computer.launch
