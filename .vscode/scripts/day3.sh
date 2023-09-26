#!/bin/bash
bash .vscode/scripts/build.sh

source install/setup.bash
ros2 launch start_creating_robots gazebo.launch.py

