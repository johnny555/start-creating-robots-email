#!/bin/bash
source /opt/ros/humble/local_setup.bash
source install/setup.bash
PYTHONWARNINGS="ignore:setup.py install is deprecated::setuptools.command.install,ignore:easy_install command is deprecated::setuptools.command.easy_install"; 
export PYTHONWARNINGS

colcon build 
