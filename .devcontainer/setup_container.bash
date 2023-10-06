#!/bin/bash

# Add kisak-mesa for gazebo hotfixes. https://github.com/gazebosim/gz-sim/issues/920
add-apt-repository ppa:kisak/kisak-mesa

apt-get update
apt-get upgrade -y
apt install -y mesa-utils

# ------------------------------------------------------
# Now download and install openvdb so STVL plugin works. 
# https://github.com/SteveMacenski/spatio_temporal_voxel_layer/issues/232
# ------------------------------------------------------
mkdir openvdb
cd openvdb

wget https://github.com/wyca-robotics/openvdb/releases/download/v8.2.0-debian/libopenvdb-dev_8.2.0-1-wyca_amd64.deb
wget https://github.com/wyca-robotics/openvdb/releases/download/v8.2.0-debian/libopenvdb-doc_8.2.0-1-wyca_all.deb
wget https://github.com/wyca-robotics/openvdb/releases/download/v8.2.0-debian/libopenvdb-tools_8.2.0-1-wyca_amd64.deb
wget https://github.com/wyca-robotics/openvdb/releases/download/v8.2.0-debian/libopenvdb8.2_8.2.0-1-wyca_amd64.deb
wget https://github.com/wyca-robotics/openvdb/releases/download/v8.2.0-debian/python3-openvdb_8.2.0-1-wyca_amd64.deb

dpkg -i libopenvdb8.2_8.2.0-1-wyca_amd64.deb
dpkg -i libopenvdb-dev_8.2.0-1-wyca_amd64.deb
dpkg -i libopenvdb-tools_8.2.0-1-wyca_amd64.deb
dpkg -i python3-openvdb_8.2.0-1-wyca_amd64.deb
dpkg -i libopenvdb-doc_8.2.0-1-wyca_all.deb

cd .. 

# -----------------------------------------------------

# Now install Gazebo ign
apt-get install -y ros-humble-ros-gz

source /opt/ros/humble/local_setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -y -r

# Create user ros, and allow it to install stuff. 
adduser --disabled-password --gecos "docker user" ros
echo 'ros ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/ros && chmod 0440 /etc/sudoers.d/ros
chown -R ros /workspace

# Install some extra stuff

apt-get install -y ros-humble-rqt-robot-steering ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup

# Do an initial build

colcon build --symlink-install

# Get python deps

sudo apt install python3-pip
pip install black

# Make it so that sourcing happens automatically
echo "source /opt/ros/humble/setup.bash" >> /home/ros/.bashrc
echo "source /workspace/install/setup.bash" >> /home/ros/.bashrc

# Suppress deprecated setuptools warning
echo "PYTHONWARNINGS=\"ignore:setup.py install is deprecated::setuptools.command.install,ignore:easy_install command is deprecated::setuptools.command.easy_install\"; export PYTHONWARNINGS" >> /home/ros/.bashrc

# Add GAZEBO path so we can easily include models
echo "export IGN_GAZEBO_RESOURCE_PATH=/workspace/install/start_creating_robots/share/start_creating_robots/worlds_and_models/" >> /home/ros/.bashrc
