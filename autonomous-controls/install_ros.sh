#!/bin/bash
set -e

echo "Installing prerequisites..."
sudo apt install -y software-properties-common curl

echo "Adding Universe repository..."
sudo add-apt-repository -y universe

echo "Adding ROS 2 GPG key..."
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "Adding ROS 2 repository..."
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "Updating package lists..."
sudo apt update

echo "Installing ROS 2 Humble Desktop and development tools..."
sudo apt install -y ros-humble-desktop ros-dev-tools

echo "Installing Gazebo Fortress and ROS-Ignition bridge packages..."
sudo apt install -y ros-humble-ros-ign-gazebo ros-humble-ros-ign-bridge ros-humble-ros-ign-interfaces

echo "ROS 2 Installation Complete!"
echo "Please restart your terminal or run 'source /opt/ros/humble/setup.bash' to start using ROS 2."
