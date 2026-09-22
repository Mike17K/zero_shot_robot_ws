#!/bin/bash
sudo apt update && sudo apt install -y ros-jazzy-moveit-msgs ros-jazzy-rmw-cyclonedds-cpp terminator xdotool xclip

apt install -y curl lsb-release gnupg
curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
  -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  > /etc/apt/sources.list.d/gazebo-stable.list
apt update
apt install -y python3-gz-transport13 python3-gz-msgs10

echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
echo "export CYCLONEDDS_URI=file:///workspaces/isaac_ros-dev/cyclonedds.xml" >> ~/.bashrc

echo "export ROS_DOMAIN_ID=40" >> ~/.bashrc
source ~/.bashrc
make rosdeps
make
source install/setup.bash