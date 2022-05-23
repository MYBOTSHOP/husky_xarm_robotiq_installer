# Pass through to the main ROS workspace of the system.
source /opt/ros/noetic/setup.bash
source /home/administrator/catkin_ws/devel/setup.bash

# Mark location of self so that robot_upstart knows where to find the setup file.
export ROBOT_SETUP=/etc/ros/setup.bash

# Setup robot upstart jobs to use the IP from the network bridge.
# export ROBOT_NETWORK=enp1s0
export HUSKY_LOGITECH=1
export HUSKY_IMU_RPY='0 0 1.5708'

# Changed Husky URDF to include MYBOTSHOP updates.
export HUSKY_URDF_EXTRAS=/home/administrator/catkin_ws/src/mbs/mbs_husky_description/urdf/accessories.urdf.xacro

