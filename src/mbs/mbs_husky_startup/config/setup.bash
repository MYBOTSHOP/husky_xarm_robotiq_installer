# Pass through to the main ROS workspace of the system.
source /opt/ros/noetic/setup.bash
source /home/administrator/catkin_ws/devel/setup.bash

# Mark location of self so that robot_upstart knows where to find the setup file.
export ROBOT_SETUP=/etc/ros/setup.bash

######
export HUSKY_LOGITECH=1
export HUSKY_IMU_RPY='3.1416 0 0'

# Changed Husky URDF to include MYBOTSHOP updates.
export HUSKY_URDF_EXTRAS=/home/administrator/catkin_ws/src/mbs/mbs_husky_description/urdf/accessories.urdf.xacro

# Lidars
export MBS_OUSTER_IP=192.168.131.1
export MBS_OUSTER_HOST=192.168.131.3
export FRONT_LASER_IP=192.168.131.2

# ROS Environment Variables
export ROS_MASTER_URI=http://192.168.131.1:11311/
export ROS_IP=192.168.131.1
export ROS_HOSTNAME=192.168.131.1
