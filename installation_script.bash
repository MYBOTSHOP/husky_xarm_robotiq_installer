#!/usr/bin/env bash

function color_echo () {
    echo "$(tput setaf 1)$1$(tput sgr0)"
}

function get_package_name () {
    IFS='/'
    package=($1)
    echo ${package[-1]}
}

function build_pkg () {
    # arg1: installation directory for the package.
    # arg2: True if a non ROS package needs to be installed systemwide.        
    package_name=$(get_package_name $1)
    color_echo "Building $package_name package."                          
    cd $1 && mkdir build && cd build && cmake .. && make
    if [ $2 == true ]; then                 
        color_echo "Installing $package_name package systemwide."                          
        sudo make install
    fi            
    cd $path/$ws
}

function install_binary_packages () {
    color_echo "Installing ros packages from binaries via rosdep."
    rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y    
    color_echo "Installing build pacakges."
    sudo apt-get install ros-noetic-geodesy\
    ros-noetic-robot-upstart\
    ros-noetic-pcl-ros\
    ros-noetic-nmea-msgs\
    ros-noetic-libg2o\
    ros-noetic-robot-localization\
    ros-noetic-teb-local-planner\
    ros-noetic-map-server\
    ros-noetic-move-base\
    ros-noetic-rviz\
    ros-noetic-roslaunch\
    ros-noetic-tf\
    ros-noetic-std-msgs\
    ros-noetic-actionlib\
    ros-noetic-geometry-msgs\
    ros-noetic-move-base-msgs\
    ros-noetic-rospy\
    ros-noetic-roscpp\
    ros-noetic-soem\
    ros-noetic-topic-tools\
    ros-noetic-socketcan-interface\
    ros-noetic-moveit-visual-tools\
    ros-noetic-moveit-simple-controller-manager\
    ros-noetic-moveit-ros-planning-interface\
    ros-noetic-combined-robot-hws\
    ros-noetic-rosserial-arduino\
    ros-noetic-imu-filter-madgwick\
    ros-noetic-moveit-ros-visualization\
    ros-noetic-moveit-planners\
    ros-noetic-husky*
}

function update_ubuntu () {
    color_echo "Updating Ubuntu"
    sudo apt-get update
}

RED='\033[0;31m'
DGREEN='\033[0;32m'
GREEN='\033[1;32m'
WHITE='\033[0;37m'
BLUE='\033[1;34m'
CYAN='\033[1;36m'
NC='\033[0m' 

current_directory=$PWD
source /opt/ros/$ROS_DISTRO/setup.bash

echo -e "${DGREEN}-------------------------------------------------------------------------------"
echo -e "  __  ____     ______   ____ _______ _____ _    _  ____  _____           _____ "
echo -e " |  \/  \ \   / /  _ \ / __ \__   __/ ____| |  | |/ __ \|  __ \         / ____|"
echo -e " | \  / |\ \_/ /| |_) | |  | | | | | (___ | |__| | |  | | |__) |  _   _| |  __ "
echo -e " | |\/| | \   / |  _ <| |  | | | |  \___ \|  __  | |  | |  ___/  | | | | | |_ |"
echo -e " | |  | |  | |  | |_) | |__| | | |  ____) | |  | | |__| | |      | |_| | |__| |"
echo -e " |_|  |_|  |_|  |____/ \____/  |_| |_____/|_|  |_|\____/|_|       \__,_|\_____|"
echo -e ""                                                                                                                                         
echo -e "-------------------------------------------------------------------------------"
echo -e "Installer Husky-XARM-ROBOTIQ "                                                                                                                                         
echo -e "-------------------------------------------------------------------------------${NC}"

echo -e "${BLUE}Enter workspace name (e.g.: catkin_ws):  ${NC}" 
read -p "" ws

echo -e "${BLUE}Enter workspace path (e.g.: /home/user): ${NC}" 
read -p "" path

echo ""
echo -e "${RED}[Note]${NC} Target OS version                 >>> ${CYAN}Ubuntu 20.04${NC} "
echo -e "${RED}[Note]${NC} Target ROS version                >>> ${CYAN}ROS Noetic${NC} "
echo -e "${RED}[Note]${NC} Catkin workspace                  >>> ${CYAN}$path/$ws${NC} "
echo -e "${RED}[Note]${NC} Non-ROS package build space:      >>> ${CYAN}$path/$ws/utils${NC} "
echo -e "${RED}[Note]${NC} Robot package build space:        >>> ${CYAN}$path/$ws/src/mbs${NC} "
echo -e "${RED}[Note]${NC} Package dependencies build space: >>> ${CYAN}$path/$ws/src/third_party${NC} "
echo ""
echo -e "PRESS ${GREEN}[ENTER]${NC} TO CONTINUE THE INSTALLATION"
echo -e "IF YOU WANT TO CANCEL, PRESS ${RED}[CTRL] + [C]${NC}"
read

if [ ! -d $path/$ws ]; then
  mkdir -p $path/$ws/src $path/$ws/utils
  cd $path/$ws/src && catkin_init_workspace && cd $current_directory
fi
cd $path/$ws
sudo rm -rf devel logs build
catkin_make

# Copy MBS packages 
cp -r $current_directory/src .
cp -r $current_directory/utils .

# Lets build non-ros dependencies 
# build_pkg utils/--- true

# Binary packages installation
install_binary_packages

# Setting up env variables for the build of different packages.
export MBS_PATH=$path/$ws
echo "export MBS_PATH=$path/$ws" >> ~/.bashrc # For future use

# Lets build everything for ROS
catkin_make
source $path/$ws/devel/setup.bash
echo "source $path/$ws/devel/setup.bash" >> ~/.bashrc

echo -e "Running startup script"
rosrun mbs_husky_startup startup_script.sh

echo -e "${RED}-------------------------------------------------------------------------------"
echo -e "Installation Complete"
echo -e "-------------------------------------------------------------------------------${NC}"