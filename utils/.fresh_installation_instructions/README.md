- Install Ubuntu 20.04
- Update Ubuntu and its packages

- Install ROS noetic

- Copy udev rules
    - sudo cp .clearpath_udev/41-clearpath.rules /etc/udev/rules.d/
    - roscd phidgets_api
    - sudo cp debian/udev /etc/udev/rules.d/99-phidgets.rules
    - sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger

- Copy .setup_for_etc_ros/setup.bash to /etc/ros/

- Update the motd

- Run the installation script