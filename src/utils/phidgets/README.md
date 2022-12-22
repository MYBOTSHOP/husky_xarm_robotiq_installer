- sudo apt-get install ros-melodic-phidgets-imu
- git clone https://github.com/ros-drivers/phidgets_drivers.git -b melodic
- rosdep install phidgets_drivers
- sudo apt-get install libusb-1.0-0 libusb-1.0-0-dev

- roscd phidgets_api
- sudo cp debian/udev /etc/udev/rules.d/99-phidgets.rules
- sudo udevadm control --reload-rules