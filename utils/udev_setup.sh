#!/usr/bin/env bash

echo "This script copies udev rules to /etc/udev/rules.d to facilitate Arduino Nano communication"
sudo cp mbs_udev/* /etc/udev/rules.d
echo "Restarting udev"
sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger
