/**
 * @file
 * @copyright Copyright (C) 2020, Blickfeld GmbH
 * All rights reserved.
 *
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree.
 *
 * ROS1 node of BlickfeldDriver
 */

#include "blickfeld_driver/blickfeld_driver.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "blickfeld_driver");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  using namespace blickfeld::ros_interop;
  BlickfeldDriver bf_lidar(nh, pnh);
  if (bf_lidar.init() == false) {
    return 1;
  }

  if (ros::ok() && bf_lidar.isRunning()) {
    ros::spin();
  }

  return 0;
}
