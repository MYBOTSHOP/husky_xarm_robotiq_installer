/**
 * @file
 * @copyright Copyright (C) 2020, Blickfeld GmbH
 * All rights reserved.
 *
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree.
 *
 * ROS1 nodelet of Blickfeld
 */

#include <memory>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "blickfeld_driver/blickfeld_driver.h"

namespace blickfeld {
namespace ros_interop {

class BlickfeldDriverNodelet : public nodelet::Nodelet {
 public:
 private:
  std::shared_ptr<BlickfeldDriver> blickfeld_driver_;

  virtual void onInit();
};

void BlickfeldDriverNodelet::onInit() {
  blickfeld_driver_.reset(new BlickfeldDriver(getNodeHandle(), getPrivateNodeHandle()));
  if (blickfeld_driver_->init() == false) {
    NODELET_ERROR("Failed to initialize BlickfeldDriverNodelet");
  }
}

}  // namespace ros_interop
}  // namespace blickfeld

PLUGINLIB_EXPORT_CLASS(blickfeld::ros_interop::BlickfeldDriverNodelet, nodelet::Nodelet)
