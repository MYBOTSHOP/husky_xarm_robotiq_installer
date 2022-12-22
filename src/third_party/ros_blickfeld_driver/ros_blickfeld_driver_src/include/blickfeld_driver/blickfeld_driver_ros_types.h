/**
 * @file
 * @copyright Copyright (C) 2020, Blickfeld GmbH
 * All rights reserved.
 *
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree.
 *
 * Header for class blickfeld driver ros types
 */

#pragma once

#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace blickfeld {
namespace ros_interop {

using PointCloud2 = sensor_msgs::PointCloud2;
using PointCloud2Ptr = sensor_msgs::PointCloud2::Ptr;
using DiagnosticStatus = diagnostic_msgs::DiagnosticStatus;
using DiagnosticStatusPtr = diagnostic_msgs::DiagnosticStatus::Ptr;
using DiagnosticKeyValue = diagnostic_msgs::KeyValue;
using PointField = sensor_msgs::PointField;
using SensorMsgImage = sensor_msgs::Image;
using SensorMsgImagePtr = sensor_msgs::Image::Ptr;
using MsgHeader = std_msgs::Header;
using MsgHeaderPtr = std_msgs::Header::Ptr;
using SensorMsgImu = sensor_msgs::Imu;
using SensorMsgImuPtr = sensor_msgs::Imu::Ptr;
using TransformMsgStamped = geometry_msgs::TransformStamped;
using TransformMsgStampedPtr = geometry_msgs::TransformStamped::Ptr;

}  // namespace ros_interop
}  // namespace blickfeld
