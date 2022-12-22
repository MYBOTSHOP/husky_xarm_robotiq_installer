/**
 * @file
 * @copyright Copyright (C) 2020, Blickfeld GmbH
 * All rights reserved.
 *
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree.
 *
 * Implementation for class BlickfeldDriverImuParser
 */

#include "blickfeld_driver_core/blickfeld_driver_imu_parser.h"

namespace blickfeld {
namespace ros_interop {

BlickfeldDriverImuParser::BlickfeldDriverImuParser(const ImuAccelerationUnit& acceleration_unit,
                                                   const std::string& frame_id)
    : acceleration_unit_(acceleration_unit), frame_id_(frame_id) {}

std::vector<std::pair<SensorMsgImuPtr, time_t>> BlickfeldDriverImuParser::parseImuSamples(
    const blickfeld::protocol::data::IMU& imu) {
  std::vector<std::pair<SensorMsgImuPtr, time_t>> imu_msgs;
  for (auto sample : imu.samples()) {
    imu_msgs.emplace_back(std::pair<SensorMsgImuPtr, time_t>());
    imu_msgs.back().first = SensorMsgImuPtr(new SensorMsgImu());
    imu_msgs.back().second = imu.start_time_ns() + sample.start_offset_ns();

    imu_msgs.back().first->header.frame_id = frame_id_;

    imu_msgs.back().first->angular_velocity.x = sample.angular_velocity(0);
    imu_msgs.back().first->angular_velocity.y = sample.angular_velocity(1);
    imu_msgs.back().first->angular_velocity.z = sample.angular_velocity(2);

    const auto unit_conversion = acceleration_unit_ == ImuAccelerationUnit::G ? 1 : g_to_meters_per_second_squared_;
    imu_msgs.back().first->linear_acceleration.x = unit_conversion * sample.acceleration(0);
    imu_msgs.back().first->linear_acceleration.y = unit_conversion * sample.acceleration(1);
    imu_msgs.back().first->linear_acceleration.z = unit_conversion * sample.acceleration(2);
  }
  return imu_msgs;
}

TransformMsgStampedPtr BlickfeldDriverImuParser::parseImuStatic(
    const google::protobuf::RepeatedField<float>& imu_acceleration) {
  const Eigen::Vector3f acceleration_calibration_unit(0, 0, -1);
  Eigen::Vector3f imu_acceleration_eigen(imu_acceleration.data());
  imu_acceleration_eigen.normalize();
  const Eigen::Vector3f rotation_vector = acceleration_calibration_unit.cross(imu_acceleration_eigen);
  const float rotation_vector_normal = rotation_vector.norm();
  Eigen::Matrix3f imu_static_rotation_offset = Eigen::Matrix3f::Identity();
  if (rotation_vector_normal < imu_minimum_error_allowed_) {
    imu_static_rotation_offset = -imu_static_rotation_offset;
  } else {
    Eigen::Matrix3f axis_cross;
    axis_cross << 0, -rotation_vector(2), rotation_vector(1),
        /*|*/ rotation_vector(2), 0, -rotation_vector(0),
        /*|*/ -rotation_vector(1), rotation_vector(0), 0;
    imu_static_rotation_offset = Eigen::Matrix3f::Identity() + axis_cross +
                                 (axis_cross * axis_cross) *
                                     (1 - acceleration_calibration_unit.dot(imu_acceleration_eigen)) /
                                     pow(rotation_vector_normal, 2);
  }
  tf2::Vector3 frame_translation(0, 0, 0);
  Eigen::Quaternionf eigen_rotation(imu_static_rotation_offset);
  tf2::Quaternion frame_orientation(eigen_rotation.x(), eigen_rotation.y(), eigen_rotation.z(), eigen_rotation.w());
  TransformMsgStampedPtr static_transform_stamped = TransformMsgStampedPtr(new TransformMsgStamped());
  static_transform_stamped->header.frame_id = frame_id_;
  static_transform_stamped->child_frame_id = frame_id_ + "_imu";
  static_transform_stamped->transform = tf2::toMsg(tf2::Transform(frame_orientation, frame_translation));
  return static_transform_stamped;
}

}  // namespace ros_interop
}  // namespace blickfeld
