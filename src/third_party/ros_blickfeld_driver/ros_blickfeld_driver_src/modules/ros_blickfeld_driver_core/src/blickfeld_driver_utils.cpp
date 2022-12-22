/**
 * @file
 * @copyright Copyright (C) 2020, Blickfeld GmbH
 * All rights reserved.
 *
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree.
 *
 * Implementation for Blickfeld Driver Utilities
 */

#include <math.h>

#include <cv_bridge/cv_bridge.h>

#include "blickfeld_driver_core/blickfeld_driver_utils.h"

namespace blickfeld {
namespace ros_interop {
namespace driver_utilities {

void cvMatToRosImageMessage(const cv::Mat &cv_mat_image, const std::string &encoding, const std::string &frame_id,
                            SensorMsgImage &ros_image) {
  MsgHeader header;
  header.frame_id = frame_id;
  cv_bridge::CvImage(header, encoding, cv_mat_image).toImageMsg(ros_image);
}

int getFlatPixelCoordinates(int point_index, const blickfeld::protocol::data::Scanline &scanline,
                            const ScanPatternT &scan_pattern, const ProjectionType &projection_type) {
  const auto &current_point = scanline.points(point_index);

  float vertical_fov = scan_pattern.vertical().fov();
  int points_per_scanline = scanline.points_size();
  const auto &scanline_center_point = scanline.points(static_cast<float>(points_per_scanline) / 2);
  float vertical_angle;
  if (projection_type == ProjectionType::AnglePreserving) {
    vertical_angle = (current_point.direction().elevation() + vertical_fov / 2);
  } else if (projection_type == ProjectionType::ScanlinePreserving) {
    vertical_angle = (scanline_center_point.direction().elevation() + vertical_fov / 2);
  }

  size_t num_scanlines = getNumberOfScanlines(scan_pattern);
  float vertical_angel_spacing = vertical_fov / static_cast<float>(num_scanlines);
  /// HINT: Compute the row index from the center point of the scanline
  int row = floor((vertical_fov - vertical_angle) / vertical_angel_spacing);
  /// HINT: Compute the column index from the horizontal angle (mirror timestamp offset compensated)
  float horizontal_fov = scan_pattern.horizontal().fov();
  float horizontal_angle = (current_point.direction().azimuth() + horizontal_fov / 2);
  float horizontal_angle_spacing = scan_pattern.pulse().angle_spacing();
  int col = floor((horizontal_angle) / horizontal_angle_spacing);
  if (col < 0 || col > points_per_scanline - 1) {
    return -1;
  } else {
    return row * points_per_scanline + col;
  }
}

size_t getNumberOfScanlines(const ScanPatternT &scan_pattern) {
  size_t num_scanlines;
  const auto &frame_mode = scan_pattern.pulse().frame_mode();
  switch (frame_mode) {
    case ScanPatternT::Pulse::FrameMode::ScanPattern_Pulse_FrameMode_ONLY_UP:
      num_scanlines = scan_pattern.vertical().scanlines_up();
      break;
    case ScanPatternT::Pulse::FrameMode::ScanPattern_Pulse_FrameMode_ONLY_DOWN:
      num_scanlines = scan_pattern.vertical().scanlines_down();
      break;
    case ScanPatternT::Pulse::FrameMode::ScanPattern_Pulse_FrameMode_COMBINE_UP_DOWN:
      num_scanlines = scan_pattern.vertical().scanlines_up() + scan_pattern.vertical().scanlines_down();
      break;
    default:
      throw std::invalid_argument("Cannot compute a 2D image for this frame mode!");
  }
  return num_scanlines;
}

cv::Size getImageSize(const ScanPatternT &scan_pattern) {
  size_t num_scanlines = getNumberOfScanlines(scan_pattern);
  double angle_spacing = static_cast<double>(scan_pattern.pulse().angle_spacing());
  double horizontal_fov = static_cast<double>(scan_pattern.horizontal().fov());
  uint32_t img_rows = static_cast<uint32_t>(num_scanlines);
  uint32_t img_cols = static_cast<uint32_t>(std::ceil(horizontal_fov / angle_spacing));
  return cv::Size(img_cols, img_rows);
}

ImuAccelerationUnit imuAccelerationUnitFromString(const std::string &imu_acceleration_unit_string) {
  if (imu_acceleration_unit_string == "g") {
    return ImuAccelerationUnit::G;
  } else if (imu_acceleration_unit_string == "meters_per_second_squared") {
    return ImuAccelerationUnit::MetersPerSecondSquared;
  } else {
    throw std::invalid_argument("imu acceleration unit unknown: " + imu_acceleration_unit_string);
  }
}

std::string imuAccelerationUnitToString(const ImuAccelerationUnit &imu_acceleration_unit) {
  switch (imu_acceleration_unit) {
    case ImuAccelerationUnit::G:
      return "g";
    case ImuAccelerationUnit::MetersPerSecondSquared:
      return "meters_per_second_squared";
    default:
      throw std::invalid_argument("imu acceleration unit unknown");
  }
}

ProjectionType projectionTypeFromString(const std::string &projection_type_string) {
  if (projection_type_string == "angle_preserving") {
    return ProjectionType::AnglePreserving;
  } else if (projection_type_string == "scanline_preserving") {
    return ProjectionType::ScanlinePreserving;
  } else {
    throw std::invalid_argument("ProjectionType unknown: " + projection_type_string);
  }
}

std::string projectionTypeToString(const ProjectionType &projection_type) {
  switch (projection_type) {
    case ProjectionType::AnglePreserving:
      return "angle_preserving";
    case ProjectionType::ScanlinePreserving:
      return "scanline_preserving";
    default:
      throw std::invalid_argument("ProjectionType unknown");
  }
}

bool publishImage(const ImageOptions &image_options) {
  return image_options.range_image || image_options.ambient_image || image_options.intensity_image;
}

ReturnOptions returnOptionsFromString(const std::string &string) {
  if (string == "strongest") {
    return ReturnOptions::Strongest;
  } else if (string == "closest") {
    return ReturnOptions::Closest;
  } else if (string == "farthest") {
    return ReturnOptions::Farthest;
  } else if (string == "all") {
    return ReturnOptions::All;
  } else {
    throw std::invalid_argument("return options unknown: " + string);
  }
}

std::string returnOptionsToString(const ReturnOptions &return_options) {
  switch (return_options) {
    case ReturnOptions::Strongest:
      return "strongest";
    case ReturnOptions::Closest:
      return "closest";
    case ReturnOptions::Farthest:
      return "farthest";
    case ReturnOptions::All:
      return "all";
    default:
      throw std::invalid_argument("returns option unknown");
  }
}
}  // namespace driver_utilities
}  // namespace ros_interop
}  // namespace blickfeld
