/**
 * @file
 * @copyright Copyright (C) 2020, Blickfeld GmbH
 * All rights reserved.
 *
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree.
 *
 * Header for class BlickfeldDriverPointCloudParser & ReturnSelector
 */

#pragma once

#include <string>

#include <cv_bridge/cv_bridge.h>

#include <blickfeld/scanner.h>

#include "blickfeld_driver/blickfeld_driver_ros_types.h"
#include "blickfeld_driver_core/blickfeld_driver_types.h"
#include "blickfeld_driver_core/blickfeld_driver_utils.h"

namespace blickfeld {
namespace ros_interop {

/**
 * @brief This class converts point cloud frame to ros point cloud and image (generated from point cloud)
 * messages
 *
 */
class BlickfeldDriverPointCloudParser {
 public:
  /**
   * @brief Construct a new Stream
   *
   * @param[in] point_cloud_options defines the fields to configure point cloud
   * @param[in] image_options defines the fields to publish images from point cloud
   */
  BlickfeldDriverPointCloudParser(const PointCloudOptions& point_cloud_options = PointCloudOptions(),
                                  const ImageOptions& image_options = ImageOptions());

  /**
   * @brief parseFrame parses a blickfeld::protocol::data::Frame message to a PointCloud2 message, and 3
   * SensorMsgImage for images
   *
   * @param[in] frame blickfeld::protocol::data::Frame
   * @return FrameStreamOutput parsed data from frame
   */
  FrameStreamOutput parseFrame(const FrameT& frame);

  /**
   * @brief Extracts the scan pattern information from a received blickfeld::protocol::data::Frame and returns it as
   * DiagnosticStatusPtr message
   *
   * @param[in] frame blickfeld::protocol::data::Frame
   * @param[in] host host name
   * @return DiagnosticStatusPtr
   */
  DiagnosticStatusPtr parseStatus(const blickfeld::protocol::data::Frame& frame, const std::string& host);

  /**
   * @brief getPointCloudOptions returns the point cloud options
   *
   * @return PointCloudOptions
   */
  PointCloudOptions getPointCloudOptions() const;

  /**
   * @brief getImageOptions returns the Image Options
   *
   * @return ImageOptions
   */
  ImageOptions getImageOptions() const;

 private:
  /**
   * @brief Create a new PointField
   *
   * @param[in] name the name of the newly created field
   * @param[in] offset the offset defined by previously created fields
   * @param[in] datatype the datatype of the field
   * @return PointField
   */
  PointField createField(std::string const& name, uint32_t offset, uint8_t datatype);

  /**
   * @brief getPointCloud2FieldIndex gets the index of a specified field (i.e., dimension/channel)
   *
   * @param[in] point cloud
   * @param[in] field_name the string defining of the field name
   * @return field index
   */
  static inline int getPointCloud2FieldIndex(const PointCloud2Ptr& point_cloud, const std::string& field_name) {
    for (size_t field_index = 0; field_index < point_cloud->fields.size(); ++field_index) {
      if (point_cloud->fields[field_index].name == field_name) {
        return static_cast<int>(field_index);
      }
    }
    return -1;
  };

  /**
   * @brief Assigns a value to a field in a PointCloud2 message
   *
   * @tparam ValueT The type of the value
   * @param[in] point_cloud PointCloud2 message
   * @param[in] point_index current index in the message
   * @param[in] field_name name of the fields
   * @param[in] value the value to be assigned
   */
  template <typename ValueT>
  void assignField(const PointCloud2Ptr& point_cloud, size_t point_index, std::string field_name, const ValueT& value) {
    uint32_t field_index = getPointCloud2FieldIndex(point_cloud, field_name);
    *reinterpret_cast<ValueT*>(
        &point_cloud->data[point_index * point_cloud->point_step + point_cloud->fields[field_index].offset]) =
        static_cast<ValueT>(value);
  }

  PointCloudOptions point_cloud_options_;
  std::vector<PointField> point_fields_;
  size_t point_size_;

  ImageOptions image_options_;

  /// Images stored in row-major format
  std::vector<float> range_image_sorted_pixels_;
  std::vector<float> ambient_image_sorted_pixels_;
  std::vector<float> intensity_image_sorted_pixels_;
  std::vector<int32_t> point_id_image_;

  static constexpr auto cv_image_encoding_ = "32FC1";
  static constexpr auto cv_image_encoding_32_bit_int_ = "CV_32SC1";
};

/**
 * @brief This class implements a special iterator, which iterates through the pulse return list, and returns a pulse
 * return which specified by @p return_options
 *
 */
class ReturnSelector {
 public:
  /**
   * @brief Construct a new Return Selector
   *
   * @param[in] returns STL compatible container of returns
   * @param[in] return_options the desired return option
   */
  ReturnSelector(const PointReturnsT& returns, const ReturnOptions& return_options);

  /**
   * @brief STL compatible begin() function
   *
   * @return iterator to first element
   */
  const ReturnSelector& begin() const;

  /**
   * @brief STL compatible end() function
   *
   * @return iterator to last element
   */
  const ReturnSelector& end() const;

  /**
   * @brief Operator for checking if the end of iteration was reached
   *
   * @return bool
   */
  bool operator!=(const ReturnSelector&) const;

  /**
   * @brief Operator for incrementing the current iterator
   *
   */
  void operator++();

  /**
   * @brief Operator for dereferencing the iterator
   *
   * @return blickfeld::protocol::data::Point_Return
   */
  blickfeld::protocol::data::Point_Return operator*() const;

 private:
  PointReturnsT::const_iterator current_iterator;
  PointReturnsT::const_iterator end_iterator;
};

}  // namespace ros_interop
}  // namespace blickfeld
