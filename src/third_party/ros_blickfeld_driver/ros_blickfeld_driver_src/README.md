# Blickfeld ROS package

This package provides an ROS node and a nodelet for publishing PointCloud2 messages from Blickfeld LiDAR devices.  
The driver is available under https://www.blickfeld.com/resources

## Supported devices

The Blickfeld ROS driver supports all currently available Blickfeld LiDARs, such as Cube 1 and Cube Range 1.

## Supported ROS Distributions

The Blickfeld ROS driver supports the following distributions

- ROS Melodic Morenia (Ubuntu 18.04)
- ROS Noetic Ninjemys (Ubuntu 20.04)

## obtain

```
git clone --recursive <URL>
```

This will clone ros_blickfeld_driver.

## Dependencies

As submodule ros_blickfeld_driver depends on:

- ros_blickfeld_driver_core v0.2.3 (resides in modules directory)

To install the Blickfeld ROS driver, please make sure that the following dependencies are installed on your system.

- [blickfeld-scanner-library (BSL) ](https://docs.blickfeld.com/cube/latest/external/blickfeld-scanner-lib/install.html) with system-wide protobuf installation. Minimum required version is **BSL Version 2.18.2**

- [ROS Melodic Installation](http://wiki.ros.org/melodic/Installation/Ubuntu) with Ubuntu 18.04 **/**
  [ROS Noetic Installation](http://wiki.ros.org/noetic/Installation/Ubuntu) with Ubuntu 20.04

- [diagnostic_updater](https://index.ros.org/p/diagnostic_updater/) can be acquired via your distribution's package manager, ${ROS_DISTRO} should be your ROS version. (e.g. melodic or noetic)

      $ sudo apt install ros-${ROS_DISTRO}-diagnostic-updater
      $ sudo apt install ros-${ROS_DISTRO}-diagnostic-msgs

  or via

      $ rosdep update
      $ rosdep install --from-paths src --ignore-src -r -y

## Build

Before building ensure that your ROS DISTRO is sourced. You will need to run this command on every new shell you open to have access to the ROS commands. ${ROS_DISTRO} should be your ROS version. (e.g. melodic or noetic)

    $ source /opt/ros/${ROS_DISTRO}/setup.bash

To build the ROS driver, request the ROS driver from the Blickfeld Sales department and decompress the archive into your ROS catkin workspace.

Run

    $ catkin build

or respectively

    $ catkin build blickfeld_driver

You may use `catkin_make` instead of `catkin build` as well.

## Running the Blickfeld ROS node

You can start the Blickfeld driver as either a node or a nodelet. For the node, you can use the launch file

    $ live_scanner.launch

and for the nodelet, the

    $ live_scanner_nodelet.launch

is provided.

You can launch the ROS driver in a **DHCP** controlled Network by providing the Hostname of the LiDAR device (you can check and set the Hostname in the WebGUI of the device) (**Make sure you have sourced your workspace**) e.g.:

    $ roslaunch blickfeld_driver live_scanner.launch host:=cube-0175

or respectively

    $ roslaunch blickfeld_driver live_scanner_nodelet.launch host:=cube-0175

You should now be able to see a PointCloud2 in Rviz on the `/bf_lidar/points_raw` topic.

If you connect the LiDAR directly to your PC or don't have a **DHCP** Server you can launch the driver by using the IP configured in the WebGUI of the Device or the Fallback-IP of the Device  
(**Make sure you sourced your workspace and adjusted your network-settings accordingly**) e.g.:

    $ roslaunch blickfeld_driver live_scanner.launch host:=cube-0175

or respectively

    $ roslaunch blickfeld_driver live_scanner_nodelet.launch host:=cube-0175

By default we do not start RViz, set the rviz parameter to true if you want to start RViz with the Blickfeld ROS driver:

    $ roslaunch blickfeld_driver live_scanner.launch host:=cube-0175 rviz:=true

See the table with all available Parameters and there default configuration.

### Parameters

Both launch files have arguments you can use:

| Argument                                         | Default                                | Note                                                                                                                                                                                                                                                                                                                                            |
| ------------------------------------------------ | -------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **host** (required)                              |                                        | The host name or the IP of the device you want to publish the point clouds from, e.g., `cube-0175`.                                                                                                                                                                                                                                             |
| lidar_frame_id                                   | `lidar`                                | The ROS TF where the point cloud should be in.                                                                                                                                                                                                                                                                                                  |
| node_name                                        | `bf_lidar`                             | Name of this ROS node.                                                                                                                                                                                                                                                                                                                          |
| remap                                            | `true`                                 | Remap this node’s input/output topics to commonly used ones. <br/>If `false`, canonical names in the form of `$(arg node_name)/foo*(in/out)`are used, depending on whether a topic is an input or and output topic.                                                                                                                             |
| rviz                                             | `false`                                | Start rviz if this argument is true.                                                                                                                                                                                                                                                                                                            |
| publish_ambient_light                            | `false`                                | Set to true if the PointCloud2 message should contain the ambient light level.                                                                                                                                                                                                                                                                  |
| publish_explicit_range                           | `false`                                | Set to true if the PointCloud2 message should contain the`range` field.                                                                                                                                                                                                                                                                         |
| publish_intensities                              | `true`                                 | Set to true if the PointCloud2 message should contain intensities.                                                                                                                                                                                                                                                                              |
| publish_no_return_points                         | `false`                                | Set to true if the PointCloud2 message should contain points in a given range for pulses without a return.                                                                                                                                                                                                                                      |
| publish_point_id                                 | `true`                                 | Add the `scanline_id` field, the `scanline_point_index` field (= the point’s number in the scan line), and the `point_id` (= frame-global point ID) to PointCloud2 message. <br/> If the device is configured to return multiple return points (multiple reflections), all these three IDs will be identical; only the `return_id` will differ. |
| publish_point_time_offset                        | `false`                                | If `true`, the PointCloud2 message will contain the timestamp field per point to represent the time offset from the start of the frame.                                                                                                                                                                                                         |
| no_return_point_range                            | `1.0`                                  | Dummy range for points of pulses without a return.                                                                                                                                                                                                                                                                                              |
| returns_publishing_options                       | `strongest`                            | different options to publish multiple returns, possible values are: `strongest` `closest` `farthest` and `all`, in case of `all`, the field `return_id` (ID for each returned point) is added to the point cloud.                                                                                                                               |
| projection_type                                  | `angle_preserving`                     | different options to project the data onto a 2D image, possible values are: `angle_preserving` or `scanline_preserving`. These indicate if a correct spherical projection is desired, or each row of the image should correspond to one scanline.                                                                                               |
| publish_ambient_image                            | `false`                                | Enables publishing of am ambient light image.                                                                                                                                                                                                                                                                                                   |
| publish_intensity_image                          | `false`                                | Enables publishing of an intensity image.                                                                                                                                                                                                                                                                                                       |
| publish_range_image                              | `false`                                | Enables publishing of a range image.                                                                                                                                                                                                                                                                                                            |
| publish_point_id_image                           | `true`                                 | Enables publishing of an image with the point ids for every pixel. This image is not meant to be visualized in rviz.                                                                                                                                                                                                                                                                            |
| imu_acceleration_unit                            | `g`                              | different imu acceleration units: `g`, `meters_per_second_squared`.                                                                                                                                                                                                                                                                        |
| publish_imu                                      | `false`                                | Enables publishing of IMU data in burst mode.                                                                                                                                                                                                                                                                                                   |
| publish_imu_static_tf_at_start                   | `false`                                | Call "Publish IMU tf transform" service once at the beginning.                                                                                                                                                                                                                                                                                  |
| use_lidar_timestamp                              | `true`                                 | Set to true if the timestamp in the ROS point cloud message should be generated from the device timestamp; otherwise the timestamp will be the ROS time when the frame was received on ROS.                                                                                                                                                     |
| use_background_subtraction                       | `false`                                | Enables on-device background subtraction.                                                                                                                                                                                                                                                                                                       |
| use_neighbor_filter                              | `false`                                | Enables on-device neighbor filter.                                                                                                                                                                                                                                                                                                              |
| background_subtraction_exponential_decay_rate    | `0.005`                                | Controls how fast objects switch between foreground and background. Exponential decay factor.                                                                                                                                                                                                                                                   |
| background_subtraction_num_initialization_frames | `10`                                   | Number of frames to initialize the background with.                                                                                                                                                                                                                                                                                             |
| ambient_image_out                                | `$(arg node_name)/ambient_image_out`   | Topic to publish the ambient image on.                                                                                                                                                                                                                                                                                                          |
| diagnostic_out                                   | `$(arg node_name)/diagnostic`          | Topic to publish the diagnostic status.                                                                                                                                                                                                                                                                                                         |
| imu_out                                          | `$(arg node_name)/imu`                 | Topic to publish the IMU burst data.                                                                                                                                                                                                                                                                                                            |
| intensity_image_out                              | `$(arg node_name)/intensity_image_out` | Topic to publish the intensity image on.                                                                                                                                                                                                                                                                                                        |
| point_cloud_out                                  | `$(arg node_name)/points_raw`          | Topic to publish the PointCloud2 message to.                                                                                                                                                                                                                                                                                                    |
| range_image_out                                  | `$(arg node_name)/range_image_out`     | Topic to publish the range image on.                                                                                                                                                                                                                                                                                                            |
|                                                  |
| point_id_image                                   | `$(arg node_name)/point_id_image`      | Topic to publish the point id image on.                                                                                                                                                                                                                                                                                                         |

## Services

The Blickfeld ROS driver advertises different services.

### Set scan pattern

This service call will set scan pattern on a device.

e.g. `rosservice call /bf_lidar/set_scan_pattern "{fov_horizontal: 72.0, fov_vertical: 30.0, angle_spacing: 0.4, scanlines_up: 28, scanlines_down: 28, frame_mode: 'COMBINE_UP_DOWN', pulse_type: 'INTERLEAVE'}" `

### Publish IMU tf transform

This service call will receive the static IMU data from device and creates a frame from the data with the frame_id `$(arg lidar_frame_id)_imu`.

`rosservice call /$(arg node_name)/publish_imu_static_tf `

## License

This package is released under a [BSD 3-Clause License](LICENSE) (see also [https://opensource.org/licenses/BSD-3-Clause](https://opensource.org/licenses/BSD-3-Clause))
