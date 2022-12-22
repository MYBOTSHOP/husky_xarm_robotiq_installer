
# Changelog
All notable changes to this project will be documented in this file.
The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

### Changed

### Fixed

### Removed

## [1.5.1] - 2022.07.05

### Added
- Add information about point_id image in README [#40410]

## [1.5.0] - 2022.06.29

### Added
- [#37666] Added return id to point cloud fields if multi return is activated (returns_publishing_options: "all")
- [#38464] Added creation of point id image
- [#39345] Added meters per second sqaured as a unit option for imu acceleration

### Changed
- [#38314] Update pointers usage

## [1.4.5] - 2022.01.24

### Fixed
- Fix intensity type

## [1.4.4] - 2022.01.21

### Changed
- [#35609] service names are configurable in live_scanner.launch and live_scanner_nodelet.launch files
- [#35909] Imu static tf service returns transformation as service response

## [1.4.3] - 2021.11.02

### Changed
- Move SetScanPattern header reference from blickfeld_driver_ros_types to blickfeld_driver

## [1.4.2] - 2021.10.28

### Changed
- Update readme

## [1.4.1] - 2021.10.28

### Fixed
- update ros blickfeld driver core submodule

## [1.4.0] - 2021.10.28

### Added
- [closes #31485] Publish IMU data as ros IMU message
- [closes #31486] Publish IMU static data as static TF transform

### Changed
- Target names and class names have blickfeld as prefix
- [closes #32741] Create custom message for setScanPatternService call in ros core
- [closes #31485] Upgrade bsl version to 2.18.2
- [closes #31485] bsl version in ci_pipeline was changed to 2.18.2

### Fixed
- Fixed missing dependency on set scan pattern service call message, fixes compile error on first compilation
- Adding the range, intensity and ambient image topic names in launch file

## [1.3.2] - 2021.07.26

### Fixed
- [closes #31131] Fix the "pure virtual method called" message

## [1.3.1] - 2021.06.30

### Fixed
- [closes #31158] Add submodule source code in the artifact

## [1.3.0] - 2021.06.25

### Added
- [closes #27988] Add support for configuration of on-device algorithms. This feature requires **BSL >= 2.17.0**.
- [closes #30587] Added support for range, intensity and ambient 2D image creation
- [closes #30443] Add ros_blickfeld_driver_core as submodule

### Changed
- [closes #30443] Move the ros1_ros2 common functionalities to a new repository (ros_blickfeld_driver_core)

## [1.2.3] - 2021.02.01

### Changed
- Updated documentation, switched Note with Default column in parameters table

## [1.2.2] - 2021.01.28

### Added
- Added support for for ROS Noetic. Tested with **BSL 2.13.0**
- Ability to support different options to **publish multiple returns** from a lidar pulse. Possible values are: strongest first last and all

### Changed
- Multi-return options to support different options
- Updated documentation
- Enforce non-standalone BSL. Shows CMake error if BSL without required protocol files is configured.
- [closes #27875] Clarify documentation about ROS driver sources

### Removed
- Removed support for ROS Kinetic

## [1.2.1] - 2020.09.08

### Changed
Fix azimuth angle for no return points

## [1.2.0] - 2020.07.17

### Added
- Enhance time debug information on ROS debug - ticket [22666]
- Automatic ci release process - ticket [23492]

## [1.1.1] - 2020.06.25

### Changed
- Updated documentation

## [1.1.0] - 2020.04.26

### Added
- Multi-return options
- BSD 3-clause license
- Debug and warning info frame index

### Changed
- Replaced the row_id with scanline_id to be compliant with protobuf_protocol
- Replaced the column_id with scanline_point_index to represent the point index in a scanline
- Unified the usage publish_time_delta

## [1.0.0] - 2020.04.06

### Added
- Ability to publish explicit range of points
- Ability to publish Blickfeld PointID (global ID in the frame), ScanLineID (= row_id) and column_id
- Publish diagnostics data
- Ability to publish point measurement time as [`start_offset_ns`](http://enduser-software.pages.muc.blickfeld.com/blickfeld-scanner-lib/protobuf_protocol.html#point) as point field
- Ability to publish multi return points
- Ability to publish under canonical topic names (names ending with `_in`/`_out` for input/output topics)

### Changed
- update the use of point cloud term - ticket [18362]

## [0.3.0] - 2019.10.30

### Added
- Ability to publish intensities, ambient light level, and point without return
- Multiple return support
- Try to reconnect after connection loss

### Changed
- Adopt to new BSL API
- Publish PointCloud2::ConstPtr to make full use of nodelets

### Removed
- Burst mode
- Old launch files
- Read from dump since BSL does not support it, yet

