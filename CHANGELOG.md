# Change Log
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).


## [v1.4.0] - 2025-03-14
### Added
- `image_compressor`.
- `twist_mux`.
- `diagnostic` installation in dockerfile.

### Changed
- Update `build.sh`.
- Rename `joy.launch` -> `gamepad_joy.launch`.
- Rename `teleop.launch` -> `gamepad_teleop.launch`.
- Update `all_nodes.launch.py`.

## v1.3.1 - 2025-03-06
### Fixed
- `socket_can_bridge`
  - Adjusted `receiver_interval_sec` and `sender_timeout_sec` for CAN communication.
- `motor_controller`
  - Move the definition of message to the constructor.

## v1.3.0 - 2024-09-05
### Added
- `road_detector`
- `lane_line_publisher`

## v1.2.0 - 2024-08-01
### Added
- `rear_potentiometer`.

## v1.1.0 - 2024-07-08
### Changed
- IMU used for Gyro Odometry from `vectornav` to `zedX`.

## v1.0.0 - 2024-05-09
### Added
- Initial release.
- Core features implemented.


[v1.4.0]: https://github.com/honda-hgrx-idcs/EC7D_AIformula_Control/tree/4c860512b23689f2473e22b0016aaa83f31c049c
