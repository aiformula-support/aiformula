# Object Publisher

## Overview
- This package subscribes to `aiformula_msgs::msg::RectMultiArray`, converts the detected objects into the odom coordinate system, and applies a Kalman filter to track them while reducing detection instability.
- The filtered objects are then published as `aiformula_msgs::msg::ObjectInfoMultiArray`.

## Features
- Kalman Filtering:Smooths object detection and reduces noise.
- Tracking in Odom Coordinate System: Objects are tracked in the odometry frame.
- Automatic Object Removal: Objects that remain unobserved for `expiration_duration` seconds are removed.

## Usage
```sh
$ colcon build --symlink-install --packages-up-to object_publisher
$ ros2 launch object_publisher object_publisher.launch.py
```

## Subscriber & Publisher
- Subscribe
    - `/aiformula_perception/object_road_detector/rect`
- Publish
    - `/aiformula_perception/object_publisher/object_info`
    - `/aiformula_visualization/object_publisher/unfiltered_object` (In debug mode)

## Parameters
### object_publisher.yaml
- `object_separation_distance` (double, default: 5.0 [m])
    - If the distance between objects is greater than this value, they are considered separate.

### tracked_object.yaml
- `process_noise_variance` (double, default: 0.001 [m^2])
    - Process noise for the Kalman filter.
- `measurement_noise_variance` (double, default: 0.1 [m^2])
    - Measurement noise for the Kalman filter.
- `initial_error_covariance` (double, default: 1.0 [m^2])
    - Initial value of the posterior error estimate covariance matrix for the Kalman filter.
    - The smaller this value is, the less the Kalman filter is affected by the observed data, making it easier to maintain the initial value. Conversely, the larger it is, the more emphasis is placed on the observed data, making it easier to update the estimation.
- `expiration_duration` (double, default: 2.5 [sec])
    - If an object remains unobserved for this duration, it will be removed.
