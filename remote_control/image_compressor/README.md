# Image Compressor

## Package Description
Subscribe `sensor_msgs::msg::Image`, compress it to jpeg format, and publish it as `sensor_msgs::msg::CompressedImage`.

## Usage
```sh
$ colcon build --symlink-install --packages-up-to image_compressor
$ ros2 launch image_compressor image_compressor.launch.py
```

## Subscriber & Publisher
- Subscribe
    - `/aiformula_sensing/zed_node/left_image/undistorted`
- Publish
    - `/aiformula_visualization/zed/left_image/compressed`

## Parameters
- `jpeg_quality` (int, default: `30`)
    - Value to be set for `IMWRITE_JPEG_QUALITY`. From 0 to 100 (the higher is the better).
