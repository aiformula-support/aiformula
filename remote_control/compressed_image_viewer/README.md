# Compressed Image Viewer

## Package Description
- Subscribe `sensor_msgs::msg::CompressedImage`, display it by `cv::imshow`.

- If `display_full_screen` is `true`
    - Subscribed image is displayed in full screen.

- If `display_full_screen` is `false`
    - Subscribed image is displayed at a size based on `window.width_ratio` and positioned according to `window.position_ratio`.


## Required Library
```sh
  A required package was not found
```
If you get the above error during build, install the appropriate version of the GTK library according to the following.

1. Check the value of `GTK_VERSION` output during build (probably 2 or 3).  
    Build method is as follows.
    ```sh
    colcon build --symlink-install --packages-up-to compressed_image_viewer
    ```
2. Install the GTK library.
    - GTK2
        ```sh
        sudo apt update  && sudo apt install -y libgtk2.0-dev
        ```
    - GTK3
        ```sh
        sudo apt update  && sudo apt install -y libgtk-3-dev
        ```

## Usage
```sh
$ colcon build --symlink-install --packages-up-to compressed_image_viewer
$ ros2 launch compressed_image_viewer compressed_image_viewer.launch.py
```

## Subscriber & Publisher
- Subscribe
    - `/aiformula_visualization/zed/left_image/compressed`

## Parameters
- `display_full_screen` (bool, default: `true`)
    - If true, display the image in full screen.

### If not in full screen mode
- `target_screen_idx` (int, default: `0`)
    - Screen index to show when multiple displays are available.

- `window.width_ratio` (double, default: `0.5`)
    - The ratio of the window width to the screen size.
    - The height is determined to maintain the aspect ratio.

- `window.position_ratio` (double, default: `(0.5, 0.5)`)
    - The ratio of the window position to the screen size.
    - `0.0`: left/top, `1.0`: right/bottom
