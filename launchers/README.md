## Launchers
This package manages launch files each node\
**Note:**  Topic name and Frame_id are maneged YAML file

| Items | /path/to/yaml_file |
| :---: | :----------------- |
| Topic Name | /config/topic_lint.yaml    |
| Frame id   | /config/frame_id_list.yaml |

### Launch Files
- `gamepad_joy.launch.py`
  - Output: `gamepad command`

- `socket_can_bridge.launch.py`
  - Input: `speed command`, Output: `wheel rotation count`

- `gamepad_teleop.launch.py`
  - Input: `gamepad command`, Output: `speed command (from gamepad)`

- `vectornav.launch.py`
  - Output: `Imu`

- `zedx_camera.launch.py`
  - Output: `Image (raw / undostorted), Imu`
