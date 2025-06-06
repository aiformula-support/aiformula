## Sample Simulator

This package provides a sample simulator for AIFormula.

![simulator_readme](https://github.com/aiformula-support/aiformula/assets/113084733/ee478919-2a6b-4ee0-a71c-0b125638ec86)

#### Acquirable data：
* camera data
* odometry data

You can control the mobility of the simulator topic: `/cmd_vel`

### Running Example
To start simulator:
  ```bash
  ros2 launch sample_simulator gazebo_simulator.launch.py
  ```

### If you want to use a gamepad in the simulator
1. Change the value of `control/speed_command/gamepad` from `"/aiformula_control/gamepad/cmd_vel"`
to `"/cmd_vel"` in `sample_launchers/config/topic_list.yaml`
2. Start `gamepad_joy.launch.py` and `gamepad_teleop.launch.py` in sample_launchers package.


### It takes a long time (about two and a half minutes) only when launching gazebo_simulator.launch.py `for the first time` !
- Once the world file is loaded into gazebo, it is cached under `~/.gazebo/model`
- After the second time, it does not take long because the cache can be used.
