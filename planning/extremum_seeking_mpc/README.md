# Model Predictive Control using Extremum Seeking Controller
This package provides a trajectory planning for AIFormula.
This packages is based on [Extremum_Seeking_Control](https://jp.mathworks.com/help/slcontrol/ug/extremum-seeking-control.html)

#### Acquirable data：
* Pointcloud data of road (right,left)
* Actual vehicle speed

### If you want to change parameter
1. Change the value in `extremum_seeking_mpc/config/extremum_seeking_mpc_params.yaml`
2. 
```
cd ~workspace/ros2_ws && colcon build --symlink-install --packages-up-to extremum_seeking_mpc && source install/setup.bash
```

## Running Example
To start extremum_seeking_mpc
```
ros2 launch extremum_seeking_mpc extremum_seeking_mpc.launch.py
```