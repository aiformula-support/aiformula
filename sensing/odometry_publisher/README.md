# odometry_publisher
- 当初はホイールエンコーダのみを用いた`wheel_odometry_publisher`を作成
- しかし，角度の精度が悪いので，ホイールエンコーダ と Imu (角度情報のみ使用) を組み合わせた`gyro_odometry_publisher`を作成
- 精度の良い`gyro_odometry_publisher`を使用することを推奨
- `wheel_odometry_publisher`も今後のために残しておく

### 使用方法
```sh
$ colcon build --symlink-install --packages-up-to odometry_publisher
$ ros2 launch odometry_publisher gyro_odometry_publisher.launch.py
```

### 機能
- `wheel_odometry_publisher`
    - Subscribe
        - `/aiformula_sensing/can/vehicle_info`
            - CAN情報
            - この中にホイールエンコーダの情報も含まれてる
    - Publish
        - `/aiformula_sensing/wheel_odometry_publisher/odom`
            - `odom` to `base_footprint`
    - TF Broadcast
        - `/tf`
            - `odom` to `base_footprint`

- `gyro_odometry_publisher`
    - Subscribe
        - `/aiformula_sensing/vectornav/imu`
        - `/aiformula_sensing/can/vehicle_info`
    - Publish
        - `/aiformula_sensing/gyro_odometry_publisher/odom`
    - TF Broadcast
        - `/tf`
    - オドメトリ算出方法
        - `Imu(t)`, `Imu(t+1)`: `Imu` から取得した時刻 `t`, `t+1` のロボットの Yaw 角
        - `Encoder(t')`: `t` ~ `t+1` 間の時刻 `t'` にホイールエンコーダから取得した回転数
        -  ロボットの Yaw 角 を 時刻 `t'` に線形補間し、その方向に `Encoder(t')` だけ進んだとしている

### ホイールエンコーダー
- id
    - 1809 (0x711): 回転数
