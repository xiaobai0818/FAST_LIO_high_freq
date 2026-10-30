# FAST_LIO_high_freq

基于 [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) 的 ROS 1 功能包，包名 `fast_lio`。主体仍是紧耦合的激光–惯性里程计：IMU 前向传播与点云去畸变，ikd-Tree 上的扫描到地图匹配，迭代误差状态卡尔曼滤波。本仓库相对上游多了两处改动，都在当前代码里。

- 初始化时把重力方向和航向对齐到世界系。
- 在激光更新之间，按 IMU 频率发布前向传播里程计。

`include/ikd-Tree` 已随仓库带上，不需要再拉 submodule。没有 Docker 脚本，也没有论文里的示例数据包。

## 重力对齐

实现在 `src/IMU_Processing.hpp` 的 `IMU_init`。初始化阶段累计 IMU，超过 10 条后打印 `IMU Initial Done` 并结束初始化。这段时间载体应尽量保持静止。

对齐用的是平均加速度，不是激光点云：

1. 把平均加速度旋到世界 +Z，也就是把静止时加速度计读到的“向上”对齐到世界天向。
2. 再把对齐后 IMU X 轴的水平投影旋到世界 +X，从而定下航向。
3. 该旋转写入初始状态 `rot`，陀螺零偏取平均角速度。重力写成 `S2(-mean_acc / ||mean_acc|| * 9.81)`，初始化完成后大致沿世界 -Z。

代码注释写的是 LiDAR X 轴。实际旋转的是 IMU 测量和 IMU 的 X 轴。配置里的外参旋转为单位阵时，两者重合。

## 高频里程计

实现在 `src/laserMapping.cpp`。第一帧激光状态更新成功之后，每来一条 IMU 就调用 `fastPredictIMU`：从最近一次激光更新的位姿、速度和零偏做中值积分，并立刻发布 `/Odom_high_freq`。每次激光滤波结束，`updateLatestStates()` 把积分起点拉回滤波后的状态。

所以 `/Odom_high_freq` 是 IMU 频率的前向传播，不是另一套独立滤波器。激光更新仍以 `/Odometry` 为准。

| 话题 | 频率 | 坐标系 | 内容 |
| --- | --- | --- | --- |
| `/Odometry` | 激光帧 | `camera_init` → `body` | 滤波后的位姿，同时发同名 TF |
| `/Odom_high_freq` | IMU | `world` → `odom_imu` | 位姿、世界系线速度、去零偏后的角速度 |
| `/cloud_registered` | 激光帧 | `camera_init` | 世界系点云 |
| `/cloud_registered_body` | 激光帧 | 机体系 | IMU 机体系点云 |
| `/path` | 抽稀后的激光帧 | `camera_init` | 仅当 `publish/path_en` 为 true |

`/Odometry` 和 `/Odom_high_freq` 的 `frame_id` 字符串不同。重力对齐完成后，两者的世界系都是初始化时定下来的那个系。

`imu_cbk` 会把 `linear_acceleration` 乘以 9.81。输入应按 g 给，Livox 内置 IMU 就是这种单位。驱动如果已经发布 m/s²，这里会被再放大约 9.81 倍。

## 依赖与编译

- ROS 1（catkin），PCL ≥ 1.8，Eigen ≥ 3.3.4
- Python 开发头文件。`CMakeLists.txt` 里 `find_package(PythonLibs REQUIRED)`
- Livox：`livox_ros_driver2`。`lidar_type == 1` 时订阅的是 `livox_ros_driver2/CustomMsg`，不是旧版 `livox_ros_driver`

```bash
cd ~/catkin_ws/src
git clone https://gitee.com/xiaobai0818/FAST_LIO_high_freq.git
cd ..
catkin_make
source devel/setup.bash
```

使用 Livox 前，先 source `livox_ros_driver2` 所在工作空间。节点可执行文件是 `fastlio_mapping`。

## 运行

雷达和 IMU 需要时间同步。外参 `extrinsic_T`、`extrinsic_R` 表示雷达在 IMU 坐标系下的位姿，旋转只接受 3×3 矩阵。外参已经标定好时，把 `extrinsic_est_en` 设为 false。

```bash
roslaunch fast_lio mapping_mid360.launch
```

`rviz` 默认打开，配置文件是 `rviz_cfg/loam_livox.rviz`。

| 雷达 | launch | 配置 | 点云话题 | IMU 话题 |
| --- | --- | --- | --- | --- |
| Livox Avia | `mapping_avia.launch` | `config/avia.yaml` | `/livox/lidar` | `/livox/imu` |
| Livox Horizon | `mapping_horizon.launch` | `config/horizon.yaml` | `/livox/lidar` | `/livox/imu` |
| Livox MID360 | `mapping_mid360.launch` | `config/mid360.yaml` | `/livox/lidar` | `/livox/imu` |
| Velodyne | `mapping_velodyne.launch` | `config/velodyne.yaml` | `/velodyne_points` | `/imu/data` |
| Ouster OS2-64 | `mapping_ouster64.launch` | `config/ouster64.yaml` | `/os_cloud_node/points` | `/os_cloud_node/imu` |

`preprocess/lidar_type`：1 是 Livox（`CustomMsg`），2 是 Velodyne，3 是 Ouster。后两者走 `sensor_msgs/PointCloud2`，还要设置 `scan_line` 和 `timestamp_unit`（0 秒，1 毫秒，2 微秒，3 纳秒）。现有 launch 里 `feature_extract_enable` 都是 0，直接用降采样后的原始点做扫描到地图。

`time_sync_en` 默认 false。没有外部对时、时间差又明显时才打开，软件同步精度有限。`time_offset_lidar_to_imu` 在两种情况下都会从 IMU 时间戳里减去。

`pcd_save/pcd_save_en` 为 true 时，节点退出后把累积点云写到 `PCD/scans.pcd`。`interval: -1` 表示全部帧进同一个文件，帧数多时内存会涨。Avia、Horizon、Velodyne、Ouster 的默认配置是打开的，MID360 是关闭的。只有 MID360 的 `publish/path_en` 默认为 true。

## 来源

滤波、去畸变和 ikd-Tree 建图来自 HKU MARS 的 FAST-LIO2。本仓库保留的本地改动是初始化重力对齐，以及 `/Odom_high_freq`。
