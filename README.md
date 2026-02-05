# FAST-LIO 中文说明文档（高频率里程计输出，带重力对齐功能）

## 相关工作与扩展应用

### SLAM 相关

1. [ikd-Tree](https://github.com/hku-mars/ikd-Tree)：面向3D近邻搜索的先进动态KD-Tree

2. [R2LIVE](https://github.com/hku-mars/r2live)：以FAST-LIO为激光雷达-惯性前端的高精度激光雷达-惯性-视觉融合方案

3. [LI_Init](https://github.com/hku-mars/LiDAR_IMU_Init)：鲁棒、实时的激光雷达-IMU外参标定与时间同步工具包

4. [FAST-LIO-LOCALIZATION](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION)：集成重定位功能模块的FAST-LIO

### 控制与规划相关

1. [IKFOM](https://github.com/hku-mars/IKFoM)：面向流形的快速高精度卡尔曼滤波工具包

2. [UAV Avoiding Dynamic Obstacles](https://github.com/hku-mars/dyn_small_obs_avoidance)：FAST-LIO在机器人规划中的落地实现之一

3. [UGV Demo](https://www.youtube.com/watch?v=wikgrQbE6Cs)：基于可微流形的模型预测控制轨迹跟踪方案

4. [Bubble Planner](https://arxiv.org/abs/2202.12177)：基于后退视距走廊的四旋翼高速平滑轨迹规划方法

## FAST-LIO 简介

FAST-LIO（Fast LiDAR-Inertial Odometry）是一款计算高效、鲁棒性强的激光雷达-惯性里程计工具包。通过紧耦合迭代扩展卡尔曼滤波融合激光雷达特征点与IMU数据，可在高速运动、噪声干扰或特征退化的复杂环境中实现稳定导航。本工具包解决了多个核心问题：

1. 面向里程计优化的快速迭代卡尔曼滤波

2. 多数平稳环境下可自动初始化

3. 并行KD-Tree搜索，降低计算开销

## FAST-LIO 2.0（2021-07-05 更新）

<div align="left">

<img src="doc/real_experiment2.gif" width=49.6% />

<img src="doc/ulhkwh_fastlio.gif" width = 49.6% >

</div>

**相关视频**：[FAST-LIO2](https://youtu.be/2OvjGnxszf8)、[FAST-LIO1](https://youtu.be/iYCY6T79oNU)

**算法流程**：

<div align="center">

<img src="doc/overview_fastlio2.svg" width=99% />

</div>

### 新增特性

1. 基于[ikd-Tree](https://github.com/hku-mars/ikd-Tree)的增量式建图，实现更快运行速度，支持超100Hz激光雷达数据输入

2. 基于原始激光雷达点云的直接里程计（扫描到地图匹配，可关闭特征提取），精度更优

3. 无需特征提取，支持多种激光雷达类型（包括机械式：Velodyne、Ouster；固态式：Livox Avia、Horizon、MID-70），可轻松扩展适配更多雷达

4. 支持外接IMU

5. 支持ARM平台，包括Khadas VIM3、英伟达TX2、树莓派4B（8G内存）

### 相关论文

- [FAST-LIO2: Fast Direct LiDAR-inertial Odometry](doc/Fast_LIO_2.pdf)

- [FAST-LIO: A Fast, Robust LiDAR-inertial Odometry Package by Tightly-Coupled Iterated Kalman Filter](https://arxiv.org/abs/2010.08196)

### 贡献者

[徐威 Wei Xu](https://github.com/XW-HKU)、[蔡逸熙 Yixi Cai](https://github.com/Ecstasy-EC)、[贺东娇 Dongjiao He](https://github.com/Joanna-HE)、[朱方程 Fangcheng Zhu](https://github.com/zfc-zfc)、[林家荣 Jiarong Lin](https://github.com/ziv-lin)、[刘政 Zheng Liu](https://github.com/Zale-Liu)、[Borong Yuan](https://github.com/borongyuan)

## 1. 依赖环境

### 1.1 Ubuntu 与 ROS

- Ubuntu 版本 ≥ 16.04

- Ubuntu 18.04 及以上版本，默认安装的PCL和Eigen即可满足FAST-LIO运行需求

- ROS 版本 ≥ Melodic，[ROS安装教程](http://wiki.ros.org/ROS/Installation)

### 1.2 PCL 与 Eigen

- PCL 版本 ≥ 1.8，[PCL安装教程](http://www.pointclouds.org/downloads/linux.html)

- Eigen 版本 ≥ 3.3.4，[Eigen安装教程](http://eigen.tuxfamily.org/index.php?title=Main_Page)

### 1.3 livox_ros_driver

按照[livox_ros_driver安装教程](https://github.com/Livox-SDK/livox_ros_driver)完成安装

**注意**：

- 因FAST-LIO需优先支持Livox系列激光雷达，运行任何FAST-LIO启动文件前，必须先安装并配置`livox_ros_driver`环境变量

- 环境变量配置方法：在`~/.bashrc`文件末尾添加`source $Livox_ros_driver_dir$/devel/setup.bash`，其中`$Livox_ros_driver_dir$`为livox ros驱动工作空间目录（若完全按照官方文档安装，通常为`ws_livox`目录）

## 2. 编译

若需通过Docker容器运行FAST-LIO2，需先在设备上安装Docker，[Docker安装教程](https://docs.docker.com/engine/install/ubuntu/)

### 2.1 Docker 容器部署

在Linux系统中通过以下命令创建自定义名称的脚本文件：

```Plain Text

touch <自定义脚本名称>.sh
```

将以下代码写入`<自定义脚本名称>.sh`文件：

```Plain Text

#!/bin/bash
mkdir docker_ws
# 支持GUI的ROS Kinetic Docker运行脚本

# 允许本地主机访问X服务器
xhost +local:

# 容器名称
CONTAINER_NAME="fastlio2"

# 启动Docker容器
docker run -itd \
  --name=$CONTAINER_NAME \
  --user mars_ugv \
  --network host \
  --ipc=host \
  -v /home/$USER/docker_ws:/home/mars_ugv/docker_ws \
  --privileged \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/etc/localtime:/etc/localtime:ro" \
  -v /dev/bus/usb:/dev/bus/usb \
  --device=/dev/dri \
  --group-add video \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --env="DISPLAY=$DISPLAY" \
  kenny0407/marslab_fastlio2:latest \
  /bin/bash
```

执行以下命令为脚本添加执行权限：

```Plain Text

sudo chmod +x <自定义脚本名称>.sh
```

执行以下命令下载镜像并创建容器：

```Plain Text

./<自定义脚本名称>.sh
```

**脚本说明**：

- 上述`docker run`命令会基于Docker Hub镜像创建带标签的容器，镜像下载时长取决于网络速度

- 脚本会创建`docker_ws`工作空间，作为Docker容器与宿主机的共享文件夹；若需运行ROS包示例，需将ROS包文件下载至宿主机`docker_ws`目录，容器内同名文件夹会同步该文件，可直接在容器内播放

- 本配置将宿主机网络共享给Docker容器，因此在宿主机或容器内执行`rostopic list`命令，输出结果完全一致

### 2.2 源码编译

克隆仓库并执行catkin_make编译：

```Plain Text

    cd ~/$ROS工作空间$/src
    git clone https://github.com/hku-mars/FAST_LIO.git
    cd FAST_LIO
    git submodule update --init
    cd ../..
    catkin_make
    source devel/setup.bash
```

- 编译前务必先配置`livox_ros_driver`环境变量（参考1.3节）

- 若需使用自定义编译的PCL库，在`~/.bashrc`中添加以下语句：

`export PCL_ROOT={自定义PCL库路径}`

## 3. 直接运行

**重要提示**：

A. 确保IMU与激光雷达完成**时间同步**，这是系统正常运行的关键

B. 若出现警告"Failed to find match for field 'time'"，表示ROS包文件中缺失每个激光雷达点的时间戳，该时间戳对前向/后向传播至关重要

C. 若已提供精确外参，建议将`extrinsic_est_en`设为false；外参初始化可参考最新工作：[**鲁棒实时激光雷达-IMU初始化**](https://github.com/hku-mars/LiDAR_IMU_Init)

### 3.1 Livox Avia 雷达运行

按照[Livox-ros-driver安装教程](https://github.com/Livox-SDK/livox_ros_driver)完成PC与Livox Avia雷达连接，随后执行：

```Plain Text

    cd ~/$FAST-LIO工作空间$
    source devel/setup.bash
    roslaunch fast_lio mapping_avia.launch
    roslaunch livox_ros_driver livox_lidar_msg.launch
```

- 对于Livox系列雷达，FAST-LIO仅支持`livox_lidar_msg.launch`采集的数据，因只有该启动文件对应的`livox_ros_driver/CustomMsg`数据结构会输出每个激光雷达点的时间戳（对点云运动去畸变至关重要）；`livox_lidar.launch`暂不支持该功能

- 若需修改数据发布帧率，需先修改[Livox-ros-driver](https://github.com/Livox-SDK/livox_ros_driver)中[livox_lidar_msg.launch](https://github.com/Livox-SDK/livox_ros_driver/blob/master/livox_ros_driver/launch/livox_lidar_msg.launch)的`publish_freq`参数，再重新编译`livox_ros_driver`功能包

### 3.2 外接IMU的Livox系列雷达运行

`mapping_avia.launch`理论上支持mid-70、mid-40等Livox系列雷达，运行前需修改以下参数：

编辑`config/avia.yaml`文件，配置以下参数：

1. 激光雷达点云话题名：`lid_topic`

2. IMU话题名：`imu_topic`

3. 平移外参：`extrinsic_T`

4. 旋转外参：`extrinsic_R`（仅支持旋转矩阵）

**参数说明**：

- FAST-LIO中外参定义为**激光雷达在IMU本体坐标系下的位姿**（IMU为基准坐标系），具体数值可查阅设备官方手册

- FAST-LIO为Livox雷达提供简易软件时间同步功能，将`time_sync_en`设为true即可开启；但仅在无外部时间同步条件时建议开启，因软件同步无法保证精度

### 3.3 Velodyne 或 Ouster 雷达运行（以Velodyne为例）

#### 步骤A：运行前配置

编辑`config/velodyne.yaml`文件，配置以下参数：

1. 激光雷达点云话题名：`lid_topic`

2. IMU话题名：`imu_topic`（支持内置/外接IMU，6轴/9轴均可）

3. 根据PointCloud2话题中`time`（Velodyne）或`t`（Ouster）字段的单位，配置`timestamp_unit`参数

4. 雷达线数（已测试16、32、64线，未测试128线及以上）：`scan_line`

5. 平移外参：`extrinsic_T`

6. 旋转外参：`extrinsic_R`（仅支持旋转矩阵）

**参数说明**：FAST-LIO中外参定义为**激光雷达在IMU本体坐标系下的位姿**（IMU为基准坐标系）

#### 步骤B：启动程序

```Plain Text

    cd ~/$FAST-LIO工作空间$
    source devel/setup.bash
    roslaunch fast_lio mapping_velodyne.launch
```

#### 步骤C：启动激光雷达ROS驱动或播放ROS包

### 3.4 PCD 文件保存

将启动文件中的`pcd_save_enable`设为`1`，FAST-LIO终止后，所有全局坐标系下的扫描点云会累积保存至`FAST_LIO/PCD/scans.pcd`文件，通过`pcl_viewer scans.pcd`可可视化点云

**pcl_viewer操作技巧**：运行pcl_viewer时，按键盘1-5键可切换可视化/着色模式：

```Plain Text

    1：随机着色
    2：X轴数值着色
    3：Y轴数值着色
    4：Z轴数值着色
    5：强度值着色
```

## 4. ROS包示例

### 4.1 Livox Avia ROS包

<div align="left">

<img src="doc/results/HKU_LG_Indoor.png" width=47% />

<img src="doc/results/HKU_MB_002.png" width = 51% >

</div>

**文件下载**：[谷歌云盘](https://drive.google.com/drive/folders/1CGYEJ9-wWjr8INyan6q1BZz_5VtGB-fP?usp=sharing)

**运行命令**：

```Plain Text

roslaunch fast_lio mapping_avia.launch
rosbag play 下载的ROS包文件.bag
```

### 4.2 Velodyne HDL-32E ROS包

**NCLT数据集**：原始bin文件可从[此处](http://robots.engin.umich.edu/nclt/)下载

我们提供[ROS包文件](https://drive.google.com/drive/folders/1blQJuAB4S80NwZmpM6oALyHWvBljPSOE?usp=sharing)及[Python脚本](https://drive.google.com/file/d/1QC9IRBv2_-cgo_AEvL62E1ml1IL9ht6J/view?usp=sharing)，可通过以下命令生成ROS包：

`python3 sensordata_to_rosbag_fastlio.py bin文件目录 生成包名称.bag`

**运行命令**：

```Plain Text

roslaunch fast_lio mapping_velodyne.launch
rosbag play 下载的ROS包文件.bag
```

## 5. 无人机部署验证

为验证FAST-LIO在实际移动机器人中的鲁棒性与计算效率，我们搭建了小型四旋翼无人机平台：搭载70°视场角Livox Avia激光雷达，以及载板为1.8GHz Intel i7-8550U CPU、8G内存的大疆妙算2-C，如下图所示。

无人机主体结构采用3D打印（铝合金或PLA材料），.stl模型文件后续将开源。

<div align="center">

<img src="doc/uav01.jpg" width=40.5% >

<img src="doc/uav_system.png" width=57% >

</div>

## 6. 致谢

感谢LOAM（J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time）、[Livox_Mapping](https://github.com/Livox-SDK/livox_mapping)、[LINS](https://github.com/ChaoqinRobotics/LINS---LiDAR-inertial-SLAM)及[Loam_Livox](https://github.com/hku-mars/loam_livox)相关工作的贡献。
> （注：文档部分内容可能由 AI 生成）
