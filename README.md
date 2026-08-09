# ROS 2 Human Following Simulation

这是一个面向 ROS 2 Humble 的人体跟随仿真项目。当前仿真后端已经迁移到
Gazebo Sim 6.18.0 Fortress，并通过 `ros_gz` 与 ROS 2 交换底盘、传感器和时钟数据。

项目可以一条命令启动 Gazebo Sim、机器人、Nav2、RViz 和人体跟随节点。无显示器的
机器也能用 headless 模式运行。人体检测优先使用可选的 MediaPipe，本机没有安装时会
自动改用 OpenCV HOG，节点仍能正常启动和处理 RGB-D 数据。

## 包结构

```text
src/
├── care_robot/              # Python 运行节点和算法测试
├── care_robot_description/  # URDF 机器人描述
├── care_robot_gazebo/       # Gazebo Sim world、model、桥接和仿真 launch
├── care_robot_bringup/      # Nav2、SLAM、地图、RViz 和系统 launch
└── docs/                    # 架构说明
```

| 包 | 构建类型 | 职责 |
| --- | --- | --- |
| `care_robot` | `ament_python` | 人体检测、跟随目标和 actor 非接触代理同步 |
| `care_robot_description` | `ament_cmake` | 机器人 URDF 与 Gazebo Sim 原生系统插件配置 |
| `care_robot_gazebo` | `ament_cmake` | Fortress 世界、模型、生成实体和 `ros_gz_bridge` |
| `care_robot_bringup` | `ament_cmake` | 完整系统、Nav2、SLAM、地图和 RViz |

更详细的数据流见 [架构说明](docs/architecture.md)。

## 已验证环境

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Sim 6.18.0 Fortress
- `ros_gz_sim`、`ros_gz_bridge` 与 `ros_gz_interfaces`
- Navigation2 与 SLAM Toolbox
- Python 3.10、OpenCV 与 NumPy

Fortress 使用下面的命令查看版本。本机没有 `gz sim` 命令属于正常情况，Fortress 仍以
`ign gazebo` 作为命令行入口。

```bash
ign gazebo --version
ros2 pkg prefix ros_gz_sim
ros2 pkg prefix ros_gz_bridge
```

## 安装依赖

进入工作空间根目录后让 rosdep 安装 ROS 依赖。

```bash
cd ~/ROS2/ros2-human-following-sim
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
```

MediaPipe 是可选依赖。需要更稳定的姿态关键点检测时再安装它。

```bash
python3 -m pip install -r src/care_robot/requirements.txt
```

没有 MediaPipe 时，`detector_backend: auto` 会选择 OpenCV HOG。也可以在
`care_robot_bringup/config/follower.yaml` 中明确填写 `hog` 或 `mediapipe`。

## 编译与测试

编译目录位于用户指定的工作空间根目录。

```bash
cd ~/ROS2/ros2-human-following-sim
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

运行测试。

```bash
colcon test --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result --verbose
```

每次打开新终端都要重新加载 ROS 2 和当前工作空间。

```bash
source /opt/ros/humble/setup.bash
source ~/ROS2/ros2-human-following-sim/install/setup.bash
```

## 启动完整系统

带 Gazebo 和 RViz 界面启动。

```bash
ros2 launch care_robot_bringup bringup.launch.py
```

服务器或远程终端建议使用无界面模式。

```bash
ros2 launch care_robot_bringup bringup.launch.py gui:=false use_rviz:=false
```

默认启动位置为 `x=-5.5`、`y=2.0`、`yaw=0.0`。AMCL 会从同一位置自动初始化，
Nav2 的生命周期节点随后进入 `active`。

## 分开启动

仅启动 Gazebo Sim、机器人和 ROS 桥接。

```bash
ros2 launch care_robot_gazebo simulation.launch.py
```

无界面启动仿真。

```bash
ros2 launch care_robot_gazebo simulation.launch.py gui:=false
```

已有仿真时，可以在另一个终端启动 Nav2 或人体跟随节点。

```bash
ros2 launch care_robot_bringup localization.launch.py use_rviz:=false
ros2 launch care_robot_bringup following.launch.py
```

建图时先启动仿真，再启动 SLAM Toolbox。

```bash
ros2 launch care_robot_gazebo simulation.launch.py gui:=false
ros2 launch care_robot_bringup mapping.launch.py use_rviz:=false
```

保存地图。

```bash
ros2 run nav2_map_server map_saver_cli -f follow_map
```

## ROS 与 Gazebo Sim 接口

`simulation.launch.py` 会启动一个集中桥接节点。

| ROS 接口 | 方向 | 用途 |
| --- | --- | --- |
| `/clock` | Gazebo Sim 到 ROS 2 | 仿真时钟 |
| `/cmd_vel` | ROS 2 到 Gazebo Sim | 底盘速度命令 |
| `/odom` | Gazebo Sim 到 ROS 2 | 里程计 |
| `/tf`、`/joint_states` | Gazebo Sim 到 ROS 2 | 底盘与轮子状态 |
| `/scan` | Gazebo Sim 到 ROS 2 | GPU 激光雷达 |
| `/robot_camera_sensor/image` | Gazebo Sim 到 ROS 2 | RGB 图像 |
| `/robot_camera_sensor/depth_image` | Gazebo Sim 到 ROS 2 | 32 位浮点深度图 |
| `/robot_camera_sensor/camera_info` | Gazebo Sim 到 ROS 2 | 相机内参 |
| `/world/default/set_pose` | ROS 2 到 Gazebo Sim | actor 非接触代理位置同步 |

`/follow_goal` 只发布用于观察的 `PoseStamped`。Nav2 命令通过
`NavigateToPose` action 发送，避免重复提交目标。

可以用下面的命令快速确认数据是否到达。

```bash
ros2 topic hz /scan
ros2 topic hz /robot_camera_sensor/image
ros2 topic echo /odom --once
ros2 lifecycle get /controller_server
```

## 配置位置

- `care_robot_bringup/config/follower.yaml` 管理检测器、图像话题、TF 和跟随行为
- `care_robot_bringup/config/nav2_params.yaml` 管理 AMCL、规划器、控制器和 costmap
- `care_robot_bringup/config/slam_toolbox.yaml` 管理在线异步建图
- `care_robot_gazebo/worlds/maze4.world` 管理默认场景和 actor 轨迹
- `care_robot_description/urdf/care_robot.urdf` 管理机器人、传感器和差速驱动

## 修改时容易漏掉的地方

桥接路径目前使用实体名 `care_robot`。修改 `robot_name` 时要同步修改
`simulation.launch.py` 中的 Gazebo 话题路径。

修改机器人默认生成位置时，也要同步修改 `nav2_params.yaml` 中的
`amcl.initial_pose`。两处不一致会让地图坐标中的初始位置产生偏差。

Fortress 不会把脚本 actor 的姿态放进常规 `/pose/info`。代理同步节点会直接读取 world
中的 waypoint，并按照仿真时钟插值后调用 `SetEntityPose`。默认人物轨迹采用 Fortress
的 `z=1.0` 地面参考，人物从小车前方 1.5 米处起步。隐藏代理不参与接触计算，不会推动
或卡住小车。修改 actor 轨迹后无需再维护一份重复参数。

原始实验节点和旧 launch 文件保存在 `care_robot/archive`，扩展名为 `.disabled`，不会
参与构建或安装。仓库当前使用 `Proprietary` 作为占位许可证，正式分发前需要由项目
所有者确定许可证。
