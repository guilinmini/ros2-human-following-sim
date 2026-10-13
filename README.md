# ROS 2 Human Following Simulation

这是一个面向 ROS 2 Jazzy 的人体跟随仿真项目。当前仿真后端已经迁移到
Gazebo Sim 8 Harmonic，并通过 `ros_gz` 与 ROS 2 交换底盘、传感器和时钟数据。

项目可以一条命令启动 Gazebo Sim、机器人、Nav2、RViz 和人体跟随节点。无显示器的
机器也能用 headless 模式运行。人体检测优先使用可选的 MediaPipe，本机没有安装时会
自动改用 OpenCV HOG，节点仍能正常启动和处理 RGB-D 数据。

## 包结构

```text
ros2-human-following-sim/
├── care_robot/              # Python 运行节点和算法测试
├── care_robot_description/  # URDF 机器人描述
├── care_robot_gazebo/       # Gazebo Sim world、model、桥接和仿真 launch
├── care_robot_bringup/      # Nav2、SLAM、地图、RViz 和系统 launch
└── docs/                    # 架构说明
```

| 包                         | 构建类型         | 职责                                             |
| -------------------------- | ---------------- | ------------------------------------------------ |
| `care_robot`             | `ament_python` | 人体检测、跟随目标和 actor 非接触代理同步        |
| `care_robot_description` | `ament_cmake`  | 机器人 URDF 与 Gazebo Sim 原生系统插件配置       |
| `care_robot_gazebo`      | `ament_cmake`  | Harmonic 世界、模型、生成实体和`ros_gz_bridge` |
| `care_robot_bringup`     | `ament_cmake`  | 完整系统、Nav2、SLAM、地图和 RViz                |

更详细的数据流见 [架构说明](docs/architecture.md)。

## 使用 Docker（推荐）

宿主机只需安装 Docker Engine 和 Compose v2.24.4 或更新版本；Windows、macOS
使用 Docker Desktop 的 Linux 容器模式。Ubuntu 22.04 + ROS 2 Humble 的用户可以
保留现有环境，容器内独立运行 Ubuntu 24.04 + Jazzy + Harmonic。
基础镜像来自 [ROS 官方镜像](https://hub.docker.com/_/ros)。

以下命令在仓库根目录执行。首次构建会下载依赖并编译、测试四个包，耗时较长。

```bash
docker compose up --build -d
```

浏览器打开 [仿真桌面](http://localhost:6080/vnc.html?autoconnect=true&resize=scale)，
即可操作 Gazebo 和 RViz。关闭浏览器不会停止仿真。
默认使用 OpenCV HOG 检测人体，无需安装可选的 MediaPipe。

```bash
# 查看启动日志。
docker compose logs -f sim

# 等待传感器、地图、TF 和 Nav2 就绪；超时会返回非零退出码。
docker compose exec sim ros-entrypoint check-simulation

# 在容器中打开已经加载 Jazzy 和项目环境的终端。
docker compose exec sim bash

# 停止并删除容器，保留保存的地图和日志。
docker compose down
```

如果 6080 端口已被占用，在仓库根目录的 `.env` 文件中写入 `NOVNC_PORT=6081`，
然后重新启动并改用 6081 端口。桌面默认仅监听宿主机的 `127.0.0.1`。

无需桌面时启动无界面版本，此时不开放网页端口：

```bash
docker compose down
docker compose -f compose.yaml -f compose.headless.yaml up --build -d
docker compose exec sim ros-entrypoint check-simulation
```

所有源码、地图和模型都打包进镜像，不挂载宿主机的 ROS 或个人目录。
运行产生的日志存放在容器的 `/data/log`，使用 Docker 命名卷持久化；
`docker compose down -v` 会同时删除该卷。
保存并导出当前地图到仓库根目录：

```bash
docker compose exec sim ros-entrypoint ros2 run nav2_map_server map_saver_cli -f /data/follow_map
docker compose cp sim:/data/follow_map.yaml ./follow_map.yaml
docker compose cp sim:/data/follow_map.pgm ./follow_map.pgm
```

默认使用 Mesa CPU 软件渲染，不要求显卡直通或宿主机 X11。相机、深度图和激光雷达
即使在无界面模式下也需要渲染，因此性能取决于 CPU，可能达不到实时速度。
Docker Desktop 可先分配 4 核 CPU、8 GB 内存，再根据实际负载调整。
配置没有固定 CPU 架构，构建时使用本机对应的基础镜像。
容器内 ROS 发现限制在容器本机，不会自动接入宿主机的 Humble 节点。

已在 Ubuntu 24.04 / AMD64 宿主机上验证镜像构建、36 项测试（0 失败、1 项跳过）、
桌面及无界面运行、浏览器 noVNC 连接、传感器和 Nav2 就绪、跟随导航目标完成，
以及重建容器后的地图持久化。Ubuntu 22.04 宿主机、Windows、macOS 和 ARM64 尚未实测。

修改项目源码后，重新执行 `docker compose up --build -d`；
无界面模式使用上面带两个 `-f` 参数的命令。

## 已验证环境

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Sim 8 Harmonic（本机验证版本 8.15.0）
- `ros_gz_sim`、`ros_gz_bridge` 与 `ros_gz_interfaces`
- Navigation2 与 SLAM Toolbox
- Python 3.12、OpenCV 与 NumPy

已验证四个包编译、colcon 测试和默认场景的无界面启动。测试汇总为 36 项、
0 错误、0 失败、1 项原有 copyright 检查跳过；运行中确认 Nav2 激活、传感器和
TF 数据可用，以及 OpenCV HOG 检测、导航目标发送与机器人移动。独立建图启动也已
验证，SLAM Toolbox 正常发布 `/map`。原生 GUI 和可选 MediaPipe 后端未在本次迁移中验证；
Docker 内的 Gazebo、RViz 和浏览器桌面已验证。

Harmonic 使用 `gz sim` 命令。先加载 Jazzy 环境，再检查 Gazebo 和桥接包：

```bash
source /opt/ros/jazzy/setup.bash
gz sim --version
ros2 pkg prefix ros_gz_sim
ros2 pkg prefix ros_gz_bridge
```

## 安装依赖

以下命令均在仓库根目录执行，项目文件使用相对路径。四个 ROS 包直接位于仓库
根目录，也可以把整个仓库放进其他
colcon 工作空间的 `src/` 下，从该工作空间根目录编译。

安装 ROS 2 Jazzy 后，让 rosdep 安装项目依赖。Jazzy 的 `ros_gz` 使用官方配套的
Gazebo Harmonic，参见 [Gazebo 与 ROS 版本配对说明](https://gazebosim.org/docs/harmonic/ros_installation/)。

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths care_robot care_robot_description care_robot_gazebo care_robot_bringup --ignore-src --rosdistro jazzy -r -y
```

MediaPipe 是可选依赖。需要更稳定的姿态关键点检测时再安装它。

```bash
sudo apt install python3-venv
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python3 -m pip install -r care_robot/requirements.txt
# 避免 colcon 扫描虚拟环境中的包。
touch .venv/COLCON_IGNORE
```

没有 MediaPipe 时，`detector_backend: auto` 会选择 OpenCV HOG。也可以在
`care_robot_bringup/config/follower.yaml` 中明确填写 `hog` 或 `mediapipe`。

## 编译与测试

使用 Bash 执行以下命令；Zsh 用户将 `setup.bash` 换为 `setup.zsh`。
如果启用了可选 MediaPipe 虚拟环境，请保持该环境激活，并用环境内的 Python 编译：
`python3 -m colcon build --symlink-install`。启动前也需激活同一环境。

从旧 ROS 版本迁移时，请使用干净的 `build/`、`install/` 和 `log/` 目录，
在新终端中只加载 Jazzy 环境后重新编译。

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

运行测试。

```bash
colcon test --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result --verbose
```

每次打开新终端，先进入仓库根目录，再加载 ROS 2 和当前工作空间。

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
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

| ROS 接口                             | 方向                | 用途                     |
| ------------------------------------ | ------------------- | ------------------------ |
| `/clock`                           | Gazebo Sim 到 ROS 2 | 仿真时钟                 |
| `/cmd_vel`                         | ROS 2 到 Gazebo Sim | 底盘速度命令             |
| `/odom`                            | Gazebo Sim 到 ROS 2 | 里程计                   |
| `/tf`、`/joint_states`           | Gazebo Sim 到 ROS 2 | 底盘与轮子状态           |
| `/scan`                            | Gazebo Sim 到 ROS 2 | GPU 激光雷达             |
| `/robot_camera_sensor/image`       | Gazebo Sim 到 ROS 2 | RGB 图像                 |
| `/robot_camera_sensor/depth_image` | Gazebo Sim 到 ROS 2 | 32 位浮点深度图          |
| `/robot_camera_sensor/camera_info` | Gazebo Sim 到 ROS 2 | 相机内参                 |
| `/world/default/set_pose`          | ROS 2 到 Gazebo Sim | actor 非接触代理位置同步 |

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
