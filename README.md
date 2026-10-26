# ODIN1 采集操作

环境：Windows + WSL2 Ubuntu 22.04，ROS 2 Humble。工作空间 `/root/odin_ws`，工具位于 `D:\workspace\odin_capture`。

设备 `N080100083` 的 SoC/SLAM 固件为 `0.12.0`，使用厂家 `0.12.0` 标签（提交号见 `driver_commit.txt`）。该标签内显示的旧版本字符串尚未更新，以 Git 提交号为准。最新驱动要求更高固件而不能直接使用；本次未升级设备固件。使用 ROS 官方签名校验的清华 ROS 2 镜像安装依赖。

## 1. 每次接入硬件

用 USB 3 数据线连接 ODIN1 并确认供电。保持 Ubuntu/WSL 窗口打开，在 Windows PowerShell 执行：

```powershell
powershell -ExecutionPolicy Bypass -File D:\workspace\odin_capture\attach.ps1
```

首次 `bind` 要求管理员 PowerShell；后续 `attach` 通常不需要。脚本按 `2207:0019` 自动识别 BUSID。断电、拔插或 WSL 关闭后可能需要重新 attach。设备交给 WSL 后 Windows 上的其他软件无法同时使用它。

## 2. 启动驱动

在 Ubuntu 终端 A：

```bash
bash /mnt/d/workspace/odin_capture/start.sh
```

等待设备连接、固件版本、标定加载及正常出数日志。日志保存在 `/root/odin_ws/logs`。脚本使用 `capture.yaml`，设备时间戳、约 10 Hz 点云/RGB、里程计模式，开启原始点云、IMU、图像、里程计和 SLAM 点云。新录制同时保存 `/odin1/image`（bgr8）和 `/odin1/image/compressed`（JPEG）。其他处理功能由下述开关控制。

驱动源码已修复 `config_file` 参数未生效的问题，并支持 `ODIN_CALIB_DIR` 指定独立标定目录；配置文件无需每次修改后重新编译。完整修改保存在 `driver.patch`。

### 深度补全、重投影与图像叠加

`start.sh` 通过 `start_nodes.py` 统一启动主驱动和开启的处理节点。所有节点使用 `capture.yaml`，需要标定的节点使用 `$ODIN_CALIB_DIR/calib.yaml`（默认 `/root/odin_ws/runtime/calib.yaml`），由主驱动从设备获取。处理节点不再固定读取源码目录的配置和标定。

启用全部三项时，在 `capture.yaml` 中设置：

```yaml
  sendrgb: 1
  sendrgbundistort: 1
  senddtof: 1
  sendodom: 1
  sendcloudslam: 1
  senddepth: 1
  sendreprojection: 1
  sendoverlay: 1
```

首次应用驱动源码修改需执行 `bash /mnt/d/workspace/odin_capture/build.sh`；之后修改配置仅需重启 `start.sh`。如果正在录制，先在录制终端按 Ctrl+C，等待校验和复制结束，再停止并重新启动驱动。不要同时运行两份驱动。

启动器检查上述依赖；缺少依赖时会报出需要开启的键。关闭启动终端时会停止整组节点；任一节点退出也会停止整组，避免处理功能失效后仍误以为正常运行。

停止时按 Ctrl+C 一次。若厂家 SDK 卡在 `Closing device...`，启动器等待 8 秒后发送 SIGTERM，再等 2 秒仍未退出则发送 SIGKILL；期间不要反复按 Ctrl+C。此机制限制退出等待时间，不代表已修复 SDK 的 USB 关闭故障。请使用 `start.sh` 启动，直接运行 `host_sdk_sample` 没有这一超时保护。强制结束后若设备不能重新出数，先重新插拔设备并在 Windows 执行 `attach.ps1`，再启动驱动。停止驱动前仍需先正常结束 bag 录制并等待校验和复制完成。

另开终端启动实时 RViz：

```bash
source /mnt/d/workspace/odin_capture/env.sh
rviz2 -d "$ODIN_TOOLS/playback.rviz" --ros-args -p use_sim_time:=false
```

在 RViz 中通过 Add → By topic 添加显示：

| 开关 | 节点可执行文件 | 输出话题 | 显示类型 |
| --- | --- | --- | --- |
| `sendrgbundistort` | 主驱动 | `/odin1/image/undistorted` | Image |
| `senddepth` | `pcd2depth_ros2_node` | `/odin1/depth_img_competetion` | Image |
| `senddepth` | `pcd2depth_ros2_node` | `/odin1/depth_img_competetion_cloud` | PointCloud2 |
| `sendreprojection` | `cloud_reprojection_ros2_node` | `/odin1/reprojected_image` | Image |
| `sendoverlay` | `image_overlay_node` | `/odin1/overlay_image` | Image |

`competetion` 是厂家代码中的原始拼写。overlay 默认将重投影图像叠加到去畸变图像上，输入和输出名称由 `overlay_*_topic` 配置指定。`record.sh` 默认同时保存深度补全图和补全点云；录制时需保持 `senddepth: 1` 及其依赖 `senddtof: 1`、`sendrgb: 1`，两路补全输出无数据时录前检查会拒绝开始录制。去畸变图、重投影图和叠加图仍不在默认录制清单中。

## 3. 录制

Ubuntu 终端 B 先检查数据，再试录：

```bash
bash /mnt/d/workspace/odin_capture/record.sh --probe
bash /mnt/d/workspace/odin_capture/record.sh --seconds 60
```

正式采集直到手动停止：

```bash
bash /mnt/d/workspace/odin_capture/record.sh --seconds 0
```

在终端 B 按 **Ctrl+C 一次**结束录制，等待校验和复制完成，再停止终端 A 的驱动。录制按 2 GiB 分包；录制前要求 Linux 和 D 盘至少各有 10 GiB 可用空间，录制期间不足 3 GiB 时提前停止。空间检查不能代替按数据量规划容量。

bag 原件：`/root/odin_bags/日期_时间/bag/`。成功校验后复制整个采集目录至 `D:\workspace\odin_bags\日期_时间\`，并校验 SHA-256。目录还包含标定、配置、版本、驱动修改补丁、运行日志、QoS 和消息统计。

本机首次试录目录为 `20261015_103511`，时长约 59.9 秒，bag 为 1,192,738,816 字节（约 1.11 GiB）。那次只保存 JPEG 图像，约每分钟 1.1 GiB。同时录制 bgr8 图像会额外增加约 60 MiB/s，未包含深度补全输出时约每分钟 4–5 GiB。当前还默认录制深度补全图和补全点云，容量需求会进一步增加，应按实际试录大小规划。

录制清单：

| 话题 | 内容 |
| --- | --- |
| `/odin1/cloud_raw` | 驱动原始点云，包含 offset_time 等字段 |
| `/odin1/imu` | IMU |
| `/odin1/image` | bgr8 图像，sensor_msgs/msg/Image，供 RViz 和算法直接订阅 |
| `/odin1/image/compressed` | 设备 JPEG 图像 |
| `/odin1/odometry` | 设备里程计 |
| `/odin1/odometry_highfreq` | 高频里程计 |
| `/odin1/cloud_slam` | 设备 SLAM 点云 |
| `/odin1/depth_img_competetion` | 深度补全图，sensor_msgs/msg/Image |
| `/odin1/depth_img_competetion_cloud` | 深度补全点云，sensor_msgs/msg/PointCloud2 |
| `/tf` | 位姿和坐标变换 |
| `/tf_static` | 若有发布则记录；当前驱动外参在 `/tf` |

`cloud_raw` 是厂家驱动提供的数据，默认保留厂家置信度过滤阈值 35；并非保证无过滤的硬件数据。`calib.yaml` 保存设备标定。设备时间戳与主机接收时间不属于同一时间轴，离线算法应使用消息 header 时间和点内 offset_time，避免混用 bag 接收时间。

每次新录制会保存 `recording_topics.json`；录后校验和严格回放验证按该次采集的清单检查，包括新增的两路深度补全输出。已有清单的旧 bag 继续按原清单校验；没有该文件的旧试录仍按 JPEG 图像清单检查，不要求 `/odin1/image` 或深度补全输出。

## 图像回放和算法使用

先停止实时驱动。Ubuntu 终端 A 一键打开图像和点云视图：

```bash
bash /mnt/d/workspace/odin_capture/view.sh
```

终端 B 回放原有试录（新 bag 则替换目录）：

```bash
bash /mnt/d/workspace/odin_capture/replay.sh /root/odin_bags/20261015_103511 --start-paused
```

等待数秒后，在终端 B 按空格开始播放，再按空格可暂停。如终端不支持按键控制，也可在终端 C 执行：

```bash
source /mnt/d/workspace/odin_capture/env.sh
ros2 service call /rosbag2_player/resume rosbag2_interfaces/srv/Resume '{}'
```

`view.sh` 会自动启动 JPEG 解码器，所以旧 bag 也能在 RViz 的 Camera 面板看到图像。新 bag 已有 `/odin1/image` 时，解码器会停止额外发布，避免重复帧。关闭该 RViz 窗口时会一起停止它启动的解码器。

如使用现有 RViz 窗口，可通过 File → Open Config 加载 `D:\workspace\odin_capture\playback.rviz`，另开终端运行 `bash /mnt/d/workspace/odin_capture/image_bridge.sh` 为旧 bag 解码。

算法直接订阅 `/odin1/image`，类型 `sensor_msgs/msg/Image`，编码 `bgr8`；Python 可用 `CvBridge().imgmsg_to_cv2(msg, desired_encoding='bgr8')` 得到 OpenCV 图像。标定见该采集目录的 `calib.yaml`，使用 `msg.header.stamp` 做时间关联。这里的 bgr8 是设备 JPEG 解码后的图像，不是额外采集的无压缩传感器原始数据，也未经去畸变。

不启动 RViz、只运行算法时，旧 bag 需要另开一个终端执行：

```bash
bash /mnt/d/workspace/odin_capture/image_bridge.sh
```

新 bag 回放时已有 Image 话题，可直接运行算法。也可以直接订阅 JPEG 话题，并用 `CvBridge().compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')` 解码，原有数据无需重新采集。

## 4. 查看与回放

替换下例中的实际采集目录。先停止实时驱动：

```bash
source /mnt/d/workspace/odin_capture/env.sh
ros2 bag info /root/odin_bags/日期_时间/bag
bash /mnt/d/workspace/odin_capture/check.sh /root/odin_bags/日期_时间
bash /mnt/d/workspace/odin_capture/replay.sh /root/odin_bags/日期_时间
```

算法需要从第一条消息完整接收时，先启动算法订阅端，再暂停启动播放器，等待连接完成后恢复：

```bash
bash /mnt/d/workspace/odin_capture/replay.sh /root/odin_bags/日期_时间 --start-paused
# 另一个已加载 env.sh 的终端，等待数秒后执行：
ros2 service call /rosbag2_player/resume rosbag2_interfaces/srv/Resume '{}'
```

可自动执行严格回放验证，逐话题对比 bag 消息数和订阅收到的消息数：

```bash
source /mnt/d/workspace/odin_capture/env.sh
python3 /mnt/d/workspace/odin_capture/verify_replay.py /root/odin_bags/日期_时间
```

首次试录已通过此验证；`replay_validation.json` 中全部话题计数一致，JPEG 可解码为 1600×1296，原始点云每帧 49,152 点。设备时间戳间隔检查未发现超过各话题中位间隔两倍的间隙；这些检查描述本次短时试录结果。

另一个已加载 `env.sh` 的终端可运行 `ros2 topic hz /odin1/imu`。需要可视化时运行 `rviz2 --ros-args -p use_sim_time:=true`，使用点云消息中的 frame 或 TF 根坐标系。设备时间与 bag 接收时间不同，RViz 的时钟设置需结合实际消息时间轴检查；离线解码验证不依赖 RViz。

## 排查

- `lsusb -d 2207:0019` 无输出：检查 Windows 的 `usbipd list`，重新执行 attach；换 USB 3 数据线/接口并检查电源。
- 已枚举但无法打开：检查 udev 权限及是否有其他程序占用设备。当前 WSL 默认用户为 root。
- 话题存在但无消息：查看驱动日志中的固件、USB、初始化、流启用错误；录制脚本会拒绝在关键数据流缺失时开始录制。
- `validation.json` 的频率基于 bag 接收时间，用于吞吐检查；它不能证明设备端绝对无丢帧。时间戳倒退会使校验失败并保留原始数据。
- WSL USB 转接的稳定性需要以实际运行结果为准；持续发生 USB 断连时先处理连接与供电，再考虑原生 Ubuntu。

重新编译：`bash /mnt/d/workspace/odin_capture/build.sh`。所有终端使用 `env.sh` 保持相同 ROS_DOMAIN_ID（默认 71）及中间件配置。
