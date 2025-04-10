# RC_BRINGUP

ROS2 的 launch 集合包，用于统一启动驱动、播放包与辅助功能。

---

## utils_bringup.launch.py

### 功能描述
- 启动辅助节点，包含：
  - `rosbridge`（ROS2-ROS1 桥接）
  - `foxglove_bridge`（Foxglove 可视化工具）
  - `robot_tf_publish`（发布静态 TF 树）
  - `rosbag`（记录当前话题数据）
  - `rosbridge_websocket` (开启websocket桥接)[相关连接](https://roslibpy.readthedocs.io/en/latest/contributing.html)
- 特殊说明：
  - rosbridge 需额外启动 `roscore`
  - rosbag 仅记录已有的点云、IMU、TF 话题,**后续添加的不会记录**
  - rosbag 自动保留最近 5 条数据至 `data` 目录，单包最大 2GB
> rosbag的限制可以在srcipts/rosbag_record.py中更改,或者通过传入参数修改

### 参数表
| 参数名              | 说明             | 默认值  |
| ------------------- | ---------------- | ------- |
| `use_rosbag_record` | 启用话题录制功能 | `false` |
| `use_tf_publish`    | 启用静态 TF 发布 | `false` |
| `use_ros1_bridge`   | 启用 ROS1 桥接   | `true`  |

---

## rosbag_with_utils.launch.py

### 功能描述
- 自动播放 `bag_play` 目录中最新的数据包（需提供文件夹结构）：
```text
  ./bag_play
  └── bag_example
      ├── test.db3
      └── metadata
```
- 同步启动 utils_bringup.launch.py，并强制覆盖以下参数：
```txt
use_rosbag_record = false
use_tf_publish = false
```
## driver_with_utils.launch.py
### 功能描述
- 启动硬件驱动节点 + 辅助节点
- 驱动实现详见 [`my_driver`](../my_driver/README.md) 功能包
- 强制参数配置：
```txt
use_rosbag_record = true
use_tf_publish = true
```