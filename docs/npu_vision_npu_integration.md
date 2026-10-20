# `cix_ros_noe_yolo` 集成说明与外部项目适配

本文说明本仓库中为使用板载 NPU 所做的衔接改动，并说明如何在**其他 ROS 2 工程**中复用、适配 [`cix_ros_noe_yolo`](../src/cix_ros_noe_yolo/) 包。关键 ROS 接口名、话题名、参数名保留英文，便于对照代码。

---

## 1. 目的与范围

### `cix_ros_noe_yolo` 负责什么

该包提供独立节点 **`npu_yolo_coco_inference`**，职责为：

- 订阅相机图像（默认 `sensor_msgs/Image`，`image_topic` 默认 `/camera/image_raw`）；
- 在 **CIX NPU** 上运行 YOLOv8n（`.cix` + NPU 运行时）与随包后处理；
- 发布 **`vision_msgs/Detection2DArray`**（默认 `/vision/detections_2d`）；
- 发布 **`std_msgs/Bool`**（默认 `/npu_vision_active`，带心跳），用于告知下游「当前由 NPU 负责检测结果」，便于与 CPU 侧 Ultralytics 等互斥。

该包**不包含**：Gazebo 仿真、Nav2、或 [`autonombot_vision`](../src/autonombot_vision/) 里的 `advanced_vision_navigator`。后者是**可选下游**，用于障碍物/可视化等同一套管线。

### 与本仓库其它组件的关系

| 组件 | 角色 |
|------|------|
| `cix_ros_noe_yolo` | NPU 推理 + 检测话题 + 活动标志 |
| `advanced_vision_navigator` | 可订阅上述话题，在 NPU 激活时跳过本地 YOLO，仍发布 `/vision_debug` 等 |
| `simulated_robot.launch.py` | 设置 DDS/域隔离；**不**启动 NPU 包，需第二终端手动启动 NPU launch |

---

## 2. 本仓库为配合 NPU 做了哪些修改

以下文件**不**内嵌 CIX NPU 运行时，只做网络域、启动与话题握手。

### 2.1 [`simulated_robot.launch.py`](../src/autonombot_launch/launch/simulated_robot.launch.py)

| 区域 | 与 NPU 的关系 |
|------|----------------|
| **`setup_network_isolation()`**（在文件 import 时执行） | 设置 `ROS_DOMAIN_ID`（当前为 **61**）、Fast DDS SHM 配置、回环相关环境变量等。第二进程（如 `npu_yolo_coco_inference.launch.py`）必须与主栈**同一套 DDS/域**，否则看不到 `/vision/detections_2d`、`/npu_vision_active`。 |
| **NPU 包** | **未**加入 launch。源码注释写明：NPU 需单独启动：`ros2 launch cix_ros_noe_yolo npu_yolo_coco_inference.launch.py …`。 |
| **`use_advanced_vision:=true`** | 启动 `advanced_vision_navigator`，传入 `use_sim_time`、`conf_thr`、`nms_thr` 等。无需在此写 NPU 专用参数；话题名使用 navigator 默认值即可（见下文对照表）。 |

### 2.2 [`advanced_vision_navigator.py`](../src/autonombot_vision/autonombot_vision/advanced_vision_navigator.py)

| 机制 | 行为 |
|------|------|
| **参数** | `npu_active_topic`（默认 `/npu_vision_active`）、`external_npu_detections_topic`（默认 `/vision/detections_2d`），须与 `npu_yolo_coco_inference` 发布的话题一致。 |
| **`Bool` 订阅（`npu_active_topic`）** | QoS：reliable + transient-local。回调 `_npu_active_cb` 设置 `_npu_claims_vision`。 |
| **`Detection2DArray` 订阅** | 回调 `_detections_2d_cb`：若 NPU 未声明激活则直接返回；否则走与本地 YOLO 相同的障碍物/标记/点云/`/vision_debug` 逻辑。 |
| **`image_callback`** | 仍处理 `/camera/image_raw` 以更新 `_last_cv_image`。若 `_npu_claims_vision` 为真，在解码更新后**直接 return**，**不**调用 `self.model(...)`（Ultralytics 空闲）。 |

**整体数据流**：终端 A 启动仿真 + navigator；终端 B 启动 `npu_yolo_coco_inference`；两者域一致后，NPU 发布检测与心跳，navigator 切换为消费 NPU 结果。

---

## 3. `cix_ros_noe_yolo` 包内要点（移植前必读）

### 3.1 构建与安装布局

- 包类型：`ament_python`（见 [`package.xml`](../src/cix_ros_noe_yolo/package.xml)）。
- [`setup.py`](../src/cix_ros_noe_yolo/setup.py) 将 `model/`（含 `yolov8n.cix`）与 `yolo_postprocess/` 安装到 `share/cix_ros_noe_yolo/...`，launch 与 `config/fastdds_shm_only.xml` 一并安装。

### 3.2 运行时依赖（未全部写在 `package.xml` 中）

[`npu_yolo_coco_engine.py`](../src/cix_ros_noe_yolo/src/npu_yolo_coco_engine.py) 需要：

- **CIX NPU 运行时**：Python 可导入 `NOE_Engine.EngineInfer`，并能加载 `.cix`。
- **`yolo_postprocess/` 目录**：内含 `object_detect_postprocess.py`、`label/coco_classes.py` 等；默认通过 `ament_index` 解析 `cix_ros_noe_yolo` 的 share 路径，也可用参数 `onnx_yolov8_root` 覆盖。

无上述环境时，节点在导入或 `EngineInfer` 初始化阶段会失败——**普通 x86 仅 CPU 开发机**通常无法跑通完整 NPU 路径。

### 3.3 节点 `npu_yolo_coco_inference`（[`npu_yolo_coco_inference.py`](../src/cix_ros_noe_yolo/src/npu_yolo_coco_inference.py)）

**主要参数：**

| 参数 | 含义 |
|------|------|
| `npu_model_path` | `.cix` 绝对路径；非空则优先使用 |
| `npu_model_name` | `npu_model_path` 为空时，在 share 下 `onnx_yolov8_n/{name}.cix` 解析（默认 `yolov8n`，可写 `yolov8s` 等，可省略 `.cix` 后缀） |
| `onnx_yolov8_root` | 覆盖 `onnx_yolov8_n` 根目录；空则用包内默认 |
| `conf_thr` / `nms_thr` | 置信度与 NMS，与后处理一致 |
| `image_topic` | 输入图像 |
| `detections_topic` | 输出的 `Detection2DArray` |
| `npu_active_topic` | 输出的「NPU 活动」`Bool` |
| `publish_npu_active_heartbeat` | 为 `true` 时定时重复发布 `true`；为 `false` 时仅启动时发布一次 `true`（仍保留 TRANSIENT_LOCAL 语义） |
| `npu_active_heartbeat_hz` | 心跳频率（有上下限裁剪；仅当 `publish_npu_active_heartbeat` 为真时生效） |
| `reenable_ultralytics_on_exit` | 节点销毁时是否向 `npu_active_topic` 发布 `false`（便于 CPU 侧恢复） |
| `npu_inference_fps_topic` | NPU 推理 FPS（`EngineInfer.get_cur_fps()`），默认 `/npu_inference_fps` |
| `pipeline_fps_topic` | 整条 pipeline 平滑 FPS，默认 `/vision/yolo_pipeline_fps` |
| `debug_image_topic` | 调试图；左上角叠加 **NPU 推理 FPS**（供 rqt） |

**QoS：**

- 检测发布：与 `qos_profile_sensor_data` 一致。
- `Bool`：reliable + transient-local，深度 1；启动后即发布一次 `true`；若 **`publish_npu_active_heartbeat`** 为真，则由定时器按 **`npu_active_heartbeat_hz`** 维持周期性 `true`。

### 3.4 Launch [`npu_yolo_coco_inference.launch.py`](../src/cix_ros_noe_yolo/launch/npu_yolo_coco_inference.launch.py)

- **不再**通过 launch 注入 `ROS_DOMAIN_ID` / Fast DDS 等环境变量；第二终端须与主栈（如 `simulated_robot.launch.py` 中 **`setup_network_isolation()`**）使用**相同**的 `ROS_DOMAIN_ID` 与 RMW 配置（在 shell 中 `export` 或使用你自己的上层 launch 统一设置）。

**注意**：若主进程与 NPU 进程 domain 不一致，将**看不到** `/vision/detections_2d`、`/npu_vision_active` 等话题。

---

## 4. 在其他 ROS 2 工程中如何适配（重点）

### 4.1 集成步骤

1. 将 **`cix_ros_noe_yolo`** 目录拷入目标工作空间的 `src/`（或 git submodule）。
2. 在**目标板**上安装 NPU 运行时与依赖，保证 `NOE_Engine` 可用。
3. `colcon build --packages-select cix_ros_noe_yolo`（或整体构建），然后 `source install/setup.bash`。
4. 启动：`ros2 launch cix_ros_noe_yolo npu_yolo_coco_inference.launch.py`，按需传入参数。

### 4.2 话题与命名

- **`image_topic`**：必须与你的相机/仿真图像话题一致（例如从 `/camera/image_raw` 改为项目中的实际名字）。
- **`detections_topic` / `npu_active_topic`**：若改名，**所有**订阅这些话题的节点都要同步（若使用本仓库的 `advanced_vision_navigator`，请设置其 `external_npu_detections_topic` 与 `npu_active_topic`）。

### 4.3 仿真时间 `use_sim_time`

- 与 Gazebo 等**仿真**且存在 `/clock` 时：主栈与 NPU 节点通常 **`use_sim_time:=true`**。
- **实机**无仿真时钟时：一般为 `false`，并与相机驱动、其余节点一致。

### 4.4 DDS / 多终端 / 多机

- 所有相关终端与机器必须使用**相同** `ROS_DOMAIN_ID` 与兼容的 RMW/Fast DDS 配置。
- 确保 NPU 终端与主栈使用**同一** `ROS_DOMAIN_ID` 与兼容的 DDS 配置，否则**看不到话题**。

### 4.5 不使用 `advanced_vision_navigator` 时

- **最少**：订阅 **`Detection2DArray`**（`detections_topic`）即可做自定义逻辑。
- 若需要与本仓库相同的「NPU 与 CPU 检测互斥」语义，需自行订阅 **`npu_active_topic`**，或参考 navigator 中 `_npu_claims_vision` 的用法。
- 若**只关心检测结果**，可以忽略 `Bool`，但应了解：节点退出时可能发布 `false`（取决于 `reenable_ultralytics_on_exit`）。

### 4.6 更换模型

- 将 **`npu_model_path`** 指向新的 `.cix` 文件。
- 若模型输出张量形状或解码方式**与 YOLOv8n 不一致**，需要修改 `NpuYoloCocoEngine.detect` 以及 `yolo_postprocess/` 下的后处理逻辑——这已超出「仅改 launch 参数」的适配范围。

---

## 5. 默认话题 / 参数与 `advanced_vision_navigator` 对照

| 项目 | `npu_yolo_coco_inference`（默认） | `advanced_vision_navigator`（对应参数） |
|------|------------------------------|----------------------------------------|
| NPU 活动标志 | 发布 `/npu_vision_active` | `npu_active_topic` → `/npu_vision_active` |
| 检测结果 | 发布 `/vision/detections_2d` | `external_npu_detections_topic` → `/vision/detections_2d` |
| NPU 推理 FPS | 发布 `/npu_inference_fps` | （可选订阅） |
| Pipeline FPS | 发布 `/vision/yolo_pipeline_fps` | （可选订阅） |
| 调试图 | 发布 `/vision/yolo_debug_image` | rqt；左上角为 NPU 推理 FPS |
| 相机图像 | 订阅 `/camera/image_raw` | 自身订阅 `/camera/image_raw`（用于 `_last_cv_image`） |

任一侧改名时，另一侧必须通过参数保持一致。

---

## 6. 相关文档

- 本仓库快速命令与两终端示例：[README.md](../README.md) 中「NPU vision inference」一节。
- CPU 侧 YOLO 与 `use_advanced_vision`：[yolo_advanced_vision.md](yolo_advanced_vision.md)。
