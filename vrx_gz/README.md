我先看一遍 `vrx_gz` 的结构和关键文件，再给你一份能对得上代码的解释。
`vrx_gz` 是这个仓库的**仿真主包**：它既提供 Gazebo Sim（gz-sim8）里的世界、模型和 C++ 仿真插件，也提供一套 Python 启动层，用来生成 SDF、把机器人放进场景，并把 Gazebo 话题桥接成 ROS 2 话题。换句话说，**船如何在水里动、比赛如何判分、传感器数据如何进入 ROS，全在这个包里决定**。

先看它的运行时数据流：

<svg viewBox="0 0 680 190" width="100%" role="img" xmlns="http://www.w3.org/2000/svg">
<title>vrx_gz 运行时数据流</title>
<desc>从 launch 层到 gz sim 仿真、ros_gz_bridge 桥接，最终输出 ROS 2 话题的四段流程。</desc>
<defs>
<marker id="arrow" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="6" markerHeight="6" orient="auto-start-reverse">
<path d="M2 1L8 5L2 9" fill="none" stroke="context-stroke" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round"/>
</marker>
</defs>
<g>
<rect x="45" y="44" width="128" height="56" rx="8" fill="#E6F1FB" stroke="#185FA5" stroke-width="0.5"/>
<text x="109" y="66" text-anchor="middle" dominant-baseline="central" font-family="var(--font-sans)" font-size="13" font-weight="500" fill="#0C447C">launch 层</text>
<text x="109" y="84" text-anchor="middle" dominant-baseline="central" font-family="var(--font-sans)" font-size="12" fill="#185FA5">解析参数与 YAML</text>
</g>
<path d="M173 72 H193" fill="none" stroke="#888780" stroke-width="1.5" marker-end="url(#arrow)"/>
<g>
<rect x="199" y="44" width="128" height="56" rx="8" fill="#E1F5EE" stroke="#0F6E56" stroke-width="0.5"/>
<text x="263" y="66" text-anchor="middle" dominant-baseline="central" font-family="var(--font-sans)" font-size="13" font-weight="500" fill="#085041">Gazebo 仿真</text>
<text x="263" y="84" text-anchor="middle" dominant-baseline="central" font-family="var(--font-sans)" font-size="12" fill="#0F6E56">world + 船 + 插件</text>
</g>
<path d="M327 72 H347" fill="none" stroke="#888780" stroke-width="1.5" marker-end="url(#arrow)"/>
<g>
<rect x="353" y="44" width="128" height="56" rx="8" fill="#FAEEDA" stroke="#854F0B" stroke-width="0.5"/>
<text x="417" y="66" text-anchor="middle" dominant-baseline="central" font-family="var(--font-sans)" font-size="13" font-weight="500" fill="#633806">ros_gz_bridge</text>
<text x="417" y="84" text-anchor="middle" dominant-baseline="central" font-family="var(--font-sans)" font-size="12" fill="#854F0B">GZ ↔ ROS 话题</text>
</g>
<path d="M481 72 H501" fill="none" stroke="#888780" stroke-width="1.5" marker-end="url(#arrow)"/>
<g>
<rect x="507" y="44" width="128" height="56" rx="8" fill="#EEEDFE" stroke="#534AB7" stroke-width="0.5"/>
<text x="571" y="66" text-anchor="middle" dominant-baseline="central" font-family="var(--font-sans)" font-size="13" font-weight="500" fill="#3C3489">ROS 2 话题</text>
<text x="571" y="84" text-anchor="middle" dominant-baseline="central" font-family="var(--font-sans)" font-size="12" fill="#534AB7">/wamv/sensors/…</text>
</g>
<text x="45" y="140" font-family="var(--font-sans)" font-size="12" fill="#5F5E5A">内含 13 个任务世界 + 72 个赛季变体、30 个模型、20+ 个 C++ 仿真插件</text>
<text x="45" y="160" font-family="var(--font-sans)" font-size="12" fill="#5F5E5A">hooks/ 注册 GZ_SIM_RESOURCE_PATH，gz sim 才能按世界名找到 SDF 与模型</text>
</svg>

## 一、它到底是什么

`vrx_gz` 是 VRX（Virtual RobotX 无人艇竞赛）的**Gazebo 侧基座**，其 `package.xml` 写的是 “VRX gazebo resources”，但实际功能远超“资源”：

| 层 | 内容 | 作用 |
|---|---|---|
| 仿真资产 | `worlds/`（13 个任务世界 + `2023_phase2/`48 个 + `2023_practice/`24 个）、`models/`（30 个：码头、浮标、障碍、比赛船、海浪面） | gz sim 加载的场景 |
| C++ 插件 | `src/*.cc` 共 20+ 个 `.so` | 水的物理、浮力、风力、波场、评分与道具交互 |
| Python 启动层 | `src/vrx_gz/`（`launch.py` `model.py` `bridges.py` `payload_bridges.py` `bridge.py`） | 生成 SDF、spawn、生成 ROS 桥接 |
| 启动入口 | `launch/*.launch.py` 4 个 | 用户实际敲的命令 |
| 环境注入 | `hooks/resource_paths.{dsv.in,sh}` | 把 worlds/models 加进 `GZ_SIM_RESOURCE_PATH`，`gz sim xxx.sdf` 才能只写名字就找到文件 |

注意 CMake 里用 `find_package(gz-sim8)`，这是 **Gazebo Harmonic**（ROS 2 Jazzy 时代），不是 Garden/Fortress。

## 二、C++ 插件分三类（这才是“仿真像不像”的关键）

**1. 水与船体动力学**
- `Wavefield.cc` + `Waves` —— 波场数学核心，生成海面高度/法线/流速场
- `WaveVisual.cc` —— 把波场渲染成可见水面
- `PolyhedraBuoyancyDrag.cc` + `PolyhedronVolume` / `ShapeVolume` —— 基于多面体几何算浮力与阻尼，所以船的浮态是**算出来的**，不是固定的
- `SimpleHydrodynamics.cc`、`Surface.cc`、`USVWind.cc` —— 附加水动力、水面判据、风扰

**2. 竞赛评分（`worlds/*.sdf` 里挂 `libXxxScoringPlugin.so`）**
`ScoringPlugin.cc` 是抽象基类（状态机：init → running → finished + 任务信息发布），派生：Stationkeeping、Wayfinding、Navigation、Gymkhana、Perception、AcousticPerception、AcousticTracking、ScanDock、Wildlife。这就是为什么你能在 ROS 侧收到 `/vrx/task/info` 和 `*_pose_error`。

**3. 道具与交互**
`BallShooterPlugin`（射球）、`LightBuoyPlugin`（灯浮标）、`PlacardPlugin`（识别牌）、`AcousticPingerPlugin`（声学信标）、`PublisherPlugin`（把仿真状态发到 gz 话题）。

## 三、Python 启动层的核心设计（看这段就懂全包）

**① `Model` 是“配置 → SDF 字符串”的工厂**（`model.py`）

```
YAML(config) → Model.FromConfig → xacro 渲染 urdf → 写 tmp/model.urdf
             → gz sdf -p model.urdf → SDF 字符串 → ros_gz_sim create -string ...
```

关键点：它**不读 models 目录里的现成 SDF**，而是每次用 `wamv_gazebo/urdf/wamv_gazebo.urdf.xacro` 现场 xacro 展开成 URDF，再让 `gz sdf -p` 转成 SDF。所以改船体应在 `vrx_urdf`/`wamv_gazebo` 的 xacro 里改，改 `models/` 下的 SDF 不生效。

**② 桥接是“从 SDF 反推”的**（`payload_from_sdf`）
用 sdformat14 解析刚生成的 SDF，遍历每个 link 的每个 sensor，得到 `{sensor_name: [link_name, sensor_type]}`，再交给 `payload_bridges.py` 按类型生成桥接：相机→image+camera_info、RGBD→4 个话题、IMU/GPS/激光、`thruster_thrust_*`、`BallShooter`、`AcousticPinger`。**加了新传感器不用手写桥接表**，这是整个包最聪明的一处。

**③ `bridges.py` 管固定话题**
`clock`、`task_info`、`pose/pose_static`（gz `Pose_V` → `TFMessage`）、`joint_states`，以及各任务专属话题（waypoints、goal、landmark 上报、color_sequence 上报）。方向由 `BridgeDirection` 编成 `@` / `[` / `]` 符号，再由 `parameter_bridge` 原生解析。

**④ spawn 后统一进命名空间**
每个模型一个 `GroupAction([PushRosNamespace(model_name), ...])`，节点名变 `/wamv/robot_state_publisher`，同时 `frame_prefix` 把 TF frame 变成 `wamv/base_link`，`/joint_states` 显式 remap 到 `/wamv/joint_states`。多船（`two_wamv.yaml`）就是靠这套隔离的。

## 四、四个 launch 入口怎么选

```bash
# 1) 一键全流程：世界 + 默认 wamv(-532,162,0) + 全部桥接
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta

# 2) 多船走 YAML
ros2 launch vrx_gz competition.launch.py config_file:=$(ros2 pkg prefix vrx_gz)/share/vrx_gz/config/two_wamv.yaml

# 3) 只起环境，不 spawn（适合先开 gz 再手动加船）
ros2 launch vrx_gz vrx_environment.launch.py world:=sydney_regatta

# 4) 纯命令行 spawn 单船，可指定 8 个 payload 槽位
ros2 launch vrx_gz spawn.launch.py name:=wamv model:=wam-v x:=-532 y:=162 slot0:=camera_front
```

`sim_mode` 三值：`full`（spawn + 桥接）、`sim`（只 spawn，不桥接）、`bridge`（只桥接，不 spawn）。用 `full` 时桥接节点挂在 `OnProcessExit(gz_spawn_entity)` 之后启动 —— 因为 `ros_gz_sim create` 是一次性进程，必须等它退出、实体存在了再桥接，否则话题还不存在。

