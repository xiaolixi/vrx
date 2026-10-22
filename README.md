# 说明
VRX是Virtual RobotX的简称  
VRX中具有波浪自适应模块无人船（wave adaptive modular vehicle，WAMV）模型，可自定义设置WAMV的推进器及搭载的传感器。

仓库地址：https://github.com/osrf/vrx

# 修改点：
- sdf的路径和rsp的remmaping修正
- 新增controller作为学习用

## 部署：ros2 jazzy
# vrx_urdf 下各个目录的作用

`vrx_urdf` 不是 ROS 包,而是一个**容器目录**,里面装着三个 ROS 2 包,共同负责 wam-v 的 URDF/SDF 模型定义与生成。

## 一、顶层结构

```
vrx_urdf/
├── vrx_gazebo/          ← ① 模型生成工具 + 配置合规检查
├── wamv_description/    ← ② 船体本体 URDF(纯描述,无 Gazebo 插件)
└── wamv_gazebo/         ← ③ Gazebo 专用 URDF(传感器+插件+水动力)
```

三个包按"**描述分离**"原则分工,层层组合。

## 二、vrx_gazebo 包:模型生成工具

### 定位
提供**工具脚本**和**合规性配置**,用于动态生成 wam-v URDF(根据用户 YAML 配置组合传感器)。

### 目录结构

```
vrx_gazebo/
├── config/
│   ├── wamv_config/
│   │   ├── component_compliance/      ← 传感器合规规则
│   │   │   ├── bounding_boxes.yaml    ← 传感器包围盒(防超出船体)
│   │   │   └── numeric.yaml           ← 数量限制(最多几个相机等)
│   │   ├── thruster_compliance/       ← 推进器合规规则
│   │   │   ├── bounding_boxes.yaml
│   │   │   └── numeric.yaml
│   │   ├── example_component_config.yaml   ← 用户配置示例(传感器布局)
│   │   └── example_thruster_config.yaml    ← 用户配置示例(推进器布局)
│   ├── rviz_vrx_final.rviz            ← RViz 预设视角(完整模型)
│   └── rviz_vrx_rsp.rviz              ← RViz 预设视角(仅 RSP)
│
├── hooks/
│   ├── resource_paths.dsv.in          ← 给 ament 注入资源路径
│   └── resource_paths.sh
│
├── launch/
│   ├── generate_wamv.launch.py        ← 从 YAML 生成 wam-v URDF
│   └── rviz.launch.py                 ← 启动 RViz 查看模型
│
├── models/                            ← 传感器/执行器 SDF 模型库
│   ├── 3d_lidar/                      ← 3D 激光雷达
│   ├── ball_shooter/                  ← 发球装置
│   ├── battery/                       ← 电池
│   ├── blue_projectile/               ← 蓝色弹丸
│   ├── cpu_cases/                     ← CPU 机箱
│   ├── mono_camera/                   ← 单目相机
│   └── sensor_post/                   ← 传感器支架
│
├── scripts/
│   ├── __init__.py
│   └── generate_wamv.py               ← 命令行生成工具
│
└── src/vrx_gazebo/
    ├── __init__.py
    ├── compliance.py                  ← 合规检查逻辑
    ├── configure_wamv.py              ← 配置解析 + URDF 组装核心
    └── utils.py                       ← 工具函数
```

### 核心作用
- 用户提供 YAML 描述想要的传感器组合
- `configure_wamv.py` 解析 YAML,调 `compliance.py` 检查合规(数量/位置不越界)
- 输出定制化的 `wamv_gazebo.urdf.xacro`

## 三、wamv_description 包:船体本体

### 定位
描述 wam-v 的**纯物理本体**——船体、浮筒、电池、发动机、螺旋桨。**不含任何 Gazebo 插件**,可在任何仿真器或纯可视化用。

### 目录结构

```
wamv_description/
├── hooks/
│   ├── resource_paths.dsv.in          ← 注入 mesh 资源路径
│   └── resource_paths.sh
│
├── models/                            ← 3D 模型资源(mesh + 贴图)
│   ├── WAM-V-Base/                    ← 船体本体(双浮筒+横梁)
│   │   ├── mesh/
│   │   │   ├── WAM-V-Base.dae         ← Collada mesh
│   │   │   ├── WAM-V_Albedo.png       ← 反照率贴图
│   │   │   ├── WAM-V_Metalness.png    ← 金属度
│   │   │   ├── WAM-V_Normal.png       ← 法线
│   │   │   └── WAM-V_Roughness.png    ← 粗糙度
│   │   └── model.config
│   ├── engine/                        ← 发动机 mesh
│   │   ├── mesh/engine.dae
│   │   └── model.config
│   └── propeller/                     ← 螺旋桨 mesh
│       ├── mesh/propeller.dae
│       └── model.config
│
└── urdf/                              ← URDF xacro 文件
    ├── wamv_base.urdf.xacro           ← 船体宏(双浮筒+横梁+关节)
    ├── battery.xacro                  ← 电池宏(左右各一)
    ├── cpu_cases.xacro                ← CPU 机箱宏
    └── thrusters/
        └── engine.xacro               ← 发动机+螺旋桨宏
```

### 核心作用
- 定义 `wamv/base_link` 等船体 link 和 joint
- 提供 mesh 文件让船看起来真实
- 被 `wamv_gazebo` 的 xacro include 组合

## 四、wamv_gazebo 包:Gazebo 专用扩展

### 定位
在 `wamv_description` 基础上**加 Gazebo 专属内容**:传感器、执行器、水动力插件、推进器布局。这是 spawn 到 gz 时实际用的 URDF。

### 目录结构

```
wamv_gazebo/
├── models/
│   └── gps/                           ← GPS 传感器 mesh
│       ├── mesh/gps.dae
│       └── model.config
│
└── urdf/
    ├── wamv_gazebo.urdf.xacro         ← 顶层入口(launch.py 默认调它)
    ├── wamv_gazebo.xacro              ← 船体本体+水动力宏
    ├── macros.xacro                   ← 通用 Gazebo 插件宏
    │
    ├── components/                    ← 传感器/执行器 xacro
    │   ├── wamv_camera.xacro          ← 普通相机
    │   ├── wamv_3d_lidar.xacro        ← 3D 激光雷达
    │   ├── lidar.xacro                ← 通用 lidar 宏
    │   ├── wamv_planar_lidar.xacro    ← 2D 平面激光
    │   ├── wamv_gps.xacro             ← GPS
    │   ├── wamv_imu.xacro             ← IMU
    │   ├── wamv_p3d.xacro             ← ground truth 位姿
    │   ├── wamv_pinger.xacro          ← 声学信标接收
    │   └── ball_shooter.xacro         ← 发球装置
    │
    ├── dynamics/
    │   └── wamv_gazebo_dynamics_plugin.xacro   ← 水动力插件(浮力/水阻/波浪)
    │
    └── thruster_layouts/              ← 推进器布局(可切换)
        ├── wamv_aft_thrusters.xacro   ← H 布局(默认, 后部双发)
        ├── wamv_t_thrusters.xacro     ← T 布局
        ├── wamv_x_thrusters.xacro     ← X 布局
        └── wamv_gazebo_thruster_config.xacro  ← 推进器插件配置宏
```

### 核心作用
- `wamv_gazebo.urdf.xacro` 是 [Model.generate()](file:///d:/data/virtualmachinedir/vrx-jazzy/src/vrx_gz/src/vrx_gz/model.py#L259) 的默认入口
- 组合 `wamv_description` 的船体 + 自己的传感器 + 水动力插件 + 推进器布局
- 通过 xacro 参数切换不同配置(`thruster_config`、`vrx_sensors_enabled` 等)

## 五、三个包的依赖关系

```
┌─────────────────────────────────────────────────┐
│  wamv_gazebo (顶层入口, launch 默认用这个)       │
│  ─ 传感器 xacro + 水动力插件 + 推进器布局        │
│  ─ xacro 参数切换配置                            │
└─────────────┬───────────────────────────────────┘
              │ include
              ▼
┌─────────────────────────────────────────────────┐
│  wamv_description (船体本体)                     │
│  ─ base_link + 浮筒 + 电池 + 发动机              │
│  ─ 提供 mesh 资源                                │
└─────────────────────────────────────────────────┘
              ▲
              │ 工具生成/检查
┌─────────────┴───────────────────────────────────┐
│  vrx_gazebo (生成工具, 可选)                     │
│  ─ 从 YAML 生成定制 xacro                        │
│  ─ 合规检查(数量/位置)                          │
│  ─ 提供传感器 SDF 模型库                         │
└─────────────────────────────────────────────────┘
```

## 六、典型使用场景

### 场景 A:直接用默认配置(launch.py 默认路径)
```
competition.launch.py
  → Model.generate()
    → xacro wamv_gazebo/urdf/wamv_gazebo.urdf.xacro
       → include wamv_description/urdf/wamv_base.urdf.xacro (船体)
       → include wamv_gazebo/urdf/components/*.xacro (传感器)
       → include wamv_gazebo/urdf/thruster_layouts/wamv_aft_thrusters.xacro
```
**只用 wamv_gazebo + wamv_description**,不碰 vrx_gazebo。

### 场景 B:自定义传感器配置
```
1. 写 my_components.yaml (要哪些传感器, 放哪)
2. ros2 launch vrx_gazebo generate_wamv.launch.py \
     component_xacro_file:=my_components.yaml
   → vrx_gazebo 调 configure_wamv.py 生成定制 xacro
3. 把生成的 xacro 传给 Model.set_urdf()
```
**三个包都用**。

### 场景 C:仅可视化
```
ros2 launch wamv_gazebo rviz.launch.py
```
直接在 RViz 里看模型,不启动 Gazebo。

## 七、设计哲学

1. **描述与仿真器解耦**:`wamv_description` 只管"船长什么样",不含任何仿真器相关代码,可复用到任意仿真器。

2. **Gazebo 扩展独立**:`wamv_gazebo` 专注 Gazebo 专属内容(插件/传感器),不污染本体描述。

3. **工具与数据分离**:`vrx_gazebo` 是工具,不包含模型本身,只读 YAML + 调度其他两个包的 xacro。

4. **xacro 组合优于硬编码**:通过参数(`thruster_config`、`vrx_sensors_enabled`)和 include 实现配置切换,避免维护多份 URDF。

## 总结表

| 包 | 一句话定位 | 主要内容 |
|---|---|---|
| `wamv_description` | 船体本体描述 | base_link + 浮筒 + 电池 + 发动机 mesh + xacro |
| `wamv_gazebo` | Gazebo 专用扩展 | 传感器 xacro + 水动力插件 + 推进器布局 |
| `vrx_gazebo` | 模型生成工具 | YAML 配置 + 合规检查 + 生成脚本 + 传感器 SDF 库 |

理解这三个包的分工,就理解了 VRX 模型系统的整个架构——**描述、扩展、工具**三层分离,通过 xacro include 组合。



## world: 
- wildlife_task: 鳄鱼、鸭嘴兽、乌龟 野生动物，可随波逐流
- wayfinding_task: 三个定位的杆
```
    <plugin
      filename="libWayfindingScoringPlugin.so"
      name="vrx::WayfindingScoringPlugin">
      <vehicle>wamv</vehicle>
      <task_name>wayfinding</task_name>
      <waypoints>
        <waypoint>
          <pose>-33.7226766699 150.6740630167 1.21756121843</pose>
        </waypoint>
        <waypoint>
          <pose>-33.7220832884 150.67379127877 1.0</pose>
        </waypoint>
        <waypoint>
          <pose>-33.7226013209 150.67504609858 1.0</pose>
        </waypoint>
      </waypoints>
      <markers>
        <scaling>0.2 0.2 2.0</scaling>
        <height>0.5</height>
      </markers>

      <task_info_topic>/vrx/task/info</task_info_topic>
      <initial_state_duration>10</initial_state_duration>
      <ready_state_duration>10</ready_state_duration>
      <running_state_duration>300</running_state_duration>
      <release_topic>/vrx/release</release_topic>
    </plugin>
```
- npark: 另一个仿真环境

# Virtual RobotX (VRX)
This repository is the home to the source code and software documentation for the VRX simulation environment, which supports simulation of unmanned surface vehicles in marine environments.
* Designed in coordination with RobotX organizers, this project provides arenas and tasks similar to those featured in past and future RobotX competitions, as well as a description of the WAM-V platform.
* For RobotX competitors this simulation environment is intended as a first step toward developing tools prototyping solutions in advance of physical on-water testing.
* We also welcome users with simulation needs beyond RobotX. As we continue to improve the environment, we hope to offer support to a wide range of potential applications.

## A new modernization development: Gazebo Harmonic and ROS 2 Jazzy

> [!NOTE]
> This development effort was executed by the
> [Honu Robotics](https://honurobotics.com) team, thanks to the sponsorship
> of [RoboNation](https://robonation.org/).

We are happy to announce that the repository has been ported to use supported
versions of Gazebo and ROS 2:
  * Code is now working with Gazebo Harmonic and ROS 2 Jazzy
  * This is the recommended configuration for new users.
  * Users who wish to continue running Gazebo Garden and ROS 2 Humble can still do so using the `humble` branch of this repository.

## The VRX Competition
The VRX environment is also the "virtual venue" for the [VRX Competition](https://github.com/osrf/vrx/wiki). Please see our Wiki for tutorials and links to registration and documentation relevant to the virtual competition.

[![VRX](images/sydney_regatta_gzsim.png)](https://vimeo.com/851696025 "Gazebo Virtual RobotX v. 2.3 - Click to Watch!")
![ROS 2 CI](https://github.com/osrf/vrx/workflows/ROS%202%20CI/badge.svg)

## Getting Started

 * Watch the [Release 2.3 Highlight Video](https://vimeo.com/851696025).
 * The [VRX Wiki](https://github.com/osrf/vrx/wiki) provides documentation and tutorials.
 * The instructions assume a basic familiarity with the ROS environment and Gazebo.  If these tools are new to you, we recommend starting with the excellent [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials)
 * For technical problems, please use the [project issue tracker](https://github.com/osrf/vrx/issues) to describe your problem or request support.

## Reference

If you use the VRX simulation in your work, please cite our summary publication, [Toward Maritime Robotic Simulation in Gazebo](https://wiki.nps.edu/display/BB/Publications?preview=/1173263776/1173263778/PID6131719.pdf):

```
@InProceedings{bingham19toward,
  Title                    = {Toward Maritime Robotic Simulation in Gazebo},
  Author                   = {Brian Bingham and Carlos Aguero and Michael McCarrin and Joseph Klamo and Joshua Malia and Kevin Allen and Tyler Lum and Marshall Rawson and Rumman Waqar},
  Booktitle                = {Proceedings of MTS/IEEE OCEANS Conference},
  Year                     = {2019},
  Address                  = {Seattle, WA},
  Month                    = {October}
}
```
## 🛠️ Getting Help and Contributing

VRX is an open source project supported by the community. If you run into issues, need help, or have suggestions:

- 💬 **Ask for help or report bugs** by opening an [issue](https://github.com/osrf/vrx/issues). Please include as much detail as possible, including:
  - Steps to reproduce the issue
  - Your system setup (OS, ROS version, etc.)
  - Relevant error messages or logs

- 🛠️ **Found a fix or improvement?** We welcome contributions! Submit a [pull request](https://github.com/osrf/vrx/pulls) with your proposed changes.

- 📫 **Please do not email the maintainers with technical questions.** Using GitHub issues helps ensure that questions and solutions are visible and searchable for the whole community.

Your feedback and participation help make VRX better for everyone — thank you for contributing!

## Contributors

We continue to receive important improvements from the community.  We have done our best to document this on our [Contributors Wiki](https://github.com/osrf/vrx/wiki/Contributors).





