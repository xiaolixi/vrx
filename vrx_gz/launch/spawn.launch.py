# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# =============================================================================
# VRX 单机器人 spawn 启动器
# -----------------------------------------------------------------------------
# 作用: 通过命令行参数 spawn 一个机器人 (wam-v/usv 或 UAV) 到已运行的 Gazebo 仿真中,
#       并可选地启动 ROS<->GZ 桥接和竞赛话题桥接
#
# 与 competition.launch.py / vrx_environment.launch.py 的区别:
#   - 这两个文件走 config_file (YAML) 方式, 适合多船/预设配置
#   - 本文件走纯命令行参数方式, 适合临时 spawn 单个机器人, 调试时更灵活
#   - 支持 8 个传感器插槽 (slot0~slot7) 及各自 RPY, 可自定义 payload 组合
#
# 典型用法:
#   1. spawn 一只 wam-v 到默认位置:
#        ros2 launch vrx_gz spawn.launch.py \
#            name:=wamv model:=wam-v x:=-532 y:=162 Y:=1
#   2. spawn 时给 slot0 装 camera:
#        ros2 launch vrx_gz spawn.launch.py \
#            name:=wamv model:=wam-v slot0:=my_camera slot0_rpy:="15 0 0"
#   3. 仅 spawn 不桥接 (纯 gz):
#        ros2 launch vrx_gz spawn.launch.py sim_mode:=sim
# =============================================================================

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration

import vrx_gz.bridges
import vrx_gz.launch

from vrx_gz.model import Model


def parse_from_cli(context, world_name):
    # ---- 读取机器人基本属性 ----
    # name: 同时作为 gz 模型名和 ROS 命名空间
    # model: SDF 模型类型 (wam-v / usv / vrx_hexrotor / vrx_quadrotor ...)
    robot_name = LaunchConfiguration('name').perform(context)
    model_type = LaunchConfiguration('model').perform(context)

    # ---- 读取位姿参数 (xyz + RPY) ----
    x_pos = LaunchConfiguration('x').perform(context)
    y_pos = LaunchConfiguration('y').perform(context)
    z_pos = LaunchConfiguration('z').perform(context)
    r_rot = LaunchConfiguration('R').perform(context)
    p_rot = LaunchConfiguration('P').perform(context)
    y_rot = LaunchConfiguration('Y').perform(context)
    # position = [x, y, z, roll, pitch, yaw], 顺序与 Model.spawn_args() 对应
    position = [x_pos, y_pos, z_pos, r_rot, p_rot, y_rot]

    # ---- 读取可选 URDF (覆盖默认 xacro) ----
    robot_urdf = LaunchConfiguration('urdf').perform(context)

    # 构造 Model 对象
    model = Model(robot_name, model_type, position)

    # 若指定了 URDF 则覆盖默认的 wamv_gazebo.urdf.xacro
    if robot_urdf and robot_urdf != '':
        model.set_urdf(robot_urdf)

    # ---- 读取 8 个传感器插槽 (slot0~slot7) ----
    # 每个插槽可装一个传感器组件, 对应 wamv_gazebo.urdf.xacro 中的 slot 变量
    slot0_payload = LaunchConfiguration('slot0').perform(context)
    slot1_payload = LaunchConfiguration('slot1').perform(context)
    slot2_payload = LaunchConfiguration('slot2').perform(context)
    slot3_payload = LaunchConfiguration('slot3').perform(context)
    slot4_payload = LaunchConfiguration('slot4').perform(context)
    slot5_payload = LaunchConfiguration('slot5').perform(context)
    slot6_payload = LaunchConfiguration('slot6').perform(context)
    slot7_payload = LaunchConfiguration('slot7').perform(context)

    # 每个插槽的安装姿态 (Roll Pitch Yaw, 单位: 度)
    slot0_rpy = LaunchConfiguration('slot0_rpy').perform(context)
    slot1_rpy = LaunchConfiguration('slot1_rpy').perform(context)
    slot2_rpy = LaunchConfiguration('slot2_rpy').perform(context)
    slot3_rpy = LaunchConfiguration('slot3_rpy').perform(context)
    slot4_rpy = LaunchConfiguration('slot4_rpy').perform(context)
    slot5_rpy = LaunchConfiguration('slot5_rpy').perform(context)
    slot6_rpy = LaunchConfiguration('slot6_rpy').perform(context)
    slot7_rpy = LaunchConfiguration('slot7_rpy').perform(context)

    # 组装 payload 字典: {slot名: {'sensor': 传感器名, 'rpy': 安装姿态}}
    payloads = {
        'slot0': {'sensor': slot0_payload, 'rpy': slot0_rpy},
        'slot1': {'sensor': slot1_payload, 'rpy': slot1_rpy},
        'slot2': {'sensor': slot2_payload, 'rpy': slot2_rpy},
        'slot3': {'sensor': slot3_payload, 'rpy': slot3_rpy},
        'slot4': {'sensor': slot4_payload, 'rpy': slot4_rpy},
        'slot5': {'sensor': slot5_payload, 'rpy': slot5_rpy},
        'slot6': {'sensor': slot6_payload, 'rpy': slot6_rpy},
        'slot7': {'sensor': slot7_payload, 'rpy': slot7_rpy},
    }

    # ---- 按模型类型设置专属属性 ----
    if model.is_UAV():
        # UAV: 读取飞行时间(分钟), 据此计算电池容量
        # take flight time in minutes
        flight_time = LaunchConfiguration('flightTime').perform(context)

        model.set_flight_time(flight_time)
    elif model.is_USV():
        # USV (如 wam-v): 根据世界名设置波场尺寸, 影响水动力学计算
        # wavefield_size 来自 WAVEFIELD_SIZE 字典 (如 sydney_regatta -> 1000)
        model.set_wavefield(world_name)

    # 设置 payload (UAV 主要用此字段; URDF 模式下后续会被 payload_from_sdf 覆盖)
    model.set_payload(payloads)
    return model


def launch(context, *args, **kwargs):
    # ---- 在执行期解析 launch 参数 ----
    world_name = LaunchConfiguration('world').perform(context)
    sim_mode = LaunchConfiguration('sim_mode').perform(context)
    # 字符串 'True'/'False' 转为布尔值
    bridge_competition_topics = LaunchConfiguration(
        'bridge_competition_topics').perform(context).lower() == 'true'

    # 从命令行参数解析出单个 Model 对象
    model = parse_from_cli(context, world_name)

    # 收集所有要启动的进程/动作
    launch_processes = []

    # ---- spawn 机器人 + ROS<->GZ 桥接 ----
    # sim_mode 为 full/sim 时调用 ros_gz_sim create 注入 SDF;
    # sim_mode 为 full/bridge 时为模型建 pose/joint_states/传感器/thruster 桥接,
    # 启动 pose_tf_broadcaster 和 robot_state_publisher, 用 PushRosNamespace 隔离命名空间
    launch_processes.extend(vrx_gz.launch.spawn(sim_mode, world_name, model))
    # ---- 竞赛话题桥接 (可选) ----
    # 仅在需要桥接(full/bridge)且开启 bridge_competition_topics 时执行
    # competition_bridges 桥接: clock, task_info; 非 competition_mode 还桥接 wind 速度/方向;
    # 并按 world_name 匹配任务类型(perception/stationkeeping/wayfinding/wildlife/
    # scan_dock_deliver/gymkhana/acoustic_tracking/follow_path)桥接对应专属话题
    if (sim_mode == 'bridge' or sim_mode == 'full') and bridge_competition_topics:
        launch_processes.extend(vrx_gz.launch.competition_bridges(world_name))

    return launch_processes


def generate_launch_description():
    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'world',
            default_value='simple_demo',
            description='Name of world'),
        DeclareLaunchArgument(
            'sim_mode',
            default_value='full',
            description='Simulation mode: "full", "sim", "bridge".'
                        'full: spawns robot and launch ros_gz bridges, '
                        'sim: spawns robot only, '
                        'bridge: launch ros_gz bridges only.'),
        DeclareLaunchArgument(
            'bridge_competition_topics',
            default_value='True',
            description='True to bridge competition topics, False to disable bridge.'),

        DeclareLaunchArgument(
            'name',
            default_value='',
            description='Name of robot to spawn'),
        DeclareLaunchArgument(
            'model',
            default_value='',
            description='SDF model to spawn'),
        DeclareLaunchArgument(
            'x',
            default_value='0',
            description='X position to spawn'),
        DeclareLaunchArgument(
            'y',
            default_value='0',
            description='y position to spawn'),
        DeclareLaunchArgument(
            'z',
            default_value='0',
            description='z position to spawn'),
        DeclareLaunchArgument(
            'R',
            default_value='0',
            description='R rotation to spawn'),
        DeclareLaunchArgument(
            'P',
            default_value='0',
            description='P rotation to spawn'),
        DeclareLaunchArgument(
            'Y',
            default_value='0',
            description='Y rotation to spawn'),
        DeclareLaunchArgument(
            'flightTime',
            default_value='10',
            description='Battery flight time in minutes (only for UAVs)'),
        DeclareLaunchArgument(
            'slot0',
            default_value='',
            description='Payload mounted to slot 0'),
        DeclareLaunchArgument(
            'slot0_rpy',
            default_value='0 0 0',
            description='Roll, Pitch, Yaw in degrees of payload mount'),
        DeclareLaunchArgument(
            'slot1',
            default_value='',
            description='Payload mounted to slot 1'),
        DeclareLaunchArgument(
            'slot1_rpy',
            default_value='0 0 0',
            description='Roll, Pitch, Yaw in degrees of payload mount'),
        DeclareLaunchArgument(
            'slot2',
            default_value='',
            description='Payload mounted to slot 2'),
        DeclareLaunchArgument(
            'slot2_rpy',
            default_value='0 0 0',
            description='Roll, Pitch, Yaw in degrees of payload mount'),
        DeclareLaunchArgument(
            'slot3',
            default_value='',
            description='Payload mounted to slot 3'),
        DeclareLaunchArgument(
            'slot3_rpy',
            default_value='0 0 0',
            description='Roll, Pitch, Yaw in degrees of payload mount'),
        DeclareLaunchArgument(
            'slot4',
            default_value='',
            description='Payload mounted to slot 4'),
        DeclareLaunchArgument(
            'slot4_rpy',
            default_value='0 0 0',
            description='Roll, Pitch, Yaw in degrees of payload mount'),
        DeclareLaunchArgument(
            'slot5',
            default_value='',
            description='Payload mounted to slot 5'),
        DeclareLaunchArgument(
            'slot5_rpy',
            default_value='0 0 0',
            description='Roll, Pitch, Yaw in degrees of payload mount'),
        DeclareLaunchArgument(
            'slot6',
            default_value='',
            description='Payload mounted to slot 6'),
        DeclareLaunchArgument(
            'slot6_rpy',
            default_value='0 0 0',
            description='Roll, Pitch, Yaw in degrees of payload mount'),
        DeclareLaunchArgument(
            'slot7',
            default_value='',
            description='Payload mounted to slot 7'),
        DeclareLaunchArgument(
            'slot7_rpy',
            default_value='0 0 0',
            description='Roll, Pitch, Yaw in degrees of payload mount'),
        DeclareLaunchArgument(
            'urdf',
            default_value='',
            description='URDF file of the wam-v model. '),
        # launch setup
        OpaqueFunction(function=launch)
    ])
