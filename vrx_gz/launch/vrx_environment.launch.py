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
# VRX 环境启动器
# -----------------------------------------------------------------------------
# 作用: 启动 Gazebo 仿真世界 + (可选)机器人 + (可选)竞赛话题桥接
#
# 与 competition.launch.py 的区别:
#   - competition.launch.py 在未提供 config_file 时会自动创建默认 wamv
#     (位置 [-532, 162, 0, 0, 0, 1])
#   - 本文件在未提供 config_file 时 models 列表为空, 不会 spawn 任何机器人,
#     仅启动仿真世界和竞赛桥接, 适合先准备环境再单独 spawn 机器人的场景
#
# 典型用法:
#   1. 只起环境:
#        ros2 launch vrx_gz vrx_environment.launch.py world:=sydney_regatta
#   2. 环境 + 多船 (走 YAML 配置):
#        ros2 launch vrx_gz vrx_environment.launch.py \
#            world:=sydney_regatta config_file:=<path>/two_wamv.yaml
#   3. 只起环境不桥接 (纯 gz):
#        ros2 launch vrx_gz vrx_environment.launch.py \
#            sim_mode:=sim bridge_competition_topics:=false
# =============================================================================


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
import os

import vrx_gz.launch
from vrx_gz.model import Model


def launch(context, *args, **kwargs):
    # ---- 在执行期解析 launch 参数 ----
    config_file = LaunchConfiguration('config_file').perform(context)
    world_name = LaunchConfiguration('world').perform(context)
    sim_mode = LaunchConfiguration('sim_mode').perform(context)
    # 字符串 'True'/'False' 转为布尔值
    bridge_competition_topics = LaunchConfiguration(
        'bridge_competition_topics').perform(context).lower() == 'true'
    robot = LaunchConfiguration('robot').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    gz_paused = LaunchConfiguration('paused').perform(context).lower() == 'true'
    competition_mode = LaunchConfiguration('competition_mode').perform(context).lower() == 'true'
    extra_gz_args = LaunchConfiguration('extra_gz_args').perform(context)

    # 收集所有要启动的进程/动作
    launch_processes = []

    # ---- 加载机器人模型 (可选) ----
    # 与 competition.launch.py 的关键差异: 未提供 config_file 时 models 保持为空,
    # 不会兜底创建默认 wamv, 因此只起环境不 spawn 机器人
    models = []
    if config_file and config_file != '':
        with open(config_file, 'r') as stream:
            # Model.FromConfig 支持 YAML 顶层为 dict(单船) 或 list(多船)
            models = Model.FromConfig(stream)

    # ---- 启动 Gazebo 仿真 ----
    # 去掉可能带的后缀, 仅保留世界名
    world_name, ext = os.path.splitext(world_name)
    # vrx_gz.launch.simulation() 通过 ros_gz_sim 的 gz_sim.launch.py 启动 gz sim,
    # 加载 {world_name}.sdf; 并启动 monitor_sim.py 监视 gz 进程,
    # gz 退出时触发整个 launch 的 Shutdown
    launch_processes.extend(vrx_gz.launch.simulation(world_name, headless,
                                                     gz_paused, extra_gz_args))
    # 取世界名 basename (去掉路径), 用于后续 spawn 和桥接的话题匹配
    world_name_base = os.path.basename(world_name)
    # ---- spawn 机器人 + ROS<->GZ 桥接 ----
    # models 为空时此步不执行任何 spawn;
    # sim_mode 为 full/sim 时调用 ros_gz_sim create 注入 SDF;
    # sim_mode 为 full/bridge 时为每个模型建 pose/joint_states/传感器/thruster 桥接,
    # 启动 pose_tf_broadcaster 和 robot_state_publisher, 用 PushRosNamespace 隔离命名空间;
    # robot 参数非空时仅 spawn 匹配名字的模型
    launch_processes.extend(vrx_gz.launch.spawn(sim_mode, world_name_base, models, robot))

    # ---- 竞赛话题桥接 (可选) ----
    # 仅在需要桥接(full/bridge)且开启 bridge_competition_topics 时执行
    if (sim_mode == 'bridge' or sim_mode == 'full') and bridge_competition_topics:
        # competition_bridges 桥接: clock, task_info; 非 competition_mode 还桥接 wind 速度/方向;
        # 并按 world_name 匹配任务类型(perception/stationkeeping/wayfinding/wildlife/
        # scan_dock_deliver/gymkhana/acoustic_tracking/follow_path)桥接对应专属话题
        launch_processes.extend(vrx_gz.launch.competition_bridges(world_name_base, competition_mode))

    return launch_processes


def generate_launch_description():
    # 声明所有 launch 参数, 然后用 OpaqueFunction 在执行期调用 launch() 解析参数
    return LaunchDescription([
        # Launch Arguments
        
        # world名称
        DeclareLaunchArgument(
            'world',
            default_value='sydney_regatta',
            description='Name of world'),
        # 模拟mode
        DeclareLaunchArgument(
            'sim_mode',
            default_value='full',
            description='Simulation mode: "full", "sim", "bridge".'
                        'full: spawns robot and launch ros_gz bridges, '
                        'sim: spawns robot only, '
                        'bridge: launch ros_gz bridges only.'),
        # 是否开启任务桥接？？TODO 和 competition_mode一起使用
        DeclareLaunchArgument(
            'bridge_competition_topics',
            default_value='True',
            description='True to bridge competition topics, False to disable bridge.'),
        # robot的config文件
        DeclareLaunchArgument(
            'config_file',
            default_value='',
            description='YAML configuration file to spawn'),
        # robot名称
        DeclareLaunchArgument(
            'robot',
            default_value='',
            description='Name of robot to spawn if specified. '
                        'This must match one of the robots in the config_file'),
        # 是否开启headless模式
        DeclareLaunchArgument(
            'headless',
            default_value='False',
            description='True to run simulation headless (no GUI). '),
        # 是否开启暂停模式
        DeclareLaunchArgument(
            'paused',
            default_value='False',
            description='True to start the simulation paused. '),
        # 是否开启比赛模式，比赛模式下没有debug话题
        DeclareLaunchArgument(
            'competition_mode',
            default_value='False',
            description='True to disable debug topics. '),
        # 额外的gz sim参数
        DeclareLaunchArgument(
            'extra_gz_args',
            default_value='',
            description='Additional arguments to be passed to gz sim. '),
        OpaqueFunction(function=launch),
    ])
