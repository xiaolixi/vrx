# Copyright 2021 Open Source Robotics Foundation, Inc.
# Copyright 2025 Honu Robotics, Inc.
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
# VRX ROS<->GZ 桥接配置工厂
# -----------------------------------------------------------------------------
# 作用: 提供一组工厂函数, 每个函数返回一个 Bridge 对象, 描述一对 GZ 话题与 ROS 话题
#       的映射关系 (话题名 + 消息类型 + 方向). 这些 Bridge 被 launch.py 中的
#       parameter_bridge 节点使用, 实现 Gazebo 与 ROS 2 之间的数据互通.
#
# Bridge 字段含义 (见 bridge.py):
#   gz_topic   : Gazebo 一侧的话题名
#   ros_topic  : ROS 一侧的话题名 (相对命名空间, 命名空间由 PushRosNamespace 加)
#   gz_type    : Gazebo 消息类型 (gz.msgs.*)
#   ros_type   : ROS 消息类型 (包名/msg/类型)
#   direction  : 桥接方向
#                GZ_TO_ROS      = GZ -> ROS  ([)
#                ROS_TO_GZ      = ROS -> GZ  (])
#                BIDIRECTIONAL  = 双向        (@)
#
# 本文件按用途分为几组:
#   1. 通用传感器与位姿: prefix / magnetometer / air_pressure / pose / pose_static
#                       / joint_states / cmd_vel / comms_tx / comms_rx
#   2. 通用环境与任务:   clock / task_info
#   3. 任务专属话题:     stationkeeping / acoustic_tracking / wayfinding /
#                       perception / gymkhana_blackbox / scan_dock_deliver
#   4. 调试话题:         usv_wind_speed / usv_wind_direction (仅非竞赛模式)
# =============================================================================

from vrx_gz.bridge import Bridge, BridgeDirection


# ---------------------------------------------------------------------------
# 通用: 传感器话题前缀生成
# GZ 传感器话题命名规则: /world/<world>/model/<model>/link/<link>/sensor/<sensor_name>/<sensor_type>
# 本函数拼出前缀, 后续具体传感器函数在其后追加 '/<name>/<type>'
# ---------------------------------------------------------------------------
def prefix(world_name, model_name, link_name):
    return f'/world/{world_name}/model/{model_name}/link/{link_name}/sensor'

# 磁力计: 输出磁场强度向量, 用于指南针/姿态估计
def magnetometer(world_name, model_name, link_name='base_link'):
    sensor_prefix = prefix(world_name, model_name, link_name)
    return Bridge(
        gz_topic=f'{sensor_prefix}/magnetometer/magnetometer',
        ros_topic='magnetic_field',
        gz_type='gz.msgs.Magnetometer',
        ros_type='sensor_msgs/msg/MagneticField',
        direction=BridgeDirection.GZ_TO_ROS)

# 气压计: 输出大气压力, 用于海拔估计
def air_pressure(world_name, model_name, link_name='base_link'):
    sensor_prefix = prefix(world_name, model_name, link_name)
    return Bridge(
        gz_topic=f'{sensor_prefix}/air_pressure/air_pressure',
        ros_topic='air_pressure',
        gz_type='gz.msgs.FluidPressure',
        ros_type='sensor_msgs/msg/FluidPressure',
        direction=BridgeDirection.GZ_TO_ROS)

# ---------------------------------------------------------------------------
# 通用: 模型位姿 (动态)
# GZ 发布模型在世界坐标系下的实时位姿, 转为 TFMessage 用于更新 tf 树
# 通常配合 pose_tf_broadcaster 一起使用
# ---------------------------------------------------------------------------
def pose(model_name):
    return Bridge(
        gz_topic=f'/model/{model_name}/pose',
        ros_topic='pose',
        gz_type='gz.msgs.Pose_V',
        ros_type='tf2_msgs/msg/TFMessage',
        direction=BridgeDirection.GZ_TO_ROS)

# 模型静态位姿: 仅发布一次, 用于固定不变的 link (如船体本身)
def pose_static(model_name):
    return Bridge(
        gz_topic=f'/model/{model_name}/pose_static',
        ros_topic='pose_static',
        gz_type='gz.msgs.Pose_V',
        ros_type='tf2_msgs/msg/TFMessage',
        direction=BridgeDirection.GZ_TO_ROS)

# 关节状态: 各关节角度/速度, 喂给 robot_state_publisher 计算 link 间 TF
def joint_states(world_name, model_name):
    return Bridge(
        gz_topic=f'/world/{world_name}/model/{model_name}/joint_state',
        ros_topic='joint_states',
        gz_type='gz.msgs.Model',
        ros_type='sensor_msgs/msg/JointState',
        direction=BridgeDirection.GZ_TO_ROS)

# 速度指令: ROS 端发 cmd_vel 控制机器人运动 (常用于 UAV 或简化控制)
def cmd_vel(model_name):
    return Bridge(
        gz_topic=f'/model/{model_name}/cmd_vel',
        ros_topic='cmd_vel',
        gz_type='gz.msgs.Twist',
        ros_type='geometry_msgs/msg/Twist',
        direction=BridgeDirection.ROS_TO_GZ)

# ---------------------------------------------------------------------------
# 通信: 多机器人之间的消息收发 (VRX comms 任务)
# 发送: ROS 端发到 /broker/msgs, GZ 仿真内分发给目标机器人
# 接收: GZ 把收到的帧推到对应模型的 /rx 话题
# ---------------------------------------------------------------------------
def comms_tx(model_name):
    return Bridge(
        gz_topic='/broker/msgs',
        ros_topic='tx',
        gz_type='gz.msgs.Dataframe',
        ros_type='ros_gz_interfaces/msg/Dataframe',
        direction=BridgeDirection.ROS_TO_GZ)

def comms_rx(model_name):
    return Bridge(
        gz_topic=f'/model/{model_name}/rx',
        ros_topic='rx',
        gz_type='gz.msgs.Dataframe',
        ros_type='ros_gz_interfaces/msg/Dataframe',
        direction=BridgeDirection.GZ_TO_ROS)

# ---------------------------------------------------------------------------
# 通用环境: 仿真时钟
# 必桥话题, 让 ROS 节点使用仿真时间 (/clock) 而非系统时间
# ---------------------------------------------------------------------------
def clock():
    return Bridge(
        gz_topic='/clock',
        ros_topic='/clock',
        gz_type='gz.msgs.Clock',
        ros_type='rosgraph_msgs/msg/Clock',
        direction=BridgeDirection.GZ_TO_ROS)

# 任务信息: 当前任务名/状态/得分/剩余时间等参数向量
def task_info():
    return Bridge(
        gz_topic=f'/vrx/task/info',
        ros_topic=f'/vrx/task/info',
        gz_type='gz.msgs.Param',
        ros_type='ros_gz_interfaces/msg/ParamVec',
        direction=BridgeDirection.GZ_TO_ROS)

# ===========================================================================
# 任务专属话题
# 每个任务有自己的子话题, 由 launch.py 的 competition_bridges() 按 world_name 匹配后桥接
# ===========================================================================

# ---- stationkeeping (定点停船任务) ----
# 目标位姿 (GZ 发布给参赛者) + 实时/平均位姿误差 (GZ 发布评分用)
def stationkeeping_goal():
    return Bridge(
        gz_topic=f'/vrx/stationkeeping/goal',
        ros_topic=f'/vrx/stationkeeping/goal',
        gz_type='gz.msgs.Pose',
        ros_type='geometry_msgs/msg/PoseStamped',
        direction=BridgeDirection.GZ_TO_ROS)

def stationkeeping_mean_pose_error():
    return Bridge(
        gz_topic=f'/vrx/stationkeeping/mean_pose_error',
        ros_topic=f'/vrx/stationkeeping/mean_pose_error',
        gz_type='gz.msgs.Float',
        ros_type='std_msgs/msg/Float32',
        direction=BridgeDirection.GZ_TO_ROS)

def stationkeeping_pose_error():
    return Bridge(
        gz_topic=f'/vrx/stationkeeping/pose_error',
        ros_topic=f'/vrx/stationkeeping/pose_error',
        gz_type='gz.msgs.Float',
        ros_type='std_msgs/msg/Float32',
        direction=BridgeDirection.GZ_TO_ROS)

# ---- acoustic_tracking (声学跟踪任务) ----
# 评分用: 平均/实时位姿误差
def acoustic_tracking_mean_pose_error():
    return Bridge(
        gz_topic=f'/vrx/acoustic_tracking/mean_pose_error',
        ros_topic=f'/vrx/acoustic_tracking/mean_pose_error',
        gz_type='gz.msgs.Float',
        ros_type='std_msgs/msg/Float32',
        direction=BridgeDirection.GZ_TO_ROS)

def acoustic_tracking_pose_error():
    return Bridge(
        gz_topic=f'/vrx/acoustic_tracking/pose_error',
        ros_topic=f'/vrx/acoustic_tracking/pose_error',
        gz_type='gz.msgs.Float',
        ros_type='std_msgs/msg/Float32',
        direction=BridgeDirection.GZ_TO_ROS)

# ---- wayfinding (航线寻找任务) ----
# waypoints: GZ 发布目标航点列表 (Pose_V -> PoseArray)
# mean_error / min_errors: 评分用
def wayfinding_waypoints():
    return Bridge(
        gz_topic=f'/vrx/wayfinding/waypoints',
        ros_topic=f'/vrx/wayfinding/waypoints',
        gz_type='gz.msgs.Pose_V',
        ros_type='geometry_msgs/msg/PoseArray',
        direction=BridgeDirection.GZ_TO_ROS)

def wayfinding_mean_error():
    return Bridge(
        gz_topic=f'/vrx/wayfinding/mean_error',
        ros_topic=f'/vrx/wayfinding/mean_error',
        gz_type='gz.msgs.Float',
        ros_type='std_msgs/msg/Float32',
        direction=BridgeDirection.GZ_TO_ROS)

def wayfinding_min_errors():
    return Bridge(
        gz_topic=f'/vrx/wayfinding/min_errors',
        ros_topic=f'/vrx/wayfinding/min_errors',
        gz_type='gz.msgs.Float_V',
        ros_type='ros_gz_interfaces/msg/Float32Array',
        direction=BridgeDirection.GZ_TO_ROS)

# ---- perception (感知任务) ----
# 参赛者识别到地标后, 通过此话题把 landmark 位姿上报给 GZ 评分
# 方向是 ROS_TO_GZ (上行上报)
def perception_reports():
    return Bridge(
        gz_topic=f'/vrx/perception/landmark',
        ros_topic=f'/vrx/perception/landmark',
        gz_type='gz.msgs.Pose',
        ros_type='geometry_msgs/msg/PoseStamped',
        direction=BridgeDirection.ROS_TO_GZ)

# ---- wildlife (海洋动物任务) ----
# animal_pose: 桥接某个动物模型的位姿, topic 参数由 launch.py 传入具体动物名
def animal_pose(topic):
    return Bridge(
        gz_topic=f'{topic}',
        ros_topic=f'{topic}',
        gz_type='gz.msgs.Pose',
        ros_type='geometry_msgs/msg/PoseStamped',
        direction=BridgeDirection.GZ_TO_ROS)

# ---- gymkhana_blackbox (黑盒障碍任务) ----
# 目标位姿 + 评分用误差
def gymkhana_blackbox_goal():
    return Bridge(
        gz_topic=f'/vrx/gymkhana_blackbox/goal',
        ros_topic=f'/vrx/gymkhana_blackbox/goal',
        gz_type='gz.msgs.Pose',
        ros_type='geometry_msgs/msg/PoseStamped',
        direction=BridgeDirection.GZ_TO_ROS)

def gymkhana_blackbox_mean_pose_error():
    return Bridge(
        gz_topic=f'/vrx/gymkhana_blackbox/mean_pose_error',
        ros_topic=f'/vrx/gymkhana_blackbox/mean_pose_error',
        gz_type='gz.msgs.Float',
        ros_type='std_msgs/msg/Float32',
        direction=BridgeDirection.GZ_TO_ROS)

def gymkhana_blackbox_pose_error():
    return Bridge(
        gz_topic=f'/vrx/gymkhana_blackbox/pose_error',
        ros_topic=f'/vrx/gymkhana_blackbox/pose_error',
        gz_type='gz.msgs.Float',
        ros_type='std_msgs/msg/Float32',
        direction=BridgeDirection.GZ_TO_ROS)

# ---- scan_dock_deliver (扫描码头投递任务) ----
# 参赛者识别灯标颜色序列后, 通过此话题把识别结果上报给 GZ 评分
# 方向是 ROS_TO_GZ (上行上报)
def color_sequence_reports():
    return Bridge(
        gz_topic=f'/vrx/scan_dock_deliver/color_sequence',
        ros_topic=f'/vrx/scan_dock_deliver/color_sequence',
        gz_type='gz.msgs.StringMsg_V',
        ros_type='ros_gz_interfaces/msg/StringVec',
        direction=BridgeDirection.ROS_TO_GZ)

# ===========================================================================
# 调试话题 (仅非竞赛模式桥接, 方便调试)
# 风速/风向: GZ 内置风场扰动参数, 用于分析船体受力情况
# ===========================================================================
def usv_wind_speed():
    return Bridge(
        gz_topic=f'/vrx/debug/wind/speed',
        ros_topic=f'/vrx/debug/wind/speed',
        gz_type='gz.msgs.Float',
        ros_type='std_msgs/msg/Float32',
        direction=BridgeDirection.GZ_TO_ROS)

def usv_wind_direction():
    return Bridge(
        gz_topic=f'/vrx/debug/wind/direction',
        ros_topic=f'/vrx/debug/wind/direction',
        gz_type='gz.msgs.Float',
        ros_type='std_msgs/msg/Float32',
        direction=BridgeDirection.GZ_TO_ROS)