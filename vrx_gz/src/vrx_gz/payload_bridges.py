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
# VRX 动态传感器/执行器桥接工厂
# -----------------------------------------------------------------------------
# 作用: 与 bridges.py (静态话题) 互补, 本文件为机器人 SDF 中解析出的每个传感器/
#       执行器自动生成对应 Bridge. 调用方 (Model.payload_bridges) 遍历从 SDF 解析出
#       的 payload 列表, 对每个条目调用 payload_bridges() 分发到具体工厂函数.
#
# 与 bridges.py 的核心区别:
#   - bridges.py: 话题名固定, 函数直接被按名调用 (如 pose(model_name))
#   - 本文件    : 话题名包含动态的 sensor_name/link_name, 由 SDF 解析结果决定
#                 ROS 话题名分层: sensors/<类型>/<传感器名>/<子话题>
#
# 话题命名规则:
#   GZ 侧: /world/<w>/model/<m>/link/<link>/sensor/<sensor_name>/<子话题>
#   ROS 侧: sensors/<type>/<sensor_name>/<子话题>  (相对命名空间)
#
# 依赖 sdformat14 的 Sensortype 枚举判断标准 Gazebo 传感器类型;
# VRX 自定义插件 (OdometryPublisher/BallShooter/thruster_*/AcousticPinger)
# 因是 USER_DEFINED 类型, 通过 sensor_name 字符串匹配识别.
# =============================================================================

from vrx_gz.bridge import Bridge, BridgeDirection

import sdformat14 as sdf


# ---------------------------------------------------------------------------
# 话题前缀生成工具
# ---------------------------------------------------------------------------

# GZ 侧传感器话题前缀: /world/<w>/model/<m>/link/<link>/sensor/<sensor_name>
def gz_prefix(world_name, model_name, link_name, sensor_name):
    return f'/world/{world_name}/model/{model_name}/link/{link_name}/sensor/{sensor_name}'


# ROS 侧传感器话题前缀: sensors/<type>/<sensor_name>
# 注: sensor_name 为空时返回 'sensors/<type>/', 由调用方再拼具体子话题
def ros_prefix(sensor_name, sensor_type):
    return f'sensors/{sensor_type}/{sensor_name}'


# ===========================================================================
# 标准传感器桥接工厂 (GZ -> ROS)
# ===========================================================================

# 普通相机图像: 输出 RGB 图像流
# ROS 话题示例: sensors/cameras/camera_front/image_raw
def image(world_name, model_name, link_name, sensor_name):
    gz_sensor_prefix = gz_prefix(world_name, model_name, link_name, sensor_name)
    ros_sensor_prefix = ros_prefix(sensor_name, 'cameras')
    return Bridge(
        gz_topic=f'{gz_sensor_prefix}/image',
        ros_topic=f'{ros_sensor_prefix}/image_raw',
        gz_type='gz.msgs.Image',
        ros_type='sensor_msgs/msg/Image',
        direction=BridgeDirection.GZ_TO_ROS)


# 深度相机: 深度图 (浮点像素值表示距离)
def depth_image(world_name, model_name, link_name, sensor_name):
    gz_sensor_prefix = gz_prefix(world_name, model_name, link_name, sensor_name)
    ros_sensor_prefix = ros_prefix(sensor_name, 'cameras')
    return Bridge(
        gz_topic=f'{gz_sensor_prefix}/depth_image',
        ros_topic=f'{ros_sensor_prefix}/depth',
        gz_type='gz.msgs.Image',
        ros_type='sensor_msgs/msg/Image',
        direction=BridgeDirection.GZ_TO_ROS)


# 相机内参: 焦距/畸变等参数, 标定用
def camera_info(world_name, model_name, link_name, sensor_name):
    gz_sensor_prefix = gz_prefix(world_name, model_name, link_name, sensor_name)
    ros_sensor_prefix = ros_prefix(sensor_name, 'cameras')
    return Bridge(
        gz_topic=f'{gz_sensor_prefix}/camera_info',
        ros_topic=f'{ros_sensor_prefix}/camera_info',
        gz_type='gz.msgs.CameraInfo',
        ros_type='sensor_msgs/msg/CameraInfo',
        direction=BridgeDirection.GZ_TO_ROS)


# 激光雷达: 2D 扫描数据 (LaserScan, 平面扫描线)
def lidar_scan(world_name, model_name, link_name, sensor_name):
    gz_sensor_prefix = gz_prefix(world_name, model_name, link_name, sensor_name)
    ros_sensor_prefix = ros_prefix(sensor_name, 'lidars')
    return Bridge(
        gz_topic=f'{gz_sensor_prefix}/scan',
        ros_topic=f'{ros_sensor_prefix}/scan',
        gz_type='gz.msgs.LaserScan',
        ros_type='sensor_msgs/msg/LaserScan',
        direction=BridgeDirection.GZ_TO_ROS)


# 激光雷达: 3D 点云 (PointCloud2, 多线激光合成)
def lidar_points(world_name, model_name, link_name, sensor_name):
    gz_sensor_prefix = gz_prefix(world_name, model_name, link_name, sensor_name)
    ros_sensor_prefix = ros_prefix(sensor_name, 'lidars')
    return Bridge(
        gz_topic=f'{gz_sensor_prefix}/scan/points',
        ros_topic=f'{ros_sensor_prefix}/points',
        gz_type='gz.msgs.PointCloudPacked',
        ros_type='sensor_msgs/msg/PointCloud2',
        direction=BridgeDirection.GZ_TO_ROS)


# 深度相机: 点云 (RGBD 合成的彩色点云)
def camera_points(world_name, model_name, link_name, sensor_name):
    gz_sensor_prefix = gz_prefix(world_name, model_name, link_name, sensor_name)
    ros_sensor_prefix = ros_prefix(sensor_name, 'cameras')
    return Bridge(
        gz_topic=f'{gz_sensor_prefix}/points',
        ros_topic=f'{ros_sensor_prefix}/points',
        gz_type='gz.msgs.PointCloudPacked',
        ros_type='sensor_msgs/msg/PointCloud2',
        direction=BridgeDirection.GZ_TO_ROS)


# RF 测距传感器: 射频信号参数 (Param_V), 用于特殊测距场景
def rfranger(world_name, model_name, link_name, sensor_name):
    gz_sensor_prefix = gz_prefix(world_name, model_name, link_name, sensor_name)
    ros_sensor_prefix = ros_prefix(sensor_name)
    return Bridge(
        gz_topic=f'{gz_sensor_prefix}/rfsensor',
        ros_topic=f'{ros_sensor_prefix}/rfsensor',
        gz_type='gz.msgs.Param_V',
        ros_type='ros_gz_interfaces/msg/ParamVec',
        direction=BridgeDirection.GZ_TO_ROS)

# IMU: 加速度 + 角速度 + 姿态, 用于姿态估计/状态反馈
# 注: ROS 话题固定为 sensors/imu/imu/data (无 sensor_name 区分, 一船一 IMU)
def imu(world_name, model_name, link_name, sensor_name):
    gz_sensor_prefix = gz_prefix(world_name, model_name, link_name, sensor_name)
    ros_sensor_prefix = ros_prefix('', 'imu')
    return Bridge(
        gz_topic=f'{gz_sensor_prefix}/imu',
        ros_topic=f'{ros_sensor_prefix}imu/data',
        gz_type='gz.msgs.IMU',
        ros_type='sensor_msgs/msg/Imu',
        direction=BridgeDirection.GZ_TO_ROS)

# GPS: 经纬度 + 高度, 用于全局定位
# 注: ROS 话题固定为 sensors/gps/gps/fix (无 sensor_name 区分, 一船一 GPS)
def navsat(world_name, model_name, link_name, sensor_name):
    gz_sensor_prefix = gz_prefix(world_name, model_name, link_name, sensor_name)
    ros_sensor_prefix = ros_prefix('', 'gps')
    return Bridge(
        gz_topic=f'{gz_sensor_prefix}/navsat',
        ros_topic=f'{ros_sensor_prefix}gps/fix',
        gz_type='gz.msgs.NavSat',
        ros_type='sensor_msgs/msg/NavSatFix',
        direction=BridgeDirection.GZ_TO_ROS)


# 接触传感器: 碰撞事件列表, 全局话题 (不绑定具体模型)
def contacts():
    return Bridge(
        gz_topic=f'/vrx/contacts',
        ros_topic=f'/vrx/contacts',
        gz_type='gz.msgs.Contacts',
        ros_type='ros_gz_interfaces/msg/Contacts',
        direction=BridgeDirection.GZ_TO_ROS)


# ===========================================================================
# 执行器与自定义插件桥接 (部分 ROS -> GZ)
# ===========================================================================

# Ground Truth 里程计: GZ 发布真值位姿 (用于评估 SLAM 精度等)
# ROS 话题: sensors/position/ground_truth_odometry
def odometry(model_name):
    ros_sensor_prefix = ros_prefix('', 'position')
    return Bridge(
        gz_topic=f'/model/{model_name}/odometry',
        ros_topic=f'{ros_sensor_prefix}ground_truth_odometry',
        gz_type='gz.msgs.Odometry',
        ros_type='nav_msgs/msg/Odometry',
        direction=BridgeDirection.GZ_TO_ROS)

# 发球装置: ROS 端发 Bool 触发射球 (scan_dock_deliver 任务用)
def ball_shooter(model_name):
    return Bridge(
        gz_topic=f'/{model_name}/shooters/ball_shooter/fire',
        ros_topic=f'shooters/ball_shooter/fire',
        gz_type='gz.msgs.Boolean',
        ros_type='std_msgs/msg/Bool',
        direction=BridgeDirection.ROS_TO_GZ)

# 推进器推力指令: ROS 端发 Float64 控制推力大小
# side 参数为 'left' / 'right' (由 thruster_layout xacro 决定)
def thrust(model_name, side):
    return Bridge(
        gz_topic=f'{model_name}/thrusters/{side}/thrust',
        ros_topic=f'thrusters/{side}/thrust',
        gz_type='gz.msgs.Double',
        ros_type='std_msgs/msg/Float64',
        direction=BridgeDirection.ROS_TO_GZ)

# 推进器转向指令: 控制发动机偏转角度 (H/T 布局才需要, X 布局无此功能)
def thrust_joint_pos(model_name, side):
    # ROS naming policy indicates that first character of a name must be an alpha
    # character. In the case below, the gz topic has the joint index 0 as the
    # first char so the following topics fail to be created on the ROS end
    # left_joint_topic = '/model/' + model_name + '/joint/left_chasis_engine_joint/0/cmd_pos'
    # right_joint_topic = '/model/' + model_name + '/joint/right_chasis_engine_joint/0/cmd_pos'
    # For now, use erb to generate unique topic names in model.sdf.erb
    return Bridge(
        gz_topic=f'{model_name}/thrusters/{side}/pos',
        ros_topic=f'thrusters/{side}/pos',
        gz_type='gz.msgs.Double',
        ros_type='std_msgs/msg/Float64',
        direction=BridgeDirection.ROS_TO_GZ)

# 声学信标接收: GZ 发布距离+方位 (acoustic_tracking 任务用)
def acoustic_pinger(model_name):
    return Bridge(
        gz_topic=f'{model_name}/sensors/acoustics/receiver/range_bearing',
        ros_topic=f'sensors/acoustics/receiver/range_bearing',
        gz_type='gz.msgs.Param',
        ros_type='ros_gz_interfaces/msg/ParamVec',
        direction=BridgeDirection.GZ_TO_ROS)

# 设置声学信标位置: ROS 端发 Vector3 改变信标位置 (调试用)
def set_acoustic_pinger():
    return Bridge(
        gz_topic=f'/pinger/set_pinger_position',
        ros_topic=f'/pinger/set_pinger_position',
        gz_type='gz.msgs.Vector3d',
        ros_type='geometry_msgs/msg/Vector3',
        direction=BridgeDirection.ROS_TO_GZ)


# ===========================================================================
# 核心分发函数: 按 sensor_type / sensor_name 选择对应桥接
# -----------------------------------------------------------------------------
# 调用方: Model.payload_bridges() 遍历 SDF 解析出的 payload 列表, 每个条目调一次
# 参数:
#   world_name   : 世界名 (如 sydney_regatta)
#   model_name   : 模型名 (如 wamv, 同时是命名空间)
#   link_name    : 传感器所在 link (如 cpu_camera_link)
#   sensor_name  : 传感器名 (如 camera_front, 或 thruster_thrust_left_engine)
#   sensor_type  : sdformat 枚举类型 (CAMERA/IMU/NAVSAT/GPU_LIDAR/RGBD_CAMERA/...)
#                  对于自定义插件, 此字段被复用为 side ('left'/'right')
# 返回: Bridge 列表 (一个传感器可能产生多个话题, 如相机有 image+camera_info)
# ===========================================================================
def payload_bridges(world_name, model_name, link_name, sensor_name, sensor_type):
    bridges = []
    # ---- 标准传感器: 按 sdf.Sensortype 枚举分发 ----
    if sensor_type == sdf.Sensortype.CAMERA:
        # 普通相机: 图像 + 内参
        bridges = [
            image(world_name, model_name, link_name, sensor_name),
            camera_info(world_name, model_name, link_name, sensor_name)
        ]
    elif sensor_type == sdf.Sensortype.IMU:
        bridges = [
            imu(world_name, model_name, link_name, sensor_name)
        ]
    elif sensor_type == sdf.Sensortype.CONTACT:
        bridges = [
            contacts(),
        ]
    elif sensor_type == sdf.Sensortype.NAVSAT:
        bridges = [
            navsat(world_name, model_name, link_name, sensor_name),
        ]
    elif sensor_type == sdf.Sensortype.GPU_LIDAR:
        # GPU 激光雷达: 2D 扫描 + 3D 点云
        bridges = [
            lidar_scan(world_name, model_name, link_name, sensor_name),
            lidar_points(world_name, model_name, link_name, sensor_name)
        ]
    elif sensor_type == sdf.Sensortype.RGBD_CAMERA:
        # RGBD 相机: 图像 + 内参 + 深度图 + 点云 (4 个话题)
        bridges = [
            image(world_name, model_name, link_name, sensor_name),
            camera_info(world_name, model_name, link_name, sensor_name),
            depth_image(world_name, model_name, link_name, sensor_name),
            camera_points(world_name, model_name, link_name, sensor_name),
        ]
    # ---- VRX 自定义插件: 按 sensor_name 字符串匹配 (USER_DEFINED 类型无枚举) ----
    elif 'OdometryPublisher' in sensor_name:
        # Ground truth 位姿发布器
        bridges = [
            odometry(model_name),
        ]
    elif 'BallShooter' in sensor_name:
        # 发球装置
        bridges = [
            ball_shooter(model_name),
        ]
    elif 'thruster_thrust_' in sensor_name:
        # 推进器推力: sensor_type 字段被复用为 side ('left'/'right')
        bridges = [
            thrust(model_name, sensor_type),
        ]
    elif 'thruster_rotate_' in sensor_name:
        # 推进器转向: sensor_type 字段被复用为 side ('left'/'right')
        bridges = [
            thrust_joint_pos(model_name, sensor_type),
        ]
    elif 'AcousticPinger' in sensor_name:
        # 声学信标: 接收 + 设置位置 (双向)
        bridges = [
            acoustic_pinger(model_name),
            set_acoustic_pinger(),
        ]

    return bridges
