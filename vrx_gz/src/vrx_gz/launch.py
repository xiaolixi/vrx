# Copyright 2022 Open Source Robotics Foundation, Inc.
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

from ament_index_python.packages import get_package_share_directory

from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess, EmitEvent
from launch.events import Shutdown

from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

import vrx_gz.bridges

import os

GYMKHANA_WORLDS = [
  'gymkhana_task'
]

PERCEPTION_WORLDS = [
  'perception_task',
  'phase2_2023_perception0',
  'phase2_2023_perception1',
  'phase2_2023_perception2',
  'phase2_2023_perception3',
  'phase2_2023_perception4',
  'phase2_2023_perception5',
  'phase3_2023_perception0',
  'phase3_2023_perception1',
  'phase3_2023_perception2',
  'phase3_2023_perception3',
  'phase3_2023_perception4',
  'phase3_2023_perception5',
  'practice_2023_perception0_task',
  'practice_2023_perception1_task',
  'practice_2023_perception2_task',
  'perception0',
  'perception1',
  'perception2',
  'perception3',
  'perception4',
  'perception5'
]

STATIONKEEPING_WORLDS = [
  'stationkeeping_task',
  'phase2_2023_stationkeeping0',
  'phase2_2023_stationkeeping1',
  'phase2_2023_stationkeeping2',
  'phase2_2023_stationkeeping3',
  'phase2_2023_stationkeeping4',
  'phase2_2023_stationkeeping5',
  'phase3_2023_stationkeeping0',
  'phase3_2023_stationkeeping1',
  'phase3_2023_stationkeeping2',
  'phase3_2023_stationkeeping3',
  'phase3_2023_stationkeeping4',
  'phase3_2023_stationkeeping5',
  'practice_2023_stationkeeping0_task',
  'practice_2023_stationkeeping1_task',
  'practice_2023_stationkeeping2_task',
  'stationkeeping0',
  'stationkeeping1',
  'stationkeeping2',
  'stationkeeping3',
  'stationkeeping4',
  'stationkeeping5'
]

WAYFINDING_WORLDS = [
  'wayfinding_task',
  'phase2_2023_wayfinding0',
  'phase2_2023_wayfinding1',
  'phase2_2023_wayfinding2',
  'phase2_2023_wayfinding3',
  'phase2_2023_wayfinding4',
  'phase2_2023_wayfinding5',
  'phase3_2023_wayfinding0',
  'phase3_2023_wayfinding1',
  'phase3_2023_wayfinding2',
  'phase3_2023_wayfinding3',
  'phase3_2023_wayfinding4',
  'phase3_2023_wayfinding5',
  'practice_2023_wayfinding0_task',
  'practice_2023_wayfinding1_task',
  'practice_2023_wayfinding2_task',
  'wayfinding0',
  'wayfinding1',
  'wayfinding2',
  'wayfinding3',
  'wayfinding4',
  'wayfinding5'
]

WILDLIFE_WORLDS = [
  'wildlife_task',
  'phase2_2023_wildlife0',
  'phase2_2023_wildlife1',
  'phase2_2023_wildlife2',
  'phase2_2023_wildlife3',
  'phase2_2023_wildlife4',
  'phase2_2023_wildlife5',
  'phase3_2023_wildlife0',
  'phase3_2023_wildlife1',
  'phase3_2023_wildlife2',
  'phase3_2023_wildlife3',
  'phase3_2023_wildlife4',
  'phase3_2023_wildlife5',
  'practice_2023_wildlife0_task',
  'practice_2023_wildlife1_task',
  'practice_2023_wildlife2_task',
  'wildlife0',
  'wildlife1',
  'wildlife2',
  'wildlife3',
  'wildlife4',
  'wildlife5'
]

SCAN_DOCK_DELIVER_WORLDS = [
  'scan_dock_deliver_task',
  'phase2_2023_scan_dock_deliver0',
  'phase2_2023_scan_dock_deliver1',
  'phase2_2023_scan_dock_deliver2',
  'phase2_2023_scan_dock_deliver3',
  'phase2_2023_scan_dock_deliver4',
  'phase2_2023_scan_dock_deliver5',
  'phase3_2023_scan_dock_deliver0',
  'phase3_2023_scan_dock_deliver1',
  'phase3_2023_scan_dock_deliver2',
  'phase3_2023_scan_dock_deliver3',
  'phase3_2023_scan_dock_deliver4',
  'phase3_2023_scan_dock_deliver5',
  'practice_2023_scan_dock_deliver0_task',
  'practice_2023_scan_dock_deliver1_task',
  'practice_2023_scan_dock_deliver2_task',
  'scan_dock_deliver0',
  'scan_dock_deliver1',
  'scan_dock_deliver2',
  'scan_dock_deliver3',
  'scan_dock_deliver4',
  'scan_dock_deliver5'
]

ACOUSTIC_TRACKING_WORLDS = [
  'acoustic_tracking_task',
  'phase2_2023_acoustic_tracking0',
  'phase2_2023_acoustic_tracking1',
  'phase2_2023_acoustic_tracking2',
  'phase2_2023_acoustic_tracking3',
  'phase2_2023_acoustic_tracking4',
  'phase2_2023_acoustic_tracking5',
  'phase3_2023_acoustic_tracking0',
  'phase3_2023_acoustic_tracking1',
  'phase3_2023_acoustic_tracking2',
  'phase3_2023_acoustic_tracking3',
  'phase3_2023_acoustic_tracking4',
  'phase3_2023_acoustic_tracking5',
  'practice_2023_acoustic_tracking0_task',
  'practice_2023_acoustic_tracking1_task',
  'practice_2023_acoustic_tracking2_task',
  'acoustic_tracking0',
  'acoustic_tracking1',
  'acoustic_tracking2',
  'acoustic_tracking3',
  'acoustic_tracking4',
  'acoustic_tracking5'
]

FOLLOWPATH_WORLDS = [
  'follow_path_task',
  'phase2_2023_follow_path0',
  'phase2_2023_follow_path1',
  'phase2_2023_follow_path2',
  'phase2_2023_follow_path3',
  'phase2_2023_follow_path4',
  'phase2_2023_follow_path5',
  'phase3_2023_follow_path0',
  'phase3_2023_follow_path1',
  'phase3_2023_follow_path2',
  'phase3_2023_follow_path3',
  'phase3_2023_follow_path4',
  'phase3_2023_follow_path5',
  'practice_2023_follow_path0_task',
  'practice_2023_follow_path1_task',
  'practice_2023_follow_path2_task',
  'follow_path0',
  'follow_path1',
  'follow_path2',
  'follow_path3',
  'follow_path4',
  'follow_path5'
]
# 启动 Gazebo 仿真环境/gz_sim.launch.py
# 设置桥接
# 创建RSP
def simulation(world_name, headless=False, paused=False, extra_gz_args=''):
    gz_args = ['-v 4']
    if not paused:
        gz_args.append('-r')

    if headless:
        gz_args.append('-s')

    gz_args.append(extra_gz_args)
    # 因为有环境变量，所以可以从环境变量中获取真实路径
    gz_args.append(f'{world_name}.sdf')

    print(f"启动 Gazebo 仿真环境/gz_sim.launch.py simulation world_name: {world_name} gz_args: {' '.join(gz_args)}")

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch'),
            '/gz_sim.launch.py']),
        launch_arguments={'gz_args': ' '.join(gz_args)}.items())

    # Register handler for shutting down ros launch when ign gazebo process exits
    # monitor_sim.py will run until it can not find the ign gazebo process.
    # Once monitor_sim.py exits, a process exit event is triggered which causes the
    # handler to emit a Shutdown event
    p = os.path.join(get_package_share_directory('vrx_ros'), 'launch',
                     'monitor_sim.py')
    monitor_sim_proc = ExecuteProcess(
        cmd=['python3', p],
        name='monitor_sim',
        output='screen',
    )
    sim_exit_event_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=monitor_sim_proc,   # 告诉系统"我要监控 monitor_sim 这个进程"
            on_exit=[  # Shutdown 这是一个特殊的系统事件，会触发整个 Launch 系统的关闭
                EmitEvent(event=Shutdown(reason='Simulation ended'))
            ]
        )
    )

    return [gz_sim, monitor_sim_proc, sim_exit_event_handler]

## 专为竞赛模式设计的桥梁
def competition_bridges(world_name, competition_mode=False):
    bridges = [
        vrx_gz.bridges.clock(),
        vrx_gz.bridges.task_info(),
    ]

    if not competition_mode:
        bridges.extend([
            vrx_gz.bridges.usv_wind_speed(),
            vrx_gz.bridges.usv_wind_direction()
        ])

    task_bridges = []
    if world_name in PERCEPTION_WORLDS:
        print(f"world_name: {world_name} in PERCEPTION_WORLDS")
        task_bridges = [
            vrx_gz.bridges.perception_reports(),
        ]
    elif world_name in STATIONKEEPING_WORLDS:
        print(f"world_name: {world_name} in STATIONKEEPING_WORLDS")
        task_bridges = [
            vrx_gz.bridges.stationkeeping_goal(),
        ]
        if not competition_mode:
            task_bridges.extend([
                vrx_gz.bridges.stationkeeping_mean_pose_error(),
                vrx_gz.bridges.stationkeeping_pose_error(),
            ])
    elif world_name in WAYFINDING_WORLDS:
        print(f"world_name: {world_name} in WAYFINDING_WORLDS")
        task_bridges = [
            vrx_gz.bridges.wayfinding_waypoints(),
        ]
        if not competition_mode:
            task_bridges.extend([
                vrx_gz.bridges.wayfinding_mean_error(),
                vrx_gz.bridges.wayfinding_min_errors(),
            ])
    elif world_name in GYMKHANA_WORLDS:
        print(f"world_name: {world_name} in GYMKHANA_WORLDS")
        task_bridges = [
            vrx_gz.bridges.gymkhana_blackbox_goal(),
        ]
        if not competition_mode:
            task_bridges.extend([
                vrx_gz.bridges.gymkhana_blackbox_mean_pose_error(),
                vrx_gz.bridges.gymkhana_blackbox_pose_error(),
            ])
    elif world_name in WILDLIFE_WORLDS:
        print(f"world_name: {world_name} in WILDLIFE_WORLDS")
        task_bridges = [
            vrx_gz.bridges.animal_pose('/vrx/wildlife/animal0/pose'),
            vrx_gz.bridges.animal_pose('/vrx/wildlife/animal1/pose'),
            vrx_gz.bridges.animal_pose('/vrx/wildlife/animal2/pose'),
        ]
    elif world_name in SCAN_DOCK_DELIVER_WORLDS:
        print(f"world_name: {world_name} in SCAN_DOCK_DELIVER_WORLDS")
        task_bridges = [
            vrx_gz.bridges.color_sequence_reports(),
        ]
    bridges.extend(task_bridges)

    nodes = []
    nodes.append(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[bridge.argument() for bridge in bridges],
        remappings=[bridge.remapping() for bridge in bridges],
    ))

    print(f"桥接竞赛的ros_gz_bridge。competition_bridges bridges: {bridges}")
    return nodes

# 启动 parameter_bridge,把长话题名 remap 成短话题名
# ros2 run ros_gz_bridge parameter_bridge \
#     /world/sydney_regatta/model/wamv/link/cpu_camera_link/sensor/camera_front/image@sensor_msgs/msg/Image[gz.msgs.Image \
#     --ros-args \
#     -r /world/sydney_regatta/model/wamv/link/cpu_camera_link/sensor/camera_front/image:=sensors/cameras/camera_front/image_raw
# <node pkg="ros_gz_bridge" exec="parameter_bridge" ...>
#     <param name="arguments" value="..."/>
#     <remap from="/world/.../image" to="sensors/cameras/image_raw"/>
# </node>
def ros_gz_bridge_function(bridges):
    arguments=[bridge.argument() for bridge in bridges]
    remappings=[bridge.remapping() for bridge in bridges]
    
    print('----------------------spawn ros_gz_bridge parameter_bridge ------------')
    print(arguments)
    print(remappings)
    print('----------------------spawn ros_gz_bridge parameter_bridge ------------ end')
    return Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=arguments,
        remappings=remappings,
    )

def robot_state_publisher_function(model, use_sim_time):
    model_dir = os.path.join(get_package_share_directory('vrx_gazebo'), 'models/' + model.model_name + '/tmp')
    urdf_file = os.path.join(model_dir, 'model.urdf')
    print(f"robot_state_publisher robot_description from urdf_file: {urdf_file}")
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    # robot_state_publisher订阅/joint_states，发布/tf
    # frame_prefix给机器人的所有坐标系（TF Frame）加个前缀。
    params = {'use_sim_time': use_sim_time, 'frame_prefix': model.model_name + '/', 'robot_description': robot_desc}
    return Node(package='robot_state_publisher',
                executable='robot_state_publisher',
                output='both',
                parameters=[params],
                remappings=[('/joint_states', '/' + model.model_name + '/joint_states')] # 订阅 /joint_states改成订阅 /wamv/joint_states
            )


def spawn(sim_mode, world_name, models, robot=None):
    print(f"spawn sim_mode: {sim_mode} world_name: {world_name} models: {models} robot: {robot}")
    if type(models) != list:
        models = [models]
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'),

    launch_processes = []
    for model in models:
        # 指定那个机器人
        if robot and model.model_name != robot:
            continue

        # Script to insert model in running simulation
        if sim_mode == 'full' or sim_mode == 'sim':
            arguments = model.spawn_args()
            gz_spawn_entity = Node(
                package='ros_gz_sim',
                executable='create',
                output='screen',
                arguments=arguments
            )
            truncated_args = [arg[:20] + '...' if len(arg) > 20 else arg for arg in arguments]
            
            print(f"spawn 机器人 ros_gz_sim create 参数有model: {model.model_name} spawn_args: {truncated_args}")
            launch_processes.append(gz_spawn_entity)

        if sim_mode == 'full' or sim_mode == 'bridge':
            ## 返回桥接、节点、自定义launch文件，但是实际上只有桥接
            bridges, nodes, custom_launches = model.bridges(world_name)
            ## 返回桥接、节点、自定义launch文件，但是实际上只有桥接和节点【相机的optical_frame_publisher】
            payload_bridges, payload_nodes, payload_launches = model.payload_bridges(world_name)

            bridges.extend(payload_bridges)
            nodes.extend(payload_nodes)

            nodes.append(ros_gz_bridge_function(bridges))

            # tf broadcaster (sensors)
            nodes.append(Node(
                package='vrx_ros',
                executable='pose_tf_broadcaster',
                output='screen',
            ))

            # robot_state_publisher (tf for wamv)
            nodes.append(robot_state_publisher_function(model, use_sim_time))
            ## TODO 注意这里是已经转换过了topic名称，然后仔加的环境前缀
            # - PushRosNamespace('wamv') = 把后续所有节点都放到 /wamv 命名空间下
            # - GroupAction([...]) = 把多个动作打包成一个整体
            # - *nodes = 把列表展开,如 *[node1, node2, node3] 变成 node1, node2, node3
            # 项目	            实际值	                        由什么决定
            # 节点名	        /wamv/robot_state_publisher	    PushRosNamespace 影响 
            # 订阅joint_states	/wamv/joint_states	            你显式 remap 的绝对路径
            # 发布TF话题	    /tf、/tf_static	                RSP 的 tf2 broadcaster 绑定根命名空间，不受 PushRosNamespace 影响 
            # TF frame名	    wamv/base_link等	           frame_prefix 参数

            group_action = GroupAction([
                PushRosNamespace(model.model_name),
                *nodes
            ])

            if sim_mode == 'full':
                handler = RegisterEventHandler(
                    event_handler=OnProcessExit(
                        target_action=gz_spawn_entity,  # gz_spawn_entity 这个节点执行的是 "一次性任务"，不是常驻进程。
                        on_exit=[group_action],
                    )
                )
                launch_processes.append(handler)
            elif sim_mode == 'bridge':
                launch_processes.append(group_action)

            # 两个launch
            launch_processes.extend(payload_launches)
            launch_processes.extend(custom_launches)

    return launch_processes
