# import launch
# from launch_ros.actions import ComposableNodeContainer
# from launch_ros.descriptions import ComposableNode

# def generate_launch_description():
#     """Launch color_pointscloud component in nvenc_multicam_container."""
    
#     container = ComposableNodeContainer(
#         name='nvenc_multicam_container',
#         namespace='',
#         package='rclcpp_components',
#         executable='component_container',
#         composable_node_descriptions=[
#             # 添加你的color_pointscloud组件
#             ComposableNode(
#                 package='color_pointscloud',
#                 plugin='color_pointscloud::ColorPointsComponent',
#                 name='color_points_component',
#                 parameters=[
#                     # 从YAML文件加载参数
#                     {'use_sim_time': False},
#                     # 可以在这里添加其他参数
#                 ],
#                 # 如果需要重映射话题
#                 remappings=[
#                     # 例如: ('input_topic', 'new_topic_name')
#                 ]
#             ),
#             # 你还可以在这里添加其他组件
#         ],
#         output='screen',
#         arguments=['--ros-args', '--log-level', 'info'],
#     )

#     return launch.LaunchDescription([container])

import launch
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # 首先启动容器（如果还没有运行）
    container = Node(
        package='rclcpp_components',
        executable='component_container',
        name='nvenc_multicam_container',
        output='screen'
    )
    
    # 然后加载组件
    load_component = ExecuteProcess(
        cmd=[
            'ros2', 'component', 'load',
            '/nvenc_multicam_container',
            'color_pointscloud',
            'color_pointscloud::ColorPointsComponent',
            '--node-name', 'color_points_component',
            # 如果需要参数
            '-p', 'use_sim_time:=false'
        ],
        output='screen'
    )
    
    return launch.LaunchDescription([load_component])