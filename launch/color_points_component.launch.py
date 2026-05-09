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
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """Launch color_pointscloud component in its own container."""
    
    container = ComposableNodeContainer(
        name='color_points_container',  # 改了一个不冲突的名字
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='color_pointscloud',
                plugin='color_pointscloud::ColorPointsComponent',
                name='color_points_component',
                parameters=[
                    {'use_sim_time': False},
                ]
            ),
        ],
        output='screen',
    )

    return launch.LaunchDescription([container])