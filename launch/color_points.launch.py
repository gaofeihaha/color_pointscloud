import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable

def generate_launch_description():
    # 获取包路径
    pkg_dir = get_package_share_directory('color_pointscloud')
    
    # 参数配置
    config_file = PathJoinSubstitution([
        pkg_dir, 'config', 'config.yaml'
    ])
    
    # 声明启动参数
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to configuration YAML file'
    )
    
    # 创建节点
    color_pointscloud_node = Node(
        package='color_pointscloud',
        executable='color_pointscloud_node',
        name='color_pointscloud_node',
        output='screen',
        parameters=[
            {
                'config_file': LaunchConfiguration('config_file')
            }
        ],
        remappings=[
            # 如果需要重新映射话题，可以在这里添加
            # ('/input_topic', '/new_topic_name'),
        ],
        arguments=[
            '--ros-args',
            '--log-level', 'info'  # 可以设置为debug, info, warn, error, fatal
        ]
    )
    # color_pointscloud_node = Node(
    # package='color_pointscloud',
    # executable='color_pointscloud_node',
    # name='color_pointscloud_node',
    # output='screen',
    # prefix=['gnome-terminal -- gdb -ex run --args'],
    # parameters=[
    #     {
    #         'config_file': LaunchConfiguration('config_file')
    #     }
    # ]
# )
    
    # 可选：添加TF相关节点
    # tf_static_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_transform_publisher',
    #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link']
    # )
    
    return LaunchDescription([
        config_file_arg,
        color_pointscloud_node,
        # tf_static_node,  # 如果需要静态TF变换
    ])