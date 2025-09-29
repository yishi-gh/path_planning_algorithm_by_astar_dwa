import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('octomap_path_planner')
    ply_file_path = "/mnt/c/Users/ADMIN/Desktop/rtabmap_cloud.ply" # 替换为你的文件路径

    # 定义一个兼容的 QOS 配置文件
    qos_profile = {
        'depth': 10,
        'reliability': 'best_effort',
        'durability': 'transient_local'
    }

    return LaunchDescription([
        Node(
            package='octomap_path_planner',
            executable='ply_publisher_node',
            name='ply_publisher_node',
            output='screen',
            parameters=[
                {'ply_file_path': ply_file_path}
            ]
            # 这里添加 QOS 参数
            # qos=qos_profile # 这通常由 Node 内部处理，不直接在 launch 文件中设置，此处仅作概念说明
        ),
        Node(
            package='octomap_path_planner',
            executable='octomap_node',
            name='octomap_node',
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_publisher',
            arguments=[
                '0', '0', '0', 
                '0', '0', '0',
                'map', 'lidar_link'
            ],
            output='screen'
        )
    ])