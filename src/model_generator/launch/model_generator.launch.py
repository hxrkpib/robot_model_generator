from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    urdf_path = '/home/lizhen/works/open_source/robot_model_generator/generator/model_generate/output/output.urdf'  # 修改为您的URDF文件名

    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF 文件不存在: {urdf_path}")

    with open(urdf_path, 'r') as urdf_file:
        robot_description_content = urdf_file.read()

    rviz_config_file = os.path.join(
        package_share_directory, 'rviz', 'default.rviz')

    return LaunchDescription([

        Node(
            package='model_generator',
            executable='model_generator',
            name='model_generator',
            output='screen',

        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}]
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            parameters=[{'rate': 200}],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=[
                '-d', rviz_config_file] if os.path.exists(rviz_config_file) else []
        )
    ])
