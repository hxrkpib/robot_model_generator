from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_share_directory = get_package_share_directory('model_generator')

    robot_description_content = '<robot name="empty"><link name="base_link"></link></robot>'

    rviz_config_file = os.path.join(
        package_share_directory, 'rviz', 'default.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'data_path',
            default_value='',  # 设置默认值
        ),

        Node(
            package='model_generator',
            executable='model_generator',
            name='model_generator',
            output='screen',
            parameters=[{'data_path': LaunchConfiguration('data_path')}],
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
