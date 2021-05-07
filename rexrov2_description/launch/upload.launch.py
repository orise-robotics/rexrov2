import os

import launch
import launch_ros.actions
from launch_ros.actions import Node
from launch_ros.substitutions import ExecutableInPackage
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command
from launch.substitutions import PathJoinSubstitution
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource

import xacro


def generate_launch_description():
    
    x = LaunchConfiguration('x', default='0.0')
    y = LaunchConfiguration('y', default='0.0')
    z = LaunchConfiguration('z', default='1.0')
    urdf_file = PathJoinSubstitution([FindPackageShare('rexrov2_description'), 'robots/teste.xacro'])

    gazebo = ExecuteProcess(
          cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
          output='screen')

    rsp = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description':
                Command([ExecutableInPackage(package='xacro', executable='xacro'), ' ', urdf_file])
            }])

    spawn_entity = Node(
            package='gazebo_ros', 
            executable='spawn_entity.py',
            arguments=['-entity', 'rexrov2', '-topic', '/robot_description',
                    '-x',  x, '-y', y, '-z', z],
            output='screen')
            
    return LaunchDescription([
        gazebo,
        rsp,
        spawn_entity,
    ])
