from launch_ros.actions import Node
from launch_ros.substitutions import ExecutableInPackage
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command
from launch.substitutions import PathJoinSubstitution
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

import os
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    x = LaunchConfiguration('x', default='0.0')
    y = LaunchConfiguration('y', default='0.0')
    z = LaunchConfiguration('z', default='1.0')
    #urdf_file = PathJoinSubstitution([FindPackageShare('rexrov2_description'), 'robots/rexrov2_default.xacro'])
    urdf_file = os.path.join(get_package_share_directory('rexrov2_description'), 'robots/rexrov2_default.xacro')
    #Command([ExecutableInPackage(package='xacro', executable='xacro'), ' ', urdf_file])
    
    robot = xacro.process(urdf_file)

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
                robot
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
