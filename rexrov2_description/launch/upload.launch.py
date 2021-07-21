# Copyright 2016 UUV Simulator Authors
# Copyright 2021 Open Rise Robotics
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

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import ExecutableInPackage
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    x = LaunchConfiguration('x', default='0.0')
    y = LaunchConfiguration('y', default='0.0')
    z = LaunchConfiguration('z', default='1.0')
    urdf_file = PathJoinSubstitution(
        [FindPackageShare('rexrov2_description'), 'robots', 'rexrov2_default.xacro'])

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
                Command([
                    ExecutableInPackage(package='xacro', executable='xacro'), ' ', urdf_file
                ])
            }])

    spawn_entity = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'rexrov2', '-topic', '/robot_description', '-x', x, '-y', y, '-z', z],
            output='screen')

    return LaunchDescription([
        gazebo,
        rsp,
        spawn_entity,
    ])
