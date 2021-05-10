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

import time
import unittest

import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node as LaunchNode
from launch_ros.substitutions import FindPackageShare
import launch_testing
import launch_testing.asserts
import rclpy
from rclpy.node import Node


def generate_test_description():
    share_dir = FindPackageShare('rexrov2_description')
    spawn_launch_path = PathJoinSubstitution([share_dir, 'launch', 'upload.launch.py'])
    spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_launch_path),
        launch_arguments={'namespace': 'ns'}.items(),
    )

    return (
        launch.LaunchDescription([
            spawn_launch,
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'spawn_launch': spawn_launch
        }
    )


class TestSpawnLaunchInterface(unittest.TestCase):

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()
        cls.this_node = Node('test_node')

    def test_node_names(self, proc_info, spawn_launch):
        self.assertEqual(1, len(spawn_launch.get_sub_entities()))
        nodes = list(
            filter(
                lambda entity: isinstance(entity, LaunchNode),
                spawn_launch.get_sub_entities()[0].entities
            )
        )

        expected_nodes = ['/ns/robot_state_publisher']

        for node in nodes:
            proc_info.assertWaitForStartup(node, timeout=5)
            self.assertTrue(node.is_node_name_fully_specified())
            self.assertIn(node.node_name, expected_nodes)

    def get_topic_names_and_types(self, expected, timeout):
        """Make sure discovery has found all 'expected' topics."""
        start = time.monotonic()
        while True:
            topics = self.this_node.get_topic_names_and_types()
            now = time.monotonic()
            if all(expected_topic in topics for expected_topic in expected):
                return topics
            elif (now - start) > timeout:
                return None

    def test_topics(self):
        expected = [
            ('/ns/robot_description', ['std_msgs/msg/String'])
        ]
        topics = self.get_topic_names_and_types(expected, timeout=0.5)

        for expected_topic in expected:
            self.assertIn(expected_topic, topics)


@launch_testing.post_shutdown_test()
class TestProcessTermination(unittest.TestCase):

    def test_exit_code(self, proc_info):
        # rclpy does not exit normally on SIGINT signal (https://github.com/ros2/rclpy/issues/527)
        launch_testing.asserts.assertExitCodes(proc_info, [launch_testing.asserts.EXIT_OK, -2])