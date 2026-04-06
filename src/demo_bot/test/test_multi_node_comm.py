import os
import time
import unittest

import launch
import launch_testing.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import rclpy
from gazebo_msgs.srv import GetModelList


def generate_test_description():
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('demo_bot'),
                'launch', 'sim.launch.py',
            )
        )
    )
    return (
        launch.LaunchDescription([
            sim_launch,
            launch_testing.actions.ReadyToTest(),
        ]),
        {},
    )


class TestMultiNodeComm(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_multi_node_comm')

    def tearDown(self):
        self.node.destroy_node()

    def test_get_model_list_service(self, proc_output):
        client = self.node.create_client(GetModelList, '/get_model_list')
        try:
            # Wait for service to become available
            ready = client.wait_for_service(timeout_sec=30.0)
            self.assertTrue(
                ready, '/get_model_list service not available after 30s')

            # Call the service
            request = GetModelList.Request()
            future = client.call_async(request)

            end = time.time() + 30
            while time.time() < end and not future.done():
                rclpy.spin_once(self.node, timeout_sec=1)

            self.assertTrue(
                future.done(),
                '/get_model_list service did not respond within 30s')

            response = future.result()
            self.assertTrue(
                response.success,
                f'Service call failed: {response.status_message}')

            self.assertIn(
                'turtlebot3_burger', response.model_names,
                f'turtlebot3_burger not found in model list: '
                f'{response.model_names}')
        finally:
            self.node.destroy_client(client)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
