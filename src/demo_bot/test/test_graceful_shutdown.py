import os
import signal
import subprocess
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


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


class TestGracefulShutdown(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_graceful_shutdown')

    def tearDown(self):
        self.node.destroy_node()

    def test_system_survives_node_kill(self, proc_output):
        # Phase 1: Confirm mover is running by receiving /cmd_vel messages
        cmd_msgs = []
        sub_cmd = self.node.create_subscription(
            Twist, '/cmd_vel', lambda msg: cmd_msgs.append(msg), 10)
        try:
            end = time.time() + 60
            while time.time() < end and len(cmd_msgs) < 5:
                rclpy.spin_once(self.node, timeout_sec=1)
            self.assertGreaterEqual(
                len(cmd_msgs), 5,
                f'Mover not publishing: got {len(cmd_msgs)} /cmd_vel msgs')
        finally:
            self.node.destroy_subscription(sub_cmd)

        # Phase 2: Kill the mover node process via SIGTERM
        result = subprocess.run(
            ['pgrep', '-f', 'mover.py'],
            capture_output=True, text=True)
        pids = [p for p in result.stdout.strip().split('\n') if p]
        self.assertGreater(
            len(pids), 0, 'Could not find mover process PID')
        os.kill(int(pids[0]), signal.SIGTERM)

        # Phase 3: Wait briefly, then verify Gazebo still publishes /odom
        time.sleep(3)
        odom_msgs = []
        sub_odom = self.node.create_subscription(
            Odometry, '/odom', lambda msg: odom_msgs.append(msg), 10)
        try:
            end = time.time() + 30
            while time.time() < end and len(odom_msgs) < 3:
                rclpy.spin_once(self.node, timeout_sec=1)
            self.assertGreaterEqual(
                len(odom_msgs), 3,
                f'After killing mover, Gazebo stopped publishing /odom: '
                f'got {len(odom_msgs)} msgs')
        finally:
            self.node.destroy_subscription(sub_odom)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, -signal.SIGTERM],
        )
