import math
import os
import time
import unittest

import launch
import launch_testing.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import rclpy
from sensor_msgs.msg import LaserScan

MIN_VALID_SCANS = 10


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


class TestSensorSanity(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_sensor_sanity')

    def tearDown(self):
        self.node.destroy_node()

    def test_lidar_publishes_valid_data(self, proc_output):
        scans = []
        sub = self.node.create_subscription(
            LaserScan, '/scan', lambda msg: scans.append(msg), 10)
        try:
            end = time.time() + 90
            while time.time() < end and len(scans) < MIN_VALID_SCANS:
                rclpy.spin_once(self.node, timeout_sec=1)

            self.assertGreaterEqual(
                len(scans), MIN_VALID_SCANS,
                f'Expected >= {MIN_VALID_SCANS} /scan messages, '
                f'got {len(scans)}')

            # Check range_min and range_max are sane
            last = scans[-1]
            self.assertGreater(
                last.range_min, 0.0,
                f'range_min should be positive, got {last.range_min}')
            self.assertGreater(
                last.range_max, last.range_min,
                f'range_max ({last.range_max}) should exceed '
                f'range_min ({last.range_min})')

            # Check that ranges are not all zeros (broken sensor)
            all_ranges = []
            for scan in scans:
                all_ranges.extend(scan.ranges)
            non_zero = [r for r in all_ranges if r > 0.0]
            self.assertGreater(
                len(non_zero), 0,
                'All LiDAR ranges are 0.0 — sensor appears broken')

            # Check that ranges are not all inf (no returns at all)
            finite = [r for r in all_ranges if not math.isinf(r)]
            self.assertGreater(
                len(finite), 0,
                'All LiDAR ranges are inf — no valid returns detected')
        finally:
            self.node.destroy_subscription(sub)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
