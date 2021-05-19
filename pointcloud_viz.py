#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSPresetProfiles, qos_profile_system_default
import sys
from utils import point_cloud2
from panda3d_viewer import Viewer, ViewerConfig

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('pointcloud_visualize')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
        self.subscription = self.create_subscription(
            PointCloud2,
            "/velodyne_points" if len(sys.argv) <= 1 else sys.argv[1],
            self.listener_callback,
            qos_profile)

        self.viewer = Viewer(show_grid=False)
        self.viewer.reset_camera((10, 10, 15), look_at=(0, 0, 0))
        self.viewer.append_group('root')
        self.viewer.append_cloud('root', 'cloud', thickness=4)

    def listener_callback(self, msg):
        points = point_cloud2.pointcloud2_to_xyz_array(msg)
        self.viewer.set_cloud_data('root', 'cloud', np.float32(points))

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

if __name__ == '__main__':
    main()

