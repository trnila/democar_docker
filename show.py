#!/usr/bin/env python3
import numpy
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
import cv_bridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSPresetProfiles, qos_profile_system_default
import sys


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
        self.subscription = self.create_subscription(
            Image,
            sys.argv[1],
            self.listener_callback,
            qos_profile)
        self.bridge = cv_bridge.CvBridge()

    def listener_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8") 
        cv2.imshow("img", img)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

if __name__ == '__main__':
    main()

