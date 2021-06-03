#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from sensor_msgs.msg import Image
import time
import cv2
import cv_bridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSPresetProfiles, qos_profile_system_default
import sys
from queue import Queue
import queue
from threading import Thread, Lock
from utils import point_cloud2
from panda3d_viewer import Viewer, ViewerConfig
from timeit import default_timer as timer

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
        self.lock = Lock()
        self.subscription = self.create_subscription(
            Image,
            '/detectnet/overlay',
            self.detection_callback,
            qos_profile)
        self.subscription = self.create_subscription(
            Image,
            '/segnet/overlay',
            self.segmentation_callback,
            qos_profile)

        self.subscription = self.create_subscription(
            PointCloud2,
            "/velodyne_points",
            self.points_callback,
            qos_profile)

        cv2.namedWindow("merged", cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty("merged",cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)


        self.bridge = cv_bridge.CvBridge()
        self.video_detect = cv2.VideoWriter('detection.avi', cv2.VideoWriter_fourcc('M','J','P','G'), 8, (640, 480))
        self.video_seg = cv2.VideoWriter('segmentation.avi', cv2.VideoWriter_fourcc('M','J','P','G'), 8, (640, 480))

        self.queues = Queue()
        config = ViewerConfig()
        config.set_window_size(640*2, 480)
        config.show_grid(False)
        config.show_axes(False)

        self.viewer = Viewer(config=config)
        self.viewer.reset_camera((-40, 0, 20), look_at=(0, 0, 0))
        self.viewer.append_group('root')
        self.viewer.append_cloud('root', 'cloud', thickness=4)

        Thread(target=self.compose_frame).start()

    def compose_frame(self):
        video = cv2.VideoWriter('merged.avi', cv2.VideoWriter_fourcc('M','J','P','G'), 8, (640 * 2, 2*480))
        frames = {}
        while True:
            try:
                while True:
                    name, img = self.queues.get(False)
                    frames[name] = img
            except queue.Empty:
                pass

            if not all(i in frames.keys() for i in ['segnet', 'detectnet', 'lidar']):
                continue

            points = point_cloud2.pointcloud2_to_xyz_array(frames['lidar'])
            self.viewer.set_cloud_data('root', 'cloud', np.float32(points))

            res = np.concatenate((frames['segnet'], frames['detectnet']), axis=1)
            res = np.concatenate((res, self.viewer.get_screenshot('BGR')), axis=0)
            #video.write(res)
            res = cv2.resize(res, (1920, 1080))
            cv2.imshow("merged", res)
            cv2.waitKey(1)

#            time.sleep(1/10)

    def points_callback(self, msg): 
        self.queues.put(('lidar', msg))

    def detection_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        #self.video_detect.write(img)
        self.queues.put(('detectnet', img))
        """
        with self.lock:
            cv2.imshow("detection", img)
            cv2.waitKey(1)
        """

    def segmentation_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8") 
        #self.video_seg.write(img)
        self.queues.put(('segnet', img))
        """
        with self.lock:
            cv2.imshow("segmentation", img)
            cv2.waitKey(1)
        """


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

if __name__ == '__main__':
    main()
