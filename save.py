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

def hsv_to_rgb(hsv):
#    hsv[:, 0] = 
#    hsv[:, 1] = hsv[:, 0]
#    hsv[:, 2] = hsv[:, 0]
    val = np.linalg.norm(hsv, axis=(1))
    m = np.min(val)
    M = np.max(val)
    val = (val - m) / (M - m)

    hsv[:, 0] = val
    hsv[:, 1] = val
    hsv[:, 2] = val

    return hsv

    hsv *= 255
    hi = np.floor(hsv[..., 0] / 60.0) % 6
    hi = hi.astype('uint8')
    v = hsv[..., 2].astype('float')
    f = (hsv[..., 0] / 60.0) - np.floor(hsv[..., 0] / 60.0)
    p = v * (1.0 - hsv[..., 1])
    q = v * (1.0 - (f * hsv[..., 1]))
    t = v * (1.0 - ((1.0 - f) * hsv[..., 1]))

    rgb = np.zeros(hsv.shape)
    rgb[hi == 0, :] = np.dstack((v, t, p))[0][hi == 0, :]
    rgb[hi == 1, :] = np.dstack((q, v, p))[0][hi == 1, :]
    rgb[hi == 2, :] = np.dstack((p, v, t))[0][hi == 2, :]
    rgb[hi == 3, :] = np.dstack((p, q, v))[0][hi == 3, :]
    rgb[hi == 4, :] = np.dstack((t, p, v))[0][hi == 4, :]
    rgb[hi == 5, :] = np.dstack((v, p, q))[0][hi == 5, :]

    return rgb / 255

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
            colors = np.ones((points.shape[0], 4), np.float32)

            import matplotlib
            
            def ff(arg):
                return matplotlib.colors.hsv_to_rgb((arg[2], 1, 1))

            inp = np.clip(np.abs(points)[:, :3] / 3, 0, 1)
#            colors[:, :3] = np.apply_along_axis(ff, 1, inp)
            colors[:, :3] = hsv_to_rgb(inp)


#            f = np.vectorize(matplotlib.colors.hsv_to_rgb)
#            f = np.vectorize(ff)
#            print(f([np.clip(np.abs(points)[:, :3] / 20, 0, 1) / 20]))
#            colors[:, :3] = f(np.clip(np.abs(points)[:, :3] / 20, 0, 1))


            self.viewer.set_cloud_data('root', 'cloud', np.float32(points), colors)

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
