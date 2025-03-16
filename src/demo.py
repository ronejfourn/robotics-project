#!/usr/bin/env python

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseStamped

import numpy as np
import cv2
from cv_bridge import CvBridge
import ros2_numpy as r2np

import time

bridge = CvBridge()

class ProcessCameraImage(Node):
    def __init__(self,):
        super().__init__('process_camera_image')
        self.sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.rgb_listener_callback,
            10,
        )
        self.sub2 = self.create_subscription(
            PointCloud2,
            '/camera/points',
            self.pc_listener_callback,
            10,
        )
        self.last_known_pc = None
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.last_pub_time = 0

    def rgb_listener_callback(self, msg):
        if self.last_known_pc is None:
            return
        if (time.time() - self.last_pub_time) < 20.0:
            return
        img_bgr = bridge.imgmsg_to_cv2(msg, 'bgra8')[..., :3]
        img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        mask_green = cv2.inRange(img_hsv, (36, 25, 25), (70, 255,255))

        contours, hierarchy = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if len(contours) == 0:
            return
        try:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
        except:
            # div by 0 and whatever else
            return

        msg = PoseStamped()
        msg.header.stamp= self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link_optical'
        msg.pose.position.x = self.last_known_pc['x'][cy,cx].item()
        msg.pose.position.y = self.last_known_pc['y'][cy,cx].item()
        msg.pose.position.z = self.last_known_pc['z'][cy,cx].item()

        # coorrect for optical axis being z forward
        msg.pose.orientation.x = -0.5
        msg.pose.orientation.y = -0.5
        msg.pose.orientation.z = -0.5
        msg.pose.orientation.w = 0.5

        self.publisher.publish(msg)
        self.get_logger().info(f'Published Goal Pose: {msg}')
        self.last_pub_time = time.time()

    def pc_listener_callback(self, msg):
        data = r2np.numpify(msg)
        self.last_known_pc = data

def main(args=None):
    rclpy.init(args=args)
    proc = ProcessCameraImage()
    rclpy.spin(proc)
    proc.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
