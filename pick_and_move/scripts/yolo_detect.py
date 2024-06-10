#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from yolov5_ros_msgs.msg import BoundingBoxes, BoundingBox
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Point


class Readyolo:
    def __init__(self):
        rospy.init_node('grasping_node', anonymous=True)

        self.bridge = CvBridge()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.rgb_image_sub = rospy.Subscriber('/camera1/rgb/image_raw', Image, self.rgb_image_callback)
        self.depth_image_sub = rospy.Subscriber('/camera1/depth/image_raw', Image, self.depth_image_callback)
        self.camera_info_sub = rospy.Subscriber('/camera1/depth/camera_info', CameraInfo, self.camera_info_callback)

        self.object_detection = rospy.Subscriber('/yolov5/BoundingBoxes', BoundingBoxes, self.yolo)
        self.pub = rospy.Publisher('/object_camera_coordinates', Point, queue_size=10)

        self.depth_intrinsics = None
        self.rgb_image = None
        self.depth_image = None

    def camera_info_callback(self, msg):
        self.depth_intrinsics = msg
        # print(self.depth_intrinsics.K[0])
        # print(self.depth_intrinsics.K[4])
        # print()

    def rgb_image_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def depth_image_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.process_yolo_results()

    def process_yolo_results(self):
        if self.depth_intrinsics is None or self.rgb_image is None or self.depth_image is None:
            return

        for box in self.bounding_boxes:
            yolo_center_point_x = (box.xmin + box.xmax) / 2
            yolo_center_point_y = (box.ymin + box.ymax) / 2

            depth = self.depth_image[int(yolo_center_point_y)][int(yolo_center_point_x)]

            camera_point = self.convert_pixel_to_camera_coordinates(yolo_center_point_x, yolo_center_point_y, depth)

            transform = self.tf_buffer.lookup_transform("cr5_base_link", "cam1_rgb_link", rospy.Time())
            base_x = transform.transform.translation.x
            base_y = transform.transform.translation.y
            base_z = transform.transform.translation.z

            # print("OI:",yolo_center_point_x,yolo_center_point_y)
            point_msg = Point(x=base_x-camera_point[1]-0.01, y=base_y-camera_point[0], z=base_z-camera_point[2]+0.025)
            self.pub.publish(point_msg)

    def convert_pixel_to_camera_coordinates(self, u, v, depth):
        if self.depth_intrinsics is None:
            return None

        fx = self.depth_intrinsics.K[0]
        fy = self.depth_intrinsics.K[4]
        cx = self.depth_intrinsics.K[2]
        cy = self.depth_intrinsics.K[5]

        camera_x = (u - cx) * depth / fx
        camera_y = (v - cy) * depth / fy
        camera_z = float(depth)
        print("camera_link:",camera_x,camera_y,camera_z)

        return [camera_x, camera_y, camera_z]

    def yolo(self, msg):
        self.bounding_boxes = msg.bounding_boxes
        self.process_yolo_results()

def main():
    readyolo = Readyolo()
    rospy.spin()

if __name__ == '__main__':
    main()
