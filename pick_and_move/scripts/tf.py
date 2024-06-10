#!/usr/bin/env python
# -*- coding: utf-8 -*-10
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Point

if __name__ == '__main__':
    rospy.init_node('transform_listener', anonymous=True)
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            # 查询夹爪坐标系到机械臂底座坐标系的变换关系
            transform = tf_buffer.lookup_transform("cr5_base_link", "grasping_frame", rospy.Time())
            print("Transform from gripper_link to base_link:")
            print("Translation: ", transform.transform.translation)
            # print("Rotation: ", transform.transform.rotation)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print("Failed to lookup transform:", e)
        
        rate.sleep()
