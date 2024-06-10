#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

def callback(data):
    # 从里程计数据中提取机器人的位置信息
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y

    # 从里程计数据中提取机器人的姿态信息
    orientation = data.pose.pose.orientation
    quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
    roll, pitch, yaw = euler_from_quaternion(quaternion)

    # 将弧度制的航向角转换为角度制
    yaw_deg = math.degrees(yaw)

    # 输出机器人在地图上的位置和航向角
    rospy.loginfo("Robot Position on Map (X, Y): ({}, {})".format(x, y))
    rospy.loginfo("Robot Yaw Angle (Degrees): {}".format(yaw_deg))

def listener():
    rospy.init_node('robot_map_pose_listener', anonymous=True)

    # 订阅里程计主题，参数为回调函数
    rospy.Subscriber("/odom", Odometry, callback)

    # 循环等待ROS消息
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
