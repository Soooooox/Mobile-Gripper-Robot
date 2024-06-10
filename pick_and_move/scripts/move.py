#!/usr/bin/env python
# -*- coding: utf-8 -*-
from actionlib.action_client import GoalManager
import rospy 
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def send_goals_python():
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()
    #定义四个发送目标点的对象
    car_goal= MoveBaseGoal()

    car_goal.target_pose.pose.position.x = 8.05
    car_goal.target_pose.pose.position.y = 0.54
    car_goal.target_pose.pose.orientation.x = -1.8448068732365062e-05
    car_goal.target_pose.pose.orientation.y = -1.803106198251614e-05
    car_goal.target_pose.pose.orientation.z = 0.7012649339067702
    car_goal.target_pose.pose.orientation.w = -0.712900758736644

    # car_goal.target_pose.pose.position.x = 8.0
    # car_goal.target_pose.pose.position.y = 0.5
    # car_goal.target_pose.pose.orientation.x = 0.0
    # car_goal.target_pose.pose.orientation.y = 0.0
    # car_goal.target_pose.pose.orientation.z = -0.7071
    # car_goal.target_pose.pose.orientation.w = 0.7071
    
    # car_goal.target_pose.pose.position.x = 2.5
    # car_goal.target_pose.pose.position.y = 1.5
    # car_goal.target_pose.pose.orientation.x = 0.0
    # car_goal.target_pose.pose.orientation.y = 0.0
    # car_goal.target_pose.pose.orientation.z = 0.7071
    # car_goal.target_pose.pose.orientation.w = 0.7071

    car_goal.target_pose.header.frame_id = "map"
    car_goal.target_pose.header.stamp = rospy.Time.now()
    client.send_goal(car_goal)

if __name__ == '__main__':
        rospy.init_node('send_goals_python',anonymous=True)    # python 语言方式下的　初始化 ROS 节点，
        result = send_goals_python()