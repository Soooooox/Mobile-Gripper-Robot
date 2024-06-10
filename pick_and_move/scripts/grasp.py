#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import Point
    
class MoveItIkDemo:
    def callback(self,msg):
        self.grasp_x = msg.x
        self.grasp_y = msg.y
        self.grasp_z = msg.z
        print("x:",self.grasp_x)
        print("y:",self.grasp_y)
        print("z:",self.grasp_z)
        # 如果msg.x小于0.3，重新获取消息
        if self.grasp_x < 0.3:
            rospy.loginfo("msg.x is less than 0.3. Receiving message again.")
            msg = rospy.wait_for_message("/object_camera_coordinates", Point)
            self.callback(msg)
            return
        
        # 获取终端link的名称
        end_effector_link = self.arm.get_end_effector_link()

        # 设置目标位置所使用的参考坐标系
        reference_frame = 'cr5_base_link'
        self.arm.set_pose_reference_frame(reference_frame)

        # 当运动规划失败后，允许重新规划
        self.arm.allow_replanning(True)

        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        self.arm.set_goal_position_tolerance(0.001)
        self.arm.set_goal_orientation_tolerance(0.01)

        # 设置允许的最大速度和加速度
        self.arm.set_max_acceleration_scaling_factor(1)
        self.arm.set_max_velocity_scaling_factor(1)

        # # 设置目标位置所使用的参考坐标系
        gripper_reference_frame = 'gripper_base_link'
        self.gripper.set_pose_reference_frame(gripper_reference_frame)

        # 当运动规划失败后，允许重新规划
        self.gripper.allow_replanning(True)

        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        self.gripper.set_goal_position_tolerance(0.001)
        self.gripper.set_goal_orientation_tolerance(0.01)

        # 设置允许的最大速度和加速度
        self.gripper.set_max_acceleration_scaling_factor(1)
        self.gripper.set_max_velocity_scaling_factor(1)

        rospy.sleep(1)

        # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于base_link坐标系
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = self.grasp_x
        target_pose.pose.position.y = self.grasp_y
        # target_pose.pose.position.z = -0.11
        target_pose.pose.position.z = 0.025
        target_pose.pose.orientation.x = 0.9999996966099037
        target_pose.pose.orientation.y = -0.0004551985760960136
        target_pose.pose.orientation.z = -0.0005154428497122243
        target_pose.pose.orientation.w = 0.00036591396122690405

        # 设置机器臂当前的状态作为运动初始状态
        self.arm.set_start_state_to_current_state()

        # 设置机械臂终端运动的目标位姿
        self.arm.set_pose_target(target_pose, end_effector_link)
        success = self.arm.go(wait=True)
        while not success:
            if success:
                break
            else:
                success = self.arm.go(wait=True)
        rospy.sleep(3)

        joint_positions = [0.0153,0.0150]
        result=self.gripper.set_joint_value_target(joint_positions)
        success = self.gripper.go(wait=True)
        while not success:
            if success:
                break
            else:
                success = self.gripper.go(wait=True)

        rospy.sleep(1)

        self.arm.set_named_target('hold')
        success = self.arm.go(wait=True)
        while not success:
            if success:
                break
            else:
                success = self.arm.go(wait=True)

        rospy.sleep(1)

    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('moveit_ik_demo')

        # 初始化需要使用move group控制的机械臂中的arm group
        self.arm = moveit_commander.MoveGroupCommander('arm_group')
        # self.gripper = moveit_commander.MoveGroupCommander('gripper_group')

        msg = rospy.wait_for_message("/object_camera_coordinates",Point)

        self.callback(msg)

        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


if __name__ == "__main__":
    MoveItIkDemo()