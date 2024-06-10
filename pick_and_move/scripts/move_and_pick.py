#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import Point
from actionlib.action_client import GoalManager
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult

class MoveItAndSendGoal:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('moveit_and_send_goal')

        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化需要使用move group控制的机械臂中的arm group
        self.arm = moveit_commander.MoveGroupCommander('arm_group')
        self.gripper = moveit_commander.MoveGroupCommander('gripper_group')

        # 订阅移动基座到达目标点的结果
        self.move_base_result = False
        self.move_base_subscriber = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.move_base_callback)

        # 控制机械臂先回到初始化位置
        self.arm.set_named_target('wait')
        self.arm.go()
        rospy.sleep(1)

        # 执行第一段代码
        self.send_goals_python()

        # 等待移动基座到达目标点
        while not self.move_base_result:
            rospy.sleep(0.1)

        # 取消订阅移动基座结果主题
        self.move_base_subscriber.unregister()

        rospy.sleep(1)
        # 控制机械臂先回到初始化位置
        self.arm.set_named_target('hold')
        self.arm.go()
        rospy.sleep(1)

        # 执行第二段代码
        msg = rospy.wait_for_message("/object_camera_coordinates", Point)
        self.callback(msg)

        rospy.sleep(0.2)

        self.arm.set_named_target('wait')
        self.arm.go()
        rospy.sleep(3)

        # 订阅移动基座到达目标点的结果
        self.move_base_result = False
        self.move_base_subscriber = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.move_base_callback)
        
        # 执行第一段代码
        self.send_goals_python2()

        # 等待移动基座到达目标点
        while not self.move_base_result:
            rospy.sleep(0.1)

        # 取消订阅移动基座结果主题
        self.move_base_subscriber.unregister()

        rospy.sleep(2)

        success = False
        self.arm.set_named_target('pick')
        self.arm.go()
        success = self.arm.go(wait=True)
        while not success:
            if success:
                break
            else:
                success = self.arm.go(wait=True)
        rospy.sleep(1)

        self.gripper.set_named_target('open')
        self.gripper.go()
        success = self.gripper.go(wait=True)
        while not success:
            if success:
                break
            else:
                success = self.gripper.go(wait=True)
        rospy.sleep(1)
        
        self.arm.set_named_target('wait')
        self.arm.go()
        rospy.sleep(1)

        self.move_base_result = False
        self.move_base_subscriber = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.move_base_callback)
        # 执行第一段代码
        self.send_goals_python3()

        # 等待移动基座到达目标点
        while not self.move_base_result:
            rospy.sleep(0.1)

        # 取消订阅移动基座结果主题
        self.move_base_subscriber.unregister()

        rospy.sleep(2)

        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)



    def move_base_callback(self, data):
        # 当移动基座到达目标点时，设置标志为True
        if data.status.status == 3:  # 3表示目标已经成功到达
            self.move_base_result = True

    def send_goals_python(self):
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()
        # 定义四个发送目标点的对象
        car_goal = MoveBaseGoal()

        car_goal.target_pose.pose.position.x = 8.05
        car_goal.target_pose.pose.position.y = 0.54
        # car_goal.target_pose.pose.orientation.x = -1.8448068732365062e-05
        # car_goal.target_pose.pose.orientation.y = -1.803106198251614e-05
        # car_goal.target_pose.pose.orientation.z = 0.7012649339067702
        # car_goal.target_pose.pose.orientation.w = -0.712900758736644
        car_goal.target_pose.pose.orientation.x = 0.0
        car_goal.target_pose.pose.orientation.y = 0.0
        car_goal.target_pose.pose.orientation.z = -0.7071
        car_goal.target_pose.pose.orientation.w = 0.7071

        car_goal.target_pose.header.frame_id = "map"
        car_goal.target_pose.header.stamp = rospy.Time.now()
        client.send_goal(car_goal)

    def send_goals_python2(self):
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()
        # 定义四个发送目标点的对象
        car_goal = MoveBaseGoal()

        car_goal.target_pose.pose.position.x = 2.679853007576538
        car_goal.target_pose.pose.position.y = 1.7335153601690219
        # car_goal.target_pose.pose.orientation.x = 0.0009620160298047264
        # car_goal.target_pose.pose.orientation.y = -0.0009470103063894415
        # car_goal.target_pose.pose.orientation.z = -0.7126945046487707
        # car_goal.target_pose.pose.orientation.w = -0.7014732501956733
        car_goal.target_pose.pose.orientation.x = 0.0
        car_goal.target_pose.pose.orientation.y = 0.0
        car_goal.target_pose.pose.orientation.z = 0.7071
        car_goal.target_pose.pose.orientation.w = 0.7071

        car_goal.target_pose.header.frame_id = "map"
        car_goal.target_pose.header.stamp = rospy.Time.now()
        client.send_goal(car_goal)

    def send_goals_python3(self):
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()
        # 定义四个发送目标点的对象
        car_goal = MoveBaseGoal()

        car_goal.target_pose.pose.position.x = 0
        car_goal.target_pose.pose.position.y = 0
        car_goal.target_pose.pose.orientation.x = 0
        car_goal.target_pose.pose.orientation.y = 0
        car_goal.target_pose.pose.orientation.z = 0
        car_goal.target_pose.pose.orientation.w = 1.0

        car_goal.target_pose.header.frame_id = "map"
        car_goal.target_pose.header.stamp = rospy.Time.now()
        client.send_goal(car_goal)

    def callback(self, msg):
        self.grasp_x = msg.x
        self.grasp_y = msg.y
        self.grasp_z = msg.z
        print("x:", self.grasp_x)
        print("y:", self.grasp_y)
        print("z:", self.grasp_z)
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

        # 设置目标位置所使用的参考坐标系
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
        target_pose.pose.position.z = -0.11
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
        rospy.sleep(0.5)

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

        rospy.sleep(0.5)
        # 关闭并退出moveit
        # moveit_commander.roscpp_shutdown()
        # moveit_commander.os._exit(0)
if __name__ == "__main__":
    MoveItAndSendGoal()
