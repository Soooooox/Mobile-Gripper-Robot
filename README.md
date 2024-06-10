# 基于ROS的移动抓取机器人仿真

## 参考文章和视频

- ROS学习：[机器人工匠阿杰 - ROS 快速入门教程](https://space.bilibili.com/411541289/channel/collectiondetail?sid=693700) 和 [赵虚左 - ROS移动机器人精致入门](https://www.bilibili.com/video/BV1Ci4y1L7ZZ/)
- Moveit!控制机械臂参考：[New Series: Simulating Your Custom Robotic Arm In ROS Noetic](https://youtu.be/T_UOIHEol0I?list=PLeEzO_sX5H6TNMBiworxO8RpQlcYfl54y) 及其文档 [Importing_Your_ROS_Package_Generated_from_SolidWorks_in_ROS_Noetic.pdf](https://github.com/ageofrobotics/Simulate_Your_Robot_Arm_In_ROS_Noetic/blob/main/Importing_Your_ROS_Package_Generated_from_SolidWorks_in_ROS_Noetic.pdf)
- 发布导航目标点参考：[搭建ROS机器人之用Python发布导航目标点](https://www.bilibili.com/video/BV1iq4y197Xh/) 和 [14_ROS动作通信2：代码实现导航至目标点](https://www.bilibili.com/video/BV1j341157YH/)
- 夹爪 gazebo 插件：[gazebo-pkgs](https://github.com/JenniferBuehler/gazebo-pkgs)
- 目标定位 yolo_detect.py 文件参考：[基于YOLO的机械臂自主抓取(一)——目标定位](https://www.bilibili.com/video/BV1YN4y1R7D2/)
- yolov5_ros 功能包参考：<https://github.com/qq44642754a/Yolov5_ros>
- yolov5的权重文件使用：[基于Moveit与yolov5的gazebo仿真（开源填坑）](https://www.bilibili.com/video/BV1DT411S73o/)

## 功能包以及文件说明

- `camera_description`：D435i深度摄像头URDF模型
  - `d435i_view.launch`：rviz显示摄像头模型
- `car_description`：机器人底盘URDF模型
  - `car_view.launch`：rviz显示底盘模型
- `cr5_description`：Dobot Cr5机械臂URDF模型（含DH系列夹爪，不含D435i）
  - `cr5_view.launch`：rviz显示机械臂模型
- `robot_description`：移动抓取机器人URDF模型
  - `arm_d435i_view.launch`：rviz显示带有深度摄像头的机械臂模型
  - `robot_view.launch`：rviz显示移动抓取机器人模型
- `arm_moveit_config`：机械臂（含有深度摄像头）的moveit配置文件
- `robot_moveit_config`：移动抓取机器人的moveit配置文件
- `gazebo-pkgs`+`general-message-pkgs`：夹爪gazebo插件，解决无法夹住物体问题
- `pick_and_move`：导航和机械臂抓取python执行代码
  - `grasp.py`：单独抓取代码
  - `hold.py`：机械臂运动到待抓取姿态代码
  - `move.py`：导航至某个地点代码
  - `position.py`：获取机器人的位姿（位置和姿态）
  - `tf.py`：获取机械臂末端相对于相机和底座的tf变换
  - `move_and_pick.py`：导航到某地点然后抓取至另一地点的代码
  - `yolo_detect.py`：yolov5的目标检测的中心点计算和机械臂末端运动目标点求解代码
- `robot_gazebo`：机器人gazebo仿真（不含导航）
  - `car_arm.launch`：启动移动抓取机器人的gazebo仿真环境
  - `arm_test.launch`：机械臂测试节点
- `robot_nav`：机器人的导航
  - `car_amcl`：amcl启动文件
  - `car_map_saver`：地图保存启动文件
  - `car_map_server`：地图服务启动文件
  - `car_nav`：机器人导航启动文件（导航只需启动此文件，其他无需要启动）
  - `car_path`：机器人路径规划启动文件
  - `car_slam`：slam启动文件
- `yolov5_ros`：yolov5的ROS功能包
  - `yolo_v5.launch`：目标检测启动文件（最好在conda环境下运行）
  - yolov5_ros功能包使用参考：<https://github.com/qq44642754a/Yolov5_ros>

## 使用

```bash
# 第一步
# 启动移动抓取机器人的gazebo仿真环境
roslaunch robot_gazebo car_arm.launch

# 第二步
# 启动机器人导航
roslaunch robot_nav car_nav.launch

# 第三步
# 进入配置好的yolov5环境并启动yolov5_ros
conda activate xxxx
roslaunch yolov5_ros yolo_v5.launch

# 第四步
# 启动机械臂目标点计算节点
rosrun pick_and_move yolo_detect.py

# 第五步
# 启动测试流程
rosrun pick_and_move move_and_pick.py
```

## 最终效果展示

[展示视频下载](./show.mp4)