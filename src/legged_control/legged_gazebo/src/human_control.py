#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import math
import time

def move_human():
    rospy.init_node('move_human_node')
    rate = rospy.Rate(30)  # 控制频率 30Hz

    # 初始化位置和速度
    x, y = 0.0, 0.0
    speed = 0.5  # 速度 m/s
    amplitude = 2.0  # 运动幅度

    while not rospy.is_shutdown():
        # 示例 1：直线往返运动
        x += speed * 0.033  # 0.033s为近似周期（30Hz）
        # if x > amplitude or x < -amplitude:
        #     speed *= -1  # 反向

        # 示例 2：圆周运动
        # t = time.time()
        # x = amplitude * math.cos(t)
        # y = amplitude * math.sin(t)

        # 发布新位置
        state_msg = ModelState()
        state_msg.model_name = "human_model"
        state_msg.pose.position.x = x
        state_msg.pose.position.y = y
        state_msg.pose.position.z = 0.0
        state_msg.pose.orientation.w = 1.0  # 无旋转

        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state(state_msg)
        except rospy.ServiceException as e:
            print("Service call failed:", e)

        rate.sleep()

if __name__ == '__main__':
    try:
        move_human()
    except rospy.ROSInterruptException:
        pass