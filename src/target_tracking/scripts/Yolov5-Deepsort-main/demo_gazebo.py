#!/usr/bin/env python3
# coding=utf-8
import rospy
import sys
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import imutils
from std_msgs.msg import String
sys.path.insert(0, "/home/pt215/GO1_test_ws/src/target_tracking/scripts/Yolov5-Deepsort-main")
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from AIDetector_pytorch import Detector
from target_tracking.msg import target
import pyrealsense2 as rs
# 导入目标检测库

from AIDetector_pytorch import Detector

class TargetTracker:
    def __init__(self):
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 同步订阅
        image_sub = Subscriber("/camera/color/image_raw", Image)
        depth_sub = Subscriber("/camera/depth/image_raw", Image)
        self.ts = ApproximateTimeSynchronizer([image_sub, depth_sub], 10, 0.1)
        self.ts.registerCallback(self.sync_callback)

        self.target_pub = rospy.Publisher("/tracking_target", target, queue_size=10)
        
    def sync_callback(self, color_msg, depth_msg):
        try:
            self.current_depth = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")
            color_image = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
            self.process_image(color_image)
        except CvBridgeError as e:
            rospy.logerr(e)

    def process_image(self, color_image):
        result = self.det.feedCap(color_image)
        result_frame = result['frame']
        bboxes = result['face_bboxes']
        
        if bboxes:
            x1, y1, x2, y2 = bboxes[0]
            center_x = int((x1 + x2)/2)
            center_y = int((y1 + y2)/2)
            
            if self.current_depth is not None:
                depth_value = self.current_depth[center_y, center_x] / 1000.0
                fx = 462.1379699707031
                fy = 462.1379699707031
                cx = 320.0
                cy = 240.0

                Z = depth_value
                X = (center_x - cx) * Z / fx
                Y = (center_y - cy) * Z / fy

                try:
                    transform = self.tf_buffer.lookup_transform(
                        "base_link",
                        "front_camera_color_optical_frame",
                        rospy.Time(0),
                        rospy.Duration(1.0)
                    
                    camera_point = PointStamped()
                    camera_point.header.frame_id = "front_camera_color_optical_frame"
                    camera_point.point.x = X
                    camera_point.point.y = Y
                    camera_point.point.z = Z

                    base_point = tf2_geometry_msgs.do_transform_point(camera_point, transform)
                    final_x = base_point.point.x
                    final_y = base_point.point.y
                    final_z = base_point.point.z
                except Exception as e:
                    rospy.logwarn(f"坐标转换失败: {e}")
                    final_x, final_y, final_z = X, Y, Z

                # 发布坐标
                tar_msg = target()
                tar_msg.x = int(final_x)
                tar_msg.y = int(final_y)
                tar_msg.dis = final_z
                self.target_pub.publish(tar_msg)
    
if __name__ == "__main__":
    tracker = TargetTracker()
    rospy.spin()