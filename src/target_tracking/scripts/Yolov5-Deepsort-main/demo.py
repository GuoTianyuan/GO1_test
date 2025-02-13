#coding=utf-8 
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import sys
sys.path.insert(0, "/home/pt215/GO1_test_ws/src/target_tracking/scripts/Yolov5-Deepsort-main")
from AIDetector_pytorch import Detector
from target_tracking.msg import target
import pyrealsense2 as rs
import numpy as np
import imutils
import cv2



if __name__ == '__main__':
    rospy.init_node("taget_tracking")
    name = 'demo'
    det = Detector()
    pub = rospy.Publisher("dis_msg",target,queue_size=10)
    # 配置并启动D435相机
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    # 获取相机的帧率
    fps = 30  # D435相机设置的帧率
    print('fps:', fps)
    t = int(1000 / fps)

    videoWriter = None
    rate = rospy.Rate(30)
    try:
        while not rospy.is_shutdown():
            # 等待获取新的帧
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame:
                continue
            # 获取相机内参
            depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            # 将彩色帧转换为numpy数组
            im = np.asanyarray(depth_frame.get_data())
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            # Stack both images horizontally
            images = np.hstack((color_image, depth_colormap))
            result = det.feedCap(images)
            result1 = result['frame']
            result1 = imutils.resize(result1, height=500)
            print('result已获取')
            result2 = result['face_bboxes']
            print(f"检测框坐标：{result2}")
            if result2:  # 检查列表是否为空
                bbox = result2[0]
                x1, y1, x2, y2 = bbox
                print(f"检测框坐标：左上角({x1}, {y1}), 右下角({x2}, {y2})")
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)
                print('坐标已获取')
                align_to = rs.stream.color
                align = rs.align(align_to)
                aligned_frames = align.process(frames)
                aligned_depth_frame = aligned_frames.get_depth_frame()
                if 0 <= center_x < depth_frame.width and 0 <= center_y < depth_frame.height:
                    depth_value = depth_frame.get_distance(center_x, center_y)
                    depth_value = aligned_depth_frame.get_distance(center_x, center_y)
                    print('深度',depth_value)
                    tar = target()
                    tar.x = center_x
                    tar.y = center_y
                    tar.dis = depth_value
                    pub.publish(tar)
                    rate.sleep()
                else:
                    print("坐标超出范围")
            if videoWriter is None:
                fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')  # opencv3.0
                videoWriter = cv2.VideoWriter('result.mp4', fourcc, fps, (result1.shape[1], result1.shape[0]))
            # print('深度：',dis)
            videoWriter.write(result1)
            cv2.imshow(name, result1)
            if cv2.waitKey(t) & 0xFF == ord('q'):
                break

            if cv2.getWindowProperty(name, cv2.WND_PROP_AUTOSIZE) < 1:
                # exict by point "x"
                break
            
    except Exception as e:
        print(e)

    finally:
        # 停止管道，释放相机资源
        pipeline.stop()
        videoWriter.release()
        cv2.destroyAllWindows()
