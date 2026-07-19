#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import rospy
import cv2
import numpy as np
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from image_geometry import PinholeCameraModel
import mediapipe as mp

class Hand3DPrinter:
    def __init__(self):
        rospy.init_node('hand_3d_printer', anonymous=True)

        # --- ROS / CV 準備 ---
        self.bridge = CvBridge()
        self.camera_model = PinholeCameraModel()
        self.cam_info_received = False

        # Mediapipe Hands モジュール
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )

        # トピック同期
        info_sub  = message_filters.Subscriber('/camera/color/camera_info', CameraInfo, buff_size=2**24)
        rgb_sub   = message_filters.Subscriber('/camera/color/image_rect_color', Image, buff_size=2**24)
        depth_sub = message_filters.Subscriber('/camera/depth/image_rect_raw', Image, buff_size=2**24)
        ts = message_filters.ApproximateTimeSynchronizer(
            [info_sub, rgb_sub, depth_sub],
            queue_size=10,
            slop=0.1
        )
        ts.registerCallback(self.callback)

        rospy.loginfo("hand_3d_printer node started.")
    
    def callback(self, cam_info, rgb_msg, depth_msg):
        start_time = time.time()
        # カメラ情報を一度だけ読み込み
        if not self.cam_info_received:
            self.camera_model.fromCameraInfo(cam_info)
            self.cam_info_received = True

        # 画像変換
        try:
            color_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge error: {e}")
            return

        h, w = cam_info.height, cam_info.width

        # Mediapipe で手検出
        rgb_for_mp = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb_for_mp)
        if not results.multi_hand_landmarks:
            return

        hand_points = []  # List[List[(x,y,z)]]

        for hand_id, hand_lms in enumerate(results.multi_hand_landmarks):
            points3d = []
            for idx, lm in enumerate(hand_lms.landmark):
                u = int(lm.x * w); v = int(lm.y * h)
                u, v = np.clip(u, 0, w-1), np.clip(v, 0, h-1)
                depth = float(depth_image[v, u])
                if np.isnan(depth) or depth <= 0: 
                    points3d.append(None)
                    continue
                x = (u - self.camera_model.cx()) * depth / self.camera_model.fx()
                y = (v - self.camera_model.cy()) * depth / self.camera_model.fy()
                z = depth
                points3d.append((x/1000.0, y/1000., z/1000.))
            hand_points.append(points3d)

        for hand_id, pts in enumerate(hand_points):
            # 1) 手のひら中心を計算
            palm_idxs = [0, 5, 9, 13, 17]  # Wrist, 各MCP
            valid = [pts[i] for i in palm_idxs if pts[i] is not None]
            if not valid:
                rospy.logwarn(f"Hand{hand_id}: palm center 求まらず")
                continue
            pc = np.mean(valid, axis=0)  # (x,y,z) の平均
            print(f"=== Hand {hand_id} ===")
            print(f"Palm center: x={pc[0]:.3f}, y={pc[1]:.3f}, z={pc[2]:.3f}")

            # 2) 各指先との距離を計算
            tip_idxs = [4, 8, 12, 16, 20]
            dists = []
            for ti in tip_idxs:
                if pts[ti] is None: 
                    continue
                d = np.linalg.norm(np.array(pts[ti]) - pc)
                dists.append(d)
            if dists:
                avg_dist = float(np.mean(dists))
                state = "OPEN" if avg_dist > 0.06 else "CLOSED"
                print(f"  Finger-tip 平均距離 = {avg_dist:.3f} m → {state}")
            else:
                print("  Finger-tip の距離が取得できません")

            # （もし全ランドマークを見たいならこちらも）
            # for idx, p in enumerate(pts):
            #     if p is None: continue
            #     print(f"  Point{idx:2d}: x={p[0]:.3f}, y={p[1]:.3f}, z={p[2]:.3f}")

            print("-------------------")

    def run(self):
        rospy.spin()
        self.hands.close()


if __name__ == '__main__':
    node = Hand3DPrinter()
    node.run()
