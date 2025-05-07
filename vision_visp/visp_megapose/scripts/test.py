#!/usr/bin/env python3
import rospy
import csv
import os
from sensor_msgs.msg import Image, CameraInfo

class SyncChecker:
    def __init__(self):
        rospy.init_node('sync_checker', anonymous=True)

        # 訂閱影像、相機資訊、深度影像、深度相機資訊
        rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.camera_info_callback)
        rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_callback)
        rospy.Subscriber("/camera/aligned_depth_to_color/camera_info", CameraInfo, self.depth_info_callback)

        self.image_time = None
        self.camera_info_time = None
        self.depth_time = None
        self.depth_info_time = None

        # CSV 檔案路徑
        self.csv_file = os.path.join(os.path.expanduser("~"), "sync_timestamps_fps15.csv")
        self.init_csv()
        
        rospy.loginfo("Sync Checker Node Started!")
    
    def init_csv(self):
        # 檢查 CSV 是否存在，若不存在則建立標題列
        if not os.path.exists(self.csv_file):
            with open(self.csv_file, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["Image_Timestamp", "CameraInfo_Timestamp", "Depth_Timestamp", "DepthInfo_Timestamp"])

    def image_callback(self, image_msg):
        self.image_time = image_msg.header.stamp.to_sec()
        self.check_sync()

    def camera_info_callback(self, camera_info_msg):
        self.camera_info_time = camera_info_msg.header.stamp.to_sec()
        self.check_sync()

    def depth_callback(self, depth_msg):
        self.depth_time = depth_msg.header.stamp.to_sec()
        self.check_sync()

    def depth_info_callback(self, depth_info_msg):
        self.depth_info_time = depth_info_msg.header.stamp.to_sec()
        self.check_sync()

    def check_sync(self):
        if None not in (self.image_time, self.camera_info_time, self.depth_time, self.depth_info_time):
            # 儲存時間戳到 CSV
            with open(self.csv_file, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([self.image_time, self.camera_info_time, self.depth_time, self.depth_info_time])
            rospy.loginfo(f"Saved timestamps: {self.image_time}, {self.camera_info_time}, {self.depth_time}, {self.depth_info_time}")

if __name__ == '__main__':
    SyncChecker()
    rospy.spin()
