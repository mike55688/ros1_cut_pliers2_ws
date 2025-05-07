#!/usr/bin/env python3
import rospy
import csv
from geometry_msgs.msg import Pose
from datetime import datetime

class PoseFrequencyLoggerROS1:
    def __init__(self, topic, output_csv):
        self.topic = topic
        self.output_csv = output_csv
        self.last_time = None
        self.hz = 0
        self.start_logging()

    def callback(self, msg):
        current_time = rospy.Time.now().to_sec()
        if self.last_time is not None:
            self.hz = 1.0 / (current_time - self.last_time)
            self.log_to_csv(self.hz)
        self.last_time = current_time

    def log_to_csv(self, hz):
        with open(self.output_csv, 'a') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([datetime.now().isoformat(), hz])

    def start_logging(self):
        with open(self.output_csv, 'w') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["Timestamp", "Hz"])
        rospy.Subscriber(self.topic, Pose, self.callback)
        rospy.loginfo(f"Started logging {self.topic} frequency to {self.output_csv}")


if __name__ == "__main__":
    rospy.init_node("pose_hz_logger")
    topic = rospy.get_param("~topic", "/pallet")
    output_csv = rospy.get_param("~output_csv", "ros1_pose_hz.csv")
    logger = PoseFrequencyLoggerROS1(topic, output_csv)
    rospy.spin()
