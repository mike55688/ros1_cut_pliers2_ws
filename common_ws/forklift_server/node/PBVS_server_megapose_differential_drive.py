#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import actionlib
import forklift_server.msg
import tf
from gpm_msg.msg import forklift
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
import math
from forklift_msg.msg import meteorcar
from visp_megapose.msg import Confidence
from custom_msgs.msg import CmdCutPliers  # 引入訊息格式

import sys
import os
script_dir = os.path.dirname( __file__ )
mymodule_dir = os.path.join( script_dir, '..', 'scripts' )
sys.path.append( mymodule_dir )
from PBVS_action_sequence_differential_drive import PBVS

from dataclasses import dataclass
@dataclass
class DetectionConfidence:
    pallet_confidence: float
    pallet_detection: bool
    shelf_confidence: float
    shelf_detection: bool

class Subscriber():
    def __init__(self):
        self.get_parameters()
        self.init_parame()
        self.create_subscriber_publisher()
        self.fnDetectionAllowed(False, False, 0.0)

    def get_parameters(self):
        # Subscriber Topic setting
        self.odom_topic = rospy.get_param(rospy.get_name() + "/odom", "/odom")
        self.shelf_topic = rospy.get_param(rospy.get_name() + "/shelf_topic", "/shelf")
        self.pallet_topic = rospy.get_param(rospy.get_name() + "/pallet_topic", "/pallet")
        self.object_filter = rospy.get_param(rospy.get_name() + "/object_filter", True)
        self.forkpos = rospy.get_param(rospy.get_name() + "/forkpos", "/forkpos")
        self.confidence_minimum = rospy.get_param(rospy.get_name() + "/confidence_minimum", 0.5)

        rospy.loginfo("Get subscriber topic parameter")
        rospy.loginfo("odom_topic: {}, type: {}".format(self.odom_topic, type(self.odom_topic)))
        rospy.loginfo("shelf_topic: {}, type: {}".format(self.shelf_topic, type(self.shelf_topic)))
        rospy.loginfo("pallet_topic: {}, type: {}".format(self.pallet_topic, type(self.pallet_topic)))
        rospy.loginfo("object_filter: {}, type: {}".format(self.object_filter, type(self.object_filter)))
        rospy.loginfo("forkpos: {}, type: {}".format(self.forkpos, type(self.forkpos)))
        rospy.loginfo("confidence_minimum: {}, type: {}".format(self.confidence_minimum, type(self.confidence_minimum)))

        # bodycamera parking setting
        self.bodycamera_tag_offset_x = rospy.get_param(rospy.get_name() + "/bodycamera_tag_offset_x", 0.0)
        self.bodycamera_parking_fork_init = rospy.get_param(rospy.get_name() + "/bodycamera_parking_fork_init", 0.0)
        self.bodycamera_ChangingDirection_threshold = rospy.get_param(rospy.get_name() + "/bodycamera_ChangingDirection_threshold", 0.0)
        self.bodycamera_desired_dist_threshold = rospy.get_param(rospy.get_name() + "/bodycamera_desired_dist_threshold", 0.0)
        
        self.bodycamera_parking_stop = rospy.get_param(rospy.get_name() + "/bodycamera_parking_stop", 0.0)
        self.bodycamera_Changingtheta_threshold = rospy.get_param(rospy.get_name() + "/bodycamera_Changingtheta_threshold", 0.0)
        self.bodycamera_decide_distance = rospy.get_param(rospy.get_name() + "/bodycamera_decide_distance", 0.0)
        self.bodycamera_back_distance = rospy.get_param(rospy.get_name() + "/bodycamera_back_distance", 0.0)

        rospy.loginfo("Get bodycamera parking parameter")
        rospy.loginfo("bodycamera_tag_offset_x: {}, type: {}".format(self.bodycamera_tag_offset_x, type(self.bodycamera_tag_offset_x)))
        rospy.loginfo("bodycamera_parking_fork_init: {}, type: {}".format(self.bodycamera_parking_fork_init, type(self.bodycamera_parking_fork_init)))
        rospy.loginfo("bodycamera_ChangingDirection_threshold: {}, type: {}".format(self.bodycamera_ChangingDirection_threshold, type(self.bodycamera_ChangingDirection_threshold)))
        rospy.loginfo("bodycamera_desired_dist_threshold: {}, type: {}".format(self.bodycamera_desired_dist_threshold, type(self.bodycamera_desired_dist_threshold)))        
        rospy.loginfo("bodycamera_parking_stop: {}, type: {}".format(self.bodycamera_parking_stop, type(self.bodycamera_parking_stop)))
        rospy.loginfo("bodycamera_Changingtheta_threshold: {}, type: {}".format(self.bodycamera_Changingtheta_threshold, type(self.bodycamera_Changingtheta_threshold)))
        rospy.loginfo("bodycamera_decide_distance: {}, type: {}".format(self.bodycamera_decide_distance, type(self.bodycamera_decide_distance)))
        rospy.loginfo("bodycamera_back_distance: {}, type: {}".format(self.bodycamera_back_distance, type(self.bodycamera_back_distance)))

        # forkcamera parking setting
        self.forkcamera_parking_fork_layer1 = rospy.get_param(rospy.get_name() + "/forkcamera_parking_fork_layer1", 0.0)
        self.forkcamera_parking_fork_layer2 = rospy.get_param(rospy.get_name() + "/forkcamera_parking_fork_layer2", 0.0)
        self.forkcamera_tag_offset_x = rospy.get_param(rospy.get_name() + "/forkcamera_tag_offset_x", 0.0)
        self.forkcamera_ChangingDirection_threshold = rospy.get_param(rospy.get_name() + "/forkcamera_ChangingDirection_threshold", 0.0)
        self.forkcamera_parking_stop = rospy.get_param(rospy.get_name() + "/forkcamera_parking_stop", 0.0)
        self.forkcamera_Changingtheta_threshold = rospy.get_param(rospy.get_name() + "/forkcamera_Changingtheta_threshold", 0.0)
        self.forkcamera_decide_distance = rospy.get_param(rospy.get_name() + "/forkcamera_decide_distance", 0.0)
        self.forkcamera_back_distance = rospy.get_param(rospy.get_name() + "/forkcamera_back_distance", 0.0)

        rospy.loginfo("Get forkcamera parking parameter")
        rospy.loginfo("forkcamera_parking_fork_layer1: {}, type: {}".format(self.forkcamera_parking_fork_layer1, type(self.forkcamera_parking_fork_layer1)))
        rospy.loginfo("forkcamera_parking_fork_layer2: {}, type: {}".format(self.forkcamera_parking_fork_layer2, type(self.forkcamera_parking_fork_layer2)))
        rospy.loginfo("forkcamera_tag_offset_x: {}, type: {}".format(self.forkcamera_tag_offset_x, type(self.forkcamera_tag_offset_x)))
        rospy.loginfo("forkcamera_ChangingDirection_threshold: {}, type: {}".format(self.forkcamera_ChangingDirection_threshold, type(self.forkcamera_ChangingDirection_threshold)))
        rospy.loginfo("forkcamera_parking_stop: {}, type: {}".format(self.forkcamera_parking_stop, type(self.forkcamera_parking_stop)))
        rospy.loginfo("forkcamera_Changingtheta_threshold: {}, type: {}".format(self.forkcamera_Changingtheta_threshold, type(self.forkcamera_Changingtheta_threshold)))
        rospy.loginfo("forkcamera_decide_distance: {}, type: {}".format(self.forkcamera_decide_distance, type(self.forkcamera_decide_distance)))
        rospy.loginfo("forkcamera_back_distance: {}, type: {}".format(self.forkcamera_back_distance, type(self.forkcamera_back_distance)))


    def init_parame(self):
        # Odometry_param
        self.is_odom_received = False
        self.robot_2d_pose_x = 0.0
        self.robot_2d_pose_y = 0.0
        self.robot_2d_theta = 0.0
        self.previous_robot_2d_theta = 0.0
        self.total_robot_2d_theta = 0.0
        # AprilTag_param
        self.shelf_or_pallet = True
        self.offset_x = 0.0
        self.marker_2d_pose_x = 0.0
        self.marker_2d_pose_y = 0.0
        self.marker_2d_pose_z = 0.0

        self.marker_2d_theta = 0.0
        # Forklift_param
        self.updownposition = 0.0

        # 新增手臂相關參數
        self.current_arm_status = None  # 儲存當前手臂狀態
        self.last_command = {"height2": -1, "length2": -1, "claw2": False, "mode": -1}  # 上次命令記錄

        # confidence_param
        self.sub_detectionConfidence = DetectionConfidence(
            pallet_confidence = 0.0,
            pallet_detection = False,
            shelf_confidence = 0.0,
            shelf_detection = False
        )
    
    def create_subscriber_publisher(self):
        if(self.object_filter):
            pallet = self.pallet_topic + "_filter"
            shelf = self.shelf_topic + "_filter"
        else:
            pallet = self.pallet_topic
            shelf = self.shelf_topic
        
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.cbGetOdom, queue_size = 1)
        self.forkpose_sub = rospy.Subscriber(self.forkpos, meteorcar, self.cbGetforkpos, queue_size = 1)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 1, latch=True)
        self.pub_fork = rospy.Publisher('/cmd_fork', meteorcar, queue_size = 1, latch=True)
        self.pallet_sub = rospy.Subscriber(pallet, Pose, self.cbGetPallet, queue_size = 1)
        self.shelf_sub = rospy.Subscriber(shelf, Pose, self.cbGetShelf, queue_size = 1)
        self.pallet_confidence_sub = rospy.Subscriber(self.pallet_topic + "_confidence", Confidence, self.cbGetPalletConfidence, queue_size = 1)
        self.shelf_confidence_sub = rospy.Subscriber(self.shelf_topic + "_confidence", Confidence, self.cbGetShelfConfidence, queue_size = 1)
        
        self.pallet_detection_pub = rospy.Publisher(self.pallet_topic + "_detection", forklift_server.msg.Detection, queue_size = 1, latch=True)
        self.shelf_detection_pub = rospy.Publisher(self.shelf_topic + "_detection", forklift_server.msg.Detection, queue_size = 1, latch=True)
        # 新增手臂相關訂閱和發布
        self.arm_status_sub = rospy.Subscriber("/arm_current_status", CmdCutPliers, self.arm_status_callback, queue_size=1)
        self.cut_pliers_sub = rospy.Subscriber("/cmd_cut_pliers", CmdCutPliers, self.cmd_cut_pliers_callback, queue_size=1)
        self.arm_control_pub = rospy.Publisher("/cmd_cut_pliers", CmdCutPliers, queue_size=1, latch=True)
    
    
    def fnDetectionAllowed(self, shelf_detection, pallet_detection, layer):
        shelf_msg = forklift_server.msg.Detection()
        shelf_msg.detection_allowed = shelf_detection
        shelf_msg.layer = layer
        self.shelf_detection_pub.publish(shelf_msg)
        
        pallet_msg = forklift_server.msg.Detection()
        pallet_msg.detection_allowed = pallet_detection
        pallet_msg.layer = layer
        self.pallet_detection_pub.publish(pallet_msg)
        rospy.sleep(0.2)
        # rospy.loginfo("shelf_msg = {}, pallet_msg = {}".format(shelf_msg, pallet_msg))

    def __del__(self):
        self.window.destroy()

    def SpinOnce(self):
        return self.robot_2d_pose_x, self.robot_2d_pose_y, self.robot_2d_theta, \
               self.marker_2d_pose_x, self.marker_2d_pose_y, self.marker_2d_pose_z, self.marker_2d_theta
    def SpinOnce_fork(self):
        return self.updownposition
    
    def SpinOnce_confidence(self):
        return self.sub_detectionConfidence

    def cbGetPallet(self, msg):
        try:
            if self.shelf_or_pallet == True:
                marker_msg = msg
                quaternion = (marker_msg.orientation.x, marker_msg.orientation.y, marker_msg.orientation.z, marker_msg.orientation.w)
                theta = tf.transformations.euler_from_quaternion(quaternion)[1]
                self.marker_2d_pose_x = -marker_msg.position.z
                self.marker_2d_pose_y = marker_msg.position.x + self.offset_x
                self.marker_2d_pose_z = marker_msg.position.y  # 更新z轴信息

                self.marker_2d_theta = -theta
                # rospy.loginfo("Pose: x={:.3f}, y={:.3f}, theta={:.3f}".format(self.marker_2d_pose_x, self.marker_2d_pose_y, self.marker_2d_theta))
            else:
                pass
        except:
            pass

    def cbGetShelf(self, msg):
        try:
            if self.shelf_or_pallet == False:
                marker_msg = msg
                quaternion = (marker_msg.orientation.x, marker_msg.orientation.y, marker_msg.orientation.z, marker_msg.orientation.w)
                theta = tf.transformations.euler_from_quaternion(quaternion)[1]
                self.marker_2d_pose_x = -marker_msg.position.z
                self.marker_2d_pose_y = marker_msg.position.x + self.offset_x
                self.marker_2d_pose_z = marker_msg.position.y  # 更新z轴信息1

                self.marker_2d_theta = -theta
                # rospy.loginfo("Pose: x={:.3f}, y={:.3f}, theta={:.3f}".format(self.marker_2d_pose_x, self.marker_2d_pose_y, self.marker_2d_theta))
            else:
                pass
        except:
            pass



    def cbGetOdom(self, msg):
        if self.is_odom_received == False:
            self.is_odom_received = True 

        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        theta = tf.transformations.euler_from_quaternion(quaternion)[2]
        if theta < 0:
            theta = theta + math.pi * 2
        if theta > math.pi * 2:
            theta = theta - math.pi * 2

        self.robot_2d_pose_x = msg.pose.pose.position.x
        self.robot_2d_pose_y = msg.pose.pose.position.y
        self.robot_2d_theta = theta

        d_theta = self.robot_2d_theta - self.previous_robot_2d_theta
        if d_theta > math.pi:
            d_theta -= 2 * math.pi
        elif d_theta < -math.pi:
            d_theta += 2 * math.pi

        self.total_robot_2d_theta += d_theta
        self.previous_robot_2d_theta = self.robot_2d_theta

        self.robot_2d_theta = self.total_robot_2d_theta

    def cbGetforkpos(self, msg):
        self.updownposition = msg.fork_position

    def cbGetShelfConfidence(self, msg):
        self.sub_detectionConfidence.shelf_confidence = msg.object_confidence
        self.sub_detectionConfidence.shelf_detection = msg.model_detection

    def cbGetPalletConfidence(self, msg):
        self.sub_detectionConfidence.pallet_confidence = msg.object_confidence
        self.sub_detectionConfidence.pallet_detection = msg.model_detection


    def arm_status_callback(self, msg):
        self.current_arm_status = msg
        # rospy.loginfo(f"更新手臂狀態: height1={msg.height1}, length1={msg.length1}, claw1={msg.claw1}")

    def cmd_cut_pliers_callback(self, msg):
        mode = msg.mode if hasattr(msg, "mode") else 0

        if self.current_arm_status is None:
            rospy.logwarn("⚠ current_arm_status 尚未初始化，忽略此指令")
            return

        if mode == 1:
            if msg.length2 >= self.current_arm_status.length2:
                rospy.logwarn(f"⚠ 後退模式啟動，但目標長度 {msg.length2} 不小於當前手臂2長度 {self.current_arm_status.length2}，忽略請求")
                return

        if mode == 0 and msg.length2 < self.last_command["length2"]:
            rospy.logwarn(f"⚠ 發現異常：目標長度 {msg.length2} 比上一個命令 {self.last_command['length2']} 更短，但仍為前伸模式，忽略請求")
            return

        if (msg.height2 == self.last_command["height2"] and
            msg.length2 == self.last_command["length2"] and
            msg.claw2 == self.last_command["claw2"] and
            mode == self.last_command["mode"]):
            rospy.loginfo("✅ 手臂2指令未變化，避免重複發布")
            return

        arm_cmd = CmdCutPliers()
        arm_cmd.height2 = msg.height2
        arm_cmd.length2 = msg.length2
        arm_cmd.claw2 = msg.claw2
        arm_cmd.enable_motor3 = True  # 啟用手臂2高度電機
        arm_cmd.enable_motor4 = True  # 啟用手臂2長度電機
        arm_cmd.mode = mode

        self.last_command = {
            "height2": msg.height2,
            "length2": msg.length2,
            "claw2": msg.claw2,
            "mode": mode
        }

        # 發布到 /cmd_cut_pliers
        self.arm_control_pub.publish(arm_cmd)
        rospy.loginfo(f"✅ 發送手臂2控制指令: height2={arm_cmd.height2}, length2={arm_cmd.length2}, claw2={arm_cmd.claw2}, mode={arm_cmd.mode}")

class PBVSAction():
    def __init__(self, name):
        self.subscriber = Subscriber()
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, forklift_server.msg.PBVSMegaposeAction, execute_cb=self.execute_callback, auto_start = False)
        self._result = forklift_server.msg.PBVSResult()
        self._as.start()

    def execute_callback(self, msg):
        # rospy.loginfo('Received goal: Command={}, layer_dist={}'.format(self.command, self.layer_dist))
        rospy.logwarn('PBVS receive command : %s' % (msg))
        self.PBVS = PBVS(self._as, self.subscriber, msg)

        if(msg.command == "fruit_docking"):
                self.subscriber.shelf_or_pallet = False  
                self.PBVS.fruit_docking()

        else:
            rospy.logwarn("Unknown command")
            self._result.result = 'fail'
            self._as.set_aborted(self._result)
            return
        
        rospy.logwarn('PBVS Succeeded')
        self._result.result = 'PBVS Succeeded'
        # self.shelf_or_pallet = False
        self._as.set_succeeded(self._result)
        self.PBVS = None


if __name__ == '__main__':
    rospy.init_node('PBVS_server')
    rospy.logwarn(rospy.get_name() + 'start')
    server = PBVSAction(rospy.get_name())
    rospy.spin()
    