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

import sys
import os
script_dir = os.path.dirname( __file__ )
mymodule_dir = os.path.join( script_dir, '..', 'scripts' )
sys.path.append( mymodule_dir )
from PBVS_megapose import PBVS

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

        # Raise the pallet setting
        self.raise_pallet_fork_init_layer1 = rospy.get_param(rospy.get_name() + "/raise_pallet_fork_init_layer1", 0.0)
        self.raise_pallet_fork_init_layer2 = rospy.get_param(rospy.get_name() + "/raise_pallet_fork_init_layer2", 0.0)
        self.raise_pallet_dead_reckoning_dist = rospy.get_param(rospy.get_name() + "/raise_pallet_dead_reckoning_dist", 0.0)
        self.raise_pallet_raise_height_layer1 = rospy.get_param(rospy.get_name() + "/raise_pallet_raise_height_layer1", 0.0)
        self.raise_pallet_raise_height_layer2 = rospy.get_param(rospy.get_name() + "/raise_pallet_raise_height_layer2", 0.0)
        self.raise_pallet_back_distance = rospy.get_param(rospy.get_name() + "/raise_pallet_back_distance", 0.0)

        rospy.loginfo("Get raise_pallet parameter")
        rospy.loginfo("raise_pallet_fork_init_layer1: {}, type: {}".format(self.raise_pallet_fork_init_layer1, type(self.raise_pallet_fork_init_layer1)))
        rospy.loginfo("raise_pallet_fork_init_layer2: {}, type: {}".format(self.raise_pallet_fork_init_layer2, type(self.raise_pallet_fork_init_layer2)))
        rospy.loginfo("raise_pallet_dead_reckoning_dist: {}, type: {}".format(self.raise_pallet_dead_reckoning_dist, type(self.raise_pallet_dead_reckoning_dist)))
        rospy.loginfo("raise_pallet_raise_height_layer1: {}, type: {}".format(self.raise_pallet_raise_height_layer1, type(self.raise_pallet_raise_height_layer1)))
        rospy.loginfo("raise_pallet_raise_height_layer2: {}, type: {}".format(self.raise_pallet_raise_height_layer2, type(self.raise_pallet_raise_height_layer2)))
        rospy.loginfo("raise_pallet_back_distance: {}, type: {}".format(self.raise_pallet_back_distance, type(self.raise_pallet_back_distance)))

        # Drop the pallet setting
        self.drop_pallet_fork_init_layer1 = rospy.get_param(rospy.get_name() + "/drop_pallet_fork_init_layer1", 0.0)
        self.drop_pallet_fork_init_layer2 = rospy.get_param(rospy.get_name() + "/drop_pallet_fork_init_layer2", 0.0)
        self.drop_pallet_dead_reckoning_dist = rospy.get_param(rospy.get_name() + "/drop_pallet_dead_reckoning_dist", 0.0)
        self.drop_pallet_drop_height_layer1 = rospy.get_param(rospy.get_name() + "/drop_pallet_drop_height_layer1", 0.0)
        self.drop_pallet_drop_height_layer2 = rospy.get_param(rospy.get_name() + "/drop_pallet_drop_height_layer2", 0.0)
        self.drop_pallet_back_distance = rospy.get_param(rospy.get_name() + "/drop_pallet_back_distance", 0.0)

        rospy.loginfo("Get drop_pallet parameter")
        rospy.loginfo("drop_pallet_fork_init_layer1: {}, type: {}".format(self.drop_pallet_fork_init_layer1, type(self.drop_pallet_fork_init_layer1)))
        rospy.loginfo("drop_pallet_fork_init_layer2: {}, type: {}".format(self.drop_pallet_fork_init_layer2, type(self.drop_pallet_fork_init_layer2)))
        rospy.loginfo("drop_pallet_dead_reckoning_dist: {}, type: {}".format(self.drop_pallet_dead_reckoning_dist, type(self.drop_pallet_dead_reckoning_dist)))
        rospy.loginfo("drop_pallet_drop_height_layer1: {}, type: {}".format(self.drop_pallet_drop_height_layer1, type(self.drop_pallet_drop_height_layer1)))
        rospy.loginfo("drop_pallet_drop_height_layer2: {}, type: {}".format(self.drop_pallet_drop_height_layer2, type(self.drop_pallet_drop_height_layer2)))
        rospy.loginfo("drop_pallet_back_distance: {}, type: {}".format(self.drop_pallet_back_distance, type(self.drop_pallet_back_distance)))

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
        self.marker_2d_theta = 0.0
        # Forklift_param
        self.updownposition = 0.0
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
               self.marker_2d_pose_x, self.marker_2d_pose_y, self.marker_2d_theta
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

        if(msg.command == "parking_bodycamera"):
            self.subscriber.shelf_or_pallet = False  # True: pallet, False: shelf
            self.PBVS.parking_bodycamera()
        elif(msg.command == "parking_forkcamera"):
            self.subscriber.shelf_or_pallet = True  # True: pallet, False: shelf
            self.PBVS.parking_forkcamera()
        elif(msg.command == "raise_pallet"):
            self.subscriber.shelf_or_pallet = False
            self.PBVS.raise_pallet()
        elif(msg.command == "drop_pallet"):
            self.subscriber.shelf_or_pallet = False
            self.PBVS.drop_pallet()
        elif(msg.command == "odom_front"):
            self.subscriber.shelf_or_pallet = False
            self.PBVS.odom_front()
        elif(msg.command == "odom_turn"):
            self.subscriber.shelf_or_pallet = False
            self.PBVS.odom_turn()
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
    