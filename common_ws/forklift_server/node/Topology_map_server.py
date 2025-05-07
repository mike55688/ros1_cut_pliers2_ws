#! /usr/bin/env python3
# -*- coding: utf-8 -*-
from time import time
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import forklift_server.msg
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose
from visualization_msgs.msg import Marker
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
import heapq
import math
from dynamic_reconfigure import client
import tf
from tf.transformations import euler_from_quaternion



class TopologyMap():
    def __init__(self, start_node):
        # self.start_node  = input("輸入起始點: ")     
        self.start_node  = start_node

    def path(self, goal):
        print("Path from {} to {}:".format(self.start_node, goal))
        self.parent, self.distance=self.dijkstra(graph,self.start_node)
        path=self.distance_path(graph,self.start_node,goal)
        self.start_node = goal
        return path
   

    # def find_point(self, goal):
    #     x, y, z, w = goal.goal.pose.position.x, goal.goal.pose.position.y, goal.goal.pose.orientation.z, goal.goal.pose.orientation.w
    #     for i in waypoints:
    #         if x == waypoints[i][0] and y == waypoints[i][1] and z == waypoints[i][2] and w == waypoints[i][3]:
    #             return i

    # 初始化起点的距离  到自身为零 到其他节点为无穷大
    def init_distance(self, graph,s): #传入图像 和起点
        self.distance={s:0}
        for vertex in graph:
            if vertex !=s:
                self.distance[vertex]=math.inf  #除到本身都为无穷大
        return self.distance
    def dijkstra(self,graph,s):
        pqueue=[]     #创建一个队列
        # 先添加一个起点到队列 和后面加入的排序  
        # 此方法把 队列里面的元素按照优先排列 调用heapop时返回优先级最高的   比如数值最小的
        heapq.heappush(pqueue,(0,s))  
        seen=set() #储存出现过的点
        self.parent={s:None}   #标记此节点的上一个节点  此节点为起点 则父节点为None  
        self.distance=self.init_distance(graph,s)

        while (len(pqueue)>0):
            pair=heapq.heappop(pqueue)  #返回一个数值最小的元组 
            dist=pair[0]  #提取距离
            vertex=pair[1] #提取节点
            seen.add(vertex) #添加出现过的节点
            nodes=graph[vertex].keys()   #提取与vertex相连的节点
            # print(nodes)
            #核心算法
            for w in nodes:
                if w not in seen:
                    if dist+graph[vertex][w]<self.distance[w]:
                        #把路径短的添加到队列 并排序
                        heapq.heappush(pqueue,(dist+graph[vertex][w],w))
                        self.parent[w]=vertex  #记录父节点
                        self.distance[w]=dist+graph[vertex][w] #更新起点到w节点的距离
        return self.parent,self.distance
    def distance_path(self,graph,s,end):
        self.parent, self.distance = self.dijkstra(graph, s)
        path=[end]  
        while self.parent[end] !=None:
            path.append(self.parent[end])
            end=self.parent[end]
        path.reverse()  
        return path

class SubscriberPose():        # 監聽車子在地圖中的位置
    def __init__(self):
        rospy.Subscriber("/rtabmap/localization_pose", PoseWithCovarianceStamped, self.Current_pose, queue_size = 1)

    def Current_pose(self, msg):
        self.robot_pose_x = msg.pose.pose.position.x
        self.robot_pose_y = msg.pose.pose.position.y

    def spin_once(self):
        return self.robot_pose_x, self.robot_pose_y


class Navigation():
    def __init__(self):
        odom = rospy.get_param(rospy.get_name() + "/odom", "/odom")
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.client.wait_for_server()
        self.sub_odom_robot = rospy.Subscriber(odom, Odometry, self.cbGetRobotOdom, queue_size = 1)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        self.init_param()

    def get_map_to_base_xyzw(self):
        listener = tf.TransformListener()
        while not rospy.is_shutdown():
            try:
                listener.waitForTransform('map', 'base_link', rospy.Time(0), rospy.Duration(4.0))
                (trans, rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0))
                # rospy.loginfo('Transform: %s, %s' % (trans[0], trans[1]))
                return trans[0], trans[1], rot[2], rot[3]
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logerr(e)
                return 0.0, 0.0, 0.0, 0.0
                        
    def move(self, mode, x, y, z, w):    
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.z = z
        goal.target_pose.pose.orientation.w = w

        self.client.send_goal(goal)
        print("Navigation to", goal)

        if 'pass' in mode :
            pass_value = 1.1
            while(1):
                # current_position_x, current_position_y = sub_pose.spin_once()
                current_position_x, current_position_y = self.get_map_to_base_xyzw()[0], self.get_map_to_base_xyzw()[1]
                distance_target = math.sqrt(((x - current_position_x) ** 2) + ((y - current_position_y) ** 2))      # 計算從目前位置到目標點的直線距離
                # rospy.loginfo("distance_target = %s" % (distance_target))
                if distance_target < pass_value :
                    break
                
            return self.client.get_result()
        
        else :
            wait = self.client.wait_for_result()

        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()
    
    def init_param(self):    
        self.trigger = True
        self.pre_odom = 0.0
        self.odom_pass = 0.0
    
    def cbGetRobotOdom(self, msg):

        # orientation_q = msg.pose.pose.orientation
        # orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        # (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        # self.current_orientation = yaw

        self.rz, self.rw = msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
        yaw_r = math.atan2(2 * self.rw * self.rz, self.rw * self.rw - self.rz * self.rz)
        if(yaw_r < 0):
            yaw_r = yaw_r + 2 * math.pi

        if(self.trigger == False):
            self.pre_odom = yaw_r
            self.odom_pass = 0.0
            self.trigger = True
        if(abs(yaw_r - self.pre_odom) > 1):
            self.odom_pass = self.odom_pass
        else:
            self.odom_pass = self.odom_pass + yaw_r - self.pre_odom
        self.pre_odom = yaw_r

    def self_spin(self, z2, w2):
        # rospy.INFO('self_spin')
        self.trigger = False       
        self.odom_pass = 0.0
        rospy.sleep(0.1)

        yaw_1 = math.atan2(2 * self.rw * self.rz, self.rw * self.rw - self.rz * self.rz)
        if(yaw_1 < 0):
            yaw_1 = yaw_1 + 2 * math.pi
        # print('yaw_1 = ', yaw_1)
        yaw_2 = math.atan2(2 * w2 * z2, w2 * w2 - z2 * z2)
        if(yaw_2 < 0):
            yaw_2 = yaw_2 + 2 * math.pi
        # print('yaw_2 = ', yaw_2)

        desire_angle = yaw_2 - yaw_1
        if(desire_angle > math.pi):
            desire_angle = desire_angle - 2 * math.pi
        elif(desire_angle < -math.pi):
            desire_angle = desire_angle + 2 * math.pi
        # print('desire_angle = ', desire_angle)

        speed = Twist()
        while(abs(self.odom_pass) < abs(desire_angle)):
            # print("odom_pass", self.odom_pass*180/math.pi)
            if(desire_angle >= 0):
                speed.angular.z = (desire_angle-self.odom_pass)*0.4
            elif(desire_angle <= 0):
                speed.angular.z = (desire_angle-self.odom_pass)*0.4

            if speed.angular.z > 0.2:
                speed.angular.z = 0.2
            elif speed.angular.z < -0.2:
                speed.angular.z = -0.2
            elif speed.angular.z > -0.05 and speed.angular.z < 0:
                speed.angular.z = -0.05
            elif speed.angular.z < 0.05 and speed.angular.z > 0:
                speed.angular.z = 0.05
            self.cmd_pub.publish(speed)
            rospy.sleep(0.01)
        

        self.cmd_pub.publish(Twist())
        self.trigger = True

class TopologyMapAction():
    _result = forklift_server.msg.TopologyMapResult()
    _feedback = forklift_server.msg.TopologyMapFeedback()

    def __init__(self, name):
        self._action_name = name        
        self.init_param()
        self.TopologyMap = TopologyMap(self.start_node)      
        self.Navigation = Navigation()
        self._as = actionlib.SimpleActionServer(self._action_name, forklift_server.msg.TopologyMapAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def init_param(self):
        global waypoints
        global graph
        self.start_node = rospy.get_param(rospy.get_name() + "/start_node", "LD3")
        waypoints = rospy.get_param(rospy.get_name() + "/waypoints")
        graph = rospy.get_param(rospy.get_name() + "/graph")
        self.last_target_pose = None
        

    def execute_cb(self, msg):
        rospy.loginfo('TopologyMap receive command : %s' % (msg))
        if msg.goal != "" or (msg.target_name != "" and msg.target_pose == None):
            if msg.goal != "":
                path = self.TopologyMap.path(msg.goal)
                print(path)
            elif msg.target_name != "":
                path = [msg.target_name]

            for i in range(len(path)):
                rospy.sleep(0.6)
                # new test for dynamic reconfigure

                if "back" in path[i] :
                    dynamic_client.update_configuration({"weight_kinematics_forward_drive":0.01})
                    dynamic_client.update_configuration({"weight_max_vel_theta":150}) 
                    dynamic_client.update_configuration({"weight_acc_lim_theta":1}) 
                    dynamic_client2.update_configuration({"orientation_mode":2})
                else : 
                    dynamic_client.update_configuration({"weight_kinematics_forward_drive":1000})
                    dynamic_client.update_configuration({"weight_max_vel_theta":200}) 
                    dynamic_client.update_configuration({"weight_acc_lim_theta":5}) 
                    dynamic_client2.update_configuration({"orientation_mode":1})


                if (i > 0 and (waypoints[path[i]][0] == waypoints[path[i-1]][0] and waypoints[path[i]][1] == waypoints[path[i-1]][1])):
                    rospy.loginfo('self_spin from %s to %s' %
                                  (path[i-1], path[i]))
                    self._feedback.feedback = str('self_spin from %s to %s' %(path[i-1], path[i]))
                    self._as.publish_feedback(self._feedback)
                    # rospy.loginfo('self_spin from %s to %s' % (path[i-1], path[i]))
                    self.Navigation.self_spin(
                        waypoints[path[i]][2], waypoints[path[i]][3])
                    i = i + 1
                    continue
                else:
                    rospy.loginfo('Navigation to %s' % path[i])
                    self._feedback.feedback = str('Navigation to %s' % path[i])
                    self._as.publish_feedback(self._feedback)
                    # if "back" in path[i]:
                    #     self.Navigation.move_back(waypoints[path[i]][0], waypoints[path[i]][1])
                    # else:
                    self.Navigation.move(path[i], waypoints[path[i]][0], waypoints[path[i]][1], waypoints[path[i]][2], waypoints[path[i]][3])
                

        elif msg.target_pose != None:

            posix = msg.target_pose.position.x
            posity = msg.target_pose.position.y
            orienz = msg.target_pose.orientation.z
            orienw = msg.target_pose.orientation.w

            if self.last_target_pose != None and self.last_target_pose.position.x == msg.target_pose.position.x and self.last_target_pose.position.y == msg.target_pose.position.y:
                self.Navigation.self_spin(orienz, orienw)
            else:
                self.Navigation.move(posix, posity, orienz, orienw)
            if self.last_target_pose==None:
                self.last_target_pose=Pose()
                
            self.last_target_pose.position.x = posix
            self.last_target_pose.position.y = posity
            self.last_target_pose.orientation.z = orienz
            self.last_target_pose.orientation.w = orienw
        
        rospy.logwarn('TopologyMap Succeeded')
        self._result.result = 'success'
        self._as.set_succeeded(self._result)


class MarkerViewer():
    def __init__(self):
        rospy.loginfo("Start View Marker")
        self.Marker_Pub = rospy.Publisher("/GoalMarker", Marker, queue_size = 100)
        self.TextMarker_Pub = rospy.Publisher("/GoalNameMarker", Marker, queue_size = 100)

        
    def PublishMarker(self, GoalName):
        marker = Marker()
        marker.action = Marker.ADD
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        marker.lifetime = rospy.Duration()
        marker.ns = GoalName
        marker.id = 0
        marker.type = Marker.ARROW
        
        marker.pose.position.x = waypoints[GoalName][0]
        marker.pose.position.y = waypoints[GoalName][1]
        marker.pose.position.z = 0.0
        
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = waypoints[GoalName][2]
        marker.pose.orientation.w = waypoints[GoalName][3]
        
        marker.scale.x = 1.0
        marker.scale.y = 0.2
        marker.scale.z = 0.1
        
        marker.color.r = 255 / 255
        marker.color.g = 255 / 255
        marker.color.b = 255 / 255
        marker.color.a = 1.0
        rospy.sleep(0.4)
        self.Marker_Pub.publish(marker)
        
        rospy.loginfo("PublishMarker %s",GoalName)
        
        # marker.type = Marker.TEXT_VIEW_FACING
        # marker.text = GoalName
        # marker.pose.position.x = waypoints[GoalName][0] - 0.3
        # marker.pose.position.y = waypoints[GoalName][1] - 0.3
        # marker.pose.position.z = 0.0
        # marker.pose.orientation.x = 0.0
        # marker.pose.orientation.y = 0.0
        # marker.pose.orientation.z = waypoints[GoalName][2]
        # marker.pose.orientation.w = waypoints[GoalName][3]
        
        # rospy.sleep(0.4)
        # self.TextMarker_Pub.publish(marker)
        
        

if __name__ == '__main__':
    rospy.init_node('TopologyMap_server')
    server = TopologyMapAction(rospy.get_name())
    sub_pose = SubscriberPose()
    dynamic_client = client.Client("move_base/TebLocalPlannerROS")
    # dynamic_client2 = client.Client("/move_base/global_costmap/inflation_layer")
    dynamic_client2 = client.Client("move_base/GlobalPlanner")
    
    MarkerViewer = MarkerViewer()
    for i in waypoints:
        MarkerViewer.PublishMarker(i)
        rospy.sleep(0.2)
    
    rospy.spin()
