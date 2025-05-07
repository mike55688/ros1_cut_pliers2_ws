#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from forklift_server.msg import PBVSMegaposeAction, PBVSMegaposeGoal, TopologyMapAction, TopologyMapGoal
from geometry_msgs.msg import PoseStamped

def PBVS_client(msg):
    client = actionlib.SimpleActionClient('PBVS_server', PBVSMegaposeAction)
    rospy.loginfo("Waiting for PBVS_server...")
    client.wait_for_server()
    command = PBVSMegaposeGoal(command=msg[1], layer_dist=msg[2])
    print("send ", command)
    client.send_goal(command)
    client.wait_for_result()
    return client.get_result()

def TopologyMap_client(msg):
    client = actionlib.SimpleActionClient('TopologyMap_server', TopologyMapAction)
    client.wait_for_server()
    goal = TopologyMapGoal(goal=msg[1])
    # print("send ", goal)
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()

def movebase_client(target,waypoints):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move_base server...")
    client.wait_for_server()

    coords = waypoints[target]
    goal = MoveBaseGoal()
    goal.target_pose = PoseStamped()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = coords[0]
    goal.target_pose.pose.position.y = coords[1]
    goal.target_pose.pose.orientation.z = coords[2]
    goal.target_pose.pose.orientation.w = coords[3]

    rospy.loginfo("Sending Navigation goal for waypoint '%s': %s", target, goal)
    client.send_goal(goal, done_cb=nav_done_cb)
    client.wait_for_result()
    
    result = client.get_result()
    rospy.loginfo("Navigation result: %s", result)
    return result

def nav_done_cb(state, result):
    if state != GoalStatus.SUCCEEDED:
        rospy.loginfo("Navigation goal failed with state: %d", state)
    else:
        rospy.loginfo("Navigation goal succeeded.")

if __name__ == '__main__':
    rospy.init_node('ctrl_server_megapose')
    rospy.logwarn("%s started", rospy.get_name())

    # 從參數伺服器取得指令列表與導航點字典（直接作為 YAML 格式參數設定）
    command = rospy.get_param("~command", [])
    waypoints = rospy.get_param("~waypoints", {})

    rospy.loginfo("Command list: %s", command)
    rospy.loginfo("Waypoints: %s", waypoints)

    for msg in command:
        rospy.sleep(1)
        if(msg[0] == 'PBVS' or msg[0] == 'odom'):
            rospy.logwarn(f"send {msg[0]}: {msg[1]}, {msg[2]}")
            result = PBVS_client(msg)
            print("PBVS_client result ", result)

        elif(msg[0] == 'MoveBase'):
            rospy.logwarn(f"send {msg[0]}: {msg[1]}")
            result = movebase_client(msg[1],waypoints)
            print("MoveBase result ", result)

        elif(msg[0] == 'TopologyMap'):
            rospy.logwarn(f"send {msg[0]}: {msg[1]}")
            result = TopologyMap_client(msg)
            print("TopologyMap result ", result)

        else:
            print("error command: ", msg)
            
    rospy.signal_shutdown("finish command list")
  
    
