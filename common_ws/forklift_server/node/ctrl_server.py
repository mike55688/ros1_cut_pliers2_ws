#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import actionlib
import forklift_server.msg
import apriltag_ros.msg

def PBVS_client(msg):
    client = actionlib.SimpleActionClient('PBVS_server', forklift_server.msg.PBVSAction)
    client.wait_for_server()
    command = forklift_server.msg.PBVSGoal(command=msg)
    # print("send ", command)
    client.send_goal(command)
    client.wait_for_result()
    return client.get_result()

def TopologyMap_client(msg):
    client = actionlib.SimpleActionClient('TopologyMap_server', forklift_server.msg.TopologyMapAction)
    client.wait_for_server()
    goal = forklift_server.msg.TopologyMapGoal(goal=msg)
    # print("send ", goal)
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()

def AprilTag_up_client(msg):
    client = actionlib.SimpleActionClient('AprilTag_up_server', apriltag_ros.msg.AprilTagAction)
    client.wait_for_server()
    goal = apriltag_ros.msg.AprilTagGoal(goal=msg)
    # print("send ", goal)
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()

def AprilTag_down_client(msg):
    client = actionlib.SimpleActionClient('AprilTag_down_server', apriltag_ros.msg.AprilTagAction)
    client.wait_for_server()
    goal = apriltag_ros.msg.AprilTagGoal(goal=msg)
    # print("send ", goal)
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()

if __name__ == '__main__':
    rospy.init_node('ctrl_server')
    rospy.logwarn(rospy.get_name() + "start")
    rospy.logwarn("your command list:\n")
    command = rospy.get_param(rospy.get_name() + "/command") 
    for i in command:
        print(i)

    for msg in command:
        rospy.sleep(1)
        if(msg[0] == 'PBVS'):
            rospy.logwarn("send PBVS: %s", msg[1])
            if(msg[1] == 'parking_bodycamera' or msg[1] == 'drop_pallet'):
                result = AprilTag_up_client(True)
                print("AprilTag_up_client result ", result)
                result = PBVS_client(msg[1])
                print("PBVS_client result ", result)
                result = AprilTag_up_client(False)
                print("AprilTag_up_client result ", result)

            elif(msg[1] == 'parking_forkcamera' or msg[1] == 'raise_pallet'):
                result = AprilTag_down_client(True)
                print("AprilTag_down_client result ", result)
                result = PBVS_client(msg[1])
                print("PBVS_client result ", result)
                result = AprilTag_down_client(False)
                print("AprilTag_down_client result ", result)

        elif(msg[0] == 'TopologyMap'):
            rospy.logwarn("send TopologyMap: %s", msg[1])
            result = TopologyMap_client(msg[1])
            print("TopologyMap result ", result)
        else:
            print("error command: ", msg)
            
    rospy.signal_shutdown("finish command list")
  
    
