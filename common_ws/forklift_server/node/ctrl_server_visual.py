#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import actionlib
import forklift_server.msg

def PBVS_client(msg):
    client = actionlib.SimpleActionClient('PBVS', forklift_server.msg.PBVSAction)
    client.wait_for_server()
    command = forklift_server.msg.PBVSGoal(command=msg)
    print("send ", command)
    client.send_goal(command)
    client.wait_for_result()
    return client.get_result()

def TopologyMap_client(msg):
    client = actionlib.SimpleActionClient('TopologyMap', forklift_server.msg.TopologyMapAction)
    client.wait_for_server()
    goal = forklift_server.msg.TopologyMapGoal(goal=msg)
    print("send ", goal)
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()

if __name__ == '__main__':
    rospy.init_node('ctrl_server')

    command =[
        ['TopologyMap', 'v17'],
        ['PBVS', 'parking_down'], 
        ['PBVS', 'up'], 
        ['TopologyMap', 'v23'],
        ['PBVS', 'parking_up'],
        ['PBVS', 'down'],
        ['TopologyMap', 'v17'],
        ['PBVS', 'parking_down'], 
        ['PBVS', 'up'], 
        ['TopologyMap', 'v23'],
        ['PBVS', 'parking_up'],
        ['PBVS', 'down']
    ]

    for msg in command:
        rospy.sleep(1)
        if(msg[0] == 'PBVS'):
            print("send PBVS: ", msg[1])
            result = PBVS_client(msg[1])
            print("result ", result)
        elif(msg[0] == 'TopologyMap'):
            print("send TopologyMap: ", msg[1])
            result = TopologyMap_client(msg[1])
            print("result ", result)


  
    
