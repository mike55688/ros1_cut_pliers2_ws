#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy


def info():
    print(type(waypoints))
    print(waypoints)
    print(type(graph))
    print(graph)

if __name__ == '__main__':
    rospy.init_node("TopologyMap_server")
    rospy.logwarn(rospy.get_name() + " started")
    global waypoints
    global graph
    waypoints = rospy.get_param(rospy.get_name() + "/waypoints")
    graph = rospy.get_param(rospy.get_name() + "/graph")
    info()
    
    rospy.sleep(5)
