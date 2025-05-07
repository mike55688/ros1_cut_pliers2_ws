# -*- coding: utf-8 -*-
import rospy
import forklift_server.msg
import actionlib
class Scriber():
    def __init__(self):
        client = actionlib.SimpleActionClient('PBVS_server', forklift_server.msg.PBVSAction)
        client.wait_for_server()
        command = forklift_server.msg.PBVSGoal(command='parking_forkcamera')
        client.send_goal(command)
        client.feedback_cb=self.cbGetFeedback
        client.done_cb=self.cbGetResult
        client.wait_for_result()
    def cbGetFeedback(self, msg):
        print(msg)
    def cbGetResult(self, msg):
        print(msg)
        
if __name__ == '__main__':
    rospy.init_node('HowToKnowActionServerFeedback')
    sub = Scriber()
    rospy.spin()