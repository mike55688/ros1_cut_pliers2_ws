#! /usr/bin/env python3
import rospy
from gpm_msg.msg import forklift, forkposition

class fork():
    forwordbackpostion = 0.0
    updownposition = 0.0
    def __init__(self):
        self.sub_fork_cmd = rospy.Subscriber('/cmd_fork', forklift, self.cbGetforkcmd, queue_size = 10)
        self.pub_fork_pos = rospy.Publisher('/forkpos', forkposition, queue_size = 10)
    
    def cbGetforkcmd(self, msg):
        if(msg.forkmotion == 1):
            self.updownposition = self.updownposition + 0.01
        elif(msg.forkmotion == 2):
            self.updownposition = self.updownposition - 0.01
        elif(msg.forkmotion == 3):
            
        

    def updat_fork(self):
        while not rospy.is_shutdown():
            self.pub_fork_pos.publish(self.forwordbackpostion, self.updownposition)
            rospy.sleep(0.1)
        
if __name__ == '__main__':
    rospy.init_node('fork_simulation', anonymous = True)
    fork = fork()
    rospy.spin()
