#! /usr/bin/env python3
import rospy
from enum import Enum
from gpm_msg.msg import forklift, forkposition

class Action():
    def __init__(self):
        self.subscriber = Subscriber()
        self.pub_fork = rospy.Publisher('/cmd_fork', forklift, queue_size = 1)
        self.forkmotion = Enum('forkmotion', 
                               'stop \
                                up \
                                down \
                                forward \
                                backword \
                                tilt_forward \
                                tilt_backword')
        self.init_parame()

        while not rospy.is_shutdown():
            self.is_sequence_finished = self.fork_forwardback(0.1)
            print("up")
            if self.is_sequence_finished == True:
                self.is_sequence_finished = False
                break
            rospy.sleep(0.1)

    def __del__(self):
        self.pub_fork.publish(self.forkmotion.stop.value)

    def init_parame(self):
        self.is_sequence_finished = False
        self.forwardbackpostion = 0.0
        self.updownposition = 0.0
        self.fork_threshold = 0.05
        
    def update_fork(self):
        self.forwardbackpostion, self.updownposition = self.subscriber.SpinOnce_fork()
    
    def fork_updown(self, desired_updownposition):
        self.update_fork()
        if self.updownposition < desired_updownposition - self.fork_threshold:
            self.pub_fork.publish(self.forkmotion.up.value)
            return False
        elif self.updownposition > desired_updownposition + self.fork_threshold:
            self.pub_fork.publish(self.forkmotion.down.value)
            return False
        else:
            self.pub_fork.publish(self.forkmotion.stop.value)
            return True

    def fork_forwardback(self, desired_forwardbackpostion):
        self.update_fork()
        if self.forwardbackpostion < desired_forwardbackpostion - self.fork_threshold:
            self.pub_fork.publish(self.forkmotion.forward.value)
            return False
        elif self.forwardbackpostion > desired_forwardbackpostion + self.fork_threshold:
            self.pub_fork.publish(self.forkmotion.backword.value)
            return False
        else:
            self.pub_fork.publish(self.forkmotion.stop.value)
            return True


class Subscriber():
    def __init__(self):
        self.init_parame()
        self.sub_forwardbackpostion = rospy.Subscriber('/forkpos', forkposition, self.cbGetforkpos, queue_size = 1)
        
        
    def init_parame(self):
        self.forwardbackpostion = 0.0
        self.updownposition = 0.0

    def SpinOnce_fork(self):
        return self.forwardbackpostion, self.updownposition

    def cbGetforkpos(self, msg):
        self.forwardbackpostion = msg.forwardbackpostion
        self.updownposition = msg.updownposition
        



if __name__ == '__main__':
    rospy.init_node('PBVS_Action_test', anonymous=True)
    Action = Action()
    rospy.spin()