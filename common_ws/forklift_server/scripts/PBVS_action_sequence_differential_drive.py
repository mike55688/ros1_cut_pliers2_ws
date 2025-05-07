# -*- coding: utf-8 -*-
import rospy
import forklift_server.msg
from enum import Enum
from PBVS_Action_megapose_differential_drive import Action
# from forklift_msg.msg import meteorcar

FruitSequence = Enum( 'FruitSequence', \
                        'cut_pliers_rises \
                        move_forward_y   \
                        cut_pliers_approach \
                        move_forward \
                        cut_pliers_dead_reckoning \
                        cut_pliers_close \
                        cut_pliers_backing \
                        cut_pliers_open \
                        cut_pliers_up \
                        cut_pliers_down \
                        cut_pliers_up_down') 

class PBVS():
    def __init__(self, _as, subscriber, mode):
        self._as = _as
        self._feedback = forklift_server.msg.PBVSMegaposeFeedback()
        self._result = forklift_server.msg.PBVSMegaposeResult()
        self.subscriber = subscriber
        self.command = mode.command
        self.layer_dist = mode.layer_dist
        self.check_wait_time = 0
        self.Action = Action(self.subscriber)


    def fruit_docking(self):  
        current_sequence = FruitSequence.cut_pliers_up_down.value

        while(not rospy.is_shutdown()):
            rospy.sleep(0.1)


            if current_sequence == FruitSequence.cut_pliers_up_down.value:    

                self.is_sequence_finished = self.Action.fnControlArmBasedOnFruitZ("bodycamera")

                if self.is_sequence_finished:
                    current_sequence = FruitSequence.cut_pliers_approach.value  
                    self.is_sequence_finished = False  


            elif current_sequence == FruitSequence.cut_pliers_approach.value:

                self.is_sequence_finished = self.Action.fnControlArmBasedOnFruitX("bodycamera", target_x=-0.13)

                if self.is_sequence_finished:
                    current_sequence = FruitSequence.cut_pliers_dead_reckoning.value  
                    self.is_sequence_finished = False  

            elif current_sequence == FruitSequence.cut_pliers_dead_reckoning.value:

                self.is_sequence_finished = self.Action.fnBlindExtendArm(84)

                if self.is_sequence_finished:
                    current_sequence = FruitSequence.cut_pliers_close.value  
                    self.is_sequence_finished = False  

            elif current_sequence == FruitSequence.cut_pliers_close.value:

                self.is_sequence_finished = self.Action.fnControlClaw(1)

                if self.is_sequence_finished:
                    current_sequence = FruitSequence.cut_pliers_up.value  
                    self.is_sequence_finished = False  


            elif current_sequence == FruitSequence.cut_pliers_up.value:
                self.is_sequence_finished = self.Action.fnControlArm(height = 110)

                if self.is_sequence_finished:
                    current_sequence = FruitSequence.cut_pliers_backing.value  
                    self.is_sequence_finished = False  




            elif current_sequence == FruitSequence.cut_pliers_backing.value:

                self.is_sequence_finished = self.Action.fnRetractArm(10)
                if self.is_sequence_finished:
                    current_sequence = FruitSequence.cut_pliers_down.value  

                    self.is_sequence_finished = False  

            elif current_sequence == FruitSequence.cut_pliers_down.value:
                self.is_sequence_finished = self.Action.fnControlArm(height = 80)

                if self.is_sequence_finished:
                    current_sequence = FruitSequence.cut_pliers_open.value  
                    self.is_sequence_finished = False  


            elif current_sequence == FruitSequence.cut_pliers_open.value:

                self.is_sequence_finished = self.Action.fnControlClaw(0)

                if self.is_sequence_finished:
                    rospy.loginfo("Process completed successfully.")
                    return





            else:
                rospy.loginfo(f"Error: {current_sequence} does not exist")
                return