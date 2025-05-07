# -*- coding: utf-8 -*-
import rospy
import forklift_server.msg
from enum import Enum
import math
import tkinter as tk
from PBVS_Action import Action
from gpm_msg.msg import forklift


class PBVS():
    ParkingSequence = Enum('ParkingSequence',
                           'init_fork \
                            changing_direction_1 \
                            Changingtheta \
                            decide \
                            back_turn \
                            back \
                            moving_nearby_parking_lot \
                            parking \
                            up_fork_dead_reckoning \
                            up_fork_init \
                            up_fork_up_half \
                            up_fork_up \
                            up_fork_down \
                            up_fork_forward \
                            up_fork_forward_half \
                            up_fork_backword \
                            up_fork_tilt_forward \
                            up_fork_tilt_backword \
                            up_fork_back \
                            up_fork_going \
                            down_fork_dead_reckoning \
                            down_fork_init \
                            down_fork_up \
                            down_fork_down \
                            down_fork_forward \
                            down_fork_backword \
                            down_fork_tilt_forward \
                            down_fork_tilt_backword \
                            down_fork_back \
                            down_fork_going \
                            stop')

    def __init__(self, _as, subscriber, mode):
        self._as = _as
        self._feedback = forklift_server.msg.PBVSFeedback()
        self._result = forklift_server.msg.PBVSResult()
        self.subscriber = subscriber
        self.mode = mode.command
        self.ActionCode = mode.ActionCode
        self.ShelfParameter = mode.ShelfParameter
        self. UpDownPosition = mode.UpDownPosition
        self. ForrwardBackwardPosition = mode.ForrwardBackwardPosition
        self. TilePositionv = mode.TilePosition
        self. MovePosition = mode.MovePosition
        self.Action = Action(self.subscriber)
        self.init_PBVS_parame()

    def init_PBVS_parame(self):
        self.is_sequence_finished = False
        if self.ActionCode == 20:
            self.current_parking_sequence = self.ParkingSequence.up_fork_init.value
            self.main_loop()
        elif self.ActionCode == 21:
            self.current_parking_sequence = self.ParkingSequence.up_fork_init.value
            self.main_loop()
        elif self.ActionCode == 22:
            self.current_parking_sequence = self.ParkingSequence.up_fork_init.value
            self.main_loop()
        elif self.ActionCode == 30:
            self.current_parking_sequence = self.ParkingSequence.up_fork_init.value
            self.main_loop()
        else:
            if self.ActionCode == 10:
                if self.ShelfParameter == 0:
                    self.mode = "parking_forkcamera"
                else:
                    self.mode = "parking_bodycamera"
            elif self.ActionCode == 90:
                if self.ShelfParameter == 1:
                    self.mode = "drop_pallet"
                else:
                    self.mode = "raise_pallet"
            else:  # [0]:未附值, [other]:給錯值 走舊流程
                pass

            if self.mode == "parking_bodycamera":
                self.subscriber.updown = True
                self.subscriber.offset_x = rospy.get_param(
                    rospy.get_name() + "/bodycamera_tag_offset_x", 0.325)
                self.init_fork = rospy.get_param(
                    rospy.get_name() + "/bodycamera_parking_fork_init", 0.392)
                self.ChangingDirection_threshold = rospy.get_param(
                    rospy.get_name() + "/bodycamera_ChangingDirection_threshold", 0.01)
                self.Parking_distance = rospy.get_param(
                    rospy.get_name() + "/bodycamera_parking_stop", 1.8)
                self.Changingtheta_threshod = rospy.get_param(
                    rospy.get_name() + "/bodycamera_Changingtheta_threshold", 0.1)
                self.decide_distance = rospy.get_param(
                    rospy.get_name() + "/bodycamera_decide_distance", 0.04)
                self.back_distance = rospy.get_param(
                    rospy.get_name() + "/bodycamera_back_distance", 3.0)
                self.current_parking_sequence = self.ParkingSequence.init_fork.value  # for 大車
                # self.current_parking_sequence = self.ParkingSequence.changing_direction_1.value # for 小車
                self.main_loop()

            elif self.mode == "parking_forkcamera":
                self.subscriber.updown = False
                self.subscriber.offset_x = rospy.get_param(
                    rospy.get_name() + "/forkcamera_tag_offset_x", 0.03)
                self.init_fork = rospy.get_param(
                    rospy.get_name() + "/forkcamera_parking_fork_init", 0.211)
                self.ChangingDirection_threshold = rospy.get_param(
                    rospy.get_name() + "/forkcamera_ChangingDirection_threshold", 0.01)
                self.Parking_distance = rospy.get_param(
                    rospy.get_name() + "/forkcamera_parking_stop", 1.43)
                self.Changingtheta_threshod = rospy.get_param(
                    rospy.get_name() + "/forkcamera_Changingtheta_threshold", 0.1)
                self.decide_distance = rospy.get_param(
                    rospy.get_name() + "/forkcamera_decide_distance", 0.04)
                self.back_distance = rospy.get_param(
                    rospy.get_name() + "/forkcamera_back_distance", 3.0)
                # self.current_parking_sequence = self.ParkingSequence.Changingtheta.value #test
                self.current_parking_sequence = self.ParkingSequence.init_fork.value  # for 大車
                # self.current_parking_sequence = self.ParkingSequence.changing_direction_1.value // for 小車
                self.main_loop()
                return

            elif self.mode == "raise_pallet":
                self.subscriber.updown = False
                self.init_fork = rospy.get_param(
                    rospy.get_name() + "/raise_pallet_fork_init", 0.4576)
                self.dead_reckoning_dist = rospy.get_param(
                    rospy.get_name() + "/raise_pallet_dead_reckoning_dist", 0.9)
                self.fork_forward_distance = rospy.get_param(
                    rospy.get_name() + "/raise_pallet_fork_forward_distance", 0.7)
                self.raise_height = rospy.get_param(
                    rospy.get_name() + "/raise_pallet_raise_height", 0.57)
                self.back_distance = rospy.get_param(
                    rospy.get_name() + "/raise_pallet_back_distance", 1.0)
                self.navigation_helght = rospy.get_param(
                    rospy.get_name() + "/raise_pallet_navigation_helght", 0.392)
                self.current_parking_sequence = self.ParkingSequence.up_fork_init.value
                self.main_loop()
                return

            elif self.mode == "drop_pallet":
                self.subscriber.updown = True
                self.init_fork = rospy.get_param(
                    rospy.get_name() + "/drop_pallet_fork_init", 0.86)
                self.dead_reckoning_dist = rospy.get_param(
                    rospy.get_name() + "/drop_pallet_dead_reckoning_dist", -0.85)
                self.fork_forward_distance = rospy.get_param(
                    rospy.get_name() + "/drop_pallet_fork_forward_distance", 0.7)
                self.drop_height = rospy.get_param(
                    rospy.get_name() + "/drop_pallet_drop_height", 0.67)
                self.back_distance = rospy.get_param(
                    rospy.get_name() + "/drop_pallet_back_distance", 1.0)
                self.navigation_helght = rospy.get_param(
                    rospy.get_name() + "/drop_pallet_navigation_helght", 0.07)
                self.current_parking_sequence = self.ParkingSequence.down_fork_init.value
                self.main_loop()
                return

            else:
                rospy.logwarn("mode is not correct")
                self._result.result = 'fail'
                self._as.set_succeeded(self._result)
                return

    def main_loop(self):
        r = rospy.Rate(10)
        while (not rospy.is_shutdown()):
            if (self.PBVS()):
                break
            r.sleep()

    def __del__(self):
        rospy.logwarn('delet PBVS')

    def PBVS(self):
        if self._as.is_preempt_requested():
            rospy.logwarn('PBVS Preempted')
            self.current_parking_sequence = self.ParkingSequence.stop.value

        self._feedback.feedback = str(
            self.ParkingSequence(self.current_parking_sequence))
        self._as.publish_feedback(self._feedback)
        # ActionCode-->[10]:shelf, [20]:up/down, [21]:forward/backward, [22]:tile, [30]:move
        if self.ActionCode == 20:
            self.is_sequence_finished = self.Action.fork_updown(
                self.UpDownPosition)
            if self.is_sequence_finished == True:
                rospy.sleep(1)
                return True
        elif self.ActionCode == 21:
            self.is_sequence_finished = self.Action.fork_forwardback(
                self.ForrwardBackwardPosition)
            if self.is_sequence_finished == True:
                rospy.sleep(1)
                return True
        elif self.ActionCode == 22:
            pass  # TODO Tile function
        elif self.ActionCode == 30:
            self.is_sequence_finished = self.Action.fnseqdead_reckoning(1)
            if self.is_sequence_finished == True:
                rospy.sleep(1)
                return True
        else:
            if self.ActionCode == 10:
                if self.ShelfParameter == 0:
                    self.mode = "parking_forkcamera"
                else:
                    self.mode == "parking_bodycamera",
            else:  # [0]:未附值, [other]:給錯值 走舊流程
                pass

            # ============parking============
            if self.current_parking_sequence == self.ParkingSequence.init_fork.value:
                self.is_sequence_finished = self.Action.fork_updown(
                    self.init_fork)

                if self.is_sequence_finished == True:
                    self.current_parking_sequence = self.ParkingSequence.changing_direction_1.value
                    self.is_sequence_finished = False
            elif self.current_parking_sequence == self.ParkingSequence.changing_direction_1.value:
                self.is_sequence_finished = self.Action.fnSeqChangingDirection(
                    self.ChangingDirection_threshold)

                if self.is_sequence_finished == True:
                    self.current_parking_sequence = self.ParkingSequence.moving_nearby_parking_lot.value
                    self.is_sequence_finished = False

            elif self.current_parking_sequence == self.ParkingSequence.moving_nearby_parking_lot.value:
                self.is_sequence_finished = self.Action.fnSeqMovingNearbyParkingLot()
                if self.is_sequence_finished == True:
                    self.current_parking_sequence = self.ParkingSequence.parking.value
                    self.is_sequence_finished = False

            elif self.current_parking_sequence == self.ParkingSequence.parking.value:
                self.is_sequence_finished = self.Action.fnSeqParking(
                    self.Parking_distance)

                if self.is_sequence_finished == True:
                    self.current_parking_sequence = self.ParkingSequence.Changingtheta.value
                    self.is_sequence_finished = False

            elif self.current_parking_sequence == self.ParkingSequence.Changingtheta.value:
                self.is_sequence_finished = self.Action.fnSeqChangingtheta(
                    self.Changingtheta_threshod)

                if self.is_sequence_finished == True:
                    self.current_parking_sequence = self.ParkingSequence.decide.value
                    self.is_sequence_finished = False

            elif self.current_parking_sequence == self.ParkingSequence.decide.value:
                self.is_sequence_finished = self.Action.fnSeqdecide(
                    self.decide_distance)

                if self.is_sequence_finished == True:
                    self.current_parking_sequence = self.ParkingSequence.stop.value
                    self.is_sequence_finished = False

                elif self.is_sequence_finished == False:
                    self.current_parking_sequence = self.ParkingSequence.back.value
                    self.is_sequence_finished = False

            # elif self.current_parking_sequence == self.ParkingSequence.back_turn.value:
            #     turn_threshod = 0.04
            #     self.is_sequence_finished = self.Action.fnseqturn(self.back_distance, turn_threshod)

            #     if self.is_sequence_finished == True:
            #         self.current_parking_sequence = self.ParkingSequence.back.value
            #         self.is_sequence_finished = False

            elif self.current_parking_sequence == self.ParkingSequence.back.value:
                self.is_sequence_finished = self.Action.fnseqmove_to_marker_dist(
                    self.back_distance)

                if self.is_sequence_finished == True:
                    self.current_parking_sequence = self.ParkingSequence.parking.value
                    self.is_sequence_finished = False
            # ============raise_pallet============
            # :TODO only raise pallet heve fork_updown_finetune()
            elif self.current_parking_sequence == self.ParkingSequence.up_fork_init.value:
                self.is_sequence_finished = self.Action.fork_updown(
                    self.init_fork)

                if self.is_sequence_finished == True:
                    rospy.sleep(0.05)
                    self.current_parking_sequence = self.ParkingSequence.up_fork_dead_reckoning.value
                    self.is_sequence_finished = False

            elif self.current_parking_sequence == self.ParkingSequence.up_fork_dead_reckoning.value:
                self.is_sequence_finished = self.Action.fnseqdead_reckoning(
                    self.dead_reckoning_dist)

                if self.is_sequence_finished == True:
                    rospy.sleep(0.05)
                    self.current_parking_sequence = self.ParkingSequence.up_fork_forward_half.value
                    self.is_sequence_finished = False

            elif self.current_parking_sequence == self.ParkingSequence.up_fork_forward_half.value:
                self.is_sequence_finished = self.Action.fork_forwardback(
                    self.fork_forward_distance/2)

                if self.is_sequence_finished == True:
                    rospy.sleep(0.05)
                    self.current_parking_sequence = self.ParkingSequence.up_fork_up_half.value
                    self.is_sequence_finished = False

            elif self.current_parking_sequence == self.ParkingSequence.up_fork_up_half.value:
                self.pub_fork = rospy.Publisher(
                    '/cmd_fork', forklift, queue_size=1)
                rospy.sleep(0.05)
                self.pub_fork.publish(3)
                rospy.sleep(0.07)
                self.pub_fork.publish(1)
                self.is_sequence_finished = True
                if self.is_sequence_finished == True:
                    rospy.sleep(0.05)
                    self.current_parking_sequence = self.ParkingSequence.up_fork_forward.value
                    self.is_sequence_finished = False

            elif self.current_parking_sequence == self.ParkingSequence.up_fork_forward.value:
                self.is_sequence_finished = self.Action.fork_forwardback(
                    self.fork_forward_distance)

                if self.is_sequence_finished == True:
                    rospy.sleep(0.05)
                    self.current_parking_sequence = self.ParkingSequence.up_fork_up.value
                    self.is_sequence_finished = False

            elif self.current_parking_sequence == self.ParkingSequence.up_fork_up.value:
                self.is_sequence_finished = self.Action.fork_updown(
                    self.raise_height)

                if self.is_sequence_finished == True:
                    rospy.sleep(0.05)
                    self.current_parking_sequence = self.ParkingSequence.up_fork_backword.value
                    self.is_sequence_finished = False

            elif self.current_parking_sequence == self.ParkingSequence.up_fork_backword.value:
                self.is_sequence_finished = self.Action.fork_forwardback(0)

                if self.is_sequence_finished == True:
                    rospy.sleep(0.05)
                    self.current_parking_sequence = self.ParkingSequence.up_fork_back.value
                    self.is_sequence_finished = False

            elif self.current_parking_sequence == self.ParkingSequence.up_fork_back.value:
                self.is_sequence_finished = self.Action.fnseqdead_reckoning(
                    self.back_distance)

                if self.is_sequence_finished == True:
                    rospy.sleep(0.05)
                    self.current_parking_sequence = self.ParkingSequence.up_fork_going.value
                    self.is_sequence_finished = False

            elif self.current_parking_sequence == self.ParkingSequence.up_fork_going.value:
                self.is_sequence_finished = self.Action.fork_updown(
                    self.navigation_helght)

                if self.is_sequence_finished == True:
                    rospy.sleep(0.05)
                    self.current_parking_sequence = self.ParkingSequence.stop.value
                    self.is_sequence_finished = False
            # ============drop_pallet============
            elif self.current_parking_sequence == self.ParkingSequence.down_fork_init.value:
                self.is_sequence_finished = self.Action.fork_updown(
                    self.init_fork)

                if self.is_sequence_finished == True:
                    rospy.sleep(0.05)
                    self.current_parking_sequence = self.ParkingSequence.down_fork_dead_reckoning.value
                    self.is_sequence_finished = False

            elif self.current_parking_sequence == self.ParkingSequence.down_fork_dead_reckoning.value:
                self.is_sequence_finished = self.Action.fnseqdead_reckoning(
                    self.dead_reckoning_dist)

                if self.is_sequence_finished == True:
                    rospy.sleep(0.05)
                    self.current_parking_sequence = self.ParkingSequence.down_fork_forward.value
                    self.is_sequence_finished = False

            elif self.current_parking_sequence == self.ParkingSequence.down_fork_forward.value:
                self.is_sequence_finished = self.Action.fork_forwardback(
                    self.fork_forward_distance)

                if self.is_sequence_finished == True:
                    rospy.sleep(0.05)
                    self.current_parking_sequence = self.ParkingSequence.down_fork_down.value
                    self.is_sequence_finished = False

            elif self.current_parking_sequence == self.ParkingSequence.down_fork_down.value:
                self.is_sequence_finished = self.Action.fork_updown(
                    self.drop_height)

                if self.is_sequence_finished == True:
                    rospy.sleep(0.05)
                    self.current_parking_sequence = self.ParkingSequence.down_fork_backword.value
                    self.is_sequence_finished = False

            elif self.current_parking_sequence == self.ParkingSequence.down_fork_backword.value:
                self.is_sequence_finished = self.Action.fork_forwardback(0.0)

                if self.is_sequence_finished == True:
                    rospy.sleep(0.05)
                    self.current_parking_sequence = self.ParkingSequence.down_fork_back.value
                    self.is_sequence_finished = False

            elif self.current_parking_sequence == self.ParkingSequence.down_fork_back.value:
                self.is_sequence_finished = self.Action.fnseqdead_reckoning(
                    self.back_distance)

                if self.is_sequence_finished == True:
                    rospy.sleep(0.05)
                    self.current_parking_sequence = self.ParkingSequence.down_fork_going.value
                    self.is_sequence_finished = False

            elif self.current_parking_sequence == self.ParkingSequence.down_fork_going.value:
                self.is_sequence_finished = self.Action.fork_updown(
                    self.navigation_helght)

                if self.is_sequence_finished == True:
                    rospy.sleep(0.05)
                    self.current_parking_sequence = self.ParkingSequence.stop.value
                    self.is_sequence_finished = False
            # ============stop============
            elif self.current_parking_sequence == self.ParkingSequence.stop.value:

                # self.window.destroy()
                rospy.sleep(1)
                return True

            return False
