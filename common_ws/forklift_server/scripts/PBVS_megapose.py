# -*- coding: utf-8 -*-
import rospy
import forklift_server.msg
from enum import Enum
from PBVS_Action_megapose import Action
# from forklift_msg.msg import meteorcar
ParkingBodyCameraSequence = Enum( 'ParkingBodyCameraSequence', \
                    'init_fork \
                    changing_direction \
                    move_nearby_parking_lot \
                    parking \
                    changingtheta \
                    decide \
                    back \
                    stop \
                    error')
ParkingForkCameraSequence = Enum( 'ParkingForkCameraSequence', \
                        'init_fork \
                        changing_direction \
                        parking \
                        changingtheta \
                        decide \
                        back \
                        stop \
                        error')
RaisePalletSequence = Enum( 'RaisePalletSequence', \
                        'init_fork \
                        dead_reckoning \
                        fork_updown \
                        back \
                        stop \
                        error')
DropPalletSequence = Enum( 'DropPalletSequence', \
                        'init_fork \
                        dead_reckoning \
                        fork_updown \
                        back \
                        stop \
                        error')
FrontSequence = Enum( 'FrontSequence', \
                        'Front \
                        stop \
                        error')
TurnSequence = Enum( 'TurnSequence', \
                        'Turn \
                        stop \
                        error')
    
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

    def parking_bodycamera(self):
        current_sequence = ParkingBodyCameraSequence.init_fork.value
        previous_sequence = None  # 用來記錄上一次的階段

        while(not rospy.is_shutdown()):
            # 如果 current_sequence 發生變化，記錄 log
            if current_sequence != previous_sequence:
                rospy.loginfo('Current Sequence: {0}'.format(ParkingBodyCameraSequence(current_sequence)))
                previous_sequence = current_sequence  # 更新 previous_sequence

            if(current_sequence == ParkingBodyCameraSequence.init_fork.value):
                # self.subscriber.fnDetectionAllowed(True, False, self.layer_dist)  # fnDetectionAllowed(self, shelf_detection, pallet_detection, layer)

                self.is_sequence_finished = self.Action.fnForkUpdown(self.subscriber.bodycamera_parking_fork_init)
                
                if self.is_sequence_finished == True:
                    current_sequence = ParkingBodyCameraSequence.changing_direction.value
                    self.is_sequence_finished = False
            
            elif(current_sequence == ParkingBodyCameraSequence.changing_direction.value):
                self.subscriber.fnDetectionAllowed(True, False, self.layer_dist)  # fnDetectionAllowed(self, shelf_detection, pallet_detection, layer)
                self.is_sequence_finished = self.Action.fnSeqChangingDirection(self.subscriber.bodycamera_ChangingDirection_threshold, "bodycamera")

                if self.is_sequence_finished == True:
                    current_sequence = ParkingBodyCameraSequence.move_nearby_parking_lot.value
                    self.is_sequence_finished = False
            
            elif(current_sequence == ParkingBodyCameraSequence.move_nearby_parking_lot.value):
                # self.subscriber.fnDetectionAllowed(True, False, self.layer_dist)  # fnDetectionAllowed(self, shelf_detection, pallet_detection, layer)
                self.is_sequence_finished = self.Action.fnSeqMovingNearbyParkingLot(self.subscriber.bodycamera_desired_dist_threshold)
                
                if self.is_sequence_finished == True:
                    current_sequence = ParkingBodyCameraSequence.parking.value
                    self.is_sequence_finished = False
            elif(current_sequence == ParkingBodyCameraSequence.parking.value):
                # self.subscriber.fnDetectionAllowed(True, False, self.layer_dist)  # fnDetectionAllowed(self, shelf_detection, pallet_detection, layer)
                self.is_sequence_finished = self.Action.fnSeqParking(self.subscriber.bodycamera_parking_stop, 1.0, "bodycamera")
                
                if self.is_sequence_finished == True:
                    current_sequence = ParkingBodyCameraSequence.changingtheta.value
                    self.is_sequence_finished = False
            elif(current_sequence == ParkingBodyCameraSequence.changingtheta.value):
                # self.subscriber.fnDetectionAllowed(True, False, self.layer_dist)  # fnDetectionAllowed(self, shelf_detection, pallet_detection, layer)
                self.is_sequence_finished = self.Action.fnSeqChangingtheta(self.subscriber.bodycamera_Changingtheta_threshold, "bodycamera")
                
                if self.is_sequence_finished == True:
                    current_sequence = ParkingBodyCameraSequence.decide.value
                    self.is_sequence_finished = False
            elif(current_sequence == ParkingBodyCameraSequence.decide.value):
                # self.subscriber.fnDetectionAllowed(True, False, self.layer_dist)  # fnDetectionAllowed(self, shelf_detection, pallet_detection, layer)
                self.is_sequence_finished = self.Action.fnSeqdecide(self.subscriber.bodycamera_decide_distance)
                
                if self.is_sequence_finished == True:
                    current_sequence = ParkingBodyCameraSequence.stop.value
                    self.is_sequence_finished = False
                elif self.is_sequence_finished == False:
                    current_sequence = ParkingBodyCameraSequence.back.value
                    self.is_sequence_finished = False
            elif(current_sequence == ParkingBodyCameraSequence.back.value):
                # self.is_sequence_finished = self.Action.fnseqMoveToMarkerDist(self.subscriber.bodycamera_back_distance)
                # self.subscriber.fnDetectionAllowed(True, False, self.layer_dist)  # fnDetectionAllowed(self, shelf_detection, pallet_detection, layer)
                self.is_sequence_finished = self.Action.fnseqDeadReckoning(-self.subscriber.bodycamera_back_distance)
                
                if self.is_sequence_finished == True:
                    rospy.Rate(1).sleep()
                    current_sequence = ParkingBodyCameraSequence.parking.value
                    self.is_sequence_finished = False

            elif(current_sequence == ParkingBodyCameraSequence.stop.value):
                self.subscriber.fnDetectionAllowed(False, False, self.layer_dist)  # fnDetectionAllowed(self, shelf_detection, pallet_detection, layer)
                if self.check_wait_time > 15 :
                    self.check_wait_time = 0
                    return
                else:
                    self.check_wait_time =self.check_wait_time  +1
            
            else:
                rospy.logerr('Error: {0} does not exist'.format(current_sequence))
                self.subscriber.fnDetectionAllowed(False, False, self.layer_dist)  # fnDetectionAllowed(self, shelf_detection, pallet_detection, layer)
                if self.check_wait_time > 15 :
                    self.check_wait_time = 0
                    return
                else:
                    self.check_wait_time =self.check_wait_time  +1
               
    def parking_forkcamera(self):
        current_sequence = ParkingForkCameraSequence.init_fork.value
        previous_sequence = None  # 用來記錄上一次的階段

        while(not rospy.is_shutdown()):
            # 如果 current_sequence 發生變化，記錄 log
            if current_sequence != previous_sequence:
                rospy.loginfo('Current Sequence: {0}'.format(ParkingForkCameraSequence(current_sequence)))
                previous_sequence = current_sequence  # 更新 previous_sequence

            if(current_sequence == ParkingForkCameraSequence.init_fork.value):
                # self.subscriber.fnDetectionAllowed(False, True, self.layer_dist)  # fnDetectionAllowed(self, shelf_detection, pallet_detection, layer)
                if self.layer_dist == 1.0:
                    self.is_sequence_finished = self.Action.fnForkUpdown(self.subscriber.forkcamera_parking_fork_layer1)
                elif self.layer_dist == 2.0:
                    self.is_sequence_finished = self.Action.fnForkUpdown(self.subscriber.forkcamera_parking_fork_layer2)
                else:
                    rospy.loginfo('Layer is not defined')
                    self.subscriber.fnDetectionAllowed(False, False, self.layer_dist)  # fnDetectionAllowed(self, shelf_detection, pallet_detection, layer)
                    return
                
                if self.is_sequence_finished == True:
                    current_sequence = ParkingForkCameraSequence.parking.value
                    self.is_sequence_finished = False
            
            elif(current_sequence == ParkingForkCameraSequence.parking.value):
                self.subscriber.fnDetectionAllowed(False, True, self.layer_dist)  # fnDetectionAllowed(self, shelf_detection, pallet_detection, layer)
                self.is_sequence_finished = self.Action.fnSeqParking(self.subscriber.forkcamera_parking_stop, 1.0, "forkcamera")
                
                if self.is_sequence_finished == True:
                    current_sequence = ParkingForkCameraSequence.changingtheta.value
                    self.is_sequence_finished = False
            elif(current_sequence == ParkingForkCameraSequence.changingtheta.value):
                # self.subscriber.fnDetectionAllowed(False, True, self.layer_dist)  # fnDetectionAllowed(self, shelf_detection, pallet_detection, layer)
                self.is_sequence_finished = self.Action.fnSeqChangingtheta(self.subscriber.forkcamera_Changingtheta_threshold, "forkcamera")
                
                if self.is_sequence_finished == True:
                    current_sequence = ParkingForkCameraSequence.decide.value
                    self.is_sequence_finished = False
            elif(current_sequence == ParkingForkCameraSequence.decide.value):
                # self.subscriber.fnDetectionAllowed(False, True, self.layer_dist)  # fnDetectionAllowed(self, shelf_detection, pallet_detection, layer)
                self.is_sequence_finished = self.Action.fnSeqdecide(self.subscriber.forkcamera_decide_distance)
                
                if self.is_sequence_finished == True:
                    current_sequence = ParkingForkCameraSequence.stop.value
                    self.is_sequence_finished = False
                elif self.is_sequence_finished == False:
                    current_sequence = ParkingForkCameraSequence.back.value
                    self.is_sequence_finished = False
            elif(current_sequence == ParkingForkCameraSequence.back.value):
                # self.is_sequence_finished = self.Action.fnseqMoveToMarkerDist(self.subscriber.forkcamera_back_distance)
                # self.subscriber.fnDetectionAllowed(False, True, self.layer_dist)  # fnDetectionAllowed(self, shelf_detection, pallet_detection, layer)
                self.is_sequence_finished = self.Action.fnseqDeadReckoning(-self.subscriber.forkcamera_back_distance)
                
                if self.is_sequence_finished == True:
                    current_sequence = ParkingForkCameraSequence.parking.value
                    self.is_sequence_finished = False

            elif(current_sequence == ParkingForkCameraSequence.stop.value):
                self.subscriber.fnDetectionAllowed(False, False, self.layer_dist)  # fnDetectionAllowed(self, shelf_string, pallet_string)
                if self.check_wait_time > 15 :
                    self.check_wait_time = 0
                    return
                else:
                    self.check_wait_time =self.check_wait_time  +1
            
            else:
                rospy.loginfo('Error: {0} does not exist'.format(current_sequence))
                self.subscriber.fnDetectionAllowed(False, False, self.layer_dist)  # fnDetectionAllowed(self, shelf_string, pallet_string)
                if self.check_wait_time > 15 :
                    self.check_wait_time = 0
                    return
                else:
                    self.check_wait_time =self.check_wait_time  +1

    def raise_pallet(self):
        current_sequence = RaisePalletSequence.init_fork.value
        previous_sequence = None  # 用來記錄上一次的階段

        while(not rospy.is_shutdown()):
            # 如果 current_sequence 發生變化，記錄 log
            if current_sequence != previous_sequence:
                rospy.loginfo('Current Sequence: {0}'.format(RaisePalletSequence(current_sequence)))
                previous_sequence = current_sequence  # 更新 previous_sequence
            self.subscriber.fnDetectionAllowed(False, False, self.layer_dist)  # fnDetectionAllowed(self, shelf_string, pallet_string)

            if(current_sequence == RaisePalletSequence.init_fork.value):
                if self.layer_dist == 1.0:
                    self.is_sequence_finished = self.Action.fnForkUpdown(self.subscriber.raise_pallet_fork_init_layer1)
                elif self.layer_dist == 2.0:
                    self.is_sequence_finished = self.Action.fnForkUpdown(self.subscriber.raise_pallet_fork_init_layer2)
                else:
                    rospy.loginfo('Layer is not defined')
                    return
                
                if self.is_sequence_finished == True:
                    current_sequence = RaisePalletSequence.dead_reckoning.value
                    feedback = str(RaisePalletSequence(current_sequence))
                    rospy.loginfo('fnseqDeadReckoning change to:{0}'.format(feedback))
                    self.is_sequence_finished = False

            elif(current_sequence == RaisePalletSequence.dead_reckoning.value):
                # rospy.loginfo('fnseqDeadReckoning: {0}'.format(self.subscriber.raise_pallet_dead_reckoning_dist))
                self.is_sequence_finished = self.Action.fnseqDeadReckoning(self.subscriber.raise_pallet_dead_reckoning_dist)
                
                if self.is_sequence_finished == True:
                    current_sequence = RaisePalletSequence.fork_updown.value
                    self.is_sequence_finished = False

            elif(current_sequence == RaisePalletSequence.fork_updown.value):
                if self.layer_dist == 1.0:
                    self.is_sequence_finished = self.Action.fnForkUpdown(self.subscriber.raise_pallet_raise_height_layer1)
                elif self.layer_dist == 2.0:
                    self.is_sequence_finished = self.Action.fnForkUpdown(self.subscriber.raise_pallet_raise_height_layer2)
                else:
                    rospy.loginfo('Layer is not defined')
                    return
                
                if self.is_sequence_finished == True:
                    current_sequence = RaisePalletSequence.back.value
                    self.is_sequence_finished = False

            elif(current_sequence == RaisePalletSequence.back.value):
                self.is_sequence_finished = self.Action.fnseqDeadReckoning(-self.subscriber.raise_pallet_back_distance)

                if self.is_sequence_finished == True:
                    current_sequence = RaisePalletSequence.stop.value
                    self.is_sequence_finished = False
            
            elif(current_sequence == RaisePalletSequence.stop.value):
                if self.check_wait_time > 15 :
                    self.check_wait_time = 0
                    return
                else:
                    self.check_wait_time =self.check_wait_time  +1
            
            else:
                rospy.loginfo('Error: {0} does not exist'.format(current_sequence))
                if self.check_wait_time > 15 :
                    self.check_wait_time = 0
                    return
                else:
                    self.check_wait_time =self.check_wait_time  +1
              
    def drop_pallet(self):
        current_sequence = DropPalletSequence.init_fork.value
        previous_sequence = None  # 用來記錄上一次的階段

        while(not rospy.is_shutdown()):
            # 如果 current_sequence 發生變化，記錄 log
            if current_sequence != previous_sequence:
                rospy.loginfo('Current Sequence: {0}'.format(DropPalletSequence(current_sequence)))
                previous_sequence = current_sequence  # 更新 previous_sequence
            self.subscriber.fnDetectionAllowed(False, False, self.layer_dist)  # fnDetectionAllowed(self, shelf_string, pallet_string)

            if(current_sequence == DropPalletSequence.init_fork.value):
                if self.layer_dist == 1.0:
                    self.is_sequence_finished = self.Action.fnForkUpdown(self.subscriber.drop_pallet_fork_init_layer1)
                elif self.layer_dist == 2.0:
                    self.is_sequence_finished = self.Action.fnForkUpdown(self.subscriber.drop_pallet_fork_init_layer2)
                else:
                    rospy.loginfo('Layer is not defined')
                    return
                
                if self.is_sequence_finished == True:
                    current_sequence = DropPalletSequence.dead_reckoning.value
                    self.is_sequence_finished = False

            elif(current_sequence == DropPalletSequence.dead_reckoning.value):
                self.is_sequence_finished = self.Action.fnseqDeadReckoning(self.subscriber.drop_pallet_dead_reckoning_dist)
                if self.is_sequence_finished == True:
                    current_sequence = DropPalletSequence.fork_updown.value
                    self.is_sequence_finished = False

            elif(current_sequence == DropPalletSequence.fork_updown.value):
                if self.layer_dist == 1.0:
                    self.is_sequence_finished = self.Action.fnForkUpdown(self.subscriber.drop_pallet_drop_height_layer1)
                elif self.layer_dist == 2.0:
                    self.is_sequence_finished = self.Action.fnForkUpdown(self.subscriber.drop_pallet_drop_height_layer2)
                else:
                    rospy.loginfo('Layer is not defined')
                    return
                
                if self.is_sequence_finished == True:
                    current_sequence = DropPalletSequence.back.value
                    self.is_sequence_finished = False

            elif(current_sequence == DropPalletSequence.back.value):
                self.is_sequence_finished = self.Action.fnseqDeadReckoning(-self.subscriber.drop_pallet_back_distance)
                if self.is_sequence_finished == True:
                    current_sequence = DropPalletSequence.stop.value
                    self.is_sequence_finished = False

            elif(current_sequence == DropPalletSequence.stop.value):
                if self.check_wait_time > 15 :
                    self.check_wait_time = 0
                    return
                else:
                    self.check_wait_time =self.check_wait_time  +1
            else:
                rospy.loginfo('Error: {0} does not exist'.format(current_sequence))
                if self.check_wait_time > 15 :
                    self.check_wait_time = 0
                    return
                else:
                    self.check_wait_time =self.check_wait_time  +1
    
    def odom_front(self):
        current_sequence = FrontSequence.Front.value
        previous_sequence = None  # 用來記錄上一次的階段

        while(not rospy.is_shutdown()):
            # 如果 current_sequence 發生變化，記錄 log
            if current_sequence != previous_sequence:
                rospy.loginfo('Current Sequence: {0}'.format(FrontSequence(current_sequence)))
                previous_sequence = current_sequence  # 更新 previous_sequence
            self.subscriber.fnDetectionAllowed(False, False, self.layer_dist)  # fnDetectionAllowed(self, shelf_string, pallet_string)

            if(current_sequence == FrontSequence.Front.value):
                self.is_sequence_finished = self.Action.fnseqDeadReckoning(-self.layer_dist)
                if self.is_sequence_finished == True:
                    current_sequence = FrontSequence.stop.value
                    self.is_sequence_finished = False

            elif(current_sequence == FrontSequence.stop.value):
                    if self.check_wait_time > 15 :
                        self.check_wait_time = 0
                        return
                    else:
                        self.check_wait_time =self.check_wait_time  +1
            else:
                rospy.loginfo('Error: {0} does not exist'.format(current_sequence))
                if self.check_wait_time > 15 :
                    self.check_wait_time = 0
                    return
                else:
                    self.check_wait_time =self.check_wait_time  +1
            
    def odom_turn(self):
        current_sequence = TurnSequence.Turn.value
        previous_sequence = None  # 用來記錄上一次的階段

        while(not rospy.is_shutdown()):
            # 如果 current_sequence 發生變化，記錄 log
            if current_sequence != previous_sequence:
                rospy.loginfo('Current Sequence: {0}'.format(TurnSequence(current_sequence)))
                previous_sequence = current_sequence  # 更新 previous_sequence
            self.subscriber.fnDetectionAllowed(False, False, self.layer_dist)  # fnDetectionAllowed(self, shelf_string, pallet_string)

            if(current_sequence == TurnSequence.Turn.value):
                self.is_sequence_finished = self.Action.fnseqDeadReckoningAngle(self.layer_dist)
                if self.is_sequence_finished == True:
                    current_sequence = TurnSequence.stop.value
                    self.is_sequence_finished = False
                
                elif(current_sequence == TurnSequence.stop.value):
                    if self.check_wait_time > 15 :
                        self.check_wait_time = 0
                        return
                    else:
                        self.check_wait_time =self.check_wait_time  +1
            else:
                rospy.loginfo('Error: {0} does not exist'.format(current_sequence))
                if self.check_wait_time > 15 :
                    self.check_wait_time = 0
                    return
                else:
                    self.check_wait_time =self.check_wait_time  +1
