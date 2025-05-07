# -*- coding: utf-8 -*-
import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist
from enum import Enum
from forklift_msg.msg import meteorcar
import statistics
import time
from custom_msgs.msg import CmdCutPliers  # 引入訊息格式

class Action():
    def __init__(self, Subscriber):
        # cmd_vel
        self.cmd_vel = cmd_vel(Subscriber)
        self.Subscriber = Subscriber
        self.NearbySequence = Enum('NearbySequence', 'initial_turn go_straight turn_right parking ')
        self.current_nearby_sequence = self.NearbySequence.initial_turn.value
        # fork_cmd
        self.pub_fork = self.Subscriber.pub_fork
        self.fork_msg = meteorcar()
        # Odometry_param
        self.is_odom_received = False
        self.robot_2d_pose_x = 0.0
        self.robot_2d_pose_y = 0.0
        self.robot_2d_theta = 0.0
        self.initial_robot_pose_x = 0.0
        self.initial_robot_pose_y = 0.0
        # AprilTag_param
        self.is_marker_pose_received = False
        self.marker_2d_pose_x = 0.0
        self.marker_2d_pose_y = 0.0
        self.marker_2d_pose_z = 0.0
        self.marker_2d_theta = 0.0
        self.initial_marker_pose_x = 0.0
        self.initial_marker_pose_y = 0.0
        self.initial_marker_pose_theta = 0.0
        # Fork_param
        self.forwardbackpostion = 0.0
        self.updownposition = 0.0
        self.fork_threshold = 0.005
        # other
        self.check_wait_time = 0
        self.is_triggered = False

        # 初始化 y_pose_history 和窗口大小
        self.y_pose_history = []
        self.moving_average_window = 5
        self.arm_control_pub = rospy.Publisher("/cmd_cut_pliers", CmdCutPliers, queue_size=10)
        # 用於儲存最新的手臂狀態
        self.current_arm_status = None
        # 訂閱 /arm_current_status 話題
        self.arm_status_sub = rospy.Subscriber("/arm_current_status", CmdCutPliers, self.arm_status_callback, queue_size=1)


    def SpinOnce(self):
        (self.robot_2d_pose_x, self.robot_2d_pose_y, self.robot_2d_theta, \
         self.marker_2d_pose_x, self.marker_2d_pose_y, self.marker_2d_pose_z, self.marker_2d_theta)=self.Subscriber.SpinOnce()
    
    
    def TFConfidence(self, object_name):#判斷TF是否可信
        # rospy.loginfo('shelf_detection: {0}'.format(self.Subscriber.sub_detectionConfidence.shelf_detection))
        # rospy.loginfo('shelf_confidence: {0}'.format(self.Subscriber.sub_detectionConfidence.shelf_confidence))
        # rospy.loginfo('confidence_minimum: {0}'.format(self.Subscriber.confidence_minimum))
        if object_name == "bodycamera":
            if (not self.Subscriber.sub_detectionConfidence.pallet_detection) or self.Subscriber.sub_detectionConfidence.pallet_confidence < self.Subscriber.confidence_minimum:
                self.cmd_vel.fnStop()
                return False
        elif object_name == "bodycamera":
            if (not self.Subscriber.sub_detectionConfidence.shelf_detection) or self.Subscriber.sub_detectionConfidence.shelf_confidence < self.Subscriber.confidence_minimum:
                self.cmd_vel.fnStop()
                return False
        return True

#-----------------------------------------------------------------------------------------------

    def refine_alignment(self, object_name, target_y=0.007, max_iterations=10, threshold=0.006):
        """
        當水果位於相機的左/右（以 y 軸衡量）時，對底盤做小幅微調，每次移動後檢查是否進入範圍。
        """
        Y_MIN = -0.002  # 允許的最小值
        Y_MAX = target_y  # 允許的最大值

        prev_y = None  # 用來追蹤上一個 y 值，確保有更新

        for i in range(max_iterations):
            self.SpinOnce()

            if not self.TFConfidence(object_name):
                self.cmd_vel.fnStop()
                rospy.logwarn(f"TF Data Not Confident for object '{object_name}' - Stopping")
                return False

            smoothed_y = self.compute_moving_average(self.marker_2d_pose_y)
            error = smoothed_y - target_y
            rospy.loginfo(f"[refine_alignment] Iter {i+1}, Camera Y = {smoothed_y:.6f}, Error = {error:.6f}")

            # 防止數據未更新，等姿態更新
            if prev_y is not None and abs(smoothed_y - prev_y) < 0.00001:
                rospy.logwarn("Pose not updated, waiting for new data...")
                rospy.sleep(4)
                continue

            prev_y = smoothed_y

            # 如果已經在範圍內，立即停止並返回
            if Y_MIN <= smoothed_y <= Y_MAX:
                self.cmd_vel.fnStop()
                rospy.loginfo(f"Camera Y = {smoothed_y:.6f} is within range [{Y_MIN}, {Y_MAX}], alignment complete!")
                return True

            # 不在範圍內，進行修正
            if smoothed_y > Y_MAX:
                self.cmd_vel.fnGoStraight_fruit()
                rospy.loginfo("Over threshold, moving backward to correct.")
            elif smoothed_y < Y_MIN:
                self.cmd_vel.fnGoBack()
                rospy.loginfo("Under threshold, moving forward to correct.")

            # 每次移動後立即停止，等數據更新並檢查
            move_duration = 0.05  # 進一步縮短移動時間，每次移動更小距離
            rospy.sleep(move_duration)
            self.cmd_vel.fnStop()
            rospy.loginfo("Stop, waiting for pose update...")

            # 動態等待數據更新
            prev_y_temp = smoothed_y
            timeout = 10.0  # 最多等待 10 秒
            start_time = time.time()
            while time.time() - start_time < timeout:
                self.SpinOnce()
                smoothed_y = self.compute_moving_average(self.marker_2d_pose_y)
                if abs(smoothed_y - prev_y_temp) > 0.00001:  # 數據已更新
                    break
                rospy.sleep(0.1)
            rospy.loginfo(f"Pose updated, new Y = {smoothed_y:.6f}")

            # 移動後立即檢查是否進入範圍
            if Y_MIN <= smoothed_y <= Y_MAX:
                self.cmd_vel.fnStop()
                rospy.loginfo(f"Camera Y = {smoothed_y:.6f} is within range [{Y_MIN}, {Y_MAX}] after move, alignment complete!")
                return True

        self.cmd_vel.fnStop()
        rospy.logwarn("Failed to Align Within Max Iterations")
        return False

    def blind_walk_backward(self, duration, speed=-0.2):
        """
        讓機器人以固定速度向後盲走 `duration` 秒，
        並在結束後停止，然後返回 True。
        """
        rospy.loginfo(f"🚀 開始盲走往後 {duration} 秒，速度 {speed} m/s")

        # 先停一下，避免累積舊指令
        self.cmd_vel.fnStop()
        rospy.sleep(0.1)  # 確保停止指令生效

        start_time = time.time()

        # 在 `duration` 秒內持續發送後退指令
        # 並在結束時停止機器人
        while (time.time() - start_time) < duration and not rospy.is_shutdown():
            self.cmd_vel.fnGoBack2()  # 持續發送後退指令
            rospy.sleep(0.05)         # 避免發送頻率過高

        self.cmd_vel.fnStop()
        return True


    def fnControlArm(self, height, timeout=8.0):
        """
        控制機械手臂2的高度，
        並持續檢查手臂2當前高度是否已達到指定目標，
        如果手臂2高度與目標在允許誤差範圍內則返回 True，
        否則在 timeout 時間內仍未達標則返回 False。

        :param height: 手臂2目標高度 (毫米，範圍 [0, 280])
        :param timeout: 等待超時秒數 (預設 5 秒)
        :return: 如果在 timeout 內手臂2高度與目標在允許誤差內則返回 True，否則返回 False
        """


        # 發布手臂2控制命令
        arm_cmd = CmdCutPliers()
        arm_cmd.height2 = height
        # arm_cmd.length2 = current_length  # 保持當前長度
        arm_cmd.claw2 = True  # 保持當前爪子狀態
        arm_cmd.enable_motor3 = True  # 啟用手臂2高度電機
        arm_cmd.enable_motor4 = True  # 啟用手臂2長度電機
        arm_cmd.mode = 0  # 前進模式
        self.arm_control_pub.publish(arm_cmd)


        # 持續檢查手臂2高度是否達到目標
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.current_arm_status is not None:
                current_height = abs(self.current_arm_status.height2)  # 處理可能的負值
                rospy.loginfo(
                    f"Current arm2 status: height2={current_height}, length2={self.current_arm_status.length2}, claw2={self.current_arm_status.claw2}"
                )
                if abs(current_height - height) <= 10:
                    rospy.loginfo("Arm2 reached target state.")
                    return True
            else:
                rospy.logwarn("尚未接收到手臂2狀態訊息。")
            rospy.sleep(0.1)  # 每 100ms 檢查一次
        rospy.logwarn("Timeout waiting for arm2 to reach target state.")
        return False


    def arm_status_callback(self, msg):
        """
        當收到 /arm_current_status 的消息時更新內部變數
        """
        self.current_arm_status = msg
        # rospy.loginfo("Received arm status: height1=%d, length1=%d, claw1=%s" %(msg.height1, msg.length1, str(msg.claw1)))
 


    def fnControlArmBasedOnFruitZ(self, object_name, lower_z=0.019, upper_z=0.025, timeout=10.0, increment=10, min_height=0, max_height=280, tolerance=4):
        start_time = time.time()
        while time.time() - start_time < timeout:
            self.SpinOnce()
            fruit_z = self.marker_2d_pose_z
            if self.current_arm_status is None:
                rospy.logwarn("尚未接收到手臂狀態訊息。")
                rospy.sleep(0.5)
                continue
            current_height = abs(self.current_arm_status.height2)  # 使用手臂2高度
            current_length = self.current_arm_status.length2      # 使用手臂2長度
            confidence = self.TFConfidence(object_name)
            if confidence is None or confidence < 0.5:
                rospy.logwarn(f"信心指數不足 ({confidence}), 暫停調整。")
                rospy.sleep(1.0)
                continue
            rospy.loginfo(
                f"當前水果 Z 值: {fruit_z:.4f}, 允許範圍: ({lower_z} ~ {upper_z}), 當前手臂2高度: {current_height}"
            )
            if lower_z <= fruit_z <= upper_z:
                rospy.loginfo("✅ 水果 Z 值已達標，停止調整高度。")
                return True
            if fruit_z < lower_z:
                new_height = current_height + abs(increment)
                rospy.loginfo(f"📈 水果過低，預計上升：{current_height} -> {new_height} mm")
            else:
                new_height = current_height - abs(increment)
                rospy.loginfo(f"📉 水果過高，預計下降：{current_height} -> {new_height} mm")
            new_height = max(min(new_height, max_height), min_height)
            if abs(new_height - current_height) > tolerance:
                msg = CmdCutPliers()
                msg.height2 = int(new_height)    # 設置手臂2高度
                msg.length2 = int(current_length)  # 設置手臂2長度
                msg.enable_motor3 = True    # 啟用手臂2高度電機
                msg.enable_motor4 = True    # 啟用手臂2長度電機
                msg.target_motor = 0        # 控制高度
                msg.motor_value = int(new_height)
                self.arm_control_pub.publish(msg)
                rospy.loginfo(f"✅ 發送手臂2控制指令: 高度={new_height}, 長度={current_length}")
            else:
                rospy.loginfo("高度變化小於容許誤差，避免重複發布指令。")
            reach_start = time.time()
            while time.time() - reach_start < 5:
                self.SpinOnce()
                current_height = abs(self.current_arm_status.height2)  # 檢查手臂2高度
                error = abs(current_height - new_height)
                if error <= tolerance:
                    rospy.loginfo(
                        f"✅ 手臂2調整成功：當前高度 {current_height} mm (目標 {new_height} mm，誤差 {error} mm)"
                    )
                    break
                else:
                    rospy.logwarn(
                        f"⏳ 當前高度 {current_height} mm，目標 {new_height} mm，誤差 {error} mm，等待中..."
                    )
                rospy.sleep(0.1)
            rospy.sleep(1)
        rospy.logwarn("❌ 超時：手臂2未能調整至符合目標水果 Z 範圍。")
        return False

    def fnControlArmBasedOnFruitX(self, object_name, target_x, timeout=10.0, increment=10, max_length=440):
        """
        根據水果的 x 軸數值持續調整手臂2前伸長度，
        當水果的 x 值大於 target_x 時，認為已達標停止調整，
        並保持手臂2高度不變。
        """
        start_time = time.time()
        
        if not hasattr(self, "last_valid_length2"):
            self.last_valid_length2 = 0  # 確保變數初始化

        while time.time() - start_time < timeout:
            # 更新水果 x 軸資訊
            self.SpinOnce()
            fruit_x = self.marker_2d_pose_x
            rospy.loginfo(
                f"當前水果 X 值: {fruit_x:.4f}, 目標: {target_x:.4f}, 手臂2前伸長度: {self.current_arm_status.length2}"
            )

            # 信心指數檢查
            confidence = self.TFConfidence(object_name)
            if confidence is None or confidence < 0.5:
                rospy.logwarn(f"信心指數不足 ({confidence}), 停止手臂2前伸。")
                return False

            # 若水果 x 值已達目標，則返回 True
            if fruit_x >= target_x:
                rospy.loginfo("水果 X 值已達標，停止手臂2前伸。")
                return True

            # 確保 `length2` 只增不減
            current_length = self.current_arm_status.length2

            # 如果 `length2=0`，使用上次有效值
            if current_length == 0:
                rospy.logwarn(f"⚠ `length2=0`，忽略此數值，保持 {self.last_valid_length2} mm")
                current_length = self.last_valid_length2
            elif current_length < self.last_valid_length2:
                rospy.logwarn(f"⚠ `length2` 變小 ({current_length} mm)，恢復到 {self.last_valid_length2} mm")
                current_length = self.last_valid_length2
            else:
                self.last_valid_length2 = current_length  # 記錄最後一次的有效長度

            # 設定新的目標長度
            target_length = min(current_length + increment, max_length)
            rospy.loginfo(f"嘗試手臂2前伸: {target_length} mm")

            # 發送控制命令
            msg = CmdCutPliers()
            msg.height2 = int(self.current_arm_status.height2)  # 保持當前手臂2高度
            msg.length2 = int(target_length)  # 設定手臂2前伸長度
            msg.enable_motor3 = True  # 啟用手臂2高度電機
            msg.enable_motor4 = True  # 啟用手臂2長度電機
            msg.target_motor = 1  # 控制長度
            msg.motor_value = int(target_length)  # 設定馬達值

            self.arm_control_pub.publish(msg)

            # 等待手臂2到達目標長度
            rospy.loginfo(f"等待手臂2到達長度: {target_length} mm")
            reach_start_time = time.time()
            while time.time() - reach_start_time < 5:  # 最多等待 5 秒
                self.SpinOnce()
                current_length = self.current_arm_status.length2

                # 確保 `length2` 只增加
                if current_length == 0:
                    rospy.logwarn(f"⚠ `length2=0`，忽略此數值，保持 {self.last_valid_length2} mm")
                    current_length = self.last_valid_length2
                elif current_length < self.last_valid_length2:
                    rospy.logwarn(f"⚠ `length2` 變小 ({current_length} mm)，恢復到 {self.last_valid_length2} mm")
                    current_length = self.last_valid_length2
                else:
                    self.last_valid_length2 = current_length  # 更新最後一次的有效長度

                if abs(current_length - target_length) <= 10:  # 允許 10mm 誤差
                    rospy.loginfo(f"✅ 手臂2已到達目標長度 {current_length} mm")
                    break
                else:
                    rospy.logwarn(f"⏳ 手臂2目前長度 {current_length} mm，目標 {target_length} mm，等待中...")
                    rospy.sleep(0.5)  # 每 500ms 檢查一次

            rospy.sleep(1)  # 延長等待時間，確保 `length2` 更新穩定

        rospy.logwarn("Timeout: 手臂2未能達到目標 X 值。")
        return False



    def fnBlindExtendArm(self, extra_length, max_length=440, timeout=7.0):
        """
        盲伸手臂2：在當前長度的基礎上，額外前伸 extra_length。
        
        :param extra_length: 需要額外前伸的距離（單位 mm）
        :param max_length: 最大可伸長度，避免超出限制（預設 440 mm）
        :param timeout: 等待手臂2到達目標長度的最大時間（秒）
        :return: True 若手臂2成功到達目標，False 若超時或發生錯誤
        """
        if hasattr(self, "blind_extend_completed") and self.blind_extend_completed:
            rospy.logwarn("⚠ `fnBlindExtendArm()` 已執行過，跳過此次呼叫")
            return False  # 防止再次執行

        start_time = time.time()

        # 取得當前手臂2長度
        current_length = self.current_arm_status.length2
        if current_length is None:
            rospy.logerr("❌ 無法獲取當前手臂2長度，盲伸失敗")
            return False

        self.last_valid_length2 = current_length  # 更新最後一次的有效長度

        # 設定目標長度
        target_length = min(current_length + extra_length, max_length)
        rospy.loginfo(f"🔵 手臂2盲伸: 當前長度={current_length} mm, 目標長度={target_length} mm")

        # 發送控制指令
        msg = CmdCutPliers()
        msg.height2 = self.current_arm_status.height2  # 保持當前手臂2高度
        msg.length2 = target_length  # 設定手臂2新的前伸長度
        msg.enable_motor3 = True  # 啟用手臂2高度電機
        msg.enable_motor4 = True  # 啟用手臂2長度電機
        msg.target_motor = 1  # 控制長度
        msg.motor_value = target_length

        self.arm_control_pub.publish(msg)

        # 等待手臂2到達目標長度
        rospy.loginfo(f"⏳ 等待手臂2到達長度 {target_length} mm")
        while time.time() - start_time < timeout:
            self.SpinOnce()
            current_length = self.current_arm_status.length2

            self.last_valid_length2 = current_length  # 更新最後一次的有效長度

            if abs(current_length - target_length) <= 10:  # 允許 10 mm 誤差
                rospy.loginfo(f"✅ 手臂2已成功盲伸至 {current_length} mm")
                self.blind_extend_completed = True
                return True

            rospy.sleep(0.5)

        rospy.logerr(f"⏰ 手臂2盲伸超時: 目標 {target_length} mm 未達成，當前 {current_length} mm")
        return False



    def fnControlClaw(self, claw_state, timeout=3):
        """
        控制手臂2剪鉗的開合 (claw2)，並等待其完成

        :param claw_state: True = 閉合剪鉗, False = 打開剪鉗
        :param timeout: 等待剪鉗動作完成的最大時間 (秒)
        :return: True 若剪鉗成功執行, False 若超時或發生錯誤
        """
        start_time = time.time()

        # 確保 claw_state 為 bool
        claw_state = bool(claw_state)

        # 發送剪鉗控制指令
        msg = CmdCutPliers()
        msg.height2 = self.current_arm_status.height2  # 保持當前手臂2高度
        msg.length2 = self.current_arm_status.length2  # 保持當前手臂2長度
        msg.claw2 = claw_state  # 確保為 bool
        msg.enable_motor3 = True  # 啟用手臂2高度電機
        msg.enable_motor4 = True  # 啟用手臂2長度電機

        self.arm_control_pub.publish(msg)
        rospy.loginfo(f"✂ 手臂2剪鉗指令發送: {'閉合' if claw_state else '打開'}")

        # 等待剪鉗狀態變更，達到目標狀態後等待2秒再返回True
        while time.time() - start_time < timeout:
            self.SpinOnce()  # 處理 ROS 回傳的狀態
            if self.current_arm_status.claw2 == claw_state:
                rospy.loginfo(f"✅ 手臂2剪鉗 {'閉合' if claw_state else '打開'} 成功，等待2秒以穩定狀態...")
                rospy.sleep(2)  # 等待2秒
                return True
            rospy.logwarn(f"⏳ 手臂2剪鉗動作中... 目標: {claw_state}, 當前: {self.current_arm_status.claw2}")
            rospy.sleep(0.1)
        
        rospy.logerr(f"⏰ 手臂2剪鉗動作超時: 目標 {claw_state}, 當前 {self.current_arm_status.claw2}")
        return False



    def fnRetractArm(self, target_length_2, timeout=12.0):
        """
        縮回手臂2至指定長度

        :param target_length_2: 目標縮回長度（單位 mm）
        :param timeout: 等待手臂2縮回的最大時間（秒）
        :return: True 若手臂2成功縮回，False 若超時或發生錯誤
        """
        if hasattr(self, "retract_executed") and self.retract_executed:
            rospy.logwarn("⚠ 手臂2已執行過後退，忽略此次請求")
            return False

        rospy.loginfo(f"📢 正在執行 fnRetractArm(), 手臂2目標長度: {target_length_2}")

        start_time = time.time()
        current_length = self.current_arm_status.length2

        if current_length is None:
            rospy.logerr("❌ 無法獲取當前手臂2長度，後退失敗")
            return False

        if target_length_2 > current_length:
            rospy.logwarn(f"⚠ 目標長度 {target_length_2} mm 大於當前手臂2長度 {current_length} mm，忽略請求")
            return False

        # 設定為已執行後退
        self.retract_executed = True

        # 發送後退訊息
        msg = CmdCutPliers()
        msg.height2 = int(self.current_arm_status.height2)
        msg.length2 = int(target_length_2)
        msg.claw2 = int(self.current_arm_status.claw2)
        msg.enable_motor3 = True  # 啟用手臂2高度電機
        msg.enable_motor4 = True  # 啟用手臂2長度電機
        msg.mode = 1  # 後退模式
        
        self.arm_control_pub.publish(msg)
        rospy.loginfo(f"🔵 已發送手臂2後退指令: {msg}")

        while time.time() - start_time < timeout:
            self.SpinOnce()
            current_length = self.current_arm_status.length2

            if abs(current_length - target_length_2) <= 10:
                rospy.loginfo(f"✅ 手臂2已成功縮回至 {current_length} mm")
                return True

            rospy.logwarn(f"⏳ 手臂2目前長度 {current_length} mm，目標 {target_length_2} mm，等待中...")
            rospy.sleep(0.5)

        rospy.logerr(f"⏰ 手臂2後退超時: 目標 {target_length_2} mm 未達成，當前 {current_length} mm")
        return False


    def compute_moving_average(self, new_value):
        """
        計算滑動平均值。
        """
        # 將新數值加入歷史紀錄
        self.y_pose_history.append(new_value)

        # 若歷史數據超過窗口大小，移除最舊數據
        if len(self.y_pose_history) > self.moving_average_window:
            self.y_pose_history.pop(0)

        # 計算平均值
        return sum(self.y_pose_history) / len(self.y_pose_history)



class cmd_vel():
    def __init__(self, Subscriber):
        self.Subscriber = Subscriber
        self.pub_cmd_vel = self.Subscriber.pub_cmd_vel
        self.front = False

    def cmd_pub(self, twist):
        if not self.front:
            twist.linear.x = -twist.linear.x

        if twist.angular.z > 0.2:
            twist.angular.z =0.2
        elif twist.angular.z < -0.2:
            twist.angular.z =-0.2
        if twist.linear.x > 0 and twist.linear.x < 0.02:
            twist.linear.x =0.02
        elif twist.linear.x < 0 and twist.linear.x > -0.02:
            twist.linear.x =-0.02  

        if twist.linear.x > 0.2:
            twist.linear.x =0.2
        elif twist.linear.x < -0.2:
            twist.linear.x =-0.2                     
        if twist.angular.z > 0 and twist.angular.z < 0.05:
            twist.angular.z =0.05
        elif twist.angular.z < 0 and twist.angular.z > -0.05:
            twist.angular.z =-0.05
        self.pub_cmd_vel.publish(twist)

    def fnStop(self):
        twist = Twist()
        self.cmd_pub(twist)

    def fnTurn(self, Kp=0.2, theta=0.):
        # Kp = 0.3 #1.0
        twist = Twist()
        twist.angular.z = Kp * theta
        self.cmd_pub(twist)

    def fnGoStraight(self, Kp=0.2, v=0.):
        twist = Twist()
        twist.linear.x = Kp * v

        self.cmd_pub(twist)

    def fnGoBack(self):
        twist = Twist()
        twist.linear.x = -0.02
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0

        self.cmd_pub(twist)



    def fnGoStraight_fruit(self):      #控制叉車前進
        twist = Twist()
        twist.linear.x = 0.01
        self.cmd_pub(twist)

  
    def fnGoBack2(self):      #控制叉車前進
        twist = Twist()
        twist.linear.x = -0.08
        self.cmd_pub(twist)