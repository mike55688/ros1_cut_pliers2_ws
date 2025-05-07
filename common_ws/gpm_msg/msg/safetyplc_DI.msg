bool IsPLCAlive
# a->b b接 常ON, b->a a接 常OFF
# S04
bool manual_auto_AUTO_s04_I1 # 手自動, 自動:ON, 手動:OFF
bool reset_PUSH_s04_I2 # 復位按鈕 按下: true
#bool rightforkendcheck_a2b_s04_I3
#bool rightforkendcheck_b2a_s04_I4
bool rightforkendcheck_s04_I3I4 # 由 s04_I3(b), s04_I3(a)共同判斷 (clear:on)
#bool leftforkendcheck_a2b_s04_I5
#bool leftforkendcheck_b2a_s04_I6
bool leftforkendcheck_s04_I5I6 # 由 s04_I5(b), s04_I6(a)共同判斷 (clear:on)
bool bumper_status_Normal_s04_I7I8 # 由 s04_I7(b), s04_I8(a)共同判斷
# S05
bool right_distance_sensor_status_s05_I3 # 
bool left_distance_sensor_status_s05_I4 #
bool EMONC1_NOTTRIGGER_s05_I5
bool EMONC2_NOTTRIGGER_s05_I6
bool relayfeedback_POWEROFF_s05_I7 #暫時抓I8訊號
# S06
bool back_2D_ready
bool back_2D_stop
bool back_2D_low2 
bool back_2D_low1 
bool Tile_Up_Position_s06_I5 
bool Tile_Mid_Position_s06_I6
bool Tile_Down_Position_s06_I7
# S07
bool front_2D_ready
bool front_2D_stop
bool front_2D_low2 
bool front_2D_low1 
bool Fork_Up_Position_NOTTRIGGER_s07_I5
bool Fork_Down_Position_NOTTRIGGER_s07_I6
bool Fork_Front_Position_NOTTRIGGER_s07_I7
bool Fork_Rear_Position_NOTTRIGGER_s07_I8
# S08
bool right_2D_ready
bool right_2D_stop
bool right_2D_low2 
bool right_2D_low1
# S09
bool left_2D_ready
bool left_2D_stop
bool left_2D_low2 
bool left_2D_low1
bool rightonposdetect_a2b_s10_I1
bool leftonposdetect_a2b_s10_I2
