from Base_Integrate import Short_Range
import threading

"""

Base_UAP : ultrasound(超音波)とpid制御. 
Base_Motor : 前進や後進などのモーターの動き
Base : 普通のカメラとキーボードからモーターを動かす
Base_Short_Range_pid : 物体検出やモーターをpid制御する関数

クラスの継承はShort_Range(BaseSR(Motor(UAP))) ってなっている　

"""

cansat = Short_Range()

thread_object_detection = threading.Thread(target=cansat.object_detection())
thread_motor = threading.Thread(target=cansat.motor())
thread_object_detection.start()
time.sleep(3)
time.sleep(1)
thread_motor.start()