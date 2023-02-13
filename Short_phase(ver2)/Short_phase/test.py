from Base_Integrate_Improve import Short_Range
from multiprocessing import Process
import RPi.GPIO as GPIO
"""
pid akirametanode [Base_Integrate_Improve] wo tukattekudasai
code wo kirei ni sitenainode nanika attara ittekudasai
Base_UAP : ultrasound(超音波)とpid制御. 
Base_Motor : 前進や後進などのモーターの動き
Base : 普通のカメラとキーボードからモーターを動かす
Base_Short_Range_pid : 物体検出やモーターをpid制御する関数

クラスの継承はShort_Range(BaseSR(Motor(UAP))) ってなっている　

"""
cansat = Short_Range()

p1 = Process(target=cansat.object_detection)
p2 = Process(target=cansat.motor_pid)
p1.start()
p2.start()
GPIO.cleanup()
print('aaaa ')

