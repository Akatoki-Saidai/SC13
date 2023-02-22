from lzma import PRESET_DEFAULT
from micropyGPS import MicropyGPS
import pigpio
from math import *
import cv2
import numpy as np
import RPi.GPIO as GPIO #GPIOインポート
import time #時間制御インポート
import threading
import csv
import atexit
import smbus
import datetime
import sys
from Adafruit_BNO055 import BNO055
import logging
import subprocess
​
#bnosetup
bno = BNO055.BNO055(rst=18)
if len(sys.argv) == 2 and sys.argv[1].lower() == '-v':# パラメータとして-vが渡された場合、冗長なデバッグロギングを有効にする
    logging.basicConfig(level=logging.DEBUG)
if not bno.begin():# BNO055を初期化し、問題が発生した場合は停止する
    raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')
status, self_test, error = bno.get_system_status()# システムの状態やセルフテストの結果を表示する
print('System status: {0}'.format(status))
print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
if status == 0x01:# システムステータスがエラーモードの場合、エラーを表示する
    print('System error: {0}'.format(error))
    print('See datasheet section 4.3.59 for the meaning.')
sw, bl, accel, mag, gyro = bno.get_revision()# Print BNO055 software revision and other diagnostic data.
print('Software version:   {0}'.format(sw))
print('Bootloader version: {0}'.format(bl))
print('Accelerometer ID:   0x{0:02X}'.format(accel))
print('Magnetometer ID:    0x{0:02X}'.format(mag))
print('Gyroscope ID:       0x{0:02X}\n'.format(gyro))
​
goal_heading = 270
phase = 1
while True:
    heading, roll, pitch = bno.read_euler()# Read the Euler angles for heading, roll, pitch (all in degrees).
    sys, gyro, accel, mag = bno.get_calibration_status() # Read the calibration status, 0=uncalibrated and 3=fully calibrated.
    qx,qy,qz,qw = bno.read_quaternion()        # Orientation as a quaternion:
    temp_c = bno.read_temp()        # Sensor temperature in degrees Celsius:
    mx,my,mz = bno.read_magnetometer()        # Magnetometer data (in micro-Teslas):
    Gx,Gy,Gz = bno.read_gyroscope()        # Gyroscope data (in degrees per second):
    ax,ay,az = bno.read_accelerometer()        # Accelerometer data (in meters per second squared):
    lx,ly,lz = bno.read_linear_acceleration()        # Linear acceleration data (i.e. acceleration from movement, not gravity--returned in meters per second squared):
    gx,gy,gz = bno.read_gravity()        # Gravity acceleration data (i.e. acceleration just from gravity--returned in meters per second squared):
​
    if phase == 1: #待機フェーズ
        if ay**2 < 200: #条件は後で変更
            time.sleep(0.01)
            #log_data.writerow([now,'im waiting'])
            print(ay**2,'im waiting' )
        elif  ay**2 > 200:
            #log_data.writerow([now,'im falling'])
            print(ay**2,'im falling')
            phase += 1
        #elif :#計測開始から450秒（7分30秒）経ったら走行フェーズ開始 （注）もし加速度センサーがちゃんと機能しなかった時の保険で使うのはありだと思う
            #phase += 1
    if phase == 2:#落下フェーズ（注）ここは今回の条件に変更した方がいい
        servo_angle(-40)
        Servo.stop()
        CCW()
        if goal_heading - heading < 1:
            stop()
            phase += 1
    if phase == 3:
        subprocess.run(['python' , 'e2e_improved.py'])#ここは切り替えるプログラム(遠距離や近距離)のファイル名を書く
        break
