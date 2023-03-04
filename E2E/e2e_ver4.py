global now_lon 
global now_lat 
#とりあえず分離フェーズからの奴を作る．
#現在実装ずみ＝＝＞（motor，９軸センサ，GPS，サーボ）
#02/26に全部の部品の個別の動作は確認．
"""
///////////////////////////////////////////////////////////////////
/   プログラムを実行するときはファイルを以下のように置いてください        /
/   [フォルダ]                                                     /          
/       ->本プログラム                                              /
/       ->佐々木のshortphasever2/shortphaseの中身全部をコピペ        /   
///////////////////////////////////////////////////////////////////

本プログラムの構成

処理用モジュール宣言
データ保存ファイル宣言，ファイル開く
モータ   
    モジュール，初期値宣言
    関数
        モータ前進，回転，後進，停止
サーボ
    モジュール，初期値宣言
    関数    サーボ回転
９軸センサー
    モジュール，初期値宣言            
    関数
        セットアップ=>BNO55_setup()
        加速度の大きさを求める関数=>BNO055_Accel_strength()
        北からの時計回りの角度を求める関数=>calculate_north_degree()    
GPS
    モジュール，初期値宣言
    関数
        緯度と経度を返す関数=>GPS_lat_lon()
        距離と機体正面からの時計回り方位を求める関数=>calc_distance_azimuth(naw_lat, naw_lon)
BME280
    モジュール，初期値宣言
    関数
        気温と気圧から高度を推定する関数calculate_altitude(pressure,temperature)
その他の関数
    機体とゴールの角度を計算

処理
1待機フェーズ
    ９軸センサと空気室センサで地面に落ちるまで待機しつつ，データを保存
2落下フェーズ
    なし！
3分離フェーズ
    特に分離を確認する手段がないのでサーボとモーターを一回起動
    サーボーでパラ分離
    モーターでちょっと前進
4長距離フェーズ
    9軸センサとGPSから機体とゴールの角度を求める
    角度におおじてモーターを動かす
    距離が10m以下になったら近距離フェーズに移行
5近距離フェーズ
    佐々木のプログラムをモジュールインポートしただけ！！


"""

#<処理用モジュール--------------------------------------------------->
    #目標値の緯度経度を
goal_lat = 30.376034
goal_lon = 130.95221
global PRESSURE
PRESSURE  =  0
ACCELL = 0
process = 0

#<データ保存ファイル------------------------------------------------->
import csv

f_BNO055 = open('BNO055_log.csv','w',encoding="utf-8")
f_GPS = open('GPS_log.csv','w',encoding="utf-8")
f_BME280 = open('BME280.csv','w',encoding="utf-8")
	#CSV形式の保存指定
# file_BNO055 = csv.writer(f_BNO055,delimiter=",")
# file_GPS = csv.writer(f_GPS,delimiter=",")
# file_BME280= csv.writer(f_BME280,delimiterf_.write('time'+','+"yaw"+','+'roll'+','+'pitch'+'\n')
f_BNO055.write('time'+','+"yaw"+','+'roll'+','+'pitch'+'\n')
f_GPS.write('time'+','+"lat"+','+'lon'+'\n')
f_BME280.write('time'+','+"temp"+','+'press'+','+'\n')
#<モーター----------------------------------------------------------->
#<<motorモジュール，グローバル変数>>
import RPi.GPIO as GPIO #GPIOインポート
GPIO.setwarnings(False)

AIN1 = 15
AIN2 = 29
BIN1 = 31
BIN2 = 33
GPIO.cleanup()#とりあえずGPIOを初期化
GPIO.setmode(GPIO.BOARD)    #物理ピン番号でGPIOを指定
GPIO.setup(AIN1,GPIO.OUT)   #AIN1番ピンで
GPIO.setup(AIN2,GPIO.OUT)   #AIN2ピンで
GPIO.setup(BIN1,GPIO.OUT)   #BIN1ピンで
GPIO.setup(BIN2,GPIO.OUT)   #BIN2ピンでGPIO出力設定

Apwm1 = GPIO.PWM(AIN1,50)
Bpwm1 = GPIO.PWM(BIN1,50)
Apwm2 = GPIO.PWM(AIN2,50)     #PWM(pin,Hz)→AIN2番ピンで50Hz
Bpwm2 = GPIO.PWM(BIN2,50)     #PWM(pin,Hz)→BIN2番ピンで50Hz出力設定

duty1 = 100
duty2 = 100	#回転速の比率とりあえず１００％
    
Apwm1.start(0)
Apwm2.start(0)
Bpwm1.start(0)
Bpwm2.start(0)

#<<モーター関数>>
def accel():#分離フェーズ用微移動(10秒移動？？)
    for i in range(5):
        Apwm1.ChangeDutyCycle(20*i)
        Apwm2.ChangeDutyCycle(0)   
        Bpwm1.ChangeDutyCycle(20*1) 
        Bpwm2.ChangeDutyCycle(0)   
        time.sleep(1)  
        
    stop()

def forward():        #前進
    #右車輪(A1,A2)=(+,0)→正回転
    Apwm1.ChangeDutyCycle(duty1)
    Apwm2.ChangeDutyCycle(0)   
    Bpwm1.ChangeDutyCycle(duty1)
    Bpwm2.ChangeDutyCycle(0)
    time.sleep(0.5)
    stop()
    time.sleep(1)
	   
    
def turn_right():    #右回転
    #右車輪(AIN1,AIN2)=(0,+)→負回転
    #右車輪(AIN1,AIN2)=(+,0)→正回転
    Apwm1.ChangeDutyCycle(0)
    Apwm2.ChangeDutyCycle(duty2)   
    Bpwm1.ChangeDutyCycle(duty2)
    Bpwm2.ChangeDutyCycle(0)
    time.sleep(0.5)
    stop()
    time.sleep(1)
           
    
def turn_left():     #左回転
    #右車輪(AIN1.AIN2)=(+,0)→正回転
    #左車輪(BIN1.BIN2)=(0,+)→負回転
    Apwm1.ChangeDutyCycle(duty2)
    Apwm2.ChangeDutyCycle(0)   
    Bpwm1.ChangeDutyCycle(0)
    Bpwm2.ChangeDutyCycle(duty2)
    time.sleep(1)
    stop()
    time.sleep(1)

def back():          #後進
    #左車輪(AIN1.AIN2)=(0,+)→負回転
    #左車輪(BIN1.BIN2)=(+,0)→負回転
    Apwm1.ChangeDutyCycle(0)
    Apwm2.ChangeDutyCycle(duty2)   
    Bpwm1.ChangeDutyCycle(0)
    Bpwm2.ChangeDutyCycle(duty2)
    time.sleep(0.5)
    stop()
    time.sleep(1)
    
def stop():           #停止
    Apwm1.ChangeDutyCycle(0)
    Apwm2.ChangeDutyCycle(0)   
    Bpwm1.ChangeDutyCycle(0)
    Bpwm2.ChangeDutyCycle(0)


#<サーボモーター-------------------------------------------------------->
#<<サーボモーターモジュール，グローバル変数>>
import time
import RPi.GPIO as GPIO
import sys  # 引数取得

# PIN_servo = 11   # for servo
#     #GPIO.setmode(GPIO.BOARD)      # GPIOへアクセスする番号をBCM番号で指定することを宣言
# GPIO.setup(PIN_servo, GPIO.OUT)     # 出力ピンに設定
# 	# servo setup
# servo = GPIO.PWM(PIN_servo, 50)
# servo.start(0)

#<<サーボモーターの関数>>
def servo():
    INTERVAL = 0.6
    PIN = 12
    FREQ = 50

    #GPIO.setmode(GPIO.BOARD)

    GPIO.setup(PIN, GPIO.OUT)
    servo = GPIO.PWM(PIN, FREQ)

    #init
    servo.start(0.0)

    for i in range(1):
      servo.ChangeDutyCycle(2.5)
      time.sleep(INTERVAL)

      servo.ChangeDutyCycle(12.0)
      time.sleep(INTERVAL)
      
#<9軸センサー------------------------------------------------------->
#<<BNO055モジュール，グローバル変数>>
import logging
import sys
import time
import numpy as np
import math
import datetime

from Adafruit_BNO055 import BNO055

bno = BNO055.BNO055()

#<<BNO055の関数>>
def BNO55_setup():
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
    
    #9軸センサーの加速度の大きさを求める関数
def BNO055_Accel_strength():
    yaw, roll, pitch = bno.read_euler()# Read the Euler angles for heading, roll, pitch (all in degrees).
    sys, gyro, accel, mag = bno.get_calibration_status() # Read the calibration status, 0=uncalibrated and 3=fully calibrated.
    # qx,qy,qz,qw = bno.read_quaternion()        # Orientation as a quaternion:
    # #temp_c = bno.read_temp()        # Sensor temperature in degrees Celsius:
    # mx,my,mz = bno.read_magnetometer()        # Magnetometer data (in micro-Teslas):
    # Gx,Gy,Gz = bno.read_gyroscope()        # Gyroscope data (in degrees per second):
    #ax,ay,az = bno.read_accelerometer()        # Accelerometer data (in meters per second squared):
    lx,ly,lz = bno.read_linear_acceleration()        # Linear acceleration data (i.e. acceleration from movement, not gravity--returned in meters per second squared):
    # gx,gy,gz = bno.read_gravity()        # Gravity acceleration data (i.e. acceleration just from gravity--returned in meters per second squared):
    #以下で三軸を表示
    f_BNO055.write('time'+','+ str(lx)+','+ str(ly)+','+ str(lz)+'\n')
    acccel_sum = np.sqrt(lx**2 + ly**2 +lz**2)
    print("accel_sum:",acccel_sum)

    return acccel_sum #条件分岐用の戻り値

    #BNP055＿北からの時計回りに３６０度で角度を計算する関数
def calculate_north_degree():

    mx,my,mz = bno.read_magnetometer()        # 三次元磁力密度
    # Calculate magnetic strength and direction.
    magnitude = math.sqrt(mx * mx + my * my + mz * mz)
    if my > 0:
        direction = math.atan2(mx, my) * 180 / math.pi
    elif my < 0:
        direction = (math.atan2(mx, my) + 2 * math.pi) * 180 / math.pi
    else:
        if mx < 0:
            direction = 270.0
        else:
            direction = 90.0
    # Convert direction to heading.
    heading = 360 - direction +90
    if heading >= 360:
        heading -= 360
    if heading < 0:
        heading += 360
        
    print("north_degree",heading)
    return int(heading)

#<GPS------------------------------------------------------->
#<<GPS初期値モジュール，グローバル変数>>
import matplotlib.pyplot as plt
#import cartopy.io.img_tiles as cimgt
import time
import pigpio
from micropyGPS import MicropyGPS
from math import radians, sin, cos, tan, atan2

pole_radius = 6356752.0    #極半径
equator_radius = 6378137.0    #赤道半径

	# シリアル通信設定
baudrate = 9600
	#通信設定でいじるとしたらここのTX,RXだけど、ピン配置変えたい場合以外はいじらなくてok
TX = 14
RX = 15

serialpi = pigpio.pi()
serialpi.set_mode(RX,pigpio.INPUT)
serialpi.set_mode(TX,pigpio.OUTPUT)

pigpio.exceptions = False
serialpi.bb_serial_read_close(RX)
pigpio.exceptions = True

serialpi.bb_serial_read_open(RX,baudrate,8)
	# gps設定
	#my_gpsにデータが格納される感じ
my_gps = MicropyGPS(9, 'dd') # 引数はタイムゾーンの時差と出力フォーマット

	# 10秒ごとに表示
	#なんか10秒ごとに表示されてない気がするのでいじってみてほしい
tm_last = 0
count = 0
	# Create stamen terrain background instance
	#request = cimgt.GoogleTiles()
fig = plt.figure()

#<<GPSの関数>>

    #緯度と経度を返すかんすう
def GPS_lat_lon():
    (count, sentence) = serialpi.bb_serial_read(RX)#ここはよくわからん、データが取れてるかどうか調べるところな気がする
    if len(sentence) > 0:
        for x in sentence:
            if 10 <= x <= 126:
                stat = my_gps.update(chr(x))
                if stat:
                    tm = my_gps.timestamp
                    tm_now = (tm[0] * 3600) + (tm[1] * 60) + int(tm[2])


                    # Create a GeoAxes in hte tile's projection
                    #m1 = fig.add_subplot(111, projection=request.crs) #画面の領域が描写される　111の意味は、1行目1列の1番目という意味　参考サイト　https://qiita.com/kenichiro_nishioka/items/8e307e164a4e0a279734

                    # Set map extent to +- 0.01º of the received position
                    #m1.set_extent([my_gps.longitude[0] + 0.01, my_gps.longitude[0] - 0.01, my_gps.latitude[0] + 0.01, my_gps.latitude[0] - 0.01]) #set_extent([西端の経度, 東端の経度, 南端の緯度, 北端の緯度 0.01のところを変えると描写されるマップの広さを変えれる

                    # Get image at desired zoom
                    #m1.add_image(request, 14) ##画像を指定するセルにはる　ここだとグーグルマップのタイルを貼っている　右の値はマップの拡大度合い
                    # Print data on console
                    #print('=' * 20)
                    #print(my_gps.date_string(), tm[0], tm[1], int(tm[2]))
                    #print("latitude:", my_gps.latitude[0], ", longitude:", my_gps.longitude[0])    
                    #time.sleep(1)#一秒停止

        # Plot the new position
                    #m1.plot(my_gps.longitude[0], my_gps.latitude[0], '.', transform=ccrs.Geodetic())
                    #plt.pause(0.1)
                    global now_lat
                    global now_lon
                    now_lat = my_gps.latitude[0]
                    now_lon = my_gps.longitude[0]
                    #print("GPS;",now_lat, now_lon)
                    f_GPS.write('time'+',' + str(now_lat)+','+str(now_lon)+'\n')
                    

    #<現在の緯度，経度から目標値までの距離，方位を計算>
def calculate_distance(naw_lat, naw_lon):

         # 緯度経度をラジアンに変換
    goal_lat_rad, goal_lon_rad, naw_lat_rad, naw_lon_rad = map(radians, [goal_lat, goal_lon, naw_lat, naw_lon])

    d_lat = goal_lat_rad - naw_lat_rad      #緯度差
    d_lon = goal_lon_rad - naw_lon_rad       #経度差
    lat_ave = (goal_lat_rad + naw_lat_rad) / 2    #平均緯度

            #距離計算_Hubeny
    E2 = (math.pow(equator_radius, 2) - math.pow(pole_radius, 2)) / math.pow(equator_radius, 2)     #離心率^2
    W = math.sqrt(1- E2 * math.pow(math.sin(lat_ave), 2))
    M = (equator_radius * (1 - E2)) / math.pow(W, 3)    #子午線曲率半径
    N = equator_radius / W    #卯酉線曲半径
    
    distance = math.sqrt(math.pow(M * d_lat, 2) + math.pow(N * d_lon * math.cos(lat_ave), 2))    #距離計測(m)
    print("distance;",distance)

    return distance
  
def calculate_azimuth(naw_lat, naw_lon):
    goal_lat_rad, goal_lon_rad, naw_lat_rad, naw_lon_rad = map(radians, [goal_lat, goal_lon, naw_lat, naw_lon])
            #方位角計算
    d_lon = goal_lon_rad - naw_lon_rad       #経度差
    x = math.cos(naw_lat_rad) * math.sin(goal_lat_rad) - math.sin(naw_lat_rad) * math.cos(goal_lat_rad) * math.cos(d_lon)
    y = math.sin(d_lon) * math.cos(goal_lat_rad)

    brng = math.degrees(math.atan2(y, x))
    azimuth = (brng + 360) % 360
    print("GPS_degree",azimuth)
    
    return  azimuth    

#<BME280空気質センサ-------------------------------------->
#<<BME初期値モジュール，グローバル変数>>
import smbus

bus_number  = 1
i2c_address = 0x76

bus = smbus.SMBus(bus_number)

digT = []
digP = []
digH = []

t_fine = 0.0
STANDARD_PRESSURE = 1013.25  # 標準大気圧（ヘクトパスカル）


#<<BME280 関数>>
def BME280_get_calib_param():
	calib = []
	
	for i in range (0x88,0x88+24):
		calib.append(bus.read_byte_data(i2c_address,i))
	calib.append(bus.read_byte_data(i2c_address,0xA1))
	for i in range (0xE1,0xE1+7):
		calib.append(bus.read_byte_data(i2c_address,i))

	digT.append((calib[1] << 8) | calib[0])
	digT.append((calib[3] << 8) | calib[2])
	digT.append((calib[5] << 8) | calib[4])
	digP.append((calib[7] << 8) | calib[6])
	digP.append((calib[9] << 8) | calib[8])
	digP.append((calib[11]<< 8) | calib[10])
	digP.append((calib[13]<< 8) | calib[12])
	digP.append((calib[15]<< 8) | calib[14])
	digP.append((calib[17]<< 8) | calib[16])
	digP.append((calib[19]<< 8) | calib[18])
	digP.append((calib[21]<< 8) | calib[20])
	digP.append((calib[23]<< 8) | calib[22])
	digH.append( calib[24] )
	digH.append((calib[26]<< 8) | calib[25])
	digH.append( calib[27] )
	digH.append((calib[28]<< 4) | (0x0F & calib[29]))
	digH.append((calib[30]<< 4) | ((calib[29] >> 4) & 0x0F))
	digH.append( calib[31] )
	
	for i in range(1,2):
		if digT[i] & 0x8000:
			digT[i] = (-digT[i] ^ 0xFFFF) + 1

	for i in range(1,8):
		if digP[i] & 0x8000:
			digP[i] = (-digP[i] ^ 0xFFFF) + 1

	for i in range(0,6):
		if digH[i] & 0x8000:
			digH[i] = (-digH[i] ^ 0xFFFF) + 1  

def compensate_P(adc_P):
	global  t_fine
	pressure = 0.0
	
	v1 = (t_fine / 2.0) - 64000.0
	v2 = (((v1 / 4.0) * (v1 / 4.0)) / 2048) * digP[5]
	v2 = v2 + ((v1 * digP[4]) * 2.0)
	v2 = (v2 / 4.0) + (digP[3] * 65536.0)
	v1 = (((digP[2] * (((v1 / 4.0) * (v1 / 4.0)) / 8192)) / 8)  + ((digP[1] * v1) / 2.0)) / 262144
	v1 = ((32768 + v1) * digP[0]) / 32768

	if v1 == 0:
		return 0
	pressure = ((1048576 - adc_P) - (v2 / 4096)) * 3125
	if pressure < 0x80000000:
		pressure = (pressure * 2.0) / v1
	else:
		pressure = (pressure / v1) * 2
	v1 = (digP[8] * (((pressure / 8.0) * (pressure / 8.0)) / 8192.0)) / 4096
	v2 = ((pressure / 4.0) * digP[7]) / 8192.0
	pressure = pressure + ((v1 + v2 + digP[6]) / 16.0)  

	#print "pressure : %7.2f hPa" % (pressure/100)
	return (pressure/100)#hpaの気圧を返す

def compensate_T(adc_T):
	global t_fine
	v1 = (adc_T / 16384.0 - digT[0] / 1024.0) * digT[1]
	v2 = (adc_T / 131072.0 - digT[0] / 8192.0) * (adc_T / 131072.0 - digT[0] / 8192.0) * digT[2]
	t_fine = v1 + v2
	temperature = t_fine / 5120.0
	#print "temp : %-6.2f ℃" % (temperature) 
	return(temperature)#気温（摂氏）を返す

def compensate_H(adc_H):
	global t_fine
	var_h = t_fine - 76800.0
	if var_h != 0:
		var_h = (adc_H - (digH[3] * 64.0 + digH[4]/16384.0 * var_h)) * (digH[1] / 65536.0 * (1.0 + digH[5] / 67108864.0 * var_h * (1.0 + digH[2] / 67108864.0 * var_h)))
	else:
		return 0
	var_h = var_h * (1.0 - digH[0] * var_h / 524288.0)
	if var_h > 100.0:
		var_h = 100.0
	elif var_h < 0.0:
		var_h = 0.0
	#print "hum : %6.2f %" % (var_h)

def writeReg(reg_address, data):
	bus.write_byte_data(i2c_address,reg_address,data)

def BME280_setup():

	osrs_t = 1			#Temperature oversampling x 1
	osrs_p = 1			#Pressure oversampling x 1
	osrs_h = 1			#Humidity oversampling x 1
	mode   = 3			#Normal mode
	t_sb   = 5			#Tstandby 1000ms
	filter = 0			#Filter off
	spi3w_en = 0			#3-wire SPI Disable

	ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode
	config_reg    = (t_sb << 5) | (filter << 2) | spi3w_en
	ctrl_hum_reg  = osrs_h

	writeReg(0xF2,ctrl_hum_reg)
	writeReg(0xF4,ctrl_meas_reg)
	writeReg(0xF5,config_reg)

    #<気温，気圧から高度を計算するプログラム>
def calculate_altitude(temperature, pressure):
    STANDARD_PRESSURE = 1013.25  # 標準大気圧（ヘクトパスカル）
    altitude = ((STANDARD_PRESSURE / pressure) ** (1 / 5.257) - 1) * (temperature + 273.15) / 0.0065
    return altitude

    #<<空気室センサー繰り返し用関数>>
def BME280_main():
    data = []
    for i in range (0xF7, 0xF7+8):
        data.append(bus.read_byte_data(i2c_address,i))
    pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
    temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
    hum_raw  = (data[6] << 8)  |  data[7]
    
    T = compensate_T(temp_raw)
    P = compensate_P(pres_raw)
    H = compensate_H(hum_raw)
    #print("tenp;",T)
    print("pres;",P)
    #print("humi;",H)
    f_BME280.write('time'+','+str(T)+','+str(P)+'\n')
    #print(calculate_altitude(T,P))
    #ALTITUDE = calculate_altitude(T,P)
    global PRESSURE 
    PRESSURE = P
    return P #気温，気圧を返す



    #<気温，気圧から高度を計算するプログラム>
def calculate_altitude(pressure, temperature):
    STANDARD_PRESSURE = 1013.25  # 標準大気圧（ヘクトパスカル）
    altitude = (((STANDARD_PRESSURE/10 / pressure) ** (1 / 5.257) - 1) * (temperature + 273.15) )/ 0.0065
    print("altitude;",altitude)
    return altitude

#<距離方位方向計算関係>
    #期待とゴールの 方向関係を計算
def degree_from_front_to_goal():
    #print("nowlat,now_lon",now_lat,now_lon)
    north_degree = calculate_north_degree()#330
    GPS_degree = calculate_azimuth(now_lat,now_lon)
    
    degree = GPS_degree - north_degree
    print("degree:" ,int(degree))
    return degree

#<近距離モジュール，フェーズ>

from Base_Integrate_Improve import Short_Range
from multiprocessing import Process



#E2E/////////////////////////////////////////////////////////////////////
	#CSV形式の保存指定
# file_BNO055 = csv.writer(f_BNO055,delimiter=",")
# file_GPS = csv.writer(f_GPS,delimiter=",")
# file_BME280= csv.writer(f_BME280,delimiter=",")
# 
# file_BNO055.writerow([my_gps.date_string(), "lx", "ly", "lz"])
# file_GPS.writerow([my_gps.date_string(), "Latitude", "Longitude"])
# file_BME280.writerow([my_gps.date_string(), "temp", "press", "humi"])

#セットアップ
BNO55_setup()
BME280_setup()
BME280_get_calib_param()
print("setup_complete")
process = 0
pressure = 0
while (process < 10):
    BME280_main()
    pressure += PRESSURE
    process += 1
    
STANDARD_PRESSURE = pressure/10
print("standard_pressure:",STANDARD_PRESSURE)

#1待機フェーズ
while(1):
    print("/"*20)
    time.sleep(1)
    accell = 0
    i = 0
    while(i <= 9):
        ACCELL = BNO055_Accel_strength()
        accell += ACCELL
        
        i +=1
    
    accell /= 10
    BME280_main()
    if (accell <= 0.4):
        if(PRESSURE <= STANDARD_PRESSURE + 5 ):
            print("separate")
            break

#2落下フェーズ
#3分離フェーズ
f_BNO055.write('waiting_end'+'\n')
f_BME280.write('waiting_end'+'\n')

print("/" * 20)
print("waiting_end")
print("servo")
servo()
print("accel")
accel()

#4長距離フェーズ
print("/" * 20)
print("long_phase")
GPS_lat_lon()

t = 0
while(calculate_distance(now_lat, now_lon) > 10):
    print("/"*20)
    print("LONG_PHASE")
    
    GPS_lat_lon()
    #calculate_distance(now_lat, now_lon)
    
    if (-60<= degree_from_front_to_goal()<=60):
        forward()
    elif(degree_from_front_to_goal() < -60):
        turn_left()
    else:
        turn_right()  
    t +=1
    if (t >= 10):
        break
f_BNO055.write('long_end'+'\n')
f_GPS.write('long_end'+'\n')
f_BME280.close()
f_BNO055.close()
f_GPS.close()        


GPIO.cleanup()
print("/"* 20)
print("shortphase")	
cansat = Short_Range()
# # 
p1 = Process(target=cansat.object_detection)
p2 = Process(target=cansat.motor_pid)
p1.start()
p2.start()
GPIO.cleanup()
print('aaaa ')

    





