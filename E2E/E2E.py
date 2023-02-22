#モジュールインポート-----------------------------------
import RPi.GPIO as GPIO #GPIOインポート
import time#時間制御インポート
import logging
import sys
import time
import numpy as np
from Adafruit_BNO055 import BNO055
import csv
import datetime
import smbus
import cartopy.crs as ccrs
import matplotlib.pyplot as plt
import cartopy.io.img_tiles as cimgt
import time
import pigpio
from micropyGPS import MicropyGPS
#モーター----------------------------------
AIN1 = 15
AIN2 = 29
BIN1 = 31
BIN2 = 33

#空気質センサーBME280---------------------------------
bus_number  = 1
i2c_address = 0x76

bus = smbus(bus_number)

digT = []
digP = []
digH = []

t_fine = 0.0

GPIO.setmode(GPIO.BOARD)    #物理ピン番号でGPIOを指定
GPIO.setup(AIN1,GPIO.OUT)   #AIN1番ピンで
GPIO.setup(AIN2,GPIO.OUT)   #AIN2ピンで
GPIO.setup(BIN1,GPIO.OUT)   #BIN1ピンで
GPIO.setup(BIN2,GPIO.OUT)   #BIN2ピンでGPIO出力設定

Apwm = GPIO.PWM(AIN2,255)     #PWM(pin,Hz)→AIN2番ピンで255Hz
Bpwm = GPIO.PWM(BIN2,255)     #PWM(pin,Hz)→BIN2番ピンで255Hz出力設定


def forward():        #前進
    #右車輪(A1,A2)=(+,0)→正回転
    Apwm.start(0)               #start(duty比)つまり出力0の状態              
    GPIO.output(AIN1,GPIO.HIGH) #AIN1を出力(pin,出力を100％) 
    Apwm.ChangeDutyCycle(0)     #ChangeDutyCycle(duty)→AIN2のduty比０に設定
    #左車輪(BIN1.BIN2)=(+,0)→正回転
    Bpwm.start(0)               
    GPIO.output(BIN1,GPIO.HIGH) #def forward()右車輪と同上
    Bpwm.ChangeDutyCycle(0)     
    
def turn_right():    #右回転
    #右車輪(AIN1,AIN2)=(0,+)→負回転
    Apwm.start(0)                
    GPIO.output(AIN1,GPIO.LOW)  #def forward()右車輪と同上
    Apwm.ChangeDutyCycle(100)   #duty比100％→255Hz（max）出力
    #左車輪(BIN1.BIN2)=(+,0)→正回転
    Bpwm.start(0)                
    GPIO.output(BIN1,GPIO.HIGH) #def forward()右車輪と同上
    Bpwm.ChangeDutyCycle(0)     
    
def turn_left():     #左回転
    #右車輪(AIN1.AIN2)=(+,0)→正回転
    Apwm.start(0)               
    GPIO.output(AIN1,GPIO.HIGH) #def forward()右車輪と同上
    Apwm.ChangeDutyCycle(0)     
    #左車輪(BIN1.BIN2)=(0,+)→負回転
    Bpwm.start(0)               
    GPIO.output(BIN1,GPIO.LOW)  #def forward()右車輪と同上
    Bpwm.ChangeDutyCycle(100)   

def back():          #後進
    #左車輪(AIN1.AIN2)=(0,+)→負回転
    Apwm.start(0)                  #def forward()右車輪と同上
    GPIO.output(AIN1,GPIO.LOW)
    Apwm.ChangeDutyCycle(75)
    #左車輪(BIN1.BIN2)=(+,0)→負回転
    Bpwm.start(0)                  #def forward()右車輪と同上
    GPIO.output(BIN1,GPIO.LOW)
    Bpwm.ChangeDutyCycle(100)
    
def stop():           #停止
    #左車輪(AIN1.AIN2)=(0,0)→停止
    Apwm.start(0)                  #def forward（）右車輪と同上
    GPIO.output(AIN1,GPIO.LOW)
    Apwm.ChangeDutyCycle(0)
    #左車輪(BIN1.BIN2)=(0,0)→停止
    Bpwm.start(0)                  #def forward()右車輪と同上
    GPIO.output(BIN1,GPIO.LOW)
    Bpwm.ChangeDutyCycle(0)



def writeReg(reg_address, data):
	bus.write_byte_data(i2c_address,reg_address,data)

def get_calib_param():
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

def readData():
	data = []
	for i in range (0xF7, 0xF7+8):
		data.append(bus.read_byte_data(i2c_address,i))
	pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
	temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
	hum_raw  = (data[6] << 8)  |  data[7]
	
	compensate_T(temp_raw)
	compensate_P(pres_raw)
	compensate_H(hum_raw)

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

	print "pressure : %7.2f hPa" % (pressure/100)

def compensate_T(adc_T):
	global t_fine
	v1 = (adc_T / 16384.0 - digT[0] / 1024.0) * digT[1]
	v2 = (adc_T / 131072.0 - digT[0] / 8192.0) * (adc_T / 131072.0 - digT[0] / 8192.0) * digT[2]
	t_fine = v1 + v2
	temperature = t_fine / 5120.0
	print "temp : %-6.2f ℃" % (temperature) 

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
	print "hum : %6.2f ％" % (var_h)


def setup():
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


#9軸センサー-----------------------------------
bno = BNO055.BNO055()
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

#GPS-----------------------------------
def main():

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
    request = cimgt.GoogleTiles()
    fig = plt.figure()

	
	
#繰り返し処理
setup()
get_calib_param()

while True:
    heading, roll, pitch = bno.read_euler()# Read the Euler angles for heading, roll, pitch (all in degrees).
    sys, gyro, accel, mag = bno.get_calibration_status() # Read the calibration status, 0=uncalibrated and 3=fully calibrated.
    qx,qy,qz,qw = bno.read_quatenion()        # Orientation as a quaternion:
    temp_c = bno.read_temp()        # Sensor temperature in degrees Celsius:
    mx,my,mz = bno.read_magnetometer()        # Magnetometer data (in micro-Teslas):
    Gx,Gy,Gz = bno.read_gyroscope()        # Gyroscope data (in degrees per second):
    ax,ay,az = bno.read_accelerometer()        # Accelerometer data (in meters per second squared):
    lx,ly,lz = bno.read_linear_acceleration()        # Linear acceleration data (i.e. acceleration from movement, not gravity--returned in meters per second squared):
    gx,gy,gz = bno.read_gravity()        # Gravity acceleration data (i.e. acceleration just from gravity--returned in meters per second squared):

    x = qx / 16384
    y = qy / 16384
    z = qz / 16384
    w = qw / 16384

    ysqr = y ** 2

    # roll (x-axis rotation)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    roll = np.arctan2(t0, t1)

    # pitch (y-axis rotation)
    t2 = +2.0 * (w * y - z * x)
    if t2 > 1.0:
        t2 = 1.0
    if t2 < -1.0:
        t2 = -1.0
    
    pitch = np.arcsin(t2)

    # yaw (z-axis rotation)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z);  
    yaw = np.arctan2(t3, t4)

    print("roll :%f", roll)
    print("pitch:%f", pitch)
    print("yaw  :%f", yaw)
#BME280
    readData()
#GPS
    (count, sentence) = serialpi.bb_serial_read(RX)#ここはよくわからん、データが取れてるかどうか調べるところな気がする
    if len(sentence) > 0:
        for x in sentence:
            if 10 <= x <= 126:
                stat = my_gps.update(chr(x))
                if stat:
                    tm = my_gps.timestamp
		tm_now = (tm[0] * 3600) + (tm[1] * 60) + int(tm[2])


		# Create a GeoAxes in hte tile's projection
		m1 = fig.add_subplot(111, projection=request.crs) #画面の領域が描写される　111の意味は、1行目1列の1番目という意味　参考サイト　https://qiita.com/kenichiro_nishioka/items/8e307e164a4e0a279734

		# Set map extent to +- 0.01º of the received position
		m1.set_extent([my_gps.longitude[0] + 0.01, my_gps.longitude[0] - 0.01, my_gps.latitude[0] + 0.01, my_gps.latitude[0] - 0.01]) #set_extent([西端の経度, 東端の経度, 南端の緯度, 北端の緯度 0.01のところを変えると描写されるマップの広さを変えれる

		# Get image at desired zoom
		m1.add_image(request, 14) ##画像を指定するセルにはる　ここだとグーグルマップのタイルを貼っている　右の値はマップの拡大度合い
		# Print data on console
		print('=' * 20)
		print(my_gps.date_string(), tm[0], tm[1], int(tm[2]))
		print("latitude:", my_gps.latitude[0], ", longitude:", my_gps.longitude[0])    
		#time.sleep(1)#一秒停止

# Plot the new position
		m1.plot(my_gps.longitude[0], my_gps.latitude[0], '.', transform=ccrs.Geodetic())
		plt.pause(0.1)

