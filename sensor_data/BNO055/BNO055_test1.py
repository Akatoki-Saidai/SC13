
import logging
import sys
import time

# BNOセンサーの接続を作成し、設定する。 以下の'bno = ...'の行のうち、1つだけをアンコメントしてください。
# 以下の 'bno = ...' の行のうち、1つだけがコメントアウトされていないことを確認してください。
# GPIO 18にシリアルUARTとRSTを接続したRaspberry Piの設定。→変更して自動でI2C通信で行う
bno = BNO055.BNO055(rst=18)
# BeagleBone Black の構成は、デフォルトの I2C 接続 (SCL=P9_19, SDA=P9_20)です。
# そして RST は P9_12 ピンに接続されています。
#bno = BNO055.BNO055(rst='P9_12')


# Enable verbose debug logging if -v is passed as a parameter.
if len(sys.argv) == 2 and sys.argv[1].lower() == '-v':
    logging.basicConfig(level=logging.DEBUG)

# Initialize the BNO055 and stop if something went wrong.初期化
if not bno.begin():
    raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

# Print system status and self test result.システム状況表示
status, self_test, error = bno.get_system_status()
print('System status: {0}'.format(status))
print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
# Print out an error if system status is in error mode.
if status == 0x01:
    print('System error: {0}'.format(error))
    print('See datasheet section 4.3.59 for the meaning.')

# Print BNO055 software revision and other diagnostic data.
sw, bl, accel, mag, gyro = bno.get_revision()
print('Software version:   {0}'.format(sw))
print('Bootloader version: {0}'.format(bl))
print('Accelerometer ID:   0x{0:02X}'.format(accel))
print('Magnetometer ID:    0x{0:02X}'.format(mag))
print('Gyroscope ID:       0x{0:02X}\n'.format(gyro))

print('Reading BNO055 data, press Ctrl-C to quit...')
while True:
    # Read the Euler angles for heading, roll, pitch (all in degrees).
    heading, roll, pitch = bno.read_euler()
    # Read the calibration status, 0=uncalibrated and 3=fully calibrated.
    sys, gyro, accel, mag = bno.get_calibration_status()
    # Print everything out.
    print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}'.format(
          heading, roll, pitch, sys, gyro, accel, mag))
    # その他、オプションで読み込むことができる値
    # オリエンテーションはクォータニオンで表現されます。
    #x,y,z,w = bno.read_quaterion()
    # センサの温度（摂氏
    #temp_c = bno.read_temp()
    # 地磁気センサーのデータ（単位：マイクロテスラ）:
    mx,my,mz = bno.read_magnetometer()
    # ジャイロスコープデータ (単位: 度/秒):
    Gx,Gy,Gz = bno.read_gyroscope()
    # 加速度センサーのデータ（単位：メートル毎秒2乗）。
    ax,ay,az = bno.read_accelerometer()
    # 線形加速度データ（重力ではなく、運動による加速度）： ax,y,z = bno.read_accelerometer()
    # 直線加速度データ（重力ではなく、移動による加速度）--メートル毎秒の2乗で返されます。
    # x,y,z = bno.read_linear_acceleration()
    # 重力加速度データ (すなわち，重力による加速度．
    # メートル毎秒2乗で返される)。
    #x,y,z = bno.read_gravity()
    # 次の読み込みまで1秒間スリープする。
    print ("magx=",mx, "magy=",my, "magz=",mz, "Gyx=",Gx, "Gyy=",Gy, "Gyz=",Gz, "accelx=",ax, "accely=",ay, "accelz=",az) 
    time.sleep(1)
    
    #参考https://qiita.com/kmaepu/items/779ab8e45bfe96230224
