import time
import smbus
import Adafruit_BME280

BME280_ADDR = 0x76 # BME280のI2Cアドレス

def main():
  bus = smbus.SMBus(1) # I2Cバスを開く
  bme280 = Adafruit_BME280.BME280(t_mode=Adafruit_BME280.BME280_OSAMPLE_8, p_mode=Adafruit_BME280.BME280_OSAMPLE_8, h_mode=Adafruit_BME280.BME280_OSAMPLE_8, address=BME280_ADDR, i2c=bus)

  while True:
    temperature = bme280.read_temperature() # 温度を取得
