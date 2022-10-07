import folium
import pandas as pd
import time
import pigpio
from micropyGPS import MicropyGPS
import csv

#データ格納
latitude = []
longtude = []

#地図生成
folium_map = folium.Map(location=["緯度を入れて" , "経度を入れて"], tiles = "CansatMap",zoom_start=15,width = 1920, height = 1280 # 地図のサイズ)

def main():
    # シリアル通信設定
    baudrate = 9600
    #通信設定でいじるとしたらここのTX,RXだけど、ピン配置変えたい場合以外はいじらなくてok
    TX = 24
    RX = 23

    serialpi = pigpio.pi()
    serialpi.set_mode(RX,pigpio.INPUT)
    serialpi.set_mode(TX,pigpio.OUTPUT)

    pigpio.exceptions = False
    serialpi.bb_serial_read_close(RX)
    pigpio.exceptions = True

    serialpi.bb_serial_read_open(RX,baudrate,8)
    # gps設定
    #my_gpsにデータが格納される感じ
    my_gps = MicropyGPS(9, 'dd')

    # 10秒ごとに表示
    #なんか10秒ごとに表示されてない気がするのでいじってみてほしい
    tm_last = 0
    count = 0
    
    with open('gps_data.csv', 'w') as f:#csvファイルへの書き込み(一行目)
        writer = csv.writer(f)
        writer.writerow(['latitude', 'longtitude', 'altitude'])
    with open('gps_data.csv','w') as f:#csvファイルへの書き込み(data)
        writer = csv.writer(f)
        while True:
            (count, sentence) = serialpi.bb_serial_read(RX)#ここはよくわからん、データが取れてるかどうか調べるところな気がする
            if len(sentence) > 0:
                for x in sentence:
                    if 10 <= x <= 126:
                        stat = my_gps.update(chr(x))
                        if stat:
                            tm = my_gps.timestamp
                            tm_now = (tm[0] * 3600) + (tm[1] * 60) + int(tm[2])
                            if (tm_now - tm_last) >= 10:
                                print('=' * 20)
                                print(my_gps.date_string(), tm[0], tm[1], int(tm[2]))
                                #-----------------追加したところ
                                latitude.append()#緯度追加
                                longtude.append()#経度追加
                                folium.Marker(
                                            location=[latitude[count],longtude[count]], #latitudeとlongtudeのcount番目の要素を指定
                                            icon=folium.Icon(color='blue')#青でマッピング   
                                                )
                                folium.PolyLine(locations=[latitude[count],longtude[count]]).add_to(folium_map)
                                #地図表示
                                folium_map
                                #地図保存
                                folium_map.save('Cansat.html')
                                #-----------------
                                print("latitude:", my_gps.latitude[0], ", longitude:", my_gps.longitude[0], "altitude:", my_gps.altitude)    
                                writer.writerow([my_gps.latitude[0], my_gps.longitude[0], my_gps.altitude])
                                time.sleep(1)#一秒停止
"""
tmは時間に関するやつだと思うけどよく知らない。使わなくてもいい可能性がある。
my_gps.latitude[0]は緯度
my_gps.longitude[0]は経度
my_gps.altitudeは高度(未確認)
whileの中のif文はいじくりまわしても多分大丈夫だと思う。
おかしくなったら元に戻す感じで試行錯誤してみてほしいです。
"""
if __name__ == "__main__":
    main()

