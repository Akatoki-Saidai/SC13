# -*- coding: utf-8 -*-
import math

pole_radius = 6356752.314245                  # 極半径
equator_radius = 6378137.0                    # 赤道半径

def cal_distance():

    # 緯度経度をラジアンに変換
    lat_1 = math.radians(lat1)
    lon_1 = math.radians(latlon_kamata[1])
    lat_2 = math.radians(latlon_yokosukachuo[0])
    lon_2 = math.radians(latlon_yokosukachuo[1])

    lat_difference = lat_kamata - lat_yokosukachuo       # 緯度差
    lon_difference = lon_kamata - lon_yokosukachuo       # 経度差
    lat_average = (lat_kamata + lat_yokosukachuo) / 2    # 平均緯度

    e2 = (math.pow(equator_radius, 2) - math.pow(pole_radius, 2)) \
            / math.pow(equator_radius, 2)  # 第一離心率^2

    w = math.sqrt(1- e2 * math.pow(math.sin(lat_average), 2))

    m = equator_radius * (1 - e2) / math.pow(w, 3) # 子午線曲率半径

    n = equator_radius / w                         # 卯酉線曲半径

    distance = math.sqrt(math.pow(m * lat_difference, 2) \
                   + math.pow(n * lon_difference * math.cos(lat_average), 2)) # 距離計測

    print(distance / 1000)