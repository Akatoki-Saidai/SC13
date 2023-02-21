import math 
from math import radians

pole_radius = 6356752.314245                  # 極半径
equator_radius = 6378137.0                    # 赤道半径

def calc_distance(lat1, lon1, lat2, lon2):

  # 緯度経度をラジアンに変換
  lat1_rad, lon1_rad, lat2_rad,  lon2_rad = map(radians, [lat1, lon1, lat2, lon2])

  lat_difference = lat1_rad - lat2_rad      # 緯度差
  lon_difference = lon1_rad - lon2_rad       # 経度差
  lat_average = (lat1_rad + lat2_rad) / 2    # 平均緯度

  e2 = (math.pow(equator_radius, 2) - math.pow(pole_radius, 2)) / math.pow(equator_radius, 2)  # 第一離心率^2

  w = math.sqrt(1- e2 * math.pow(math.sin(lat_average), 2))

  m = equator_radius * (1 - e2) / math.pow(w, 3) # 子午線曲率半径

  n = equator_radius / w                         # 卯酉線曲半径
  distance = math.sqrt(math.pow(m * lat_difference, 2) + math.pow(n * lon_difference * math.cos(lat_average), 2)) # 距離計測(m)

  return distance