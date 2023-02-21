import math 
from math import radians, sin, cos, tan, atan2

pole_radius = 6356752.0    #極半径
equator_radius = 6378137.0    #赤道半径

def calc_distance_azimuth(naw_lat, naw_lon, goal_lat, goal_lon):

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

  
  #方位角計算
  x = math.cos(naw_lat_rad) * math.sin(goal_lat_rad) - math.sin(naw_lat_rad) * math.cos(goal_lat_rad) * math.cos(d_lon)
  y = math.sin(d_lon) * math.cos(goal_lat_rad)

  brng = math.degrees(math.atan2(y, x))
  azimuth = (brng + 360) % 360
  
  return distance, azimuth