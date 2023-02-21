from math import sin, cos, sqrt, atan2, radians

def distance(lat1, lon1, lat2, lon2):

    #地球の半径（km）
    R = 6371

    #度数からラジアンに変換
    lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])

    #ヒュベニの公式
    dlat = lat2 - lat1  #2点間の緯度差
    dlon = lon2 - lon1  #2点間の軽度差
    a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    distance = (R * c) *1000

    return distance

print(distance(35.678376273798136, 139.7144072265726, 35.67749951124794, 139.7139760005817))