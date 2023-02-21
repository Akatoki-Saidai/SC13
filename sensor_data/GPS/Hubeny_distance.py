# -*- coding: utf-8 -*-
import math

pole_radius = 6356752.314245                  # �ɔ��a
equator_radius = 6378137.0                    # �ԓ����a

def cal_distance():

    # �ܓx�o�x�����W�A���ɕϊ�
    lat_1 = math.radians(lat1)
    lon_1 = math.radians(latlon_kamata[1])
    lat_2 = math.radians(latlon_yokosukachuo[0])
    lon_2 = math.radians(latlon_yokosukachuo[1])

    lat_difference = lat_kamata - lat_yokosukachuo       # �ܓx��
    lon_difference = lon_kamata - lon_yokosukachuo       # �o�x��
    lat_average = (lat_kamata + lat_yokosukachuo) / 2    # ���ψܓx

    e2 = (math.pow(equator_radius, 2) - math.pow(pole_radius, 2)) \
            / math.pow(equator_radius, 2)  # ��ꗣ�S��^2

    w = math.sqrt(1- e2 * math.pow(math.sin(lat_average), 2))

    m = equator_radius * (1 - e2) / math.pow(w, 3) # �q�ߐ��ȗ����a

    n = equator_radius / w                         # �K�ѐ��Ȕ��a

    distance = math.sqrt(math.pow(m * lat_difference, 2) \
                   + math.pow(n * lon_difference * math.cos(lat_average), 2)) # �����v��

    print(distance / 1000)