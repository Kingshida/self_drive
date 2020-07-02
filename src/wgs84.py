#coding=utf-8

import math
from math import sin, cos
import numpy as np


class WGS84():
    def __init__(self, lon, lat, h):
        # WGS84坐标系参数
        self.a = 6378137         # 地球长轴半径
        self.f = 1/298.257223563  # 匾率
        self.b = self.a - self.a * self.f       # 短轴半径
        self.sqr_e = (math.pow(self.a,2)-math.pow(self.b, 2)) / math.pow(self.a, 2)
        self.h = h   # 高程
        self.p = self.calc_xyz(lon, lat, self.h)    # 坐标原点
        self.lon = math.radians(lon)
        self.lat = math.radians(lat)
        self.rotation = self.euler_to_rotation(0.0, math.pi/2-self.lat, self.lon, 'rzyx')
        print(self.rotation)
        self.translation = -self.p

    
    def euler_to_rotation(self, roll, pitch, yaw, mod = 'rzyx'):
        # 绕x旋转
        r_x = np.array([
            [1, 0, 0],
            [0, cos(roll), -sin(roll)],
            [0, sin(roll), cos(roll)]
            ])
        # 绕y旋转
        r_y = np.array([
            [cos(pitch), 0, sin(pitch)],
            [0, 1, 0],
            [-sin(pitch),0, cos(pitch)]
            ])
        # 绕z旋转
        r_z = np.array([
            [cos(yaw), -sin(yaw), 0],
            [sin(yaw), cos(yaw), 0],
            [0, 0, 1]
            ])
        if mod == 'rzyx':
            r = np.dot(r_z, np.dot(r_y, r_x))
        else:
            r = None
        return r

    def calc_xyz(self, lon, lat, h):
        # 中国x<0, y>0, z>0
        # print(lon,lat, h)
        lon = math.radians(lon)
        lat = math.radians(lat)
        n = self.a / math.sqrt(1 - self.sqr_e * math.sin(lat))
        x = (n + h) * math.cos(lat) * math.cos(lon)
        y = (n + h) * math.cos(lat) * math.sin(lon)
        z = (n * (1 - self.sqr_e) + h) * math.sin(lat)
        p = np.array([x, y, z]).T
        return p


