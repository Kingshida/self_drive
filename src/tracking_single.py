# coding:utf-8
"""

Path tracking simulation with pure pursuit steering and PID speed control.

author: Atsushi Sakai (@Atsushi_twi)
        Guillaume Jacquenot (@Gjacquenot)

"""
import math
import time
import sys
import json
import requests
import redis
import numpy as np
from redisHandler import redisHandler

# Parameters
k = 0.5  # look forward gain
Lfc = 0.50  # [m] look-ahead distance
Kp = 5.0  # speed proportional gain
dt = 0.2  # [s] time tick
WB = 0.2  # [m] wheel base of vehicle

show_animation = False


class State:
    # 状态类
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

    def update(self, a, delta):
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.v / WB * math.tan(delta) * dt
        self.v += a * dt

    def calc_distance(self, point_x, point_y):
        dx = self.x - point_x
        dy = self.y - point_y
        return math.hypot(dx, dy)


class States:
    # 状态list
    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.t = []

    def append(self, t, state):
        self.x.append(state.x)
        self.y.append(state.y)
        self.yaw.append(state.yaw)
        self.v.append(state.v)
        self.t.append(t)


def proportional_control(target, current):
    # 速度的比例控制
    a = Kp * (target - current)
    return a


class TargetCourse:
    # 跟踪目标类
    def __init__(self, cx, cy, cyaw):
        self.cx = cx
        self.cy = cy
        self.cyaw = cyaw
        self.old_nearest_point_index = None

    def search_target_index(self, state):
        if self.old_nearest_point_index is None:
            # 搜寻上一个最近点下标,仅在第一次搜寻
            dx = [state.x - icx for icx in self.cx]
            dy = [state.y - icy for icy in self.cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        else:
            # 更新上一个最近点下标
            ind = self.old_nearest_point_index
            # 实际点与当前点距离
            distance_this_index = state.calc_distance(self.cx[ind],
                                                      self.cy[ind])
            while True:
                # 实际与下一点距离
                length = len(self.cx)
                if ind + 1 >= length:
                    ind = length -1
                    break
                distance_next_index = state.calc_distance(self.cx[ind + 1],
                                                          self.cy[ind + 1])
                if distance_this_index < distance_next_index:
                    break
                ind = ind + 1 if (ind + 1) < length else ind
                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind

        # Lf = k * state.v + Lfc  # 根据当前速度更新向前规划距离
        Lf = Lfc  # 根据当前速度更新向前规划距离

        # 搜寻向前规划终点下标
        while Lf > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                # 最后一点为终点
                break
            ind += 1

        # print('%.2f, %.2f'%(self.cx[ind], self.cy[ind]))

        return ind, Lf


def pure_pursuit_steer_control(state, trajectory, pind):
    # 方向舵控制
    ind, Lf = trajectory.search_target_index(state)
    if pind >= ind:
        # 去最大index
        ind = pind

    if ind < len(trajectory.cx):
        tx = trajectory.cx[ind]
        ty = trajectory.cy[ind]
    else:  # 朝向目标
        tx = trajectory.cx[-1]
        ty = trajectory.cy[-1]
        ind = len(trajectory.cx) - 1
    dx = trajectory.cx[ind] - state.x
    dy = trajectory.cy[ind] - state.y
    dyaw = trajectory.cyaw[ind] - state.yaw
    alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw

    delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)

    return delta, ind


def diff_speed_control(state, trajectory, pind):
    # 轮差控制
    ind, Lf = trajectory.search_target_index(state)
    if pind >= ind:
        # 去最大index
        ind = pind

    if ind < len(trajectory.cx):
        tx = trajectory.cx[ind]
        ty = trajectory.cy[ind]
    else:  # 朝向目标
        tx = trajectory.cx[-1]
        ty = trajectory.cy[-1]
        ind = len(trajectory.cx) - 1
    # 当前坐标 
    x0 = state.x
    y0 = state.y
    yaw0 = state.yaw
    # 目标点坐标
    x1  = trajectory.cx[ind]
    y1  = trajectory.cy[ind]
    yaw1  = trajectory.cyaw[ind]
    # 目标点姿态向量，为单位向量
    p_b = np.array([math.cos(yaw1), math.sin(yaw1)])
    # p_b向量摸
    len_b = np.linalg.norm(p_b)
    # 当前点到目标点向量
    p_a = np.array([x1-x0, y1-y0])
    # 当前点到目标点距离, 即p_a向量模
    len_a = np.linalg.norm(p_a)
    # p_a向量在p_b向量上的投影向量模，锐角>0, 钝角<0
    len_c = p_b.dot(p_a)
    # 投影向量
    p_c = len_c * p_b
    # 目标到p_b垂足点-->目标点向量
    p_e = p_a - p_c
    # 向量模，即当前点到目标向量距离, 与当前点到目标点距离的比值
    if len_a > 0.005:
        d = np.linalg.norm(p_e) / len_a
    else:
        d = 0
    '''
    # p_b x P_a 扩展成3d坐标向量，则叉乘可得与之垂直的向量
    # 取第三项符号为向量方向关系, 左边为正，右边为负
    # 000-090: k > 0, dot_ba > 0, 1 1
    # 090-180: k > 0, dot_ba < 0, 1 0
    # 180-270: k < 0, dot_ba < 0, 0 0
    # 270-360: k < 0, dot_ba > 0, 0 1
    '''
    k = p_b[0] * p_a[1] - p_b[1] * p_a[0]
    # p_b --> p_a 夹角
    dot_ba = p_b.dot(p_a)
    yaw_ba = np.arccos(dot_ba / (len_a * len_b))
    if k < 0:
        # 距离
        d = -d
        # 角度180-360
        yaw_ba = 6.28 - yaw_ba

    return d, yaw_ba




def speed_pid(pre_d, d, dyaw, dt):
    # 轮差机器人pid控制
    if not pre_d:
        pre_d = d
    kp = 0.800
    kd = 0.8000
    angle = math.radians(60)
    k = 0.0
    k = kp * d + kd * (d - pre_d) / dt

    '''
    if dyaw > angle and dyaw < 3.14:
        k = 1.57
    elif dyaw < 6.3 - angle and dyaw >= 3.14:
        k = -1.57
    else:
        k = kp * d + kd * (d - pre_d) / dt
        if k > 1.5:
            k = 1.5
        elif k < -1.5:
            k = -1.5
    '''
    # print(k, dyaw)
    return k


class tracking(redisHandler):
    '''
    # tracking 路径跟踪
    '''
    def __init__(self):
        redisHandler.__init__(self)
        self.url = 'http://117.184.129.18:8000/planning/query/?key='
        self.sub_topics = ['rtk_out', 'tracking_in']
        self.pub_topics = ['tracking_out']
        self.start_sub()

    def run(self):
        # 路径跟踪方法
        rc = self.rc
        # movebase data格式
        delta_goal = 0.25
        data_speed = {
                'header':'speed',
                'data':{
                    'y': 1,
                    'angle': 0,
                    'speed': 80
                    }
                }
        data_main = {
                'header':'auto_off',
                'data': ''
                }
        # 路径点
        cx = [-99.05, -99.0, -98.95, -98.89, -98.84, -98.79, -98.74, -98.68, -98.63, -98.58, -98.53, -98.48, -98.42, -98.37, -98.32, -98.27, -98.21, -98.16, -98.11, -98.06, -98.0, -97.95, -97.9, -97.85, -97.8, -97.74, -97.69, -97.64, -97.59, -97.53, -97.48, -97.43, -97.38, -97.33, -97.27, -97.22, -97.17, -97.12, -97.06, -96.88, -96.55, -96.12, -95.63, -95.13, -94.63, -94.14, -93.64, -93.14, -92.64, -92.14, -91.65, -91.15, -90.65, -90.15, -89.65, -89.16, -88.66, -88.16, -87.66, -87.16, -86.66, -86.17, -85.67, -85.27, -85.27, -84.78, -84.28, -83.79, -83.29, -82.8, -82.31, -81.81, -81.32, -80.82, -80.33, -79.83, -79.34, -78.85, -78.35, -77.86, -77.36, -76.87, -76.38, -75.88, -75.39, -74.89, -74.4, -73.91, -73.41, -72.92, -72.42, -71.93, -71.44, -70.94, -70.45, -69.95, -69.46, -68.96, -68.47, -67.98, -67.48, -66.99, -66.49, -66.0, -65.51, -65.01, -64.52, -64.02, -63.53, -63.04, -62.54, -62.05, -61.55, -61.06, -60.57, -60.07, -59.58, -59.08, -58.59, -58.09, -57.6, -57.11, -56.61, -56.12, -55.62, -55.13, -54.64, -54.14, -53.65, -53.15, -52.66, -52.17, -51.67, -51.18, -50.68, -50.19, -49.7, -49.2, -48.71, -48.21, -47.72, -47.22, -46.73, -46.24, -45.74, -45.25, -44.75, -44.26, -43.77, -43.27, -42.78, -42.28, -41.79, -41.3, -40.8, -40.31, -39.81, -39.32, -38.83, -38.33, -37.84, -37.34, -36.85, -36.35, -35.86, -35.37, -34.87, -34.38, -33.88, -33.39, -32.9, -32.4, -31.91, -31.41, -30.92, -30.43, -29.93, -29.44, -29.42, -29.42, -28.94, -28.53, -28.23, -28.09, -28.12, -28.21, -28.3, -28.39, -28.48, -28.58, -28.67, -28.76, -28.85, -28.94, -29.03, -29.21, -29.53, -29.96, -30.45, -30.6, -30.6, -31.09, -31.59, -32.08, -32.57, -33.07, -33.56, -34.05, -34.55, -35.04, -35.53, -36.03, -36.52, -37.01, -37.51, -38.0, -38.49, -38.99, -39.48, -39.98, -40.47, -40.96, -41.46, -41.95, -42.44, -42.94, -43.43, -43.92, -44.42, -44.91, -45.4, -45.9, -46.39, -46.88, -47.38, -47.87, -48.36, -48.86, -49.35, -49.84, -50.34, -50.83, -51.32, -51.82, -52.31, -52.8, -53.3, -53.79, -54.28, -54.78, -55.27, -55.76, -56.26, -56.75, -57.24, -57.74, -58.23, -58.72, -59.22, -59.71, -60.2, -60.7, -61.19, -61.68, -62.18, -62.67, -63.16, -63.66, -64.15, -64.65, -65.14, -65.63, -66.13, -66.62, -67.11, -67.61, -68.1, -68.59, -69.09, -69.58, -70.07, -70.57, -71.06, -71.55, -72.05, -72.54, -73.03, -73.53, -74.02, -74.51, -75.01, -75.5, -75.99, -76.49, -76.98, -77.47, -77.97, -78.46, -78.95, -79.45, -79.94, -80.43, -80.93, -81.42, -81.91, -82.41, -82.9, -83.39, -83.89, -84.38, -84.87, -85.37, -85.86, -85.86, -86.36, -86.86, -87.35, -87.85, -88.33, -88.74, -89.04, -89.18, -89.15, -89.02, -88.89, -88.76, -88.63, -88.5, -88.37, -88.24, -88.11, -87.99, -87.98, -88.15, -88.46, -88.88, -89.3, -89.3, -89.8, -90.3, -90.79, -91.29, -91.79, -92.29, -92.78, -93.28, -93.78, -94.28, -94.33]
        cy = [145.0, 144.5, 144.01, 143.51, 143.01, 142.51, 142.02, 141.52, 141.02, 140.52, 140.03, 139.53, 139.03, 138.54, 138.04, 137.54, 137.04, 136.55, 136.05, 135.55, 135.05, 134.56, 134.06, 133.56, 133.07, 132.57, 132.07, 131.57, 131.08, 130.58, 130.08, 129.58, 129.09, 128.59, 128.09, 127.6, 127.1, 126.6, 126.1, 125.64, 125.26, 125.02, 124.93, 124.97, 125.01, 125.05, 125.1, 125.14, 125.19, 125.23, 125.27, 125.32, 125.36, 125.4, 125.45, 125.49, 125.53, 125.58, 125.62, 125.66, 125.71, 125.75, 125.8, 125.83, 125.83, 125.9, 125.98, 126.06, 126.13, 126.21, 126.29, 126.36, 126.44, 126.52, 126.59, 126.67, 126.75, 126.82, 126.9, 126.98, 127.05, 127.13, 127.21, 127.28, 127.36, 127.44, 127.51, 127.59, 127.67, 127.74, 127.82, 127.9, 127.97, 128.05, 128.13, 128.2, 128.28, 128.36, 128.43, 128.51, 128.59, 128.66, 128.74, 128.82, 128.89, 128.97, 129.05, 129.12, 129.2, 129.28, 129.35, 129.43, 129.51, 129.58, 129.66, 129.74, 129.81, 129.89, 129.97, 130.04, 130.12, 130.2, 130.27, 130.35, 130.43, 130.5, 130.58, 130.66, 130.73, 130.81, 130.89, 130.96, 131.04, 131.12, 131.19, 131.27, 131.35, 131.42, 131.5, 131.58, 131.65, 131.73, 131.81, 131.88, 131.96, 132.04, 132.11, 132.19, 132.27, 132.34, 132.42, 132.5, 132.57, 132.65, 132.73, 132.8, 132.88, 132.96, 133.03, 133.11, 133.19, 133.26, 133.34, 133.42, 133.49, 133.57, 133.65, 133.72, 133.8, 133.88, 133.95, 134.03, 134.11, 134.18, 134.26, 134.34, 134.41, 134.49, 134.49, 134.49, 134.62, 134.9, 135.3, 135.78, 136.27, 136.77, 137.26, 137.75, 138.24, 138.73, 139.22, 139.71, 140.21, 140.7, 141.19, 141.65, 142.03, 142.28, 142.38, 142.37, 142.37, 142.29, 142.21, 142.13, 142.05, 141.97, 141.89, 141.81, 141.73, 141.65, 141.56, 141.48, 141.4, 141.32, 141.24, 141.16, 141.08, 141.0, 140.92, 140.84, 140.75, 140.67, 140.59, 140.51, 140.43, 140.35, 140.27, 140.19, 140.11, 140.03, 139.95, 139.86, 139.78, 139.7, 139.62, 139.54, 139.46, 139.38, 139.3, 139.22, 139.14, 139.05, 138.97, 138.89, 138.81, 138.73, 138.65, 138.57, 138.49, 138.41, 138.33, 138.24, 138.16, 138.08, 138.0, 137.92, 137.84, 137.76, 137.68, 137.6, 137.52, 137.43, 137.35, 137.27, 137.19, 137.11, 137.03, 136.95, 136.87, 136.79, 136.71, 136.63, 136.54, 136.46, 136.38, 136.3, 136.22, 136.14, 136.06, 135.98, 135.9, 135.82, 135.73, 135.65, 135.57, 135.49, 135.41, 135.33, 135.25, 135.17, 135.09, 135.01, 134.92, 134.84, 134.76, 134.68, 134.6, 134.52, 134.44, 134.36, 134.28, 134.2, 134.12, 134.03, 133.95, 133.87, 133.79, 133.71, 133.63, 133.55, 133.47, 133.39, 133.31, 133.31, 133.27, 133.22, 133.18, 133.14, 133.01, 132.73, 132.33, 131.85, 131.35, 130.87, 130.39, 129.9, 129.42, 128.94, 128.46, 127.97, 127.49, 127.01, 126.51, 126.04, 125.65, 125.39, 125.29, 125.29, 125.24, 125.2, 125.15, 125.1, 125.05, 125.0, 124.95, 124.9, 124.85, 124.81, 124.8]
        cyaw = [-1.47, -1.47, -1.47, -1.47, -1.47, -1.47, -1.47, -1.47, -1.47, -1.47, -1.47, -1.47, -1.47, -1.47, -1.47, -1.47, -1.47, -1.47, -1.47, -1.47, -1.47, -1.47, -1.47, -1.47, -1.47, -1.47, -1.47, -1.47, -1.47, -1.47, -1.47, -1.47, -1.47, -1.47, -1.47, -1.47, -1.47, -1.47, -1.37, -1.03, -0.69, -0.35, -0.01, 0.09, 0.09, 0.09, 0.09, 0.09, 0.09, 0.09, 0.09, 0.09, 0.09, 0.09, 0.09, 0.09, 0.09, 0.09, 0.09, 0.09, 0.09, 0.09, 0.09, 0.09, 0.09, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.1, 0.09, 0.09, 0.43, 0.77, 1.11, 1.45, 1.76, 1.76, 1.76, 1.76, 1.76, 1.76, 1.76, 1.76, 1.76, 1.76, 1.76, 2.1, 2.44, 2.79, 3.13, -3.05, -3.05, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -2.98, -3.05, -3.05, -3.05, -3.05, -3.05, -3.05, -2.71, -2.37, -2.03, -1.69, -1.35, -1.31, -1.31, -1.31, -1.31, -1.31, -1.31, -1.31, -1.31, -1.4, -1.74, -2.08, -2.42, -2.76, -3.05, -3.05, -3.04, -3.04, -3.04, -3.04, -3.04, -3.04, -3.04, -3.04, -3.04, -3.04, -3.05]
        goal = [cx[-1], cy[-1]]
        # 初始化起点状态
        state = State(x=cx[0], y=cy[0], yaw=6.0, v=0.0)

        target_speed = 10.0 / 3.6  # [m/s]

        T = 500.0  # max simulation time

        lastIndex = len(cx) - 1
        t = 0.0
        target_course = TargetCourse(cx, cy, cyaw)
        # 向前规划终点路径的indx
        target_ind, _ = target_course.search_target_index(state)

        # while T >= time and lastIndex > target_ind:
        pre_d = None     # 前一个点的误差
        kd_t = 0.0      # 一个周期时间
        pre_time = time.time()  # 上一周期时间
        tracking_flag = True
        # x 轴单位向量
        p_a = np.array([1, 0])
        # 上一点
        pre_position = None
        while True:
            data = self.q_get_nowait()
            if data:
                # 收到订阅的消息
                header = data['header']
                data = data['data']
                # hearder = None
            else:
                header = None
            # print(header)
            if header == 'rtk_position':
                # rtk 位置信息
                position = data
                if position['rtk_mod'] == 4:
                    # 
                    x = position['p'][0]
                    y = position['p'][1]
                    if pre_position is None:
                        pre_position = {
                                'p':[x, y],
                                'is_yaw': False
                                }
                    # 当前点与之前点向量p_b
                    p_b = np.array([x-pre_position['p'][0], y-pre_position['p'][1]])
                    len_b = np.linalg.norm(p_b)
                    # len_b >= 20cm 距离有效,记录当前方向
                    if len_b >= 0.2:
                        dot_ab = p_a.dot(p_b)
                        # len_a = 1 单位向量
                        yaw_ab = np.arccos(dot_ab / (1 * len_b))
                        k_ab = p_b[1] * p_a[0] - p_b[0] * p_a[1]
                        if k_ab < 0:
                            yaw_ab = 6.28 - yaw-ab
                        state.yaw = yaw_ab
                        pre_position['p'] = [x, y]
                        pre_position['is_yaw'] = True
                    state.x = x
                    state.y = y
                    # state.yaw = position['angle']
                    # 路径跟踪
                    if lastIndex > target_ind and tracking_flag:
                        # 计算控制输入
                        # 当前速度与目标速度差×比例因子
                        ai = proportional_control(target_speed, state.v)
                        # 方向控制量，即目标下标
                        di, target_ind = pure_pursuit_steer_control(
                            state, target_course, target_ind)
                    delta_pose = math.sqrt(math.pow(goal[0]-state.x, 2) + math.pow(goal[1]-state.y, 2))
                    if delta_pose < delta_goal and (target_ind + 1 >= lastIndex):
                        # 到达目标点
                        data_speed['angle'] = 0
                        data_speed['y'] = 0
                        rc.publish('move_base_in', json.dumps(data_speed))
                        rc.publish('tcp_out', json.dumps(data_main))
                        print('get_goal')
                        time.sleep(1)
                        rc.publish('move_base_in', json.dumps(data_speed))
                        rc.publish('tcp_out', json.dumps(data_main))
                        tracking_flag = False
                        pre_d = None
                    else:
                        d, dyaw = diff_speed_control(state, target_course, target_ind)
                        now = time.time()
                        kd_t = now - pre_time   # 时间周期
                        pre_time = now
                        #　pid控制参数
                        pid_para = speed_pid(pre_d, d, dyaw, kd_t)
                        pre_d = d
                        data_speed['data']['angle'] = pid_para
                        data_speed['data']['y'] = 1
                        if tracking_flag:
                            # 自动时，发速度指令至电机
                            rc.publish('move_base_in', json.dumps(data_speed))
                            time.sleep(0.2)
                            print('pid: %.2f, d: %.2f, yaw: %.2f'%(pid_para, d, dyaw))
            elif header == 'auto_on':
                # 开启自动, 获取路径任务信息
                url = self.url + data
                response = requests.get(url)
                if response.status_code == 200:
                    try:
                        r_json = response.json()
                        cx = r_json['cx']
                        cx_len = len(cx)
                        cy = r_json['cy']
                        cyaw = r_json['cyaw']
                        if len(cy) == cx_len and len(cyaw) == cx_len:
                            tracking_flag = True
                            lastIndex = len(cx) - 1
                            t = 0.0
                            target_course = TargetCourse(cx, cy, cyaw)
                            # 向前规划终点路径的indx
                            target_ind, _ = target_course.search_target_index(state)
                            goal = [cx[-1], cy[-1]]
                        else:
                            tracking_flag = False
                    except Exception as e:
                        tracking_flag = False
                        print(e)

            elif header == 'auto_off':
                # 关闭自动
                tracking_flag = False
                pre_d = None
            elif header == 'get_pos':
                # 当前坐标发送
                pos_msg = {
                        'header': 'cur_pos',
                        'data': [state.x, state.y, state.yaw]
                        }
                rc.publish('tcp_in', json.dumps(pos_msg))
            elif header == 'pos_init':
                # 初始化机器人位姿, 以20%的速度向前运动
                pre_t = time.time()
                while True:
                    now = time.time()
                    data_speed['data']['angle'] = 0
                    data_speed['data']['y'] = 1
                    t_speed = data_speed['data']['speed']
                    data_speed['data']['speed'] = 20
                    # 发速度指令至电机
                    rc.publish('move_base_in', json.dumps(data_speed))
                    time.sleep(0.2)
                    print('pid: %.2f, d: %.2f, yaw: %.2f'%(pid_para, d, dyaw))
                    data_speed['data']['speed'] = t_speed
                    if now - pre_t > 2:
                        break
                # 电机停止
                data_speed['data']['angle'] = 0
                data_speed['data']['y'] = 0
                rc.publish('move_base_in', json.dumps(data_speed))

        # Test
        assert lastIndex >= target_ind, "Cannot goal"




if __name__ == '__main__':
    print("Pure pursuit path tracking start")
    node_track = tracking()
    node_track.run()
