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
# import matplotlib.pyplot as plt
import numpy as np
from redisHandler import redisHandler
from config import config


class tracking(redisHandler):
    '''
    # tracking 路径跟踪
    '''
    def __init__(self):
        redisHandler.__init__(self)
        self.url = 'http://117.184.129.18:8000/planning/query/?key='
        self.sub_topics = ['tracking_in']
        self.pub_topics = ['rtk_out']
        self.__position = {
                'p':[-51.005,164.012],
                'angle': 4.8,
                'precision': 0.0,
                # 'angle_precision':0.0,
                'rtcm':'',
                'rtk_mod':4
                }
        self.start_sub()

    def get_mission(self, key):
        # 从服务器获取任务
        url = self.url + key
        response = requests.get(url)
        if response.status_code == 200:
            try:
                r_json = response.json()
                cx = r_json['cx']
                cx_len = len(cx)
                cy = r_json['cy']
                cyaw = r_json['cyaw']
                if len(cy) == cx_len and len(cyaw) == cx_len:
                    # 向前规划终点路径的indx
                    print('get_mission: {}'.format(key))
                    return [cx, cy, cyaw]
            except Exception as e:
                print(e)
        return None


    def run(self):
        # 路径跟踪方法
        pos_index = 0
        cx = [0]
        cy = [0]
        cyaw = [0]
        len_cx = 1
        tracking_flag = False
        pub_data = {
                    'header': 'rtk_position',
                    'data':self.__position
                    }
        num = 5
        while True:
            data = self.q_get_nowait()
            if not tracking_flag:
                # 如果在自动未运行, sleep
                self.pub_all(pub_data)
                time.sleep(0.1)
            else:
                if pos_index < len_cx - 1:
                    x = cx[pos_index]
                    y = cy[pos_index]
                    yaw = cyaw[pos_index]
                    pub_data['data']['p'] = [x, y]
                    pub_data['data']['angle'] = yaw
                    pos_index += 1
                    n_x = cx[pos_index]
                    n_y = cy[pos_index]
                    d = math.sqrt((n_x - x)** 2 + (n_y - y) ** 2) / num
                    dx = math.cos(yaw) * d
                    dy = math.sin(yaw) * d
                    for i in range(num):
                        x = round(x + dx * i, 3)
                        y = round(y + dy * i, 3)
                        pub_data['data']['p']  = [x, y]
                        self.pub_all(pub_data)
                        time.sleep(0.2)

            if data:
                # 收到订阅的消息
                header = data['header']
                data = data['data']
                # hearder = None
            else:
                header = None
            # print(header)
            if header == 'auto_on':
                # 开启自动, 获取路径任务信息
                if tracking_flag:
                    # 如果在自动运行中，则跳过
                    continue
                mission = self.get_mission(data)
                if mission:
                    cx, cy, cyaw = mission
                    len_cx = len(cx)
                    tracking_flag = True
                else:
                    tracking_flag = False
                pos_index = 0
                del mission
            elif header == 'auto_continue':
                # 继续上次自动任务
                print('aotu_continue', tracking_flag)
                if tracking_flag:
                    # 如果在自动运行中，则跳过
                    continue
                tracking_flag = True
            elif header == 'auto_off':
                # 关闭自动
                tracking_flag = False


if __name__ == '__main__':
    node_track = tracking()
    node_track.run()
