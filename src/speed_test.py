# encoding: utf-8
"""

Mobile robot motion planning sample with Dynamic Window Approach

author: Bran Yuan

"""

import math
import time
import sys
import json
import requests
import redis
import scipy.spatial

import matplotlib.pyplot as plt
import numpy as np

from copy import deepcopy
from enum import Enum

from redisHandler import redisHandler

show_animation = True



class dwa(redisHandler):
    '''
    # dwa 路径规划
    '''
    def __init__(self):
        redisHandler.__init__(self)
        
        self.sub_topics = ['tracking_in']
        self.pub_topics = ['ctrl_in']

        self.start_sub()


    def run(robot_type=RobotType.rectangle):
        print(__file__ + " start!!")
        # 自动运行标志
        tracking_flag = False
        # 初始化机器人状态 [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
        x = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        pre_x = None
        pre_t = None
        while True:
            # redis 消息
            data = self.q_get_nowait()
            header = None
            if data:
                header = data['header']
                data = data['data']

            if header == 'rtk_position':
                now = time.time()
                if not tracking_flag:
                    # 如果自动未开启，则跳过
                    continue
                # 自动运行主模块
                position = data
                if position['rtk_mod'] == 4:
                    # 自动时，发速度指令至电机
                    x[0] = position['p'][0]
                    x[1] = position['p'][1]
                    x[2] = position['angle']
                    if pre_t is None:
                        # 时间初始化
                        pre_t = now
                    if pre_x is None:
                        # 前一状态初始化
                        pre_x = deepcopy(x)
                    else:
                        # 线速度，角速度计算
                        tmp_dx = x[0] - pre_x[0]
                        tmp_dy = x[1] - pre_x[1]
                        tmp_dist = math.hypot(tmp_dx, tmp_dy)
                        tmp_dt = now - pre_t
                        if tmp_dt < 0.02:
                            # 防止分母为0
                            x[3] = 0
                            x[4] = 0
                        else:
                            x[3] = tmp_dist / tmp_dt
                            x[4] = (x[2] - pre_x[2]) / tmp_dt
                        # 速度，角速度
                        print(round(x[3], 3), round(x[4], 3))
                    pre_t = now




if __name__ == '__main__':
    # main(robot_type=RobotType.circle)
    robot = dwa()
    robot.run()
