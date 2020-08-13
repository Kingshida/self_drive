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
import scipy.spatial
import numpy as np
from redisHandler import redisHandler
from config import config


class track_cmd(redisHandler):
    '''
    # tracking 路径跟踪
    '''
    def __init__(self):
        redisHandler.__init__(self)
        self.url = 'http://117.184.129.18:8000/planning/query/?key='
        self.sub_topics = ['rtk_out', 'tracking_in']
        self.pub_topics = ['ctrl_in']
        self.config = config('para_cmd.yaml') 
        # 上一次命令内容
        self.last_dict_cmd = []
        self.last_list_cmd = []
        # 任务目标kdtree
        self.kdtree = None
        # 任务列表
        self.cmd_list=[]
        # 目标点误差, list型
        self.delta = 0.3
        self.delta_goal_list = 1.4
        # dict类型数据目标点误差
        self.delta_goal = 0.4
        # 发布至ctrl cmd的格式
        self.data_cmd = {
                'header':'cmd',
                'data': ''
                }
        # 任务运行记录格式
        self.data_config = {
                    'key':'none',
                    'cmd':[]
                    }
        # 任务记录的字典key值
        self.para_mission_key = 'mission'
        if not self.para_mission_key in self.config.config:
            self.config.set_para(self.para_mission_key, self.data_config)
        self.start_sub()

    def get_mission(self, key):
        # 从服务器获取任务
        url = self.url + key
        response = requests.get(url)
        cmd_list = []
        if response.status_code == 200:
            try:
                r_json = response.json()
                cmd_list = r_json['cmd']
            except Exception as e:
                print(e)
        # time.sleep(30)
        return cmd_list

    def exc_cmd_list(self, x, y, cmd_list):
        for i in range(len(cmd_list)):
            item = cmd_list[i]
            cmd = item[1]
            if self.last_list_cmd == cmd:
                # 与上次执行的命令一样,跳过
                continue
            # 当前点p0
            x0 = item[0][0]
            y0 = item[0][1]
            # 当前点垂直点角度
            yaw0 = item[0][2]
            # 当前点-->目标点向量p_b
            p_b = np.array([x-x0, y-y0])
            len_b = np.linalg.norm(p_b)
            if len_b <= self.delta_goal_list:
                # 前进方向p_d
                p_d = np.array([math.cos(yaw0), math.sin(yaw0)])
                yaw1 = yaw0 + 1.5708
                # 向量p_a
                p_a = np.array([math.cos(yaw1), math.sin(yaw1)])
                len_a = np.linalg.norm(p_a)
                # 向量p_b在向量p_a上投影的模，角角>0, 钝角<0
                len_c = p_a.dot(p_b)
                # 投影向量
                p_c = len_c * p_a
                # 目标到p_a垂足点-->目标点向量
                p_e = p_b - p_c
                len_e = np.linalg.norm(p_e)
                # dot_ab = p_a.dot(p_b)
                # k_ab = p_b[1] * p_a[0] - p_b[0] * p_a[1]
                k = np.inner(p_d, p_b)
                # print(round(len_e, 2), round(len_b, 2), round(k, 2))
                if len_e <= self.delta and k > 0:
                    # 当前点在yaw0的锐角方向，且误差小于delta_goal_list
                    # 保存运行记录
                    self.data_config['cmd'] = cmd
                    self.config.set_para(self.para_mission_key, self.data_config)

                    self.last_list_cmd = cmd
                    print('list cmd is: {}'.format(cmd))
                    for i_cmd in cmd:
                        if 'speed:' in i_cmd:
                            # 改变速度
                            try:
                                speed = float(i_cmd.split(":")[-1])
                                tmp_data = {
                                        'header':'speed',
                                        'data': speed
                                        }
                                self.rc.publish('tracking_in', json.dumps(tmp_data))
                            except Exception as e:
                                print(e)
                            continue
                        self.data_cmd['data'] = i_cmd
                        self.pub_all(self.data_cmd)
                        self.last_list_cmd = i_cmd
                        time.sleep(0.2)
                    return i
        return None


    def exc_cmd_dict(self, x, y, yaw, cmd_list):
        # 当前点(车中心点)沿着yaw反方向平移(即车尾位置)
        c, s = math.cos(yaw), math.sin(yaw)
        rx = 1.20 
        ry = 0 
        tx = c * rx - s * ry
        ty = s * rx + c * ry
        x -= tx
        y -= ty
        # 与当前点垂直的点姿态
        yaw += 1.5708
        # 姿态向量，为单位向量
        p_b = np.array([math.cos(yaw), math.sin(yaw)])
        # p_b向量摸
        len_b = np.linalg.norm(p_b)
        # 目标半径范围 /m
        r = 3
        # 查询半径范围内点
        # print(x, y)
        # print(self.kdtree.data)
        inds = self.kdtree.query_ball_point([x, y], r)
        #print(self.kdtree.query([x, y]))
        # 靠近目标标志
        is_close = False
        # 命令下标
        cmd_ind = 0
        for pind in inds:
            # 检索点与车尾位置，垂直于车尾的向量的距离是否小于delta_goal
            point = self.kdtree.data[pind]
            # 目标点
            x1, y1 = point[0], point[1]
            # 当前点到目标点向量
            p_a = np.array([x1-x, y1-y])
            # 当前点到目标点距离, 即p_a向量模
            len_a = np.linalg.norm(p_a)
            # p_a向量在p_b向量上的投影向量模，锐角>0, 钝角<0
            len_c = p_b.dot(p_a)
            # 投影向量
            p_c = len_c * p_b
            # 目标到p_b垂足点-->目标点向量
            p_e = p_a - p_c
            len_e = np.linalg.norm(p_e)
            '''
            # 向量模，即当前点到目标向量距离, 与当前点到目标点距离的比值
            if len_a > 0.005:
                d = np.linalg.norm(p_e) / len_a
            else:
                d = 0
            d = abs(d)
            # print(point,d)
            '''
            if len_e <= self.delta_goal:
                is_close = True

        if is_close:
            cmd_ind = 1
        cmd = cmd_list['cmd'][cmd_ind]
        if self.last_dict_cmd == cmd:
            # 与上次执行的命令一样,跳过
            return
        # 保存运行记录
        # self.data_config['cmd'] = cmd
        # self.config.set_para(self.para_mission_key, self.data_config)
        self.last_dict_cmd = cmd
        print('dict cmd is: {}'.format(cmd))
        for i_cmd in cmd:
            self.data_cmd['data'] = i_cmd
            self.pub_all(self.data_cmd)
            time.sleep(0.2)


    def run(self):
        # 路径跟踪方法
        # 命令路径点表
        tracking_flag = False
        # 命令执行位置公差, 单位m
        while True:
            time.sleep(0.1)
            data = self.q_get_nowait()
            header = None
            if data:
                header = data['header']
                data = data['data']
            if header == 'rtk_position':
                if not tracking_flag:
                    # 如果自动未开启，则跳过
                    continue
                # 自动运行主模块
                position = data
                if position['rtk_mod'] == 4:
                    # 自动时，发速度指令至电机
                    x = position['p'][0]
                    y = position['p'][1]
                    yaw = position['angle']
                    try:
                        for i in range(len(self.cmd_list)):
                            cmd_list = self.cmd_list[i]
                            if isinstance(cmd_list, list):
                                ind = self.exc_cmd_list(x, y, cmd_list)
                                if ind is not None:
                                    # pass
                                    self.cmd_list[i].pop(ind)
                            elif isinstance(cmd_list, dict):
                                self.exc_cmd_dict(x, y, yaw, cmd_list)
                    except Exception as e:
                        print(e)

            elif header == 'auto_on':
                print('cmd auto on')
                # 开启自动, 获取任务信息
                if tracking_flag:
                    # 如果在自动运行中，则跳过
                    continue
                self.cmd_list = self.get_mission(data)
                if self.cmd_list:
                    tracking_flag = True
                    self.data_config['key'] = data
                    self.config.set_para(self.para_mission_key, self.data_config)
                    if isinstance(self.cmd_list[0], dict):
                        # 任务目标是dict类型时
                        cx = self.cmd_list[0]['cx']
                        cy = self.cmd_list[0]['cy']
                        # 任务目标点kdtree
                        self.kdtree = scipy.spatial.cKDTree(np.vstack((cx, cy)).T)


            elif header == 'auto_continue':
                # 继续上次任务
                if tracking_flag:
                    # 如果在自动运行中，则跳过
                    continue
                self.data_config = self.config.config[self.para_mission_key]
                data = self.data_config['key']
                tmp_cmd = self.data_config['cmd']
                self.cmd_list = self.get_mission(data)
                if self.cmd_list:
                    tracking_flag = True
                    for i_cmd in tmp_cmd:
                        # 恢复上次任务的命令
                        self.data_cmd['data'] = i_cmd
                        self.pub_all(self.data_cmd)
                        time.sleep(0.2)
                    if isinstance(self.cmd_list[0], dict):
                        # 任务目标是dict类型时
                        cx = self.cmd_list[0]['cx']
                        cy = self.cmd_list[0]['cy']
                        # 任务目标点kdtree
                        self.kdtree = scipy.spatial.cKDTree(np.vstack((cx, cy)).T)
                print('auto continue', tracking_flag)
                del tmp_cmd

            elif header == 'auto_off':
                # 关闭自动
                print('auto_off')
                tracking_flag = False


if __name__ == '__main__':
    print("Pure pursuit path tracking start")
    node_track = track_cmd()
    node_track.run()
