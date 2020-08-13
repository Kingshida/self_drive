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
        # return math.sqrt(dx**2 + dy**2)


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
            # 欧几里得范数
            d = np.hypot(dx, dy)
            # print(d)
            #　最小值下标
            ind = np.argmin(d)
            # 求第二小距离下标
            ind_max = np.argmax(d)
            d[ind] = d[ind_max]
            ind_2nd = np.argmin(d)
            if ind - 5 > ind_2nd:
                # 如果第二小的下标比最小下标小很多
                # 取第二小下标，防止起点路径与后续路径重合时
                # 取到后续路径的最小点为开始点，从而丢失部分路径
                ind = ind_2nd
            self.old_nearest_point_index = ind
        else:
            # 更新上一个最近点下标
            ind = self.old_nearest_point_index
            # 实际点与当前点距离
            distance_this_index = state.calc_distance(self.cx[ind], self.cy[ind])
            length = len(self.cx)
            while True:
                # 实际与下一点距离
                if ind + 1 >= length:
                    # 当前点已经最后一点
                    ind = length -1
                    break
                distance_next_index = state.calc_distance(self.cx[ind + 1], self.cy[ind + 1])
                # print(ind, round(distance_this_index, 3), round(distance_next_index, 3), self.cx[ind], self.cy[ind],self.cx[ind + 1], self.cy[ind + 1])
                '''
                plt.cla()               
                # ox = [-105, -85, -85, -105]
                # oy = [132, 132, 155, 155]
                ox = [-100, -95, -95, -100]
                oy = [132, 132, 140, 140]
                plt.plot(ox, oy, "-r", label="course")
                plt.plot(state.x, state.y, "xr", label="target")
                plt.plot(self.cx[ind], self.cy[ind], "xg", label="target")
                plt.plot(self.cx[ind+1], self.cy[ind+1], "xb", label="target")
                plt.axis("equal")
                plt.grid(True)
                plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
                plt.pause(0.001)
                '''
                if distance_this_index < distance_next_index:
                    # 找到最近点
                    break
                ind = ind + 1 if (ind + 1) < length else ind
                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind

        # Lf = k * state.v + Lfc  # 根据当前速度更新向前规划距离
        Lf = Lfc  # 向前规划距离

        # 搜寻向前规划终点下标
        while Lf > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                # 最后一点为终点
                break
            ind += 1

        # print('%.2f, %.2f'%(self.cx[ind], self.cy[ind]))
	self.old_nearest_point_index = ind-1
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
        # ind只能向前前进
        # print('--------------------')
        ind = pind

    if ind < len(trajectory.cx):
        tx = trajectory.cx[ind]
        ty = trajectory.cy[ind]
    else:
        # 最后一点下标
        tx = trajectory.cx[-1]
        ty = trajectory.cy[-1]
        ind = len(trajectory.cx) - 1
    # 当前坐标 
    x0 = state.x
    y0 = state.y
    yaw0 = state.yaw
    # 当前点(车中心点)沿着yaw0方向平移0.5m(即车头位置), 放大角度变化导致的位置变化
    c, s = math.cos(yaw0), math.sin(yaw0)
    rx = 0.5
    ry = 0
    tx = c * rx - s * ry
    ty = s * rx + c * ry
    x0 += tx
    y0 += ty
    # 目标点坐标,相应平移tx, ty
    x1  = trajectory.cx[ind] + tx
    y1  = trajectory.cy[ind] + ty
    yaw1  = trajectory.cyaw[ind]
    # 当前点姿态向量，为单位向量
    p_b = np.array([math.cos(yaw0), math.sin(yaw0)])
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
    
    yaw_bc = (yaw0 % (2 * math.pi) - yaw1 % (2 * math.pi)) % (2 * math.pi)
    direction = 1
    # print(yaw0, yaw1, yaw_bc)
    # if dot_ba < 0 and dot_bc > 0:
    if (yaw_bc < 1.57 or yaw_bc > 4.71) and dot_ba < 0:
        # 点在后方，目标姿态同向，需倒车
        direction = -1

    return d, yaw_ba, direction, ind, dot_ba

'''
def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
    """
    Plot arrow
    """

    if not isinstance(x, float):
        for ix, iy, iyaw in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)
# '''


def speed_pid(pre_d, d, dyaw, dt, direction):
    # 轮差机器人pid控制
    if not pre_d:
        pre_d = d
    kp = 0.900
    kd = 0.8000
    
    k = 0.0
    angle = math.radians(30)
    if direction > 0:
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
    elif direction < 0:
        if dyaw > 3.14 + angle:
            k = -1.57
        elif dyaw < 3.14 - angle:
            k = 1.57
        else:
            k = kp * d + kd * (d - pre_d) / dt
            if k > 1.5:
                k = 1.5
            elif k < -1.5:
                k = -1.5
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
        self.config = config('para_track.yaml')
        self.start_sub()

    def get_mission(self, key, state):
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
                    lastIndex = cx_len - 1
                    goal = [cx[-1], cy[-1]]
                    # 路径点
                    target_course = TargetCourse(cx, cy, cyaw)
                    # 向前规划终点路径的indx
                    print('get_mission: {}'.format(key))
                    target_ind, _ = target_course.search_target_index(state)
                    return [cx, cy, cyaw, lastIndex, goal, target_course, target_ind]
            except Exception as e:
                print(e)
        return None


    def run(self):
        # 路径跟踪方法
        rc = self.rc
        # 目标误差
        delta_goal = 0.25
        # 最大速度
	max_speed = 90
        speed_scale = 1
        # to movebase通信格式
        data_speed = {
                'header':'speed',
                'data':{
                    'y': 1,
                    'angle': 0,
                    'speed': max_speed
                    }
                }
        # redis 通信格式
        data_main = {
                'header':'auto_off',
                'data': ''
                }
        # 记录任务运行过程的配置信息
        data_config = {
                    'key':'none',
                    'target_ind':0,
                    'old_ind':0,
                    'path_len':0
                    }
        para_mission_key = 'mission'
        if not para_mission_key in self.config.config:
            self.config.set_para(para_mission_key, data_config)
        # 路径点
        cx = [-99.05, -99.0, -98.95]
        cy = [145.0, 144.5, 144.01]
        cyaw = [-1.47, -1.47, -1.47]
        goal = [cx[-1], cy[-1]]
        # 初始化起点状态
        state = State(x=cx[0], y=cy[0], yaw=6.0, v=0.0)

        target_speed = 10.0 / 3.6  # [m/s]

        T = 500.0  # max simulation time

        lastIndex = len(cx) - 1
        t = 0.0
        target_course = TargetCourse(cx, cy, cyaw)
        # 第一个向前规划路径点的indx
        target_ind, _ = target_course.search_target_index(state)

        # while T >= time and lastIndex > target_ind:
        pre_d = None     # 前一个点的误差
        kd_t = 0.0      # 一个周期时间
        pre_time = time.time()  # 上一周期时间
        tracking_flag = False
        tmp_flag = False
        while True:
            data = self.q_get_nowait()
            if not tracking_flag:
                # 如果在自动未运行, sleep
                time.sleep(0.1)
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
                # if position['rtk_mod'] == 4 or position['rtk_mod'] == 5:
                if position['rtk_mod'] == 4:
                    state.x = position['p'][0]
                    state.y = position['p'][1]
                    state.yaw = position['angle']
                    if not tracking_flag:
                        # 如果在自动未运行, 跳过
                        continue
                    # 自动运行主要程序模块
                    delta_pose = math.sqrt(math.pow(goal[0]-state.x, 2) + math.pow(goal[1]-state.y, 2))

                    if (delta_pose < delta_goal * 20) and (target_ind + 20 >= lastIndex) and tracking_flag:
                        # 接近目标点，减速
                        data_speed['data']['speed'] = max_speed * 0.3
                    if delta_pose < delta_goal and (target_ind + 1 >= lastIndex) and tracking_flag:
                        # 到达目标点
                        data_speed['data']['angle'] = 0
                        data_speed['data']['y'] = 0
                        rc.publish('tcp_out', json.dumps(data_main))
                        print('get_goal')
                        time.sleep(1)
                        rc.publish('move_base_in', json.dumps(data_speed))
                        rc.publish('tcp_out', json.dumps(data_main))
                        tracking_flag = False
                        pre_d = None
                        data_speed['data']['speed'] = max_speed * speed_scale
                    else:
                        # 获取跟踪路径信息 
                        d, dyaw, direction, target_ind, dotba = diff_speed_control(state, target_course, target_ind)
                        now = time.time()
                        kd_t = now - pre_time   # 时间周期
                        pre_time = now
                        #　pid控制参数
                        pid_para = speed_pid(pre_d, d, dyaw, kd_t, direction)
                        pre_d = d
                        data_speed['data']['angle'] = pid_para
                        data_speed['data']['y'] = direction
                        if tracking_flag:
                            # 自动开启
                            # 存储自动运行记录
                            data_config['target_ind'] = int(target_ind)
                            data_config['old_ind'] = int(target_course.old_nearest_point_index)
                            self.config.set_para(para_mission_key, data_config)
                            # 发速度指令至电机
                            rc.publish('move_base_in', json.dumps(data_speed))
                            time.sleep(0.2)
                            print('direction: %d, pid: %.2f, d: %.2f, yaw: %.2f, dotab: %.2f, ind: %d'%(direction, pid_para, d, dyaw, dotba, target_ind))
            elif header == 'auto_on':
                # 开启自动, 获取路径任务信息
                if tracking_flag:
                    # 如果在自动运行中，则跳过
                    continue
                mission = self.get_mission(data, state)
                if mission:
                    cx, cy, cyaw, lastIndex, goal, target_course, target_ind = mission
                    t = 0.0
                    data_config = {
                                'key':data,
                                'target_ind':0,
                                'old_ind':0,
                                'path_len':len(cx)
                                }
                    # 存储本次mission信息
                    self.config.set_para(para_mission_key, data_config)
                    tracking_flag = True
                else:
                    tracking_flag = False
                del mission
            elif header == 'auto_continue':
                # 继续上次自动任务
                print('aotu_continue', tracking_flag)
                if tracking_flag:
                    # 如果在自动运行中，则跳过
                    continue
                data_config = self.config.config[para_mission_key]
                data = data_config['key']
                path_len = data_config['path_len']
                mission = self.get_mission(data, state)
                if mission:
                    cx, cy, cyaw, lastIndex, goal, target_course, target_ind = mission
                    print('near the last point',len(cx) == path_len and target_ind + 3 < path_len)
                    if len(cx) == path_len and target_ind + 3 < path_len:
                        # 验证服务器路径未更改，且上次断点位置不在最终目标附近
                        target_ind = self.config.config[para_mission_key]['target_ind']
                        tmp_x = cx[target_ind]
                        tmp_y = cy[target_ind]
                        tmp_d = math.sqrt((tmp_x-state.x) ** 2 + (tmp_y-state.y) ** 2)
                        print('tmp_d', tmp_d)
                        if tmp_d < 1.50:
                            # 当前位置与目标位置距离小于1.5m方可继续上一次任务
                            target_course.old_nearest_point_index = self.config.config[para_mission_key]['old_ind']
                            t = 0.0
                            tracking_flag = True
                        del tmp_x, tmp_y, tmp_d
            elif header == 'auto_off':
                # 关闭自动
                tracking_flag = False
                pre_d = None
                # 停止运行
                data_speed['data']['angle'] = 0
                data_speed['data']['y'] = 0
                rc.publish('move_base_in', json.dumps(data_speed))
                # rc.publish('tcp_out', json.dumps(data_speed))
            elif header == 'get_pos':
                # 当前坐标发送
                pos_msg = {
                        'header': 'cur_pos',
                        'data': [state.x, state.y, state.yaw]
                        }
                rc.publish('tcp_in', json.dumps(pos_msg))

            elif header == 'speed':
                # 最高速度
                speed_scale = data
                data_speed['data']['speed'] = max_speed * speed_scale
                print(data_speed)

            '''
            if show_animation:  # pragma: no cover
                plt.cla()
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect(
                    'key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
                
                # ox = [-105, -85, -85, -105]
                # oy = [132, 132, 155, 155]
                ox = [-100, -95, -95, -100]
                oy = [132, 132, 140, 140]
                plt.plot(ox, oy, "-r", label="course")
                # plot_arrow(state.x, state.y, state.yaw)
                plt.plot(state.x, state.y, "xr", label="cur")
                tmpind = target_course.old_nearest_point_index
                # plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
                # plt.plot(cx[tmpind], cy[tmpind], "xb", label="target")
                plt.axis("equal")
                plt.grid(True)
                plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
                plt.pause(0.001)
            # '''

        # Test
        assert lastIndex >= target_ind, "Cannot goal"




if __name__ == '__main__':
    print("Pure pursuit path tracking start")
    node_track = tracking()
    node_track.run()
