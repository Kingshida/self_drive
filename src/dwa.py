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


def dwa_control(x, config, goal, ob):
    """
    Dynamic Window Approach control
    """
    # 得到dynamic window参数, vmin, vmax
    # dw = calc_dynamic_window(x, config)
    dw = calc_dw_speed(x[3], config)
    # 计算轨迹,得到最优路径及最优速度
    u, trajectory = calc_control_and_trajectory(x, dw, config, goal, ob)
    return u, trajectory


class RobotType(Enum):
    circle = 0
    rectangle = 1


class Config:
    """
    仿真参数类
    """

    def __init__(self):
        # 机器人参数
        self.wheel_width = 1.5      # 轮间距[m/s]
        self.max_speed = 0.60  # [m/s]
        self.min_speed = -0.60  # [m/s]
        self.max_yawrate = self.max_speed * self.wheel_width / 2  # [rad/s]
        self.max_accel = 0.6  # [m/ss]
        self.max_dyawrate = self.max_yawrate  # [rad/ss]
        self.v_reso = 0.1  # 速度分辨率[m/s]
        self.yawrate_reso = 5 * math.pi / 180.0  # 角速度分辨率[rad]
        self.dt = 0.2  # [s] 提前规划的时间分辨率
        self.predict_time = 5.0  # [s]提前规划总时间
        self.to_goal_cost_gain = 0.2
        self.dist_cost_gain = 0.5
        self.to_omega_cost_gain = 0.0
        self.speed_cost_gain = 1.50
        self.obstacle_cost_gain = 0.30
        self.robot_type = RobotType.circle
        self.pre_traj = None        # 上一次规划的路径最后一点状态

        # 圆形机器人半径，也是两种外形机器人到达目标位置的误差
        self.robot_radius = 1.2  # [m] 用于障碍物检测范围，半径

        # 方形机器人模型：前，后，左, 右
        self.robot_f = 0.5
        self.robot_b = 0.5
        self.robot_l = 0.5
        self.robot_r = 0.5

    @property
    def robot_type(self):
        return self._robot_type

    @robot_type.setter
    def robot_type(self, value):
        if not isinstance(value, RobotType):
            raise TypeError("robot_type must be an instance of RobotType")
        self._robot_type = value


def motion(x, u, dt):
    """
    # 运动模型
    # x: [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    # u: v, yaw
    """
    x[2] += u[1] * dt
    x[0] += u[0] * math.cos(x[2]) * dt
    x[1] += u[0] * math.sin(x[2]) * dt
    x[3] = u[0]
    x[4] = u[1]

    return x


def calc_dynamic_window(x, config):
    """
    基于当前状态x计算Dynamic window 参数
    """

    # 机器人规格的Dynamic window参数
    Vs = [config.min_speed, config.max_speed,
          -config.max_yawrate, config.max_yawrate]

    # 运动模型的Dynamic window参数
    Vd = [x[3] - config.max_accel * config.dt,
          x[3] + config.max_accel * config.dt,
          x[4] - config.max_dyawrate * config.dt,
          x[4] + config.max_dyawrate * config.dt]

    #  [vmin, vmax, yaw_rate min, yaw_rate max]
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

    return dw


def calc_dw_speed(v, config):
    """
    基于当前状态v计算Dynamic window 参数
    """

    # 机器人的Dynamic window参数
    Vs = [config.min_speed, config.max_speed]

    # 运动模型的Dynamic window参数
    Vd = [v - config.max_accel * config.dt,
          v + config.max_accel * config.dt,
          ]
    #  [vmin, vmax]
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1])]

    return dw

def calc_dw_yawrate(x, v, config):
    """
    基于当前状态x计算Dynamic window 参数
    """
    # 机器人规格的Dynamic window参数
    # 速度变化
    dv = abs(v - x[3])
    # 时间dt内最大速度变化
    dv_max = config.max_accel * config.dt
    # 剩余可用于改变角度的速度变化量
    rest_dv = dv_max - abs(dv)
    if rest_dv <= 0:
        # 角速度不可变
        dw = [0, 0]
    else:
        dyaw = rest_dv / (config.wheel_width/2)
        yaw_max = x[4] + dyaw
        yaw_min = x[4] - dyaw
        dw = [yaw_min, yaw_max]
    return dw
    

def predict_trajectory(x_init, v, y, config, goal):
    """
    基于当前input规划路径
    """
    # 当前机器人状态
    x = np.array(x_init)
    # 路径
    traj = np.array(x)
    cur_time = 0
    while cur_time <= config.predict_time:
        # 计算当前速度经过dt后的状态x
        x = motion(x, [v, y], config.dt)
        traj = np.vstack((traj, x))
        cur_time += config.dt
    return traj


def calc_control_and_trajectory(x, dw, config, goal, ob):
    """
    基于dynamic window计算最终输入
    """

    x_init = x[:]   # 初始化路径,当前状态x
    min_cost = float("inf") # 正无穷
    best_u = [0.0, 0.0]
    best_trajectory = np.array([x]) # 最佳路径

    # 评估输入的所有采样点的dynamic window 路径
    for v in np.arange(dw[0], dw[1], config.v_reso):
        # 速度遍历外循环
        dw_yawrate = calc_dw_yawrate(x, v, config)
        # for y in np.arange(dw[2], dw[3], config.yawrate_reso):
        for y in np.arange(dw_yawrate[0], dw_yawrate[1], config.yawrate_reso):
            # 角速度遍历内循环
            # 基于当前状态及目标速度v, y经过一段时间后的轨迹
            trajectory = predict_trajectory(x_init, v, y, config, goal)
            # 计算路径代价
            # 目标点位姿偏差代价
            angle_cost, dist_cost = calc_to_pose_cost(trajectory, goal, config.pre_traj)
            to_pose_cost = config.to_goal_cost_gain * angle_cost
            dist_cost = config.dist_cost_gain * dist_cost
            # 角速度变化代价
            to_omega_cost = config.to_omega_cost_gain * calc_to_omega_cost(trajectory[-1], config.pre_traj, dw[-1])
            # 速度代价, 即速度优先
            speed_cost = config.speed_cost_gain * (config.max_speed - trajectory[-1, 3])
            # 障碍物代价
            ob_cost = config.obstacle_cost_gain * calc_obstacle_cost(trajectory, ob, config)
            if ob_cost > 5:
                speed_cost = 0
            # 总代价
            final_cost = to_pose_cost + speed_cost + ob_cost + dist_cost + to_omega_cost
            print("to_goal: %.2f, speed: %.2f, obmega: %.2f, ob_cost: %.2f, dist_cost: %.2f" % (to_pose_cost, speed_cost, to_omega_cost, ob_cost, dist_cost))
            # 找到最小代价路径
            if min_cost >= final_cost:
                min_cost = final_cost
                if final_cost > 9990:
                    # 与障碍物接触
                    y = 0
                    v = 0
                best_u = [v, y]
                best_trajectory = trajectory
                config.pre_traj = best_trajectory[-1]
    # 返回速度和路径
    return best_u, best_trajectory


def calc_obstacle_cost(trajectory, ob, config):
    """
        calc obstacle cost inf: collision
    """
    ox = ob[:, 0]
    oy = ob[:, 1]
    dx = trajectory[:, 0] - ox[:, None]
    dy = trajectory[:, 1] - oy[:, None]
    r = np.hypot(dx, dy)
    # 碰撞检测
    ob_r = 0
    if config.robot_type == RobotType.rectangle:
        # 方形机器人
        yaw = trajectory[:, 2]
        rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        rot = np.transpose(rot, [2, 0, 1])
        local_ob = ob[:, None] - trajectory[:, 0:2]
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])
        # local_ob = np.array([local_ob @ x for x in rot])
        local_ob = np.array([local_ob.dot(x) for x in rot])
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])
        upper_check = local_ob[:, 0] <= config.robot_f 
        right_check = local_ob[:, 1] <= config.robot_r
        bottom_check = local_ob[:, 0] >= -config.robot_b
        left_check = local_ob[:, 1] >= -config.robot_l
        ob_r = max((config.robot_f ,config.robot_b, config.robot_l, config.robot_r))
        if (np.logical_and(np.logical_and(upper_check, right_check), np.logical_and(bottom_check, left_check))).any():
            return 9999 # float("Inf")
    elif config.robot_type == RobotType.circle:
        # 圆形机器人
        ob_r = config.robot_radius
        if (r <= config.robot_radius).any():
            return 9999 # float("Inf")

    min_r = np.min(r) - ob_r
    if min_r > 20:
        min_r = 0.0
    else:
        min_r = 1.0 / min_r
    return min_r


def calc_to_pose_cost(trajectory, goal, pre_traj):

    # 计算到达目标点的方向误差代价值
    dx = goal[0] - trajectory[-1, 0]
    dy = goal[1] - trajectory[-1, 1]
    dist = math.sqrt(dx ** 2 + dy ** 2)
    # print("%.4f, %.4f" % (dx, dy))
    # 轨迹最后一点-->目标点向量与x夹角
    error_angle = math.atan2(dy, dx)
    # 轨迹最后一点角度与error_angle角度差
    cost = error_angle - trajectory[-1, 2]
    # 0 to pi
    cost = (cost + math.pi) % (2 * math.pi) - math.pi
    cost = abs(cost)
    # print("%.4f, %.4f" % (cost, position_error))

    # 计算距离目标点变化的代价
    # 上一次轨迹与目标点距离
    dist_cost = 0.0
    if cost > 2.6:
        # 角度大于150度时，启用远离目标检测
        try:
            dx1 = goal[0] - pre_traj[0]
            dy1 = goal[1] - pre_traj[1]
            # 与目标点距离变化
            pre_dist = math.sqrt(dx1 ** 2 + dy1 ** 2)
            # 距离越远，远离目标点变化的作用越大
            dist_cost = (dist - pre_dist) * dist
        except:
            pass

    # cost /= dist

    return cost, dist_cost

def calc_to_omega_cost(trajectory, pre_traj, max_yawrate):
    """
    计算角速度变化的代价
    """
    try:
        if not max_yawrate < 0.01:
            # 用当前周期最大速度来归一化角速度变化量
            cost = abs((trajectory[-1] - pre_traj[-1]) / max_yawrate)
        else:
            cost = 0.0
    except:
        cost = 0.0
     # print(cost)
    return cost


def plot_arrow(x, y, yaw, length=0.5, width=0.1):  # pragma: no cover
    plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
              head_length=width, head_width=width)
    plt.plot(x, y)


def plot_robot(x, y, yaw, config):  # pragma: no cover
    if config.robot_type == RobotType.rectangle:
        outline = np.array([[-config.robot_b, config.robot_f, (config.robot_f), -config.robot_b, -config.robot_b],
                            [config.robot_l, config.robot_l, - config.robot_r, -config.robot_r,config.robot_l]
                            ])
        Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                         [-math.sin(yaw), math.cos(yaw)]])
        outline = (outline.T.dot(Rot1)).T
        outline[0, :] += x
        outline[1, :] += y
        plt.plot(np.array(outline[0, :]).flatten(),
                 np.array(outline[1, :]).flatten(), "-k")
    elif config.robot_type == RobotType.circle:
        circle = plt.Circle((x, y), config.robot_radius, color="b")
        plt.gcf().gca().add_artist(circle)
        out_x, out_y = (np.array([x, y]) +
                        np.array([np.cos(yaw), np.sin(yaw)]) * config.robot_radius)
        plt.plot([x, out_x], [y, out_y], "-k")





class dwa(redisHandler):
    '''
    # dwa 路径规划
    '''
    def __init__(self):
        redisHandler.__init__(self)
        self.url = 'http://117.184.129.18:8000/planning/query/?key='
        
        self.sub_topics = ['rtk_out', 'tracking_in']
        self.pub_topics = ['ctrl_in']

        # 配置参数
        self.config = Config()
        self.config.robot_type = robot_type
        # 目标点误差
        self.delta_goal = 0.40

        # 任务目标kdtree
        self.goal_kdtree = None
        # 障碍物kdtree
        self.ob_kdtree = None

        # to movebase通信格式
        self.max_speed = 80
        self.data_speed = {
                'header':'speed',
                'data':{
                    'y': 1,
                    'angle': 0,
                    'speed': self.max_speed
                    }
                }

        self.start_sub()


    def get_mission(self, key):
        # 从服务器获取任务
        url = self.url + key
        response = requests.get(url)
        if response.status_code == 200:
            try:
                r_json = response.json()
                ob_list = r_json['cmd']
                goal_list = zip(r_json['cx'], r_json['cy'])
                self.ob_kdtree = scipy.spatial.cKDTree(np.array(ob_list))
                self.goal_kdtree = scipy.spatial.cKDTree(np.array(goal_list))
                return True
            except Exception as e:
                print(e)
        return False



    def dwa_plan(self, x, goal, ob):
        # dwa 规划轨迹
        u, predicted_trajectory = dwa_control(x, self.config, goal, ob)
        # 仿真，根据当前x, u计算dt后的x
        # x = motion(x, u, config.dt)  # 机器人仿真
        end_x = predicted_trajectory[-1]
        s_x = predicted_trajectory[0]

        # 到目标点距离
        dx_g = end_x[0]-goal[0]
        dy_g = end_x[1]-goal[1]
        # dx_g = x[0]-goal[0]
        # dy_g = x[1]-goal[1]
        dist_to_goal = math.hypot(dx_g, dy_g)

        if show_animation:
            plt.cla()
            # esc 停止动画
            plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(predicted_trajectory[:, 0], predicted_trajectory[:, 1], "-g")
            plt.plot(x[0], x[1], "xr")
            plt.plot(goal[0], goal[1], "xb")
            plt.plot(ob[:, 0], ob[:, 1], "ok")
            plot_robot(x[0], x[1], x[2], self.config)
            plot_arrow(x[0], x[1], x[2])
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.0001)

        return s_x,  dist_to_goal


    def run(robot_type=RobotType.rectangle):
        print(__file__ + " start!!")
        # 自动运行标志
        tracking_flag = False
        # 初始化机器人状态 [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
        x = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        # x = np.array([0.0, 0.0, math.pi / 8.0, 0.0, 0.0])
        # 目标位置 [x(m), y(m)]
        goal_list = np.array([[2,-0.25],[3,0],[4,0],[5,0],[6,0],[7,0],[8,2],[8,3],[10,3],[15,6],[18,10],[0, 10]])
        goal = goal_list[5]
        # 障碍物 [x(m) y(m), ....]
        ob = np.array([[-1, -1],
                       [0, 2.5],
                       [2, 0.80],
                       [4.0, 2.0],
                       [5.0, 4.0],
                       [3.0, 5.0],
                       [6.0, 6.0],
                       [5.0, 9.0],
                       [8.0, 9.0],
                       [7.0, 9.0],
                       [8.0, 10.0],
                       [9.0, 11.0],
                       [12.0, 13.0],
                       [12.0, 12.0],
                       [15.0, 15.0],
                       [13.0, 13.0]
                       ])
        ob_radar = np.array([])

        # input [forward speed, yaw_rate]
        
        # for goal in goal_list:
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
                    pre_t = now

                    # dwa轨迹规划
                    goal_x, dits_to_goal = self.dwa_plan(x, goal, ob)
                    speed = goal_x[3] / self.config.max_speed
                    yaw = goal_x[4] / self.config.max_yawrate
                    self.data_speed['data']['angle'] = yaw
                    if len(ob_radar) == 0:
                        # 雷达错误
                        self.data_speed['data']['y'] = 0
                    else:
                        self.data_speed['data']['y'] = 1
                    self.data_speed['data']['speed'] = speed
                    rc.publish('move_base_in', json.dumps(self.data_speed))

            elif header == 'radar_ob' or 'radar_err':
                # 开启自动, 获取任务信息
                ob_radar = np.array(data)


            elif header == 'auto_on':
                # 开启自动, 获取任务信息
                if tracking_flag:
                    # 如果在自动运行中，则跳过
                    continue
                if self.get_mission(data)
                    tracking_flag = True
                else:
                    tracking_flag = False

            elif header == 'auto_off':
                # 关闭自动， 停止运行
                tracking_flag = False
                self.data_speed['data']['angle'] = 0
                self.data_speed['data']['y'] = 0
                rc.publish('move_base_in', json.dumps(self.data_speed))

            elif header == 'get_pos':
                # 当前坐标发送
                pos_msg = {
                        'header': 'cur_pos',
                        'data': [x[0], x[1], x[2]]
                        }
                rc.publish('tcp_in', json.dumps(pos_msg))












if __name__ == '__main__':
    # main(robot_type=RobotType.circle)
    robot = dwa()
    robot.run()
