# _*_ coding: UTF-8 _*_

import sys
import json
import time
from math import cos, sin
import serialCmd
from motorTongyiIxL import Motor
# from motorDMK import Motor
# from motor055a import Motor
from redisHandler import redisHandler


class moveBase(redisHandler):
    """
    机器人底盘
    """
    def init(self):
        # 障碍物传感器启用开关
        self.ob_isvalid = False
        self.pub_topics = ['move_base_out']
        self.sub_topics = ['move_base_in']
        self.left_wheel = Motor(1, '/dev/ttyS2')
        # self.left_wheel1 = Motor(1, '/dev/ttyS2')
        self.right_wheel = Motor(1, '/dev/ttyS3')
        # self.right_wheel1 = Motor(2, '/dev/ttyS3')
        self.base_info = {'left':[0,0,0,0], 'right':[0,0,0,0]}
        self.init_motor()
        self.start_sub()
    
    def init_motor(self):
        self.left_wheel.ini_motor()
        self.right_wheel.ini_motor()
    
    def run(self):
        # 主进程
        self.init()
        pre_time = time.time()
        pre_t_ob = time.time() 
        timeout = 2.0
        # 有障碍物标志
        ob_flag = False
        while True:
            try:
                now = time.time()
                if now - pre_t_ob >= timeout:
                    # 超时未收到无障碍物标志
                    ob_flag = True
                if now - pre_time >= timeout or (ob_flag and self.ob_isvalid):
                    # 有障碍物或者控制命令超时未收到
                    pre_time = now
                    print('time out')
                    self.write_speed(0, 0, 0)
                data= self.q_get_nowait()
                if not data:
                    time.sleep(0.1)
                    continue
                header = data['header']
                data = data['data']
                if header == 'init':
                    # 初始化
                    pre_time = time.time()
                    self.init_motor()
                elif header == 'speed' and (not (ob_flag and self.ob_isvalid)):
                    # 无障碍物，或者障碍物传感器未启用，速度发送
                    pre_time = time.time()
                    self.write_speed(data['y'], data['angle'], data['speed'])
                elif header == 'heartbeat':
                    # 心跳
                    pre_time = time.time()
                    base_info = self.read_base_info()
                    data_pub = {
                            'header':'base_info',
                            'data':base_info
                            }
                    self.pub_all(data_pub)
                elif header == 'get_base_info':
                    # 获取基本信息
                    base_info = self.read_base_info()
                    data_pub = {
                            'header':'base_info',
                            'data':base_info
                            }
                    self.pub_all(data_pub)
                elif header == 'ob_invalid':
                    # 无障碍物
                    pre_t_ob = time.time()
                    ob_flag = False
                elif header == 'ob_on':
                    # 开启障碍物检测
                    self.ob_isvalid = True
                elif header == 'ob_off':
                    # 关闭障碍物检测
                    self.ob_isvalid = False
            except Exception as e:
                print('move base err')
                # print(e)

    def write_speed(self, y, angle, speed):
        """
        :param y:
        :param angle:
        :param speed
        :return:
        """
        # 根据车体样式更改，前后，左右
        # y = -y
        angle = -angle
        l = 0.0
        r = 0.0
        angle = 2 * sin(angle)
        if angle >= 0:
            l = 1
            r = - 1 + angle
        else:
            r = -1
            l = 1 + angle
        l  = l * speed / 100.0 * y
        # direction adjust
        r  = -r * speed / 100.0 * y
        print('speed_l: ', l, 'speed_r: ', r)
        self.left_wheel.write_speed(l)
        # self.left_wheel1.write_speed(l)
        self.right_wheel.write_speed(r)
        # self.right_wheel1.write_speed(r)
        print(self.read_base_info())
        return True

    def read_speed(self):
        """
        :return: v, v_yaw, 单位： m/s, rad/s
        """
        v_l = self.left_wheel.read_speed()
        v_r = self.right_wheel.read_speed()
        return v_l, v_r

    def read_base_info(self):
        """
        :return: v, u, i, err_code 单位： r/min, V, A
        """
        info_l = self.left_wheel.read_base_info()
        info_r = self.right_wheel.read_base_info()
        return {'left': info_l, 'right': info_r}


if __name__ == '__main__':
    car = moveBase()
    car.run()
