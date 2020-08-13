# coding:utf-8
import math
import time
import sys
import json
import redis
import numpy as np
from redisHandler import redisHandler
from motor055a import Motor
from sensorLinear import sensorLinear


class trackSwing(redisHandler):
    '''
    # tracking 路径跟踪
    '''
    def __init__(self):
        redisHandler.__init__(self)

        self.sub_topics = ['track_cmd_in']
        self.pub_topics = ['ctrl_in']
        
        # 电机工作角度
        self.mission_angle = 150385
        # 位移传感器工作长度mm
        self.mission_length = 125
        

        self.motor = Motor(1, '/dev/ttyS5')
        self.motor_init()

        self.sensor = sensorLinear(3, '/dev/ttyS5')

        self.start_sub()


    def motor_init(self):
        # motor先回原点，再到任务点
        while not self.motor.go_zero():
            print('go zero')
            time.sleep(5)

    def swing(self, speed):
        # 摆动， 速度和方向
        pos = 0
        if speed >= 0:
            pos = self.mission_angle
        else:
            speed = -speed
        self.motor.write_speed(speed)
        self.motor.go_abpos(pos)
    
    def pid_para(self, pre_d0, pre_d1, d, dt):
        # pid参数
        kp = 0.0222*0.20
        kd = 0.0003
        if pre_d0 is None:
            pre_d0 = d
        if pre_d1 is None:
            pre_d1 = pre_d0
        k = 0.0
        d_p = kp * d
        d_d = kd * ((d - pre_d0) - (pre_d0 - pre_d1)) / dt
        d_d = kd * (d - pre_d0) / dt
        k = d_p + d_d
        if k > 0 and d > 0:
            k *= 2.00
        print('d: {}, d_p: {}, d_d: {}, k: {}'.format(round(d, 3), round(d_p, 3), round(d_d, 3), round(k, 3)))
        return k

    def run(self):
        # 路径跟踪方法
        # 命令路径点表
        tracking_flag = True
        # 命令执行位置公差, 单位m
        pre_t = time.time()
        pre_d0 = None
        pre_d1 = None
        d = 0.0
        print('start')
        while True:
            time.sleep(0.02)
            data = self.q_get_nowait()
            if data:
                header = data['header']
            else:
                header = None
            if header == 'auto_on' or header == 'auto_continue':
                # 开启自动,展开摆臂
                self.swing(0.2)
                if not tracking_flag:
                    # 如果在自动运行中，则跳过
                    tracking_flag = True
            elif header == 'auto_off':
                # 关闭自动
                tracking_flag = False
            if tracking_flag:
                now = time.time()
                dt = now - pre_t
                pre_t = now
                length = self.sensor.read_pos()
                d = length - self.mission_length
                if length > 42949600:
                    d = 0
                    pre_d0 = 0
                    pre_d1 = 0
                speed = self.pid_para(pre_d0, pre_d1, d, dt)
                pre_d1 = pre_d0
                pre_d0 = d
                self.swing(speed)


if __name__ == '__main__':
    node_track = trackSwing()
    node_track.run()
