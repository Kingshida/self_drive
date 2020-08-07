# _*_ coding: UTF-8 _*_

import sys
import json
import time
import serialCmd
import matplotlib.pyplot as plt
from redisHandler import redisHandler

class ctrl(redisHandler):
    """
    机器人下位机控制器
    """
    def init(self):
        self.sub_topics = ['radar_out']
        # self.pub_topics = ['radar_out']
        self.start_sub()

    def plot_ob(self, ox, oy):
        plt.cla()
        plt.axis('equal')
        plt.plot(ox, oy, "+r", label="course")
        plt.pause(0.001)
   
    def run(self):
        self.init()
        while True:
            try:
                data = self.q_get()
                if data:
                    ox, oy = data['data']
                    self.plot_ob(ox, oy)
                    # print(ox, oy)
            except Exception as e:
                print('ctrl err')
                print(e)


if __name__ == '__main__':
    cmd = ctrl()
    cmd.run()
