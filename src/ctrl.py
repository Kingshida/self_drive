# _*_ coding: UTF-8 _*_

import sys
import json
import time
import serialCmd
from redisHandler import redisHandler

class ctrl(redisHandler):
    """
    机器人下位机控制器
    """
    def init(self):
        self.ser = serialCmd.serialCmd()    # 下位机控制端口
        self.sub_topics = ['ctrl_in']
        self.start_sub()
   
    def run(self):
        self.init()
        while True:
            try:
                data = self.q_get()
                if data:
                    data = data['data']
                    data = '*' + str(data) + '#'
                    # print(data)
                    if not self.ser.serial.isOpen():
                        self.ser.open_port()
                    self.ser.send_cmd(data)
            except Exception as e:
                print('ctrl err')
                print(e)


if __name__ == '__main__':
    cmd = ctrl()
    cmd.run()
