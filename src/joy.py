# _*_ coding: UTF-8 _*_

import sys
import time
import json
import serial
from redisHandler import redisHandler

class joy(redisHandler):
    """
    机器人主函数
    """
    def init(self, port = '/dev/ttyS4'):
        print('joy init')
        self.ser = serial.Serial(port=port, baudrate=9600, timeout=0.2, write_timeout=0.2)
        self.joy_on = False

    def read_data(self):
        res = None
        if self.ser.in_waiting:
            data = self.ser.read_until('#')
            if data:
                if data[0] == '*':
                    if data == '*JOYON#':
                        # 手柄打开
                        self.ser.write(data)
                        self.joy_on = True
                    else:
                        # ctrl_in消息
                        res = {
                                'header':'ctrl',
                                'data':data
                                }
                elif data[0] == '$':
                    # 底盘控制消息
                    data = data[1:-1]
                    if ',' in data:
                        data = data.split(',')
                        speed = int(data[0])
                        angle = int(data[1]) / 100.0
                        y = 0
                        if speed > 0:
                            y = 1
                        elif speed < 0:
                            y = -1
                            speed = -speed
                        if speed < 6:
                            speed = 0
                        res = {
                                'header':'speed',
                                'data':{
                                    'y':y,
                                    'angle':angle,
                                    'speed':speed
                                    }
                                }
                    else:
                        res = {
                                'header': data,
                                'data':''
                                }
        return res


   
    def run(self):
        self.init()
        pre_time = time.time()
        joy_timeout = 100
        while True:
            try:
                now = time.time()
                if now - pre_time > joy_timeout and self.joy_on:
                    # 手柄超时关闭
                    pre_time = now
                    self.joy_on = False
                    self.ser.write('*JOYOFF#')
                    print('JOYOFF')
                data = self.read_data()
                if data and self.joy_on:
                    # print(data)
                    pre_time = now
                    self.rc.publish('tcp_out', json.dumps(data))

                time.sleep(0.1)
            except Exception as e:
                print(e)


if __name__ == '__main__':
    main_p = joy()
    main_p.run()
